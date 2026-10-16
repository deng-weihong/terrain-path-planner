import assert from 'node:assert/strict'
import { mkdir, writeFile } from 'node:fs/promises'
import { chromium } from 'playwright'
import { Matrix4, Vector3 } from 'three'

const baseUrl = process.env.PATH_PLANNER_URL || 'http://127.0.0.1:5181'
const artifactDir = new URL('../../artifacts/', import.meta.url)
const initialDataset = process.env.PATH_PLANNER_TEST_LOCAL === '1' ? 'site-map' : 'obstacles'
await mkdir(artifactDir, { recursive: true })
const browser = await chromium.launch({ headless: true, args: ['--no-sandbox', '--enable-webgl', '--ignore-gpu-blocklist', '--use-angle=swiftshader', '--enable-unsafe-swiftshader'] })
const checks = []
let activePage

async function settled(page) {
  await page.locator('.validation-overview strong').waitFor({ timeout: 90000 })
  await page.waitForFunction(() => !document.querySelector('.computing-indicator'), null, { timeout: 90000 })
  await page.locator('canvas[data-ready="true"]').waitFor({ timeout: 30000 })
}

async function pixels(page) {
  await page.evaluate(() => new Promise(resolve => requestAnimationFrame(() => requestAnimationFrame(resolve))))
  return page.locator('canvas').evaluate(canvas => {
    const gl = canvas.getContext('webgl2')
    if (!gl) throw new Error('Three.js WebGL2 context missing')
    const data = new Uint8Array(canvas.width * canvas.height * 4)
    gl.readPixels(0, 0, canvas.width, canvas.height, gl.RGBA, gl.UNSIGNED_BYTE, data)
    let route = 0; let reference = 0; let map = 0
    for (let i = 0; i < data.length; i += 4) {
      const [r, g, b] = data.slice(i, i + 3)
      if (r < 60 && g > 80 && g < 170 && b > 50 && b < 150) route++
      if (r > 150 && g > 80 && g < 185 && b < 80) reference++
      if (r > 70 && r < 240 && g > 95 && b > 90 && (g > r + 10 || b > r + 15)) map++
    }
    return { route, reference, map, width: canvas.width, height: canvas.height,
      engine: canvas.dataset.engine, voxels: Number(canvas.dataset.voxels), frames: Number(canvas.dataset.frames) }
  })
}

async function layout(page, name) {
  const problems = await page.evaluate(() => {
    const issues = []
    if (document.documentElement.scrollWidth > innerWidth + 1) issues.push('horizontal page overflow')
    for (const element of document.querySelectorAll('button, .metric, .parameter, .section-heading, .endpoint, .endpoint-bar, .map-toolbar')) {
      if (element.getBoundingClientRect().width && element.scrollWidth > element.clientWidth + 2) issues.push(`overflow: ${element.textContent}`)
    }
    return issues
  })
  assert.deepEqual(problems, [], name)
  const stats = await pixels(page)
  assert(stats.route > 100, `${name}: optimized path missing: ${JSON.stringify(stats)}`)
  assert(stats.reference > 70, `${name}: baseline path missing: ${JSON.stringify(stats)}`)
  assert(stats.map > 500, `${name}: map missing: ${JSON.stringify(stats)}`)
  checks.push({ name, pixels: stats })
  await page.screenshot({ path: new URL(`${name}.png`, artifactDir).pathname, fullPage: true })
}

async function mapPosition(page, dataset, point) {
  await page.getByRole('button', { name: '俯视视角', exact: true }).click()
  await page.getByRole('button', { name: '完整地图', exact: true }).click()
  await pixels(page)
  const box = await page.locator('canvas').boundingBox()
  const camera = await page.locator('canvas').evaluate(canvas => JSON.parse(canvas.dataset.view))
  const position = new Vector3(point[0], point[1], ((point[2] ?? dataset.heightRange.max) + dataset.resolution / 2) * camera.heightScale)
  position.applyMatrix4(new Matrix4().fromArray(camera.view)).applyMatrix4(new Matrix4().fromArray(camera.projection))
  return { x: box.x + (position.x + 1) * box.width / 2, y: box.y + (1 - position.y) * box.height / 2 }
}

async function pickEndpoint(page, dataset, name, point) {
  await page.getByRole('button', { name, exact: true }).click()
  const position = await mapPosition(page, dataset, point)
  const picked = page.waitForResponse(response => response.url().endsWith('/pick'))
  await page.mouse.click(position.x, position.y)
  const response = await picked
  assert.equal(response.status(), 200)
  const sent = response.request().postDataJSON()
  const result = await response.json()
  assert.equal(sent.z, result.point[2], 'raycast selected a different height layer')
  await page.waitForFunction(() => document.querySelector('.endpoint-bar')?.getAttribute('aria-busy') === 'false')
  return result
}

async function exportResult(page) {
  const [download] = await Promise.all([
    page.waitForEvent('download'), page.getByRole('button', { name: '导出', exact: true }).click(),
  ])
  const stream = await download.createReadStream()
  const chunks = []
  for await (const chunk of stream) chunks.push(chunk)
  return JSON.parse(Buffer.concat(chunks).toString())
}

async function manualEndpoints(page, context, viewport) {
  const dataset = await (await context.request.get(`${baseUrl}/api/datasets/zigzag`)).json()
  await page.getByLabel('验证地图', { exact: true }).selectOption('zigzag')
  await settled(page)
  const start = await pickEndpoint(page, dataset, '选择起点', [2.05, 3.05])
  assert.equal(await page.getByRole('button', { name: '选择终点', exact: true }).getAttribute('aria-pressed'), 'true')
  if (viewport === 'mobile') await page.getByRole('button', { name: '算法参数', exact: true }).click()
  assert.equal(await page.getByRole('button', { name: '运行优化', exact: true }).isDisabled(), true)
  const goal = await pickEndpoint(page, dataset, '选择终点', [20.05, 9.05])
  assert.equal(await page.getByRole('button', { name: '平移地图', exact: true }).getAttribute('aria-pressed'), 'true')
  assert.equal(await page.locator('.validation-overview strong').count(), 0, 'old result remained after selecting endpoints')
  await page.getByRole('button', { name: '规划路线', exact: true }).click()
  await settled(page)
  const planned = await exportResult(page)
  assert.equal(planned.planning.engine, 'traversable-graph-v1')
  assert.deepEqual(planned.sourceRoute[0], start.point)
  assert.deepEqual(planned.sourceRoute.at(-1), goal.point)
  assert.deepEqual(planned.path[0], start.point)
  assert.deepEqual(planned.path.at(-1), goal.point)
  assert.equal(planned.validation.newViolationCount, 0)
  await layout(page, `${viewport}-manual-endpoints`)

  await page.getByRole('button', { name: '运行优化', exact: true }).click()
  await settled(page)
  const repeated = await exportResult(page)
  assert.deepEqual(repeated.sourceRoute, planned.sourceRoute, 'rerun lost original graph vertices')
  assert.deepEqual(repeated.planning, planned.planning)

  const before = await page.getByTestId('start-endpoint').innerText()
  await page.getByRole('button', { name: '选择起点', exact: true }).click()
  const position = await mapPosition(page, dataset, [6, 6])
  let requests = 0
  const countPick = request => { if (request.url().endsWith('/pick')) requests++ }
  page.on('request', countPick)
  await page.mouse.move(position.x, position.y)
  await page.mouse.down()
  await page.mouse.move(position.x + 30, position.y + 20, { steps: 5 })
  await page.mouse.up()
  await pixels(page)
  assert.equal(requests, 0, 'drag accidentally selected an endpoint')
  await page.mouse.down()
  await page.locator('canvas').dispatchEvent('pointercancel', { pointerId: 1, clientX: position.x, clientY: position.y, button: 0 })
  await page.mouse.up()
  assert.equal(requests, 0, 'pointer cancellation selected an endpoint')
  if (viewport === 'mobile') {
    const touch = await context.newCDPSession(page)
    const center = { x: Math.round(position.x), y: Math.round(position.y) }
    await touch.send('Input.dispatchTouchEvent', { type: 'touchStart', touchPoints: [
      { x: center.x - 15, y: center.y, id: 1 }, { x: center.x + 15, y: center.y, id: 2 },
    ] })
    await touch.send('Input.dispatchTouchEvent', { type: 'touchMove', touchPoints: [
      { x: center.x - 25, y: center.y, id: 1 }, { x: center.x + 25, y: center.y, id: 2 },
    ] })
    await touch.send('Input.dispatchTouchEvent', { type: 'touchEnd', touchPoints: [] })
    await pixels(page)
    assert.equal(requests, 0, 'pinch accidentally selected an endpoint')
    await touch.detach()
  }
  page.off('request', countPick)
  assert.equal(await page.getByTestId('start-endpoint').innerText(), before)
  await page.keyboard.press('Escape')
  assert.equal(await page.getByRole('button', { name: '平移地图', exact: true }).getAttribute('aria-pressed'), 'true')

  const missing = await mapPosition(page, dataset, [-1, 5])
  await page.getByRole('button', { name: '选择起点', exact: true }).click()
  await page.mouse.click(missing.x, missing.y)
  await page.locator('.error-banner').waitFor()
  assert.match(await page.locator('.error-banner').innerText(), /没有可通行体素/)
  assert.equal(await page.getByTestId('start-endpoint').innerText(), before)
  await page.getByRole('button', { name: '关闭错误', exact: true }).click()

  await page.getByRole('button', { name: '交换起终点', exact: true }).click()
  await page.getByRole('button', { name: '规划路线', exact: true }).click()
  await settled(page)
  const swapped = await exportResult(page)
  assert.deepEqual(swapped.path[0], goal.point)
  assert.deepEqual(swapped.path.at(-1), start.point)
  await page.getByRole('button', { name: '恢复样例路线', exact: true }).click()
  await settled(page)
  const restored = await exportResult(page)
  assert.equal(restored.planning, null)
  assert.deepEqual(restored.sourceRoute, dataset.route)
  checks.push({ name: `${viewport}-endpoint-interactions`, result: 'pick, plan, rerun, invalid click, pan, cancel, escape, swap, restore passed' })
}

async function focusedScreenshot(page, context, target, filename, zoom = 14) {
  const dataset = await (await context.request.get(`${baseUrl}/api/datasets/${initialDataset}`)).json()
  const position = await mapPosition(page, dataset, target)
  await page.getByRole('button', { name: '平移地图', exact: true }).click()
  const box = await page.locator('canvas').boundingBox()
  const center = { x: box.x + box.width / 2, y: box.y + box.height / 2 }
  await page.mouse.move(position.x, position.y)
  await page.mouse.down()
  await page.mouse.move(center.x, center.y, { steps: 12 })
  await page.mouse.up()
  for (let i = 0; i < Math.round(Math.log(zoom) / Math.log(1.5)); i++) await page.getByRole('button', { name: '放大', exact: true }).click()
  const close = await pixels(page)
  assert(close.map > 500, `${filename}: close-up map is blank`)
  await page.locator('canvas').screenshot({ path: new URL(filename, artifactDir).pathname })
  await page.getByRole('button', { name: '完整地图', exact: true }).click()
}

async function threeDimensionalViews(page, context, viewport) {
  const dataset = await (await context.request.get(`${baseUrl}/api/datasets/${initialDataset}`)).json()
  const initial = await pixels(page)
  assert.equal(initial.engine, 'threejs')
  assert.equal(initial.voxels, dataset.voxelCount)
  assert.equal(await page.getByRole('button', { name: '三维视角', exact: true }).getAttribute('aria-pressed'), 'true')
  const before = await page.locator('canvas').getAttribute('data-view')
  const box = await page.locator('canvas').boundingBox()
  await page.getByRole('button', { name: '旋转地图', exact: true }).click()
  await page.mouse.move(box.x + box.width * .55, box.y + box.height * .55)
  await page.mouse.down()
  await page.mouse.move(box.x + box.width * .55 + 45, box.y + box.height * .55 + 20, { steps: 8 })
  await page.mouse.up()
  const rotated = await pixels(page)
  assert.notEqual(await page.locator('canvas').getAttribute('data-view'), before, 'orbit did not move the camera')
  assert(rotated.frames > initial.frames + 2, '3D view did not update while rotating')
  assert(rotated.map > 500, 'map disappeared after rotation')
  await page.screenshot({ path: new URL(`${viewport}-3d-rotated.png`, artifactDir).pathname, fullPage: true })
  await page.getByRole('button', { name: '三维视角', exact: true }).click()
  await page.getByRole('button', { name: '完整地图', exact: true }).click()
  await page.getByLabel('高程显示倍率', { exact: true }).fill('5')
  const amplified = await pixels(page)
  assert.equal(JSON.parse(await page.locator('canvas').getAttribute('data-view')).heightScale, 5)
  assert(amplified.map > 500, 'height scaling made the map disappear')
  await page.getByRole('button', { name: '点云显示', exact: true }).click()
  const cloud = await pixels(page)
  assert(cloud.map > 500, 'point cloud is blank')
  await page.screenshot({ path: new URL(`${viewport}-3d-point-cloud.png`, artifactDir).pathname, fullPage: true })
  await page.getByRole('button', { name: '体素显示', exact: true }).click()
  if (viewport === 'desktop') {
    const column = await (await context.request.post(`${baseUrl}/api/datasets/${initialDataset}/pick`, {
      data: { x: dataset.route[0][0], y: dataset.route[0][1] },
    })).json()
    const selected = await pickEndpoint(page, dataset, '选择起点', [column.point[0], column.point[1], Math.max(...column.heights)])
    assert.equal(selected.point[2], Math.max(...column.heights), 'scaled raycast did not preserve visible height')
    await page.getByRole('button', { name: '恢复样例路线', exact: true }).click()
    await settled(page)
  }
  await page.getByLabel('高程显示倍率', { exact: true }).fill('1')
  await page.getByRole('button', { name: '三维视角', exact: true }).click()
  await page.getByRole('button', { name: '完整地图', exact: true }).click()
  await pixels(page)
  checks.push({ name: `${viewport}-threejs`, result: 'all XYZ voxels, orbit, redraw, points, voxels, height scaling passed', voxels: dataset.voxelCount })
}

try {
  for (const viewport of [{ name: 'desktop', width: 1440, height: 1000 }, { name: 'laptop', width: 1280, height: 800 }, { name: 'mobile', width: 390, height: 844 }]) {
    const context = await browser.newContext({ viewport, deviceScaleFactor: 1, acceptDownloads: true, hasTouch: viewport.name === 'mobile' })
    const page = await context.newPage()
    activePage = page
    const errors = []
    page.on('pageerror', error => errors.push(error.message))
    await page.goto(baseUrl)
    await settled(page)
    if (initialDataset !== 'obstacles') {
      await page.getByLabel('验证地图', { exact: true }).selectOption(initialDataset)
      await settled(page)
    }
    console.log(`${viewport.name}: map and 3D controls`)
    await layout(page, `${viewport.name}-${initialDataset}`)
    assert.equal(await page.locator('.validation-overview strong').innerText(), '0')
    await threeDimensionalViews(page, context, viewport.name)

    if (viewport.name === 'desktop') {
      assert.equal(await page.title(), 'Terrain Path Planner')
      assert.doesNotMatch(await page.locator('body').innerText(), /实验室|实验环境|本次实验/)
      await page.getByText('XY C² 连续', { exact: true }).waitFor()
      if (initialDataset === 'site-map') {
        await focusedScreenshot(page, context, [-22.35, -24.05], 'desktop-reported-bend-v3.png')
        await page.getByRole('button', { name: '优化', exact: true }).click()
        await focusedScreenshot(page, context, [-11.5, -9.2], 'desktop-unnecessary-bend-v3.png')
        await focusedScreenshot(page, context, [-50.5, -80.4], 'desktop-boundary-corner-v3.png', 17)
      }
      await page.getByRole('button', { name: '对比', exact: true }).click()
      await page.getByRole('button', { name: '原始', exact: true }).click()
      const original = await pixels(page)
      await page.getByRole('button', { name: '优化', exact: true }).click()
      const optimized = await pixels(page)
      assert(optimized.route > original.route * 2, 'view mode did not change visible paths')
      await page.getByRole('button', { name: '对比', exact: true }).click()
      await page.getByRole('button', { name: '放大', exact: true }).click()
      await page.getByRole('button', { name: '完整地图', exact: true }).click()
      await page.getByRole('button', { name: '聚焦路线', exact: true }).click()
      await page.getByLabel('保留段', { exact: true }).check()
      await page.getByLabel('修复前候选', { exact: true }).check()
      await page.screenshot({ path: new URL('desktop-validation-layers.png', artifactDir).pathname })
      await page.getByLabel('保留段', { exact: true }).uncheck()
      await page.getByLabel('修复前候选', { exact: true }).uncheck()
      await page.getByLabel('最大偏离数值').fill('1.0')
      await page.getByRole('button', { name: '运行优化', exact: true }).click()
      await settled(page)
      assert.equal(await page.locator('.history-item').count(), 2)

      const downloadReady = page.waitForEvent('download')
      await page.getByRole('button', { name: '导出', exact: true }).click()
      const download = await downloadReady
      const stream = await download.createReadStream()
      const chunks = []
      for await (const chunk of stream) chunks.push(chunk)
      const exported = JSON.parse(Buffer.concat(chunks).toString())
      assert.equal(exported.options.max_deviation, 1)
      assert.equal(exported.validation.newViolationCount, 0)
      assert.equal(exported.validation.clearanceViolationCount, 0)
      assert.equal(exported.solver.continuity, 'C2')
      assert.equal(exported.algorithm, 'corridor-shortcut-bspline-v3')
      assert(exported.shortcut.segments > 0)
      if (initialDataset === 'site-map') assert(exported.solver.heightRematchedSamples > 0)
      assert(exported.path.length > 100)
      await writeFile(new URL(`${initialDataset}-result.json`, artifactDir), JSON.stringify(exported, null, 2))

      await page.locator('input[type=file]').setInputFiles({ name: 'bad-route.json', mimeType: 'application/json', buffer: Buffer.from('{"path":[[0,0],[1,1]]}') })
      await page.locator('.error-banner').waitFor()
      await page.getByRole('button', { name: '关闭错误', exact: true }).click()
      const fixture = await (await context.request.get(`${baseUrl}/api/datasets/${initialDataset}`)).json()
      await page.locator('input[type=file]').setInputFiles({ name: 'imported-reference.json', mimeType: 'application/json', buffer: Buffer.from(JSON.stringify(fixture.route)) })
      await settled(page)
      assert.equal(await page.locator('.route-filename').innerText(), 'imported-reference.json')

      const start = await pickEndpoint(page, fixture, '选择起点', fixture.route[0])
      const goal = await pickEndpoint(page, fixture, '选择终点', fixture.route.at(-1))
      await page.getByRole('button', { name: '规划路线', exact: true }).click()
      await settled(page)
      const manual = await exportResult(page)
      assert.deepEqual(manual.sourceRoute[0], start.point)
      assert.deepEqual(manual.sourceRoute.at(-1), goal.point)
      assert.equal(manual.planning.engine, 'traversable-graph-v1')
      await writeFile(new URL(`${initialDataset}-manual-result.json`, artifactDir), JSON.stringify(manual, null, 2))
      await layout(page, `desktop-${initialDataset}-manual`)
      checks.push({ name: `${initialDataset}-manual-plan`, timeMs: manual.planning.timeMs, iterations: manual.planning.iterations,
        newViolations: manual.validation.newViolationCount, sharpTurns: manual.optimized.denseSharpTurns })

      await page.getByRole('button', { name: '保守', exact: true }).click()
      await page.getByLabel('验证地图', { exact: true }).selectOption('obstacles')
      await settled(page)
      assert.equal(await page.locator('.status-tag').innerText(), '仍有急转路段')
      assert.equal(await page.getByLabel('急转点', { exact: true }).isChecked(), true)
      await page.screenshot({ path: new URL('desktop-sharp-point-warning.png', artifactDir).pathname })
      await page.getByRole('button', { name: '均衡', exact: true }).click()

      for (const id of ['zigzag', 'obstacles']) {
        await page.getByLabel('验证地图', { exact: true }).selectOption(id)
        await settled(page)
        await layout(page, `desktop-${id}`)
        assert.equal(await page.locator('.status-tag').innerText(), '几何校验通过')
      }
      await manualEndpoints(page, context, viewport.name)
      await page.route('**/api/optimize', route => route.fulfill({ status: 500, contentType: 'application/json', body: '{"detail":"模拟计算失败"}' }))
      await page.getByRole('button', { name: '运行优化', exact: true }).click()
      await page.locator('.error-banner').waitFor()
      assert.match(await page.locator('.error-banner').innerText(), /模拟计算失败/)
      assert.equal(await page.getByRole('button', { name: '运行优化', exact: true }).isDisabled(), false)
      checks.push({ name: 'interactions', result: 'modes, layers, parameter rerun, history, export, import, datasets, error recovery passed' })
      await page.unroute('**/api/optimize')
      const voxelUrl = '**/api/datasets/obstacles/voxels.bin?*'
      await page.route(voxelUrl, route => route.fulfill({ status: 503, body: 'map temporarily unavailable' }))
      await page.reload()
      await page.locator('.scene-state.failed').waitFor()
      assert.match(await page.locator('.scene-state.failed').innerText(), /三维地图加载失败/)
      await page.unroute(voxelUrl)
      await page.getByRole('button', { name: '重新加载', exact: true }).click()
      await settled(page)
      const publicMap = await (await context.request.get(`${baseUrl}/api/datasets/obstacles`)).json()
      assert.equal((await pixels(page)).voxels, publicMap.voxelCount)
      checks.push({ name: 'threejs-map-recovery', result: 'binary map load failure and fresh WebGL retry passed' })
    }
    if (viewport.name === 'mobile') {
      await manualEndpoints(page, context, viewport.name)
      if (!await page.getByRole('button', { name: '运行优化', exact: true }).isVisible()) await page.getByRole('button', { name: '算法参数', exact: true }).click()
      assert.equal(await page.getByRole('button', { name: '运行优化', exact: true }).isVisible(), true)
      await page.getByRole('button', { name: '保守', exact: true }).click()
      await page.getByRole('button', { name: '运行优化', exact: true }).click()
      await settled(page)
      await layout(page, 'mobile-parameters')
    }
    assert.deepEqual(errors, [], `${viewport.name}: browser errors`)
    await context.close()
  }
  await writeFile(new URL('browser-checks.json', artifactDir), JSON.stringify(checks, null, 2))
  console.log(JSON.stringify(checks, null, 2))
} catch (error) {
  if (activePage && !activePage.isClosed()) {
    await activePage.screenshot({ path: new URL('browser-failure.png', artifactDir).pathname, fullPage: true })
    await writeFile(new URL('browser-failure.txt', artifactDir), await activePage.locator('body').innerText())
  }
  throw error
} finally { await browser.close() }
