import { spawn } from 'node:child_process'
import { once } from 'node:events'
import { createServer } from 'node:net'
import { fileURLToPath } from 'node:url'

const root = fileURLToPath(new URL('../../', import.meta.url))
let server
let output = ''

async function startApi() {
  const probe = createServer()
  probe.listen(0, '127.0.0.1')
  await once(probe, 'listening')
  const { port } = probe.address()
  await new Promise((resolve, reject) => probe.close(error => error ? reject(error) : resolve()))
  const url = `http://127.0.0.1:${port}`
  server = spawn(process.execPath, ['scripts/python.mjs', '-m', 'terrain_planner.viewer', '--port', String(port)], {
    cwd: root,
    env: { ...process.env, PATH_PLANNER_NO_LOCAL_DATA: process.env.PATH_PLANNER_TEST_LOCAL === '1' ? '0' : '1' },
    stdio: ['ignore', 'pipe', 'pipe'],
  })
  for (const stream of [server.stdout, server.stderr]) stream.on('data', data => { output = (output + data).slice(-16000) })
  const deadline = Date.now() + 30000
  while (Date.now() < deadline) {
    if (server.exitCode !== null) throw new Error(`Viewer exited before startup:\n${output}`)
    try {
      const response = await fetch(`${url}/api/health`, { signal: AbortSignal.timeout(1000) })
      if (response.ok && (await response.json()).project === 'terrain-path-planner') return url
    } catch { /* Wait for the local API to bind. */ }
    await new Promise(resolve => setTimeout(resolve, 100))
  }
  throw new Error(`Viewer did not become ready:\n${output}`)
}

try {
  const url = process.env.PATH_PLANNER_URL || await startApi()
  const check = spawn(process.execPath, ['tests/web/browser-check.mjs'], {
    cwd: root, env: { ...process.env, PATH_PLANNER_URL: url }, stdio: 'inherit',
  })
  const [code] = await once(check, 'exit')
  process.exitCode = code ?? 1
} catch (error) {
  console.error(error)
  process.exitCode = 1
} finally {
  if (server && server.exitCode === null) {
    const closed = once(server, 'exit')
    server.kill('SIGTERM')
    await closed
  }
}
