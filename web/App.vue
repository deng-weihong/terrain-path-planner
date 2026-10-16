<script setup lang="ts">
import { computed, nextTick, onBeforeUnmount, onMounted, reactive, ref } from 'vue'
import { Activity, ArrowDownToLine, ArrowLeftRight, Check, ChevronDown, CircleAlert, CircleCheck,
  Flag, Hand, History, LoaderCircle, Map, MapPin, Play, Rotate3D, RotateCcw, Route, Settings2,
  ShieldCheck, Upload, X } from '@lucide/vue'
import MapCanvas from './MapCanvas.vue'
import CurvatureChart from './CurvatureChart.vue'
import type { Dataset, Endpoint, Engine, Metrics, Options, PickMode, Point, Result, ViewMode } from './types'

const choices = ref<{ id: string; name: string }[]>([])
const dataset = ref<Dataset | null>(null)
const selectedDataset = ref('obstacles')
const engines = ref<{ id: Engine; name: string; available: boolean }[]>([{ id: 'python', name: 'Python', available: true }])
const engine = ref<Engine>('python')
const options = reactive<Options>({ smoothness: 6, max_deviation: 0.8, clearance: 0.39, control_spacing: 1.5 })
const result = ref<Result | null>(null)
const history = ref<Result[]>([])
const importedRoute = ref<Point[] | null>(null)
const importedName = ref('')
const start = ref<Endpoint | null>(null)
const goal = ref<Endpoint | null>(null)
const pickMode = ref<PickMode>('orbit')
const picking = ref(false)
const manualSelection = ref(false)
const plannedRoute = ref<Point[] | null>(null)
const selectionPlanning = ref<Result['planning']>(null)
const phase = ref('optimizing')
const mapCanvas = ref<InstanceType<typeof MapCanvas>>()
const mode = ref<ViewMode>('compare')
const showRetained = ref(false)
const showInvalid = ref(true)
const showSharp = ref(true)
const showRaw = ref(false)
const busy = ref(false)
const loading = ref(true)
const error = ref('')
const elapsed = ref(0)
const fileInput = ref<HTMLInputElement>()
const settingsOpen = ref(false)
let generation = 0
let disposed = false
let clock: ReturnType<typeof setInterval> | undefined
const sameOptions = (a: Options, b: Options) =>
  (['smoothness', 'max_deviation', 'clearance', 'control_spacing'] as const).every(key => a[key] === b[key])
const dirty = computed(() => result.value && (!sameOptions(options, result.value.options)
  || engine.value !== (result.value.engine ?? 'python')))
const route = computed(() => manualSelection.value ? plannedRoute.value ?? [] : importedRoute.value ?? dataset.value?.route ?? [])
const oldMetrics = computed(() => result.value?.baseline ?? (!manualSelection.value && !importedRoute.value ? dataset.value?.baseline : undefined))
const controlsDisabled = computed(() => busy.value || loading.value || picking.value)
const canOptimize = computed(() => dataset.value && route.value.length >= 2 && !controlsDisabled.value)
const phaseLabel = computed(() => phase.value === 'planning' ? '搜索路线' : '优化计算')
const metrics = computed(() => result.value?.optimized)
const reduction = (before: number, after: number) => (before > 0 ? (1 - after / before) * 100 : 0)
const curveImprovement = computed(() => metrics.value && oldMetrics.value ? reduction(oldMetrics.value.curvatureRms, metrics.value.curvatureRms) : 0)
const status = computed(() => {
  const r = result.value
  if (!r) return { label: '待验证', level: 'neutral' }
  if (r.validation.newViolationCount) return { label: '发现新增问题', level: 'danger' }
  if (r.optimized.denseSharpTurns || r.solver.minimumSpeed < 0.05) return { label: '仍有急转路段', level: 'warning' }
  if (!r.validation.valid) return { label: '原始地形待复核', level: 'warning' }
  return { label: '几何校验通过', level: 'success' }
})
const presets: { label: string; values: Options }[] = [
  { label: '保守', values: { smoothness: 2, max_deviation: 0.4, clearance: 0.39, control_spacing: 1 } },
  { label: '均衡', values: { smoothness: 6, max_deviation: 0.8, clearance: 0.39, control_spacing: 1.5 } },
  { label: '强平滑', values: { smoothness: 30, max_deviation: 1.5, clearance: 0.39, control_spacing: 2.5 } },
]
const validations = computed(() => result.value ? [
  { label: '地图外采样', value: result.value.validation.outsideCount, unit: '个' },
  { label: '高程不匹配', value: result.value.validation.heightViolationCount, unit: '个' },
  { label: '斜向支撑异常', value: result.value.validation.diagonalViolationCount, unit: '个' },
  { label: '新增高差异常', value: result.value.validation.transitionViolationCount, unit: '个' },
  { label: '偏离超限', value: result.value.validation.deviationViolationCount, unit: '个' },
  { label: '低于边距下限', value: result.value.validation.clearanceViolationCount, unit: '个' },
] : [])

async function api<T>(url: string, init?: RequestInit): Promise<T> {
  const response = await fetch(url, init)
  if (!response.ok) {
    const message = await response.json().catch(() => null)
    throw new Error(typeof message?.detail === 'string' ? message.detail : `请求失败 (${response.status})`)
  }
  return response.json()
}

async function loadDataset(id: string) {
  const current = ++generation
  loading.value = true; error.value = ''; result.value = null
  importedRoute.value = null; importedName.value = ''; history.value = []
  manualSelection.value = false; plannedRoute.value = null; selectionPlanning.value = null
  start.value = null; goal.value = null; pickMode.value = 'orbit'; dataset.value = null
  try {
    const data = await api<Dataset>(`/api/datasets/${id}`)
    if (disposed || current !== generation) return
    dataset.value = data
    setEndpoints(data.route)
    loading.value = false
    await run()
  } catch (e) {
    if (current === generation) error.value = e instanceof Error ? e.message : '地图加载失败'
  } finally { if (current === generation) loading.value = false }
}

async function run(plan = false) {
  if (!dataset.value || controlsDisabled.value || (!plan && route.value.length < 2)) return
  if (plan && (!start.value || !goal.value)) return
  busy.value = true; error.value = ''; elapsed.value = 0
  phase.value = plan ? 'planning' : 'optimizing'
  pickMode.value = 'pan'
  const started = performance.now()
  clock = setInterval(() => { elapsed.value = (performance.now() - started) / 1000 }, 100)
  const current = generation
  try {
    const job = await api<{ id: string }>('/api/optimize', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ dataset: dataset.value.id, engine: engine.value, ...options,
        ...(plan ? { start: start.value!.point, goal: goal.value!.point }
          : manualSelection.value || importedRoute.value ? { route: route.value } : {}),
      }),
    })
    while (!disposed && current === generation) {
      const state = await api<{ status: string; phase?: string; result?: Result; error?: string }>(`/api/jobs/${job.id}`)
      phase.value = state.phase ?? phase.value
      if (state.status === 'failed') throw new Error(state.error || '算法运行失败')
      if (state.status === 'completed' && state.result) {
        if (plan) {
          manualSelection.value = true
          importedRoute.value = null; importedName.value = ''
          plannedRoute.value = state.result.sourceRoute
          selectionPlanning.value = state.result.planning
        } else if (manualSelection.value) state.result.planning = selectionPlanning.value
        result.value = state.result
        history.value = [state.result, ...history.value].slice(0, 5)
        if (plan) { await nextTick(); mapCanvas.value?.fit(true) }
        return
      }
      if (performance.now() - started > 120000) throw new Error('计算超时，结果可通过任务接口继续查询')
      await new Promise(resolve => setTimeout(resolve, 250))
    }
  } catch (e) {
    error.value = e instanceof Error ? e.message : '算法运行失败'
  } finally {
    busy.value = false
    clearInterval(clock)
  }
}

function restore(item: Result) {
  result.value = item
  engine.value = item.engine ?? 'python'
  Object.assign(options, item.options)
  pickMode.value = 'pan'
  manualSelection.value = !!item.planning
  selectionPlanning.value = item.planning
  plannedRoute.value = item.planning ? item.sourceRoute : null
  importedRoute.value = item.planning ? null : item.sourceRoute
  importedName.value = ''
  setEndpoints(item.sourceRoute)
}

function setEndpoints(points: Point[]) {
  start.value = { point: [...points[0]], heights: [points[0][2]] }
  goal.value = { point: [...points[points.length - 1]], heights: [points[points.length - 1][2]] }
}

function invalidateRoute() {
  manualSelection.value = true; plannedRoute.value = null; selectionPlanning.value = null
  importedRoute.value = null; importedName.value = ''; result.value = null; error.value = ''
}

async function pickEndpoint(point: { x: number; y: number; z?: number }) {
  if (!dataset.value || controlsDisabled.value || (pickMode.value !== 'start' && pickMode.value !== 'goal')) return
  const target = pickMode.value
  const current = generation
  picking.value = true; error.value = ''
  try {
    const endpoint = await api<Endpoint>(`/api/datasets/${dataset.value.id}/pick`, {
      method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(point),
    })
    if (disposed || current !== generation) return
    invalidateRoute()
    if (target === 'start') { start.value = endpoint; pickMode.value = 'goal' }
    else { goal.value = endpoint; pickMode.value = 'pan' }
  } catch (e) { error.value = e instanceof Error ? e.message : '位置选择失败' }
  finally { picking.value = false }
}

function swapEndpoints() {
  const previous = start.value
  start.value = goal.value; goal.value = previous
  invalidateRoute(); pickMode.value = 'pan'
}

async function restoreSample() {
  if (!dataset.value || controlsDisabled.value) return
  manualSelection.value = false; plannedRoute.value = null; selectionPlanning.value = null
  importedRoute.value = null; importedName.value = ''; result.value = null; history.value = []
  pickMode.value = 'pan'; setEndpoints(dataset.value.route)
  await nextTick(); mapCanvas.value?.fit()
  await run()
}

function keydown(event: KeyboardEvent) { if (event.key === 'Escape') pickMode.value = 'pan' }

async function importFile(event: Event) {
  const input = event.target as HTMLInputElement
  const file = input.files?.[0]
  if (!file) return
  try {
    const value = JSON.parse(await file.text())
    const source = Array.isArray(value) ? value : value.pathPoints ?? value.path ?? value.route
    if (!Array.isArray(source) || source.length < 2 || source.length > 20000) throw new Error('路线需要包含 2–20,000 个 XYZ 点')
    const points = source.map((p: unknown) => {
      const item = p as Point & { x: number; y: number; z: number }
      const row = Array.isArray(p) ? p : [item.x, item.y, item.z]
      if (row.length !== 3 || row.some(v => typeof v !== 'number' || !Number.isFinite(v))) throw new Error('每个路径点必须是有效的 XYZ 坐标')
      return row as Point
    })
    importedRoute.value = points; importedName.value = file.name
    manualSelection.value = false; plannedRoute.value = null; selectionPlanning.value = null
    pickMode.value = 'pan'; setEndpoints(points)
    result.value = null; history.value = []
    await run()
  } catch (e) { error.value = e instanceof Error ? e.message : '路线读取失败' }
  finally { input.value = '' }
}

function exportResult() {
  if (!result.value) return
  const blob = new Blob([JSON.stringify({ ...result.value, referenceName: importedName.value || dataset.value?.name }, null, 2)], { type: 'application/json' })
  const link = document.createElement('a')
  link.href = URL.createObjectURL(blob)
  link.download = `path-planner-${result.value.dataset}-${result.value.jobId.slice(0, 8)}.json`
  link.click()
  setTimeout(() => URL.revokeObjectURL(link.href), 1000)
}

function number(value?: number, digits = 2) { return value === undefined ? '—' : value.toFixed(digits) }
function deltaText(field: keyof Metrics) {
  const before = oldMetrics.value?.[field]; const after = metrics.value?.[field]
  if (typeof before !== 'number' || typeof after !== 'number') return ''
  const percent = reduction(before, after)
  return `${percent >= 0 ? '↓' : '↑'} ${Math.abs(percent).toFixed(1)}%`
}

onMounted(async () => {
  window.addEventListener('keydown', keydown)
  try {
    const [maps, available] = await Promise.all([
      api<{ id: string; name: string }[]>('/api/datasets'),
      api<{ id: Engine; name: string; available: boolean }[]>('/api/engines'),
    ])
    choices.value = maps; engines.value = available
    await loadDataset(selectedDataset.value)
  } catch (e) { loading.value = false; error.value = e instanceof Error ? e.message : '服务连接失败' }
})
onBeforeUnmount(() => { disposed = true; clearInterval(clock); window.removeEventListener('keydown', keydown) })
</script>

<template>
  <div class="app-shell">
    <header class="app-header">
      <div class="brand"><div class="brand-mark"><Route :size="22" /></div><div><strong>Terrain Path Planner</strong><span>路径规划</span></div></div>
      <span class="project-version">v0.1.0</span>
    </header>

    <div class="context-bar">
      <div class="dataset-select"><Map :size="16" /><select v-model="selectedDataset" aria-label="验证地图" :disabled="controlsDisabled" @change="loadDataset(selectedDataset)"><option v-for="choice in choices" :key="choice.id" :value="choice.id">{{ choice.name }}</option></select><ChevronDown :size="14" /></div>
      <span v-if="dataset" class="dataset-meta">{{ dataset.resolution.toFixed(2) }} m 栅格<span>·</span>{{ dataset.areaM2.toLocaleString() }} m²</span>
      <div class="context-actions">
        <input ref="fileInput" type="file" accept="application/json,.json" hidden @change="importFile" />
        <button class="text-button import-button" title="导入路线" aria-label="导入路线" :disabled="controlsDisabled" @click="fileInput?.click()"><Upload :size="15" />导入路线</button>
        <button class="text-button" title="导出" aria-label="导出" :disabled="!result || controlsDisabled" @click="exportResult"><ArrowDownToLine :size="15" /><span>导出</span></button>
        <button class="icon-button mobile-settings" title="算法参数" aria-label="算法参数" @click="settingsOpen = !settingsOpen"><Settings2 :size="18" /></button>
      </div>
    </div>

    <div v-if="error" class="error-banner" role="alert"><CircleAlert :size="16" /><span>{{ error }}</span><button class="icon-button" title="关闭错误" aria-label="关闭错误" @click="error = ''"><X :size="16" /></button></div>

    <main class="planner-layout">
      <aside class="sidebar" :class="{ 'mobile-open': settingsOpen }">
        <section class="parameter-section">
          <div class="section-heading"><h2><Settings2 :size="16" />优化参数</h2><button class="icon-button" title="恢复默认参数" aria-label="恢复默认参数" :disabled="busy" @click="Object.assign(options, presets[1].values)"><RotateCcw :size="14" /></button></div>
          <div class="engine-select"><label for="engine">计算引擎</label><select id="engine" v-model="engine" aria-label="计算引擎" :disabled="controlsDisabled"><option v-for="item in engines" :key="item.id" :value="item.id" :disabled="!item.available">{{ item.name }}{{ item.available ? '' : ' · 未构建' }}</option></select></div>
          <div class="presets" aria-label="参数预设"><button v-for="preset in presets" :key="preset.label" :class="{ active: sameOptions(options, preset.values) }" :disabled="busy" @click="Object.assign(options, preset.values)">{{ preset.label }}</button></div>
          <div class="parameter"><label for="smoothness">平滑强度</label><output>{{ options.smoothness.toFixed(1) }}</output><input id="smoothness" v-model.number="options.smoothness" :disabled="busy" type="range" min="0.1" max="60" step="0.1" /></div>
          <div class="parameter"><label for="deviation">最大偏离</label><div class="number-unit"><input id="deviation-number" v-model.number="options.max_deviation" aria-label="最大偏离数值" :disabled="busy" type="number" min="0.05" max="3" step="0.05" /><span>m</span></div><input id="deviation" v-model.number="options.max_deviation" :disabled="busy" type="range" min="0.05" max="3" step="0.05" /></div>
          <div class="parameter"><label for="clearance">目标边界距离</label><div class="number-unit"><input v-model.number="options.clearance" aria-label="目标边界距离数值" :disabled="busy" type="number" min="0" max="1" step="0.01" /><span>m</span></div><input id="clearance" v-model.number="options.clearance" :disabled="busy" type="range" min="0" max="1" step="0.01" /></div>
          <div class="parameter"><label for="spacing">控制点间距</label><div class="number-unit"><input v-model.number="options.control_spacing" aria-label="控制点间距数值" :disabled="busy" type="number" min="0.4" max="4" step="0.1" /><span>m</span></div><input id="spacing" v-model.number="options.control_spacing" :disabled="busy" type="range" min="0.4" max="4" step="0.1" /></div>
          <button class="run-button" :disabled="!canOptimize" @click="run()"><LoaderCircle v-if="busy" :size="17" class="spin" /><Play v-else :size="16" />{{ busy ? `${phaseLabel} ${elapsed.toFixed(1)} s` : '运行优化' }}</button>
          <div class="parameter-caption"><span v-if="dirty" class="pending-dot" /><span>{{ dirty ? '参数已更改 · 结果待更新' : busy ? phaseLabel : manualSelection && !plannedRoute ? '待规划路线' : '全路径平滑' }}</span></div>
        </section>

        <section class="validation-section">
          <div class="section-heading"><h2><ShieldCheck :size="16" />校验结果</h2><span class="status-tag" :class="status.level">{{ status.label }}</span></div>
          <template v-if="result">
            <div class="validation-overview" :class="result.validation.newViolationCount ? 'danger' : 'success'"><CircleCheck v-if="!result.validation.newViolationCount" :size="17" /><CircleAlert v-else :size="17" /><span>新增问题</span><strong>{{ result.validation.newViolationCount }}</strong></div>
            <dl class="detail-list"><div v-for="item in validations" :key="item.label"><dt>{{ item.label }}</dt><dd :class="{ 'warning-text': item.value > 0 }">{{ item.value }} <small>{{ item.unit }}</small></dd></div></dl>
            <div v-if="result.validation.inheritedViolationCount" class="inherited-note"><CircleAlert :size="14" /><span>原始路线保留 {{ result.validation.inheritedViolationCount }} 个待核验采样点</span></div>
            <dl class="detail-list secondary-details"><div><dt>局部最大转角 / 5 cm</dt><dd :class="{ 'warning-text': result.optimized.denseSharpTurns > 0 }">{{ number(result.optimized.maxHeadingChangeDeg, 2) }}°</dd></div><div><dt>急转采样（&gt;5°）</dt><dd :class="{ 'warning-text': result.optimized.denseSharpTurns > 0 }">{{ result.optimized.denseSharpTurns }} 个</dd></div><div><dt>平面曲线连续性</dt><dd :class="{ 'warning-text': result.solver.minimumSpeed < 0.05 }">{{ result.solver.minimumSpeed < 0.05 ? '切线待核验' : 'XY C² 连续' }}</dd></div><div><dt>去绕行直达段</dt><dd>{{ result.shortcut.segments }} 段</dd></div><div><dt>高程重匹配</dt><dd>{{ result.solver.heightRematchedSamples }} 点</dd></div><div><dt>采样数 / 最大间隔</dt><dd>{{ result.validation.sampleCount.toLocaleString() }} / {{ number(result.validation.sampleSpacingM, 3) }} m</dd></div><div><dt>最大实际偏离</dt><dd>{{ number(result.validation.maxDeviationM, 3) }} m</dd></div><div><dt>近似保留路段</dt><dd>{{ result.solver.retainedPercent.toFixed(1) }}%</dd></div><div><dt>求解收敛</dt><dd :class="{ 'warning-text': !result.solver.converged }">{{ result.solver.converged ? '已收敛' : '未收敛' }} · {{ result.solver.iterations }} 次</dd></div><div><dt>完整 3D 校验 / 实机</dt><dd class="muted">未验证</dd></div></dl>
          </template>
          <div v-else class="empty-state">{{ busy ? '正在计算校验结果' : '暂无结果' }}</div>
        </section>

        <section v-if="history.length" class="history-section">
          <div class="section-heading"><h2><History :size="16" />运行记录</h2><small>{{ history.length }} 次</small></div>
          <button v-for="(item, index) in history" :key="item.jobId" class="history-item" :class="{ selected: result?.jobId === item.jobId }" :disabled="busy" @click="restore(item)"><span class="history-number">{{ String(history.length - index).padStart(2, '0') }}</span><span><strong>强度 {{ item.options.smoothness }} · 偏离 {{ item.options.max_deviation }} m</strong><small>{{ item.engine === 'cpp' ? 'C++' : 'Python' }} · 曲率 RMS {{ item.optimized.curvatureRms.toFixed(3) }}</small></span><Check v-if="result?.jobId === item.jobId" :size="14" /></button>
        </section>
      </aside>

      <section class="workspace">
        <div class="metrics-strip">
          <div class="metric"><span>曲率 RMS <small>m⁻¹</small></span><div><strong>{{ number(metrics?.curvatureRms, 3) }}</strong><small :class="{ negative: curveImprovement && curveImprovement < 0 }">{{ deltaText('curvatureRms') }}</small></div><span class="metric-baseline">原始 {{ number(oldMetrics?.curvatureRms, 3) }}</span></div>
          <div class="metric"><span>累计转向 <small>°</small></span><div><strong>{{ number(metrics?.turningDeg, 0) }}</strong><small :class="{ negative: metrics && oldMetrics && metrics.turningDeg > oldMetrics.turningDeg }">{{ deltaText('turningDeg') }}</small></div><span class="metric-baseline">原始 {{ number(oldMetrics?.turningDeg, 0) }}°</span></div>
          <div class="metric"><span>平面长度 <small>m</small></span><div><strong>{{ number(metrics?.lengthM, 2) }}</strong><small>{{ deltaText('lengthM') }}</small></div><span class="metric-baseline">原始 {{ number(oldMetrics?.lengthM, 2) }} m</span></div>
          <div class="metric"><span>最小边距 <small>m</small></span><div><strong>{{ number(metrics?.minClearanceM, 2) }}</strong><small v-if="metrics" :class="{ negative: metrics.minClearanceM < (result?.options.clearance ?? 0) }">{{ metrics.minClearanceM < (result?.options.clearance ?? 0) ? '未达目标' : '达到目标' }}</small></div><span class="metric-baseline">原始 {{ number(oldMetrics?.minClearanceM, 2) }} m</span></div>
        </div>

        <div class="map-workspace">
          <div class="map-toolbar">
            <div class="segmented" aria-label="路线视图"><button v-for="tab in [{ id: 'compare', label: '对比' }, { id: 'baseline', label: '原始' }, { id: 'optimized', label: '优化' }]" :key="tab.id" :class="{ active: mode === tab.id }" @click="mode = tab.id as ViewMode">{{ tab.label }}</button></div>
            <div class="pick-tools" role="group" aria-label="地图操作模式">
              <button class="icon-button" :class="{ active: pickMode === 'orbit' }" title="旋转地图" aria-label="旋转地图" :aria-pressed="pickMode === 'orbit'" :disabled="controlsDisabled" @click="pickMode = 'orbit'"><Rotate3D :size="17" /></button>
              <button class="icon-button" :class="{ active: pickMode === 'pan' }" title="平移地图" aria-label="平移地图" :aria-pressed="pickMode === 'pan'" :disabled="controlsDisabled" @click="pickMode = 'pan'"><Hand :size="16" /></button>
              <button class="icon-button start-tool" :class="{ active: pickMode === 'start' }" title="选择起点" aria-label="选择起点" :aria-pressed="pickMode === 'start'" :disabled="controlsDisabled || !dataset" @click="pickMode = 'start'"><MapPin :size="17" /></button>
              <button class="icon-button goal-tool" :class="{ active: pickMode === 'goal' }" title="选择终点" aria-label="选择终点" :aria-pressed="pickMode === 'goal'" :disabled="controlsDisabled || !dataset" @click="pickMode = 'goal'"><Flag :size="16" /></button>
            </div>
            <span v-if="importedName" class="route-filename" :title="importedName">{{ importedName }}</span>
            <div v-if="result" class="solve-time"><Activity :size="13" />{{ ((result.planning?.timeMs ?? 0) + result.solver.timeMs).toFixed(0) }} ms</div><span v-if="dirty" class="stale-tag">上次结果</span>
          </div>
          <div class="endpoint-bar" :aria-busy="picking">
            <div class="endpoint start-endpoint" data-testid="start-endpoint">
              <MapPin :size="15" /><span class="endpoint-label">起点</span>
              <span class="endpoint-xy">{{ start ? `${number(start.point[0])}, ${number(start.point[1])}` : '未选择' }}</span>
              <label v-if="start" class="endpoint-height"><span>Z</span><select v-if="start.heights.length > 1" v-model.number="start.point[2]" aria-label="起点高程" :disabled="controlsDisabled" @change="invalidateRoute"><option v-for="z in start.heights" :key="z" :value="z">{{ z.toFixed(2) }} m</option></select><span v-else>{{ number(start.point[2]) }} m</span></label>
            </div>
            <button class="icon-button endpoint-swap" title="交换起终点" aria-label="交换起终点" :disabled="controlsDisabled || !start || !goal" @click="swapEndpoints"><ArrowLeftRight :size="16" /></button>
            <div class="endpoint goal-endpoint" data-testid="goal-endpoint">
              <Flag :size="15" /><span class="endpoint-label">终点</span>
              <span class="endpoint-xy">{{ goal ? `${number(goal.point[0])}, ${number(goal.point[1])}` : '未选择' }}</span>
              <label v-if="goal" class="endpoint-height"><span>Z</span><select v-if="goal.heights.length > 1" v-model.number="goal.point[2]" aria-label="终点高程" :disabled="controlsDisabled" @change="invalidateRoute"><option v-for="z in goal.heights" :key="z" :value="z">{{ z.toFixed(2) }} m</option></select><span v-else>{{ number(goal.point[2]) }} m</span></label>
            </div>
            <div class="endpoint-actions">
              <button class="plan-button" :disabled="controlsDisabled || !dataset || !start || !goal" @click="run(true)"><LoaderCircle v-if="busy" :size="15" class="spin" /><Route v-else :size="15" />规划路线</button>
              <button class="icon-button" title="恢复样例路线" aria-label="恢复样例路线" :disabled="controlsDisabled || !dataset" @click="restoreSample"><RotateCcw :size="15" /></button>
            </div>
          </div>
          <MapCanvas v-if="dataset" ref="mapCanvas" :key="dataset.id" :dataset="dataset" :route="route" :result="result" :mode="mode" :show-retained="showRetained" :show-invalid="showInvalid" :show-sharp="showSharp" :show-raw="showRaw" :start="start?.point ?? null" :goal="goal?.point ?? null" :pick-mode="pickMode" :picking-disabled="controlsDisabled" @pick="pickEndpoint" @map-error="error = $event" />
          <div v-else class="map-placeholder"><LoaderCircle v-if="loading" :size="26" class="spin" /><Map v-else :size="30" /><span>{{ loading ? '加载地图' : '地图尚未加载' }}</span><button v-if="!loading" class="text-button" @click="loadDataset(selectedDataset)">重新加载</button></div>
          <div class="map-legend"><span class="legend-item"><i class="line-swatch baseline" />原始路线</span><span class="legend-item"><i class="line-swatch optimized" />优化路线</span><span class="legend-divider" /><label><input v-model="showRetained" type="checkbox" /><i class="line-swatch retained" />保留段</label><label><input v-model="showInvalid" type="checkbox" /><i class="point-swatch" />待核验点</label><label><input v-model="showSharp" type="checkbox" /><i class="turn-swatch" />急转点</label><label class="raw-toggle"><input v-model="showRaw" type="checkbox" /><i class="line-swatch raw" />修复前候选</label></div>
          <div v-if="busy || picking" class="computing-indicator"><LoaderCircle :size="14" class="spin" />{{ picking ? '读取地面高程' : `${phaseLabel}中 ${elapsed.toFixed(1)} s` }}</div>
        </div>
        <CurvatureChart v-if="oldMetrics" :baseline="oldMetrics" :optimized="metrics" />
        <div v-else class="curvature-panel empty-state">{{ manualSelection ? '待规划路线' : '暂无路线数据' }}</div>
        <footer class="workspace-footer"><span><span class="connection-dot" />{{ dataset ? `${dataset.voxelCount.toLocaleString()} 体素` : '连接中' }}<span class="footer-divider">/</span>{{ route.length.toLocaleString() }} 原始路径点</span><span v-if="result">边距不足占比 {{ result.optimized.belowClearancePercent.toFixed(1) }}%<span class="footer-divider">/</span>二维几何与高程检查</span></footer>
      </section>
    </main>
  </div>
</template>
