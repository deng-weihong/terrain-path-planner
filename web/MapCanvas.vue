<script setup lang="ts">
import { nextTick, onBeforeUnmount, onMounted, ref, watch } from 'vue'
import { Box, Boxes, Crosshair, Focus, LoaderCircle, Maximize, Minus, PanelsTopLeft, Plus, RotateCcw, Scan } from '@lucide/vue'
import { MapScene, type CameraView, type MapStyle } from './mapScene'
import type { Dataset, PickMode, Point, Result, ViewMode } from './types'

const props = defineProps<{
  dataset: Dataset
  route: Point[]
  result: Result | null
  mode: ViewMode
  showRetained: boolean
  showInvalid: boolean
  showSharp: boolean
  showRaw: boolean
  start: Point | null
  goal: Point | null
  pickMode: PickMode
  pickingDisabled: boolean
}>()
const emit = defineEmits<{ mapError: [message: string]; pick: [point: { x: number; y: number; z: number }] }>()
const wrapper = ref<HTMLDivElement>()
const canvas = ref<HTMLCanvasElement>()
const canvasKey = ref(0)
const cursor = ref<Point | null>(null)
const scaleLabel = ref('10 m')
const scaleWidth = ref(64)
const cameraView = ref<CameraView>('perspective')
const mapStyle = ref<MapStyle>('voxels')
const heightScale = ref(1)
const loading = ref(true)
const progress = ref(0)
const loadError = ref('')
let scene: MapScene | null = null
let observer: ResizeObserver | null = null
let controller: AbortController | null = null
let generation = 0
let disposed = false
let hoverFrame = 0
let hoverLocation: { x: number; y: number } | null = null
const pointers = new Set<number>()
let tap: { id: number; x: number; y: number; moved: boolean } | null = null

function scaleChanged(metersPerPixel: number) {
  const meters = 10 ** Math.floor(Math.log10(70 * metersPerPixel))
  const spacing = [1, 2, 5, 10].map(n => n * meters).find(n => n / metersPerPixel >= 50) ?? meters * 10
  scaleLabel.value = `${spacing < 1 ? spacing.toFixed(2) : spacing} m`
  scaleWidth.value = spacing / metersPerPixel
}

function updateLayers() { scene?.updateLayers(props) }

async function loadMap() {
  const current = ++generation
  controller?.abort()
  controller = new AbortController()
  const signal = controller.signal
  loading.value = true; progress.value = 0; loadError.value = ''; cursor.value = null
  const oldScene = scene
  scene = null
  oldScene?.dispose()
  canvasKey.value++
  await nextTick()
  if (disposed || current !== generation) return
  try {
    scene = new MapScene(canvas.value!, props.dataset, scaleChanged)
    const box = wrapper.value!.getBoundingClientRect()
    scene.resize(box.width, box.height)
    scene.setNavigation(props.pickMode)
    const response = await fetch(props.dataset.voxelUrl, { signal })
    if (!response.ok) throw new Error(`三维地图加载失败 (${response.status})`)
    const expected = props.dataset.voxelCount * 12
    const buffer = new Uint8Array(expected)
    const reader = response.body?.getReader()
    if (!reader) throw new Error('三维地图数据为空')
    let received = 0
    while (true) {
      const { done, value } = await reader.read()
      if (done) break
      if (received + value.byteLength > expected) { await reader.cancel(); throw new Error('三维地图数据长度不正确') }
      buffer.set(value, received)
      received += value.byteLength
      progress.value = Math.round(received / expected * 100)
    }
    if (received !== expected) throw new Error('三维地图数据未完整下载')
    if (disposed || current !== generation) return
    scene.load(buffer.buffer)
    scene.setStyle(mapStyle.value)
    scene.setHeightScale(heightScale.value)
    scene.setView(cameraView.value)
    scene.fit()
    updateLayers()
    loading.value = false
  } catch (error) {
    if (disposed || current !== generation || signal.aborted) return
    loading.value = false
    loadError.value = error instanceof Error ? error.message : '三维地图加载失败'
  }
}

function pointerDown(event: PointerEvent) {
  if (event.button !== 0) { tap = null; return }
  tap = pointers.size === 0 ? { id: event.pointerId, x: event.clientX, y: event.clientY, moved: false } : null
  pointers.add(event.pointerId)
}

function pointerMove(event: PointerEvent) {
  if (tap && Math.hypot(event.clientX - tap.x, event.clientY - tap.y) > 5) tap.moved = true
  if (pointers.size || loading.value || loadError.value) { cursor.value = null; return }
  hoverLocation = { x: event.clientX, y: event.clientY }
  if (!hoverFrame) hoverFrame = requestAnimationFrame(() => {
    hoverFrame = 0
    if (hoverLocation && scene) cursor.value = scene.pick(hoverLocation.x, hoverLocation.y)
  })
}

function pointerUp(event: PointerEvent) {
  if (tap?.id === event.pointerId && !tap.moved && pointers.size === 1 && !loading.value && !loadError.value
      && (props.pickMode === 'start' || props.pickMode === 'goal') && !props.pickingDisabled) {
    const point = scene?.pick(event.clientX, event.clientY)
    if (point) emit('pick', { x: point[0], y: point[1], z: point[2] })
    else emit('mapError', '所选位置没有可通行体素，请重新选择')
  }
  pointerCancel(event)
}

function pointerCancel(event: PointerEvent) { tap = null; pointers.delete(event.pointerId) }
function pointerLeave() { hoverLocation = null; cursor.value = null }
function contextLost(event: Event) {
  event.preventDefault()
  if (scene && !disposed && event.target === canvas.value) loadError.value = '三维渲染上下文已中断'
}
function fit(routeOnly = false) { scene?.fit(routeOnly) }
function zoom(factor: number) { scene?.zoom(factor) }

onMounted(() => {
  observer = new ResizeObserver(entries => {
    const box = entries[0].contentRect
    scene?.resize(box.width, box.height)
  })
  observer.observe(wrapper.value!)
  loadMap()
})
watch(() => props.dataset.id, loadMap)
watch(() => [props.result, props.mode, props.showRetained, props.showInvalid, props.showSharp, props.showRaw, props.route, props.start?.join(','), props.goal?.join(',')], updateLayers, { deep: false })
watch(() => props.pickMode, mode => { scene?.setNavigation(mode); tap = null })
watch(cameraView, view => scene?.setView(view))
watch(mapStyle, style => scene?.setStyle(style))
watch(heightScale, value => scene?.setHeightScale(value))
onBeforeUnmount(() => {
  disposed = true; generation++; controller?.abort(); observer?.disconnect()
  cancelAnimationFrame(hoverFrame)
  scene?.dispose(); scene = null
})
defineExpose({ fit })
</script>

<template>
  <div ref="wrapper" class="map-surface map-3d" :class="{ 'picking-mode': pickMode === 'start' || pickMode === 'goal' }">
    <canvas :key="canvasKey" ref="canvas" aria-label="三维路径地图" @pointerdown="pointerDown" @pointermove="pointerMove"
      @pointerup="pointerUp" @pointercancel="pointerCancel" @pointerleave="pointerLeave" @webglcontextlost="contextLost" />
    <div class="scene-controls" aria-label="三维地图视图">
      <button class="icon-button" :class="{ active: cameraView === 'perspective' }" title="三维视角" aria-label="三维视角" :aria-pressed="cameraView === 'perspective'" @click="cameraView = 'perspective'"><Box :size="17" /></button>
      <button class="icon-button" :class="{ active: cameraView === 'top' }" title="俯视视角" aria-label="俯视视角" :aria-pressed="cameraView === 'top'" @click="cameraView = 'top'"><PanelsTopLeft :size="17" /></button>
      <span class="scene-divider" />
      <button class="icon-button" :class="{ active: mapStyle === 'voxels' }" title="体素显示" aria-label="体素显示" :aria-pressed="mapStyle === 'voxels'" @click="mapStyle = 'voxels'"><Boxes :size="17" /></button>
      <button class="icon-button" :class="{ active: mapStyle === 'points' }" title="点云显示" aria-label="点云显示" :aria-pressed="mapStyle === 'points'" @click="mapStyle = 'points'"><Scan :size="17" /></button>
      <span class="scene-divider" />
      <label class="height-scale" title="高程显示倍率"><span>Z</span><input v-model.number="heightScale" aria-label="高程显示倍率" type="range" min="1" max="10" step="1" /><output>{{ heightScale }}×</output></label>
    </div>
    <div class="map-tools" aria-label="地图工具">
      <button class="icon-button" title="放大" aria-label="放大" @click="zoom(1.5)"><Plus :size="18" /></button>
      <button class="icon-button" title="缩小" aria-label="缩小" @click="zoom(1 / 1.5)"><Minus :size="18" /></button>
      <span class="tool-divider" />
      <button class="icon-button" title="完整地图" aria-label="完整地图" @click="fit()"><Maximize :size="17" /></button>
      <button class="icon-button" title="聚焦路线" aria-label="聚焦路线" @click="fit(true)"><Focus :size="18" /></button>
    </div>
    <div class="height-legend" aria-label="地图高程"><span>{{ dataset.heightRange.max.toFixed(2) }} m</span><i /><span>{{ dataset.heightRange.min.toFixed(2) }} m</span></div>
    <div class="map-scale"><span :style="{ width: `${scaleWidth}px` }" />{{ scaleLabel }}</div>
    <div class="map-coordinate"><Crosshair :size="12" /><span v-if="cursor">{{ cursor.map(v => v.toFixed(2)).join(', ') }} m</span><span v-else>XYZ · Z {{ heightScale }}×</span></div>
    <div v-if="loading || loadError" class="scene-state" :class="{ failed: loadError }" :role="loadError ? 'alert' : 'status'">
      <LoaderCircle v-if="loading" :size="22" class="spin" /><span>{{ loadError || `加载三维地图 ${progress}%` }}</span>
      <button v-if="loadError" class="text-button" @click="loadMap"><RotateCcw :size="15" />重新加载</button>
    </div>
  </div>
</template>
