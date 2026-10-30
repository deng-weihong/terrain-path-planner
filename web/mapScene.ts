import * as THREE from 'three'
import { OrbitControls } from 'three/addons/controls/OrbitControls.js'
import { Line2 } from 'three/addons/lines/Line2.js'
import { LineGeometry } from 'three/addons/lines/LineGeometry.js'
import { LineMaterial } from 'three/addons/lines/LineMaterial.js'
import { LineSegments2 } from 'three/addons/lines/LineSegments2.js'
import { LineSegmentsGeometry } from 'three/addons/lines/LineSegmentsGeometry.js'
import type { Dataset, PickMode, Point, Result, ViewMode } from './types'

export type CameraView = 'perspective' | 'top'
export type MapStyle = 'voxels' | 'points'
export interface Layers {
  route: Point[]
  result: Result | null
  mode: ViewMode
  showRetained: boolean
  showInvalid: boolean
  showSharp: boolean
  showRaw: boolean
  showGround: boolean
  start: Point | null
  goal: Point | null
}

const HEIGHT_COLORS = ['#729ab2', '#88b6b1', '#b9cfad', '#ddbc85'].map(color => new THREE.Color(color))

function disposeObjects(root: THREE.Object3D) {
  const geometries = new Set<THREE.BufferGeometry>()
  const materials = new Set<THREE.Material>()
  const textures = new Set<THREE.Texture>()
  root.traverse(object => {
    const mesh = object as THREE.Mesh
    if (mesh.geometry) geometries.add(mesh.geometry)
    if (mesh.material) {
      for (const material of Array.isArray(mesh.material) ? mesh.material : [mesh.material]) {
        materials.add(material)
        if ('map' in material && material.map instanceof THREE.Texture) textures.add(material.map)
      }
    }
    if (object instanceof THREE.InstancedMesh) object.dispose()
  })
  geometries.forEach(geometry => geometry.dispose())
  textures.forEach(texture => texture.dispose())
  materials.forEach(material => material.dispose())
  root.clear()
}

export class MapScene {
  readonly renderer: THREE.WebGLRenderer
  readonly camera = new THREE.PerspectiveCamera(42, 1, .01, 10000)
  readonly controls: OrbitControls
  private readonly scene = new THREE.Scene()
  private readonly world = new THREE.Group()
  private readonly terrain = new THREE.Group()
  private readonly routes = new THREE.Group()
  private readonly markers = new THREE.Group()
  private readonly raycaster = new THREE.Raycaster()
  private readonly bounds = new THREE.Box3()
  private readonly meshTiles: THREE.InstancedMesh[] = []
  private grid: THREE.GridHelper | null = null
  private keys = new Int32Array()
  private frame = 0
  private frames = 0
  private width = 1
  private height = 1
  private disposed = false
  private verticalScale = 1
  private representation: MapStyle = 'voxels'
  private cameraView: CameraView = 'perspective'
  private pointCloud: THREE.Points | null = null
  private routePoints: Point[] = []

  constructor(private canvas: HTMLCanvasElement, private dataset: Dataset,
              private changed: (metersPerPixel: number) => void) {
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true, preserveDrawingBuffer: true, powerPreference: 'high-performance' })
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2))
    this.renderer.setClearColor('#f1f5f5')
    this.renderer.outputColorSpace = THREE.SRGBColorSpace
    this.camera.up.set(0, 0, 1)
    this.camera.position.set(60, -80, 100)
    this.controls = new OrbitControls(this.camera, canvas)
    this.controls.enableDamping = true
    this.controls.dampingFactor = .18
    this.controls.minDistance = .15
    this.controls.maxDistance = 2000
    this.controls.maxPolarAngle = Math.PI / 2 - .015
    this.controls.screenSpacePanning = true
    this.controls.addEventListener('change', this.requestRender)
    this.world.add(this.terrain, this.routes, this.markers)
    this.scene.add(this.world)
    this.scene.add(new THREE.HemisphereLight('#ffffff', '#8a9b9d', 2.2))
    const sunlight = new THREE.DirectionalLight('#ffffff', 1.7)
    sunlight.position.set(-30, -40, 70)
    this.scene.add(sunlight)
    this.setNavigation('orbit')
    canvas.dataset.engine = 'threejs'
  }

  load(buffer: ArrayBuffer) {
    if (this.dataset.voxelEncoding !== 'int32-le-xyz' || buffer.byteLength !== this.dataset.voxelCount * 12) {
      throw new Error('三维体素数据格式或长度不正确')
    }
    const count = this.dataset.voxelCount
    this.keys = new Int32Array(count * 3)
    const data = new DataView(buffer)
    const positions = new Float32Array(count * 3)
    const colors = new Float32Array(count * 3)
    const tiles = new Map<string, number[]>()
    const resolution = this.dataset.resolution
    const zRange = this.dataset.heightRange
    const color = new THREE.Color()
    this.bounds.makeEmpty()
    for (let i = 0; i < count; i++) {
      const point = new THREE.Vector3()
      for (let axis = 0; axis < 3; axis++) {
        const key = data.getInt32((i * 3 + axis) * 4, true)
        this.keys[i * 3 + axis] = key
        positions[i * 3 + axis] = (key + .5) * resolution
        point.setComponent(axis, positions[i * 3 + axis])
      }
      this.bounds.expandByPoint(point)
      const t = Math.max(0, Math.min(1, (point.z - zRange.min) / Math.max(resolution, zRange.max - zRange.min))) * (HEIGHT_COLORS.length - 1)
      const lower = Math.min(HEIGHT_COLORS.length - 2, Math.floor(t))
      color.copy(HEIGHT_COLORS[lower]).lerp(HEIGHT_COLORS[lower + 1], t - lower)
      color.toArray(colors, i * 3)
      const tileKey = `${Math.floor(this.keys[i * 3] / 32)},${Math.floor(this.keys[i * 3 + 1] / 32)}`
      const tile = tiles.get(tileKey)
      if (tile) tile.push(i)
      else tiles.set(tileKey, [i])
    }
    this.bounds.expandByScalar(resolution / 2)
    const geometry = new THREE.BoxGeometry(resolution, resolution, resolution)
    const material = new THREE.MeshLambertMaterial({ color: '#ffffff' })
    const matrix = new THREE.Matrix4()
    // Spatial tiles keep native Three.js raycasting local on the 190k-voxel map.
    for (const indices of tiles.values()) {
      const mesh = new THREE.InstancedMesh(geometry, material, indices.length)
      mesh.userData.indices = indices
      for (let j = 0; j < indices.length; j++) {
        const i = indices[j]
        matrix.makeTranslation(positions[i * 3], positions[i * 3 + 1], positions[i * 3 + 2])
        mesh.setMatrixAt(j, matrix)
        color.fromArray(colors, i * 3)
        mesh.setColorAt(j, color)
      }
      mesh.instanceMatrix.needsUpdate = true
      if (mesh.instanceColor) mesh.instanceColor.needsUpdate = true
      mesh.computeBoundingSphere()
      this.meshTiles.push(mesh)
      this.terrain.add(mesh)
    }
    const pointGeometry = new THREE.BufferGeometry()
    pointGeometry.setAttribute('position', new THREE.BufferAttribute(positions, 3))
    pointGeometry.setAttribute('color', new THREE.BufferAttribute(colors, 3))
    this.pointCloud = new THREE.Points(pointGeometry, new THREE.PointsMaterial({
      size: resolution / Math.tan(THREE.MathUtils.degToRad(this.camera.fov / 2)), vertexColors: true,
    }))
    this.terrain.add(this.pointCloud)
    this.setStyle(this.representation)
    const size = this.bounds.getSize(new THREE.Vector3())
    const span = Math.max(size.x, size.y, 10)
    const gridSize = Math.ceil(span / 10) * 10 + 20
    this.grid = new THREE.GridHelper(gridSize, Math.min(100, gridSize / (span > 40 ? 5 : 1)), '#c8d5d7', '#e0e7e8')
    this.grid.rotation.x = Math.PI / 2
    const center = this.bounds.getCenter(new THREE.Vector3())
    this.grid.position.set(center.x, center.y, this.bounds.min.z - resolution)
    this.scene.add(this.grid)
    this.canvas.dataset.voxels = String(count)
    this.fit()
  }

  resize(width: number, height: number) {
    this.width = Math.max(1, width)
    this.height = Math.max(1, height)
    this.renderer.setSize(this.width, this.height, false)
    this.camera.aspect = this.width / this.height
    this.camera.updateProjectionMatrix()
    this.routes.traverse(object => {
      const material = (object as Line2).material
      if (material instanceof LineMaterial) material.resolution.set(this.width, this.height)
    })
    this.markers.traverse(object => {
      if (object instanceof THREE.Sprite) this.sizeSprite(object)
    })
    this.requestRender()
  }

  setNavigation(mode: PickMode) {
    this.controls.mouseButtons.LEFT = mode === 'orbit' ? THREE.MOUSE.ROTATE : THREE.MOUSE.PAN
    this.controls.mouseButtons.RIGHT = mode === 'orbit' ? THREE.MOUSE.PAN : THREE.MOUSE.ROTATE
    this.controls.touches.ONE = mode === 'orbit' ? THREE.TOUCH.ROTATE : THREE.TOUCH.PAN
    this.controls.touches.TWO = THREE.TOUCH.DOLLY_PAN
  }

  setView(view: CameraView) {
    this.settleCamera()
    this.cameraView = view
    this.controls.enableRotate = view === 'perspective'
    const distance = this.camera.position.distanceTo(this.controls.target)
    const direction = view === 'top' ? new THREE.Vector3(0, -.00001, 1) : new THREE.Vector3(.55, -.7, .85).normalize()
    this.camera.position.copy(this.controls.target).addScaledVector(direction, distance)
    this.controls.update()
    this.requestRender()
  }

  setStyle(style: MapStyle) {
    this.representation = style
    this.requestRender()
  }

  setHeightScale(value: number) {
    const old = this.verticalScale
    this.verticalScale = value
    this.world.scale.z = value
    const dz = this.controls.target.z * (value / old - 1)
    this.controls.target.z += dz
    this.camera.position.z += dz
    if (this.grid) this.grid.position.z = this.bounds.min.z * value - this.dataset.resolution
    this.world.updateMatrixWorld(true)
    this.controls.update()
    this.requestRender()
  }

  fit(routeOnly = false) {
    if (this.bounds.isEmpty()) return
    this.settleCamera()
    const box = this.bounds.clone()
    if (routeOnly && this.routePoints.length) {
      box.setFromPoints(this.routePoints.map(point => new THREE.Vector3(...point)))
      box.expandByVector(new THREE.Vector3(2, 2, .2))
    }
    box.min.z *= this.verticalScale
    box.max.z *= this.verticalScale
    const center = box.getCenter(new THREE.Vector3())
    const direction = this.cameraView === 'top' ? new THREE.Vector3(0, -.00001, 1).normalize()
      : this.camera.position.clone().sub(this.controls.target).normalize()
    const right = new THREE.Vector3().crossVectors(this.camera.up, direction).normalize()
    const up = new THREE.Vector3().crossVectors(direction, right).normalize()
    let horizontal = 0; let vertical = 0; let depth = 0
    for (const x of [box.min.x, box.max.x]) for (const y of [box.min.y, box.max.y]) for (const z of [box.min.z, box.max.z]) {
      const delta = new THREE.Vector3(x, y, z).sub(center)
      horizontal = Math.max(horizontal, Math.abs(delta.dot(right)))
      vertical = Math.max(vertical, Math.abs(delta.dot(up)))
      depth = Math.max(depth, Math.abs(delta.dot(direction)))
    }
    const tan = Math.tan(THREE.MathUtils.degToRad(this.camera.fov / 2))
    const distance = Math.max(horizontal / (tan * this.camera.aspect * Math.max(.3, 1 - 72 / this.width)),
      vertical / (tan * Math.max(.3, 1 - 72 / this.height))) + depth + 1
    this.controls.target.copy(center)
    this.camera.position.copy(center).addScaledVector(direction, distance)
    this.controls.update()
    this.requestRender()
  }

  zoom(factor: number) {
    this.settleCamera()
    const direction = this.camera.position.clone().sub(this.controls.target)
    const distance = THREE.MathUtils.clamp(direction.length() / factor, this.controls.minDistance, this.controls.maxDistance)
    this.camera.position.copy(this.controls.target).add(direction.setLength(distance))
    this.controls.update()
    this.requestRender()
  }

  private settleCamera() {
    this.controls.enableDamping = false
    this.controls.update()
    this.controls.enableDamping = true
  }

  pick(clientX: number, clientY: number): Point | null {
    const box = this.canvas.getBoundingClientRect()
    this.camera.updateMatrixWorld()
    this.world.updateMatrixWorld(true)
    this.raycaster.setFromCamera(new THREE.Vector2((clientX - box.left) / box.width * 2 - 1,
      -(clientY - box.top) / box.height * 2 + 1), this.camera)
    const hit = this.raycaster.intersectObjects(this.meshTiles, false)[0]
    if (!hit || hit.instanceId === undefined) return null
    const index = hit.object.userData.indices[hit.instanceId] as number
    const r = this.dataset.resolution
    return [(this.keys[index * 3] + .5) * r, (this.keys[index * 3 + 1] + .5) * r, (this.keys[index * 3 + 2] + .5) * r]
  }

  private line(points: Point[], color: string, width: number, dashed = false, segments = false, lift = .10) {
    if (points.length < 2) return
    const positions = points.flatMap(point => [point[0], point[1], point[2] + lift])
    const geometry = segments ? new LineSegmentsGeometry() : new LineGeometry()
    geometry.setPositions(positions)
    const material = new LineMaterial({ color, linewidth: width, dashed, dashSize: .6, gapSize: .4,
      depthWrite: false, resolution: new THREE.Vector2(this.width, this.height) })
    const line = segments ? new LineSegments2(geometry, material) : new Line2(geometry as LineGeometry, material)
    line.computeLineDistances()
    line.renderOrder = width > 6 ? 1 : 2
    this.routes.add(line)
  }

  private marker(point: Point, color: string, label = '', warning = false) {
    const image = document.createElement('canvas')
    image.width = image.height = 64
    const ctx = image.getContext('2d')!
    ctx.beginPath()
    if (warning) {
      ctx.moveTo(32, 6); ctx.lineTo(58, 32); ctx.lineTo(32, 58); ctx.lineTo(6, 32); ctx.closePath()
    } else ctx.arc(32, 32, 26, 0, Math.PI * 2)
    ctx.fillStyle = color; ctx.fill()
    ctx.strokeStyle = '#fff'; ctx.lineWidth = 5; ctx.stroke()
    if (label) {
      ctx.fillStyle = '#fff'; ctx.font = '600 28px sans-serif'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle'
      ctx.fillText(label, 32, 34)
    }
    const texture = new THREE.CanvasTexture(image)
    texture.colorSpace = THREE.SRGBColorSpace
    const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: texture, sizeAttenuation: false, depthTest: false, depthWrite: false }))
    sprite.position.set(point[0], point[1], point[2] + .15)
    sprite.userData.pixels = label ? 28 : 16
    sprite.renderOrder = 5
    this.sizeSprite(sprite)
    this.markers.add(sprite)
  }

  private sizeSprite(sprite: THREE.Sprite) {
    sprite.scale.setScalar(sprite.userData.pixels * 2 * Math.tan(THREE.MathUtils.degToRad(this.camera.fov / 2)) / this.height)
  }

  updateLayers(layers: Layers) {
    disposeObjects(this.routes)
    disposeObjects(this.markers)
    const reference = layers.result?.reference ?? layers.route
    const result = layers.result
    this.routePoints = result?.navigationPath ?? result?.path ?? layers.route
    if (layers.mode !== 'optimized' || !result) {
      this.line(reference, '#ffffff', layers.mode === 'compare' ? 8 : 7)
      this.line(reference, '#c6801e', layers.mode === 'compare' ? 6 : 3.5, true)
    }
    if (result && layers.showRaw) this.line(result.rawCandidate, '#8a66aa', 2, true)
    if (result && layers.mode !== 'baseline') {
      const navigation = result.navigationPath ?? result.path
      const lift = result.navigationPath ? 0 : .10
      if (layers.showGround) this.line(result.path, '#667b87', 2)
      if (layers.mode !== 'compare') this.line(navigation, '#ffffff', 7, false, false, lift)
      this.line(navigation, '#087f73', 3.2, false, false, lift)
      if (layers.showRetained) {
        const segments: Point[] = []
        const retained = new Set(result.retainedIndices)
        for (const i of result.retainedIndices) if (retained.has(i + 1)) segments.push(result.path[i], result.path[i + 1])
        this.line(segments, '#e3b04a', 4, false, true)
      }
      if (layers.showInvalid) for (const i of result.validation.invalidIndices) this.marker(result.path[i], '#c53e3e')
      if (layers.showSharp) for (const point of result.optimized.sharpTurnPoints) this.marker(point, '#c76b28', '', true)
    }
    if (layers.start) this.marker(layers.start, '#087f73', '起')
    if (layers.goal) this.marker(layers.goal, '#b94343', '终')
    this.requestRender()
  }

  requestRender = () => {
    if (this.disposed || this.frame) return
    this.frame = requestAnimationFrame(() => {
      this.frame = 0
      this.controls.update()
      const metersPerPixel = 2 * this.camera.position.distanceTo(this.controls.target)
        * Math.tan(THREE.MathUtils.degToRad(this.camera.fov / 2)) / this.height
      // Subpixel voxels use the same complete XYZ set as points; close views keep cube faces.
      const usePoints = this.representation === 'points' || this.dataset.resolution / metersPerPixel < 1.25
      this.meshTiles.forEach(mesh => { mesh.visible = !usePoints })
      if (this.pointCloud) this.pointCloud.visible = usePoints
      this.renderer.render(this.scene, this.camera)
      this.canvas.dataset.frames = String(++this.frames)
      this.canvas.dataset.view = JSON.stringify({ view: this.camera.matrixWorldInverse.toArray(),
        projection: this.camera.projectionMatrix.toArray(), heightScale: this.verticalScale, mode: this.cameraView })
      if (this.keys.length) this.canvas.dataset.ready = 'true'
      this.canvas.dataset.representation = usePoints ? 'points' : 'voxels'
      this.canvas.dataset.drawCalls = String(this.renderer.info.render.calls)
      this.changed(metersPerPixel)
    })
  }

  dispose() {
    this.disposed = true
    cancelAnimationFrame(this.frame)
    this.controls.removeEventListener('change', this.requestRender)
    this.controls.dispose()
    disposeObjects(this.world)
    if (this.grid) { this.grid.geometry.dispose(); (this.grid.material as THREE.Material).dispose() }
    this.renderer.dispose()
    this.renderer.forceContextLoss()
  }
}
