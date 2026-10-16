export type Point = [number, number, number]
export type ViewMode = 'compare' | 'baseline' | 'optimized'
export type PickMode = 'orbit' | 'pan' | 'start' | 'goal'

export interface Endpoint {
  point: Point
  heights: number[]
}

export interface Options {
  smoothness: number
  max_deviation: number
  clearance: number
  control_spacing: number
}

export interface Metrics {
  lengthM: number
  length3dM: number
  turningDeg: number
  curvatureRms: number
  curvatureP95: number
  sharpTurns: number
  maxHeadingChangeDeg: number
  denseSharpTurns: number
  sharpTurnPoints: Point[]
  minClearanceM: number
  clearanceP05M: number
  belowClearancePercent: number
  profile: { s: number[]; curvature: number[] }
}

export interface Dataset {
  id: string
  name: string
  mapSha256: string
  resolution: number
  voxelCount: number
  areaM2: number
  bounds: { minX: number; minY: number; maxX: number; maxY: number }
  imageUrl: string
  voxelUrl: string
  voxelEncoding: 'int32-le-xyz'
  heightRange: { min: number; max: number }
  route: Point[]
  baseline: Metrics
}

export interface Validation {
  valid: boolean
  noNewViolations: boolean
  outsideCount: number
  heightViolationCount: number
  diagonalViolationCount: number
  transitionViolationCount: number
  deviationViolationCount: number
  clearanceViolationCount: number
  clearanceFloorM: number
  clearanceTargetM: number
  inheritedViolationCount: number
  newViolationCount: number
  invalidIndices: number[]
  maxDeviationM: number
  sampleCount: number
  sampleSpacingM: number
}

export interface Result {
  jobId: string
  dataset: string
  mapSha256: string
  algorithm: string
  options: Options
  baseline: Metrics
  optimized: Metrics
  validation: Validation
  baselineValidation: Validation
  path: Point[]
  sourceRoute: Point[]
  planning: {
    engine: string
    start: Point
    goal: Point
    iterations: number
    timeMs: number
    waypointCount: number
  } | null
  reference: Point[]
  rawCandidate: Point[]
  shortcut: {
    segments: number
    lengthSavedM: number
    path: Point[]
  }
  retainedIndices: number[]
  protectedSpans: { startM: number; endM: number }[]
  solver: {
    converged: boolean
    iterations: number
    message: string
    repairPasses: number
    controlPoints: number
    timeMs: number
    retainedPercent: number
    continuity: string
    refinementPasses: number
    minimumSpeed: number
    heightRematchedSamples: number
  }
}
