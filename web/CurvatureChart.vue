<script setup lang="ts">
import { computed } from 'vue'
import type { Metrics } from './types'

const props = defineProps<{ baseline: Metrics; optimized?: Metrics }>()
const length = computed(() => Math.max(props.baseline.lengthM, props.optimized?.lengthM ?? 0))
const limit = computed(() => {
  const all = [...props.baseline.profile.curvature, ...(props.optimized?.profile.curvature ?? [])].sort((a, b) => a - b)
  return Math.max(0.3, Math.min(6, all[Math.floor(all.length * 0.99)] || 1))
})
function line(metric: Metrics) {
  return metric.profile.curvature.map((value, i) => `${metric.profile.s[i] / length.value * 1000},${57 - Math.min(value, limit.value) / limit.value * 49}`).join(' ')
}
</script>

<template>
  <section class="curvature-panel" aria-label="沿程曲率对比">
    <div class="chart-title"><span>沿程曲率 <small>m⁻¹</small></span><small>{{ limit.toFixed(1) }} 上限</small></div>
    <svg viewBox="0 0 1000 70" preserveAspectRatio="none" role="img" aria-label="原始和优化路线的沿程绝对曲率">
      <path d="M0 8H1000 M0 32H1000 M0 57H1000" fill="none" stroke="#e7ecee" stroke-width="1" vector-effect="non-scaling-stroke" />
      <polyline :points="line(baseline)" fill="none" stroke="#c6801e" stroke-width="1.3" opacity="0.65" vector-effect="non-scaling-stroke" />
      <polyline v-if="optimized" :points="line(optimized)" fill="none" stroke="#087f73" stroke-width="1.6" vector-effect="non-scaling-stroke" />
    </svg>
    <div class="chart-axis"><span>0 m</span><span>{{ (length / 2).toFixed(0) }} m</span><span>{{ length.toFixed(1) }} m</span></div>
  </section>
</template>
