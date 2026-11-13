<template>
  <canvas ref="cv" class="scan" />
</template>

<script setup>
import { onMounted, onBeforeUnmount, ref } from 'vue'
import { robot } from '../robot.js'

const props = defineProps({ range: { type: Number, default: 3 } })   // metres to edge
const cv = ref(null)
let raf = 0

function draw() {
  const c = cv.value
  if (!c) return
  const dpr = window.devicePixelRatio || 1
  const w = c.clientWidth, h = c.clientHeight
  if (c.width !== w * dpr) { c.width = w * dpr; c.height = h * dpr }
  const g = c.getContext('2d')
  g.setTransform(dpr, 0, 0, dpr, 0, 0)
  g.clearRect(0, 0, w, h)
  const cx = w / 2, cy = h / 2
  const s = Math.min(w, h) / 2 / props.range
  g.strokeStyle = '#2a3140'
  g.lineWidth = 1
  for (let r = 1; r <= props.range; r++) {
    g.beginPath(); g.arc(cx, cy, r * s, 0, Math.PI * 2); g.stroke()
  }
  // robot footprint, forward = up
  g.fillStyle = '#3b82f6'
  g.fillRect(cx - 0.14 * s, cy - 0.15 * s, 0.28 * s, 0.30 * s)
  g.fillStyle = '#93c5fd'
  g.beginPath(); g.moveTo(cx, cy - 0.22 * s); g.lineTo(cx - 0.06 * s, cy - 0.12 * s); g.lineTo(cx + 0.06 * s, cy - 0.12 * s); g.fill()
  g.fillStyle = '#f87171'
  for (const [x, y] of robot.scan) {
    // base frame: x forward, y left -> screen: up, left
    const px = cx - y * s, py = cy - x * s
    if (px >= 0 && px < w && py >= 0 && py < h) g.fillRect(px - 1.5, py - 1.5, 3, 3)
  }
  const clr = robot.state?.safety?.clearance
  if (clr != null) {
    g.fillStyle = '#facc15'; g.font = '12px sans-serif'
    g.fillText(`前方 ${clr.toFixed(2)} m`, 8, h - 8)
  }
  raf = requestAnimationFrame(draw)
}

onMounted(() => { raf = requestAnimationFrame(draw) })
onBeforeUnmount(() => cancelAnimationFrame(raf))
</script>

<style scoped>
.scan { width: 100%; height: 100%; display: block; }
</style>
