<template>
  <div class="joy" ref="pad" :class="{ held }"
       @pointerdown="down" @pointermove="move" @pointerup="up" @pointercancel="up" @lostpointercapture="up">
    <div class="ring" />
    <div v-if="axis === 'x'" class="guide-x" />
    <div class="knob" :style="{ transform: `translate(${kx}px, ${ky}px)` }" />
    <div class="label">{{ label }}</div>
  </div>
</template>

<script setup>
import { ref } from 'vue'

const props = defineProps({
  axis: { type: String, default: 'xy' },   // 'xy' | 'x'
  label: { type: String, default: '' },
})
const emit = defineEmits(['move'])
const pad = ref(null)
const held = ref(false)
const kx = ref(0)
const ky = ref(0)
let pid = null

function update(ev) {
  const r = pad.value.getBoundingClientRect()
  const rad = r.width / 2
  let dx = ev.clientX - (r.left + rad)
  let dy = ev.clientY - (r.top + r.height / 2)
  if (props.axis === 'x') dy = 0
  const d = Math.hypot(dx, dy)
  const lim = rad * 0.72
  if (d > lim) { dx *= lim / d; dy *= lim / d }
  kx.value = dx
  ky.value = dy
  // dead zone 8 %, then rescale to keep fine control near centre
  const shape = (v) => {
    const a = Math.abs(v)
    return a < 0.08 ? 0 : Math.sign(v) * ((a - 0.08) / 0.92) ** 1.5
  }
  emit('move', shape(dx / lim), shape(-dy / lim))
}

function down(ev) {
  if (pid !== null) return
  pid = ev.pointerId
  pad.value.setPointerCapture(pid)
  held.value = true
  update(ev)
}
function move(ev) {
  if (ev.pointerId === pid) update(ev)
}
function up(ev) {
  if (ev.pointerId !== pid) return
  pid = null
  held.value = false
  kx.value = ky.value = 0
  emit('move', 0, 0)
}
</script>

<style scoped>
.joy { position: relative; width: 190px; height: 190px; touch-action: none; user-select: none; }
.ring { position: absolute; inset: 0; border-radius: 50%; background: radial-gradient(circle, #1f2430 0%, #161a22 70%);
  border: 2px solid #2d3444; }
.held .ring { border-color: #3b82f6; }
.guide-x { position: absolute; left: 14%; right: 14%; top: calc(50% - 3px); height: 6px; border-radius: 3px; background: #2d3444; }
.knob { position: absolute; left: calc(50% - 38px); top: calc(50% - 38px); width: 76px; height: 76px; border-radius: 50%;
  background: linear-gradient(160deg, #4b5568, #2b3240); box-shadow: 0 4px 14px #0009; pointer-events: none; }
.held .knob { background: linear-gradient(160deg, #60a5fa, #2563eb); }
.label { position: absolute; bottom: -22px; width: 100%; text-align: center; font-size: 12px; color: #7d8597; }
</style>
