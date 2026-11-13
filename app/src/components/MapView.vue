<template>
  <canvas ref="cv" class="map" :class="{ pick: !!tool }" @pointerdown="down" @pointermove="move" @pointerup="up" @pointercancel="cancel" />
</template>

<script setup>
// SLAM map (gateway /map.png) with the robot pose, live scan and Nav2 path, map frame: x right, y up.
// With a tool set, tap = position, drag = heading; emits pick {x, y, yaw|null}.
// tool 'erase': drag paints a stroke; emits erase [[x, y], ...] (map frame) on release.
// cross {x, y, yaw?, kind}: crosshair lines whose intersection is the point being chosen;
// preview: route [[x, y], ...] planned for it (not driven yet).
// route: patrol waypoints being edited (the running patrol comes from state.nav.route); home: state.nav.home.
import { onMounted, onBeforeUnmount, ref, watch } from 'vue'
import { robot, httpBase } from '../robot.js'

const props = defineProps({
  tool: { type: String, default: '' },
  cross: { type: Object, default: null },
  preview: { type: Array, default: null },
  route: { type: Array, default: null },
})
const COL = { pose: '#f59e0b', goal: '#22c55e', route: '#f97316' }
const ERASE_R = 0.15        // m, same as the gateway's eraser
const emit = defineEmits(['pick', 'erase'])
const cv = ref(null)
let view = null            // {s, cx, cy, w, h} of the last frame
let drag = null            // {x0, y0, x1, y1} canvas px
let stroke = null          // eraser: [[x, y], ...] map frame

function toMap(px, py) {
  return { x: view.cx + (px - view.w / 2) / view.s, y: view.cy - (py - view.h / 2) / view.s }
}
function local(e) {
  const r = cv.value.getBoundingClientRect()
  return [e.clientX - r.left, e.clientY - r.top]
}
function down(e) {
  if (!props.tool || !view) return
  cv.value.setPointerCapture(e.pointerId)
  const [x, y] = local(e)
  if (props.tool === 'erase') { const p = toMap(x, y); stroke = [[p.x, p.y]]; return }
  drag = { x0: x, y0: y, x1: x, y1: y }
}
function move(e) {
  if (stroke) {
    const [x, y] = local(e), p = toMap(x, y), q = stroke[stroke.length - 1]
    if (Math.hypot(p.x - q[0], p.y - q[1]) > 0.04) stroke.push([p.x, p.y])
    return
  }
  if (!drag) return
  ;[drag.x1, drag.y1] = local(e)
}
function cancel() { drag = null; stroke = null }
function up() {
  if (stroke) {
    const st = stroke.map(([x, y]) => [+x.toFixed(3), +y.toFixed(3)]); stroke = null
    emit('erase', st)
    return
  }
  if (!drag) return
  const d = drag; drag = null
  const p = toMap(d.x0, d.y0)
  const moved = Math.hypot(d.x1 - d.x0, d.y1 - d.y0) > 15
  // canvas y points down, map y up
  const yaw = moved ? Math.atan2(-(d.y1 - d.y0), d.x1 - d.x0) : null
  emit('pick', { x: p.x, y: p.y, yaw })
}
function arrow(g, x, y, ang, len, color) {
  const ex = x + Math.cos(ang) * len, ey = y - Math.sin(ang) * len
  g.strokeStyle = color; g.fillStyle = color; g.lineWidth = 3
  g.beginPath(); g.moveTo(x, y); g.lineTo(ex, ey); g.stroke()
  g.beginPath(); g.arc(x, y, 5, 0, Math.PI * 2); g.fill()
  g.beginPath()
  g.moveTo(ex, ey)
  g.lineTo(ex - Math.cos(ang - 0.5) * 10, ey + Math.sin(ang - 0.5) * 10)
  g.lineTo(ex - Math.cos(ang + 0.5) * 10, ey + Math.sin(ang + 0.5) * 10)
  g.fill()
}
let raf = 0
let img = null
let imgSeq = -1

function loadMap(meta) {
  if (!meta || meta.seq === imgSeq) return
  const im = new Image()
  im.onload = () => { img = im; imgSeq = meta.seq }
  im.src = `${httpBase()}/map.png?seq=${meta.seq}`
}
watch(() => robot.state?.map?.seq, () => loadMap(robot.state?.map), { immediate: true })

function draw() {
  const c = cv.value
  if (!c) return
  const dpr = window.devicePixelRatio || 1
  const w = c.clientWidth, h = c.clientHeight
  if (c.width !== w * dpr) { c.width = w * dpr; c.height = h * dpr }
  const g = c.getContext('2d')
  g.setTransform(dpr, 0, 0, dpr, 0, 0)
  g.clearRect(0, 0, w, h)
  const m = robot.state?.map, pose = robot.state?.pose
  if (!m || !img) {
    g.fillStyle = '#6b7385'; g.font = '13px sans-serif'
    g.fillText('等待地图…', 10, 20)
    raf = requestAnimationFrame(draw)
    return
  }
  // fit the whole map (at least 4 m across) with a small margin
  const mw = m.w * m.res, mh = m.h * m.res
  const s = Math.min((w - 16) / Math.max(mw, 4), (h - 16) / Math.max(mh, 4))
  const cx = m.ox + mw / 2, cy = m.oy + mh / 2
  const X = (x) => w / 2 + (x - cx) * s
  const Y = (y) => h / 2 - (y - cy) * s
  view = { s, cx, cy, w, h }
  g.imageSmoothingEnabled = false
  g.drawImage(img, X(m.ox), Y(m.oy + mh), mw * s, mh * s)

  const nav = robot.state?.nav
  if (nav?.path?.length > 1) {
    g.strokeStyle = '#22c55e'; g.lineWidth = 3
    g.beginPath()
    nav.path.forEach(([px, py], i) => (i ? g.lineTo(X(px), Y(py)) : g.moveTo(X(px), Y(py))))
    g.stroke()
  }
  if (nav?.goal && ['pending', 'active', 'failed', 'rejected'].includes(nav.status)) {
    arrow(g, X(nav.goal.x), Y(nav.goal.y), nav.goal.yaw, 22, nav.status === 'active' || nav.status === 'pending' ? '#22c55e' : '#ef4444')
  }
  // home: blue flag pole + heading tick
  if (nav?.home) {
    const hx = X(nav.home.x), hy = Y(nav.home.y)
    g.strokeStyle = '#1d4ed8'; g.fillStyle = '#1d4ed8'; g.lineWidth = 2
    g.beginPath(); g.moveTo(hx, hy); g.lineTo(hx, hy - 22); g.stroke()
    g.beginPath(); g.moveTo(hx, hy - 22); g.lineTo(hx + 14, hy - 17); g.lineTo(hx, hy - 12); g.fill()
    g.beginPath(); g.arc(hx, hy, 4, 0, Math.PI * 2); g.fill()
    g.font = 'bold 11px sans-serif'; g.fillText('起点', hx + 4, hy + 14)
  }
  // patrol waypoints: numbered orange dots joined in order (current target ringed)
  const rt = nav?.route, pts = rt?.pts || props.route
  if (pts?.length) {
    g.strokeStyle = '#f97316'; g.lineWidth = 1.5; g.setLineDash([3, 4])
    g.beginPath()
    pts.forEach(([px, py], i) => (i ? g.lineTo(X(px), Y(py)) : g.moveTo(X(px), Y(py))))
    if (rt?.loop && pts.length > 2) g.closePath()
    g.stroke(); g.setLineDash([])
    pts.forEach(([px, py], i) => {
      g.fillStyle = '#f97316'
      g.beginPath(); g.arc(X(px), Y(py), 9, 0, Math.PI * 2); g.fill()
      if (rt && i === rt.i) { g.strokeStyle = '#fff'; g.lineWidth = 2; g.beginPath(); g.arc(X(px), Y(py), 12, 0, Math.PI * 2); g.stroke() }
      g.fillStyle = '#fff'; g.font = 'bold 11px sans-serif'; g.textAlign = 'center'; g.textBaseline = 'middle'
      g.fillText(String(i + 1), X(px), Y(py) + 0.5)
      g.textAlign = 'start'; g.textBaseline = 'alphabetic'
    })
  }
  if (props.preview?.length > 1) {
    g.strokeStyle = '#06b6d4'; g.lineWidth = 3; g.setLineDash([8, 6])
    g.beginPath()
    props.preview.forEach(([px, py], i) => (i ? g.lineTo(X(px), Y(py)) : g.moveTo(X(px), Y(py))))
    g.stroke(); g.setLineDash([])
  }
  if (pose) {
    const cs = Math.cos(pose.yaw), sn = Math.sin(pose.yaw)
    g.fillStyle = '#f87171'
    for (const [bx, by] of robot.scan) {
      const px = X(pose.x + bx * cs - by * sn), py = Y(pose.y + bx * sn + by * cs)
      g.fillRect(px - 1, py - 1, 2, 2)
    }
    // robot: footprint circle + heading arrow
    const r = Math.max(4, 0.15 * s)
    g.fillStyle = '#3b82f6'
    g.beginPath(); g.arc(X(pose.x), Y(pose.y), r, 0, Math.PI * 2); g.fill()
    g.strokeStyle = '#fff'; g.lineWidth = 2
    g.beginPath(); g.moveTo(X(pose.x), Y(pose.y))
    g.lineTo(X(pose.x) + cs * r * 1.8, Y(pose.y) - sn * r * 1.8); g.stroke()
  }
  if (stroke) {
    g.strokeStyle = 'rgba(236, 72, 153, 0.5)'; g.lineWidth = Math.max(6, 2 * ERASE_R * s)
    g.lineCap = 'round'; g.lineJoin = 'round'
    g.beginPath()
    stroke.forEach(([px, py], i) => (i ? g.lineTo(X(px), Y(py)) : g.moveTo(X(px), Y(py))))
    if (stroke.length === 1) g.lineTo(X(stroke[0][0]) + 0.1, Y(stroke[0][1]))
    g.stroke(); g.lineCap = 'butt'; g.lineJoin = 'miter'
  }
  const cr = props.cross
  if (cr) {
    const col = COL[cr.kind] || '#22c55e'
    const cxp = X(cr.x), cyp = Y(cr.y)
    g.strokeStyle = col; g.lineWidth = 1.5; g.setLineDash([6, 4])
    g.beginPath(); g.moveTo(cxp, 0); g.lineTo(cxp, h); g.moveTo(0, cyp); g.lineTo(w, cyp); g.stroke()
    g.setLineDash([])
    if (cr.yaw != null) arrow(g, cxp, cyp, cr.yaw, 26, col)
    else { g.strokeStyle = col; g.lineWidth = 3; g.beginPath(); g.arc(cxp, cyp, 9, 0, Math.PI * 2); g.stroke() }
  }
  if (drag) {
    const moved = Math.hypot(drag.x1 - drag.x0, drag.y1 - drag.y0) > 15
    const ang = moved ? Math.atan2(-(drag.y1 - drag.y0), drag.x1 - drag.x0) : 0
    const col = COL[props.tool] || '#22c55e'
    if (moved) arrow(g, drag.x0, drag.y0, ang, Math.max(22, Math.hypot(drag.x1 - drag.x0, drag.y1 - drag.y0)), col)
    else { g.fillStyle = col; g.beginPath(); g.arc(drag.x0, drag.y0, 6, 0, Math.PI * 2); g.fill() }
  }
  g.fillStyle = '#9aa3b5'; g.font = '12px sans-serif'
  g.fillText(`${mw.toFixed(1)} × ${mh.toFixed(1)} m`, 8, h - 8)
  raf = requestAnimationFrame(draw)
}

onMounted(() => { raf = requestAnimationFrame(draw) })
onBeforeUnmount(() => cancelAnimationFrame(raf))
</script>

<style scoped>
.map { width: 100%; height: 100%; display: block; background: #cdcdcd; touch-action: none; }
.map.pick { cursor: crosshair; }
</style>
