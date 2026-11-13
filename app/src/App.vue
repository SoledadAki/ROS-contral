<template>
  <div class="app">
    <!-- top bar -->
    <header class="bar">
      <span class="dot" :class="robot.connected ? (link ? 'ok' : 'warn') : 'bad'" />
      <b>{{ robot.info?.name || 'C21' }}</b>
      <span class="chip">{{ robot.connected ? (link ? '底盘在线' : '底盘无响应') : '未连接 ' + robot.host }}</span>
      <span class="chip" v-if="batt" :class="{ low: batt.pct < 20 }">🔋 {{ batt.v.toFixed(1) }} V · {{ batt.pct }}%</span>
      <span class="chip" v-if="safety.source">控制源 {{ safety.source }}</span>
      <span class="chip" v-if="estopOn" style="background:#7f1d1d">急停中</span>
      <span class="chip" v-if="guardMsg" :class="guardMsg.cls">{{ guardMsg.text }}</span>
      <span class="grow" />
      <span class="chip mono" v-if="odom">x {{ odom.x.toFixed(2) }} y {{ odom.y.toFixed(2) }} θ {{ (odom.yaw * 57.3).toFixed(0) }}°</span>
    </header>

    <div class="body">
      <!-- video + radar -->
      <section class="view">
        <img v-if="robot.connected && videoOn" :key="videoKey" class="video" :src="videoUrl" @error="videoErr" />
        <div v-else class="video placeholder">{{ robot.connected ? '视频已关闭' : '等待连接…' }}</div>
        <div class="vsel">
          <button v-for="s in videoSources" :key="s" :class="{ on: videoOn && vsrc === s }" @click="pickVideo(s)">
            {{ VIDEO_NAMES[s] }}</button>
          <button :class="{ on: !videoOn }" @click="videoOn = false">关</button>
        </div>
        <div v-if="hasMap" class="radar map" :class="{ big: radarBig }">
          <MapView :tool="navTool" :cross="crossShown" :preview="preview" :route="navTool === 'route' ? routePts : null" @pick="onPick" @erase="onErase" />
          <button class="mapsize" @click="radarBig = !radarBig">{{ radarBig ? '缩小' : '放大' }}</button>
        </div>
        <div v-else class="radar" :class="{ big: radarBig }" @click="radarBig = !radarBig"><ScanView :range="radarBig ? 4 : 3" /></div>

        <!-- sticks -->
        <div class="stick left"><Joystick label="平移 / 前后" @move="onMove" /></div>
        <div class="stick right"><Joystick axis="x" label="转向" @move="onTurn" /></div>
        <button class="estop" :class="{ on: estopOn }" @click="toggleEstop">{{ estopOn ? '解除急停' : '急停' }}</button>
      </section>

      <!-- side panel -->
      <aside class="side">
        <nav class="tabs">
          <button v-for="t in tabs" :key="t.k" :class="{ on: tab === t.k }" @click="tab = t.k">{{ t.n }}</button>
        </nav>

        <div v-if="tab === 'ctl'" class="pane">
          <label>速度档 {{ speed.toFixed(2) }} m/s</label>
          <input type="range" min="0.05" max="0.6" step="0.05" v-model.number="speed" @change="pushLimits" />
          <label>转速档 {{ turn.toFixed(1) }} rad/s</label>
          <input type="range" min="0.3" max="2.5" step="0.1" v-model.number="turn" @change="pushLimits" />
          <label class="row"><input type="checkbox" v-model="strafe" /> 左摇杆横移</label>
          <label class="row"><input type="checkbox" :checked="safety.scan_guard" @change="setGuard($event.target.checked)" />
            雷达防撞</label>

          <label>云台 {{ (gim * 57.3).toFixed(0) }}°</label>
          <input type="range" min="-0.78" max="0.78" step="0.02" v-model.number="gim" @input="sendGimbal" />
          <div class="btns">
            <button @click="gim = 0; sendGimbal()">云台回中</button>
            <button @click="beep(1)">鸣笛</button>
          </div>
        </div>

        <div v-if="tab === 'mode'" class="pane">
          <div v-if="!modes.length" class="muted small">{{ robot.connected ? '没有功能列表' : '未连接' }}</div>
          <div v-for="m in modes" :key="m.name" class="mode">
            <span>{{ m.label.split('（')[0] }}</span>
            <button :class="{ on: running.includes(m.name) }" @click="toggleMode(m)">
              {{ running.includes(m.name) ? '停止' : '启动' }}</button>
            <select v-if="m.name === 'navigation' && !running.includes(m.name)" class="text" v-model="navMap">
              <option v-if="!maps.length" value="">{{ mapsErr ? '（读取地图列表失败）' : '（还没有保存的地图）' }}</option>
              <option v-for="mp in maps" :key="mp.name" :value="mp.name">{{ mp.name }}</option>
            </select>
            <div v-if="m.name === 'navigation' && mapsErr" class="err">{{ mapsErr }}</div>
            <div v-if="modeErr[m.name]" class="err">{{ modeErr[m.name] }}</div>
          </div>
          <div v-if="running.includes('mapping')" class="crossbox">
            <input class="text" v-model="mapName" placeholder="地图名（可不填）" maxlength="40" />
            <div class="btns"><button class="go" :disabled="!hasMap || saving" @click="saveMap">{{ saving ? '保存中…' : '保存地图' }}</button></div>
            <div v-if="!hasMap" class="muted small">等待第一帧地图…</div>
            <div v-if="saveAck" :class="saveAck.ok ? 'muted small' : 'err'">{{ saveAck.msg }}</div>
          </div>
          <div v-if="behavior" class="crossbox">
            <div class="small">{{ behavior.text }}</div>
          </div>
          <div v-if="running.includes('navigation')" class="mapbox">
            <div class="btns">
              <button :class="{ on: navTool === 'pose' }" @click="pickTool('pose')">设定当前位置</button>
              <button :class="{ on: navTool === 'goal' }" @click="pickTool('goal')">选目标</button>
              <button :class="{ on: navTool === 'route' }" @click="pickTool('route')">巡航路线</button>
              <button :class="{ on: navTool === 'erase' }" @click="pickTool('erase')">擦除</button>
            </div>
            <div class="btns">
              <button @click="goHome">回起点</button>
              <button @click="setHome">把这里设为起点</button>
              <button @click="send({ t: 'nav_cancel' })">取消导航</button>
            </div>
            <div v-if="navTool === 'erase'" class="crossbox">
              <div class="btns">
                <button @click="restoreMap">恢复原图</button>
                <button @click="navTool = ''">完成</button>
              </div>
            </div>
            <div v-else-if="navTool" class="crossbox">
              <label>竖线 左 ↔ 右 · x {{ cross.x.toFixed(2) }} m</label>
              <div class="nudge">
                <button @click="nudge('x', -1)">◀</button>
                <input type="range" :min="ext.x0" :max="ext.x1" step="0.05" v-model.number="cross.x" />
                <button @click="nudge('x', 1)">▶</button>
              </div>
              <label>横线 下 ↔ 上 · y {{ cross.y.toFixed(2) }} m</label>
              <div class="nudge">
                <button @click="nudge('y', -1)">▼</button>
                <input type="range" :min="ext.y0" :max="ext.y1" step="0.05" v-model.number="cross.y" />
                <button @click="nudge('y', 1)">▲</button>
              </div>
              <template v-if="navTool === 'pose'">
                <label>车头朝向 {{ yawDeg }}°</label>
                <div class="nudge">
                  <button @click="nudge('yaw', 1)">↺</button>
                  <input type="range" min="-180" max="180" step="5" v-model.number="yawDeg" />
                  <button @click="nudge('yaw', -1)">↻</button>
                </div>
                <div class="btns"><button class="go" @click="confirmPose">确定位置</button><button @click="navTool = ''">取消</button></div>
              </template>
              <template v-else-if="navTool === 'route'">
                <div class="btns">
                  <button @click="addPoint">加入此点</button>
                  <button @click="addHome">加入起点</button>
                  <button :disabled="!routePts.length" @click="routePts.pop()">撤销</button>
                  <button :disabled="!routePts.length" @click="routePts.splice(0)">清空</button>
                </div>
                <label class="chk"><input type="checkbox" v-model="routeLoop" /> 循环</label>
                <div class="small">已加 {{ routePts.length }} 个点。<span :class="{ err: routePlan.ok === false }">{{ routePlan.msg }}</span></div>
                <div class="btns"><button class="go" :disabled="!routePlan.ok" @click="startRoute">开始巡航</button><button @click="navTool = ''">关闭</button></div>
              </template>
              <template v-else>
                <div class="small" :class="{ err: plan.ok === false }">{{ plan.msg }}</div>
                <div class="btns"><button class="go" :disabled="!plan.ok" @click="go">出发</button><button @click="navTool = ''">取消</button></div>
              </template>
            </div>
            <div class="small">{{ navText }}</div>
            <div v-if="navAck" :class="navAck.ok ? 'muted small' : 'err'">{{ navAck.msg }}</div>
          </div>
        </div>

        <div v-if="tab === 'st'" class="pane mono small">
          <div>底盘链路: {{ link ? '正常' : '无数据' }} · {{ base.rx_hz ?? '-' }} Hz · 坏帧 {{ base.bad_frames ?? '-' }}</div>
          <div>串口: {{ base.port_open ? '已打开' : '未打开' }} · 陀螺零偏: {{ base.gyro_bias_ready ? '已标定' : '标定中' }}</div>
          <div>电池原始值: {{ base.battery_raw ?? '-' }} · 超声原始: {{ base.ultrasonic_raw ?? '-' }}</div>
          <div>下发速度: {{ (base.cmd || []).join(', ') }}</div>
          <div>仲裁输出: {{ (safety.out || []).join(', ') }} · 限速 {{ safety.max_linear }} / {{ safety.max_angular }}</div>
          <div>雷达防撞: {{ safety.scan_guard ? '开' : '关' }} · 前方 {{ safety.clearance ?? '-' }} m</div>
          <h4>日志</h4>
          <div v-for="(l, i) in robot.log" :key="i">{{ l }}</div>
        </div>

        <div v-if="tab === 'set'" class="pane">
          <label>小车地址 (IP:端口)</label>
          <input class="text" v-model="hostEdit" />
          <div class="btns">
            <button @click="setHost(hostEdit)">连接</button>
          </div>
          <label>视频宽度 {{ vw }} px · 质量 {{ vq }}</label>
          <input type="range" min="320" max="1280" step="160" v-model.number="vw" @change="videoKey++" />
          <input type="range" min="30" max="90" step="10" v-model.number="vq" @change="videoKey++" />
          <div class="muted small">APP 版本 {{ BUILD }}</div>
        </div>
      </aside>
    </div>
  </div>
</template>

<script setup>
import { computed, reactive, ref, watch } from 'vue'
import Joystick from './components/Joystick.vue'
import ScanView from './components/ScanView.vue'
import MapView from './components/MapView.vue'
import { robot, setVelocity, estop, gimbal, beep, setLimits, setMode, setHost, httpBase, send } from './robot.js'

const tabs = [{ k: 'ctl', n: '遥控' }, { k: 'mode', n: '功能' }, { k: 'st', n: '状态' }, { k: 'set', n: '设置' }]
const tab = ref('ctl')

const base = computed(() => robot.state?.base || {})
const safety = computed(() => robot.state?.safety || {})
const link = computed(() => !!base.value.link)
const batt = computed(() => robot.state?.battery)
const odom = computed(() => robot.state?.odom)
const estopOn = computed(() => !!robot.state?.estop)
const modes = computed(() => robot.info?.modes || [])
const running = computed(() => robot.state?.modes?.running || [])
const modeErr = computed(() => robot.state?.modes?.errors || {})

// ---- driving ----
const speed = ref(Number(localStorage.getItem('c21.speed') || 0.2))
const turn = ref(Number(localStorage.getItem('c21.turn') || 1.0))
const strafe = ref(localStorage.getItem('c21.strafe') !== '0')
let stick = { x: 0, y: 0 }, rot = 0

function apply() {
  const vx = stick.y * speed.value
  const vy = strafe.value ? -stick.x * speed.value : 0
  let wz = -rot * turn.value
  if (!strafe.value && rot === 0) wz = -stick.x * turn.value
  if (estopOn.value) return setVelocity(0, 0, 0)
  setVelocity(vx, vy, wz)
}
const onMove = (x, y) => { stick = { x, y }; apply() }
const onTurn = (x) => { rot = x; apply() }

function pushLimits() {
  localStorage.setItem('c21.speed', speed.value)
  localStorage.setItem('c21.turn', turn.value)
  setLimits(speed.value, turn.value)
}
watch(strafe, (v) => localStorage.setItem('c21.strafe', v ? '1' : '0'))
// sync arbiter limits with this tablet's choice on every (re)connect
watch(() => robot.connected, (c) => { if (c) pushLimits() })

// ---- lidar guard ----
function setGuard(on) {
  if (!on && !confirm('关闭雷达防撞后小车不会自动避让障碍，确定关闭？')) return
  send({ t: 'limits', scan_guard: on })
}
const guardMsg = computed(() => {
  const s = safety.value
  if (!s.scan_guard) return robot.connected && s.estop !== undefined ? { text: '防撞已关', cls: 'warn' } : null
  if (s.guard_factor == null || s.guard_factor >= 1) return null
  if (s.blind && (s.clearance == null || s.clearance > 0.5)) return { text: '雷达盲区·慢速', cls: 'warn' }
  if (s.guard_factor <= 0) return { text: s.clearance == null ? '雷达无数据·禁止平移' : `障碍 ${s.clearance.toFixed(2)} m·已停`, cls: 'low' }
  return { text: `障碍 ${s.clearance.toFixed(2)} m·减速`, cls: 'warn' }
})

function toggleEstop() {
  if (estopOn.value) estop(false)
  else { setVelocity(0, 0, 0); estop(true) }
}

// ---- gimbal ----
const gim = ref(0)
let gimT = 0, gimPending = null
function sendGimbal() {
  const wait = 60 - (Date.now() - gimT)
  if (wait > 0) { gimPending ??= setTimeout(() => { gimPending = null; sendGimbal() }, wait); return }
  gimT = Date.now()
  gimbal(gim.value)
}
watch(() => robot.state?.gimbal, (g) => { if (g != null && Date.now() - gimT > 1000) gim.value = g })

// ---- video ----
const videoOn = ref(true)
const vsrc = ref('color')
const vw = ref(Number(localStorage.getItem('c21.vw') || 640))
const vq = ref(Number(localStorage.getItem('c21.vq') || 60))
const videoKey = ref(0)
watch([vw, vq], () => { localStorage.setItem('c21.vw', vw.value); localStorage.setItem('c21.vq', vq.value) })
const videoUrl = computed(() => `${httpBase()}/video?src=${vsrc.value}&w=${vw.value}&q=${vq.value}&fps=15&k=${videoKey.value}`)
function pickVideo(s) { vsrc.value = s; videoOn.value = true; videoKey.value++ }
// 特征点: colour + ORB feature points; with vision follow on it also boxes people
// (white = seen, yellow = locking on, green = confirmed target)
const VIDEO_NAMES = { color: '彩色', depth: '深度', features: '特征点' }
const visionOn = computed(() => running.value.includes('vision_follow'))
const videoSources = ['color', 'depth', 'features']
watch(visionOn, (on) => {
  if (on && vsrc.value !== 'features') pickVideo('features')
})
function videoErr() { setTimeout(() => videoKey.value++, 2000) }

const radarBig = ref(false)

// ---- mapping ----
const hasMap = computed(() => !!robot.state?.map)
watch(hasMap, (v) => { radarBig.value = v })
const mapName = ref('')
const saveAck = computed(() => robot.acks.map_save)
const saving = ref(false)
watch(saveAck, (a) => { if (a) { saving.value = false; loadMaps() } })
function saveMap() {
  const name = mapName.value.trim()
  if (name && maps.value.some((m) => m.name === name) && !confirm(`已有地图“${name}”，覆盖它？`)) return
  delete robot.acks.map_save
  saving.value = true
  setTimeout(() => { saving.value = false }, 15000)
  send({ t: 'map_save', name })
}

// ---- navigation ----
const maps = ref([])
const mapsErr = ref('')
const navMap = ref(localStorage.getItem('c21.navMap') || '')
async function loadMaps() {
  try {
    const r = await fetch(`${httpBase()}/api/maps`, { cache: 'no-store' })
    if (!r.ok) throw new Error(`HTTP ${r.status}`)
    maps.value = (await r.json()).sort((a, b) => b.mtime - a.mtime)
    mapsErr.value = ''
    if (!maps.value.some((m) => m.name === navMap.value)) navMap.value = maps.value[0]?.name || ''
  } catch (e) {
    if (robot.connected) mapsErr.value = `读取地图列表失败：${e.message || e}`
  }
}
watch(tab, (t) => { if (t === 'mode') loadMaps() })
watch(() => robot.connected, (c) => { if (c) loadMaps() }, { immediate: true })
function toggleMode(m) {
  const on = !running.value.includes(m.name)
  if (m.name === 'navigation' && on) {
    if (!navMap.value) return
    localStorage.setItem('c21.navMap', navMap.value)
    setMode(m.name, true, { map: navMap.value })
  } else {
    if (on && m.motion && !window.confirm(`启动“${m.label}”后小车会自己动。
确认周围安全、车在地面上？`)) return
    setMode(m.name, on)
  }
}
const behavior = computed(() => robot.state?.behavior || null)
const navTool = ref('')
watch(() => running.value.includes('navigation'), (v) => { if (!v) navTool.value = '' })
const cross = reactive({ x: 0, y: 0, yaw: null })
const crossShown = computed(() => (navTool.value && navTool.value !== 'erase' ? { ...cross, kind: navTool.value } : null))
const ext = computed(() => {
  const m = robot.state?.map
  if (!m) return { x0: -5, x1: 5, y0: -5, y1: 5 }
  return { x0: +m.ox.toFixed(2), x1: +(m.ox + m.w * m.res).toFixed(2), y0: +m.oy.toFixed(2), y1: +(m.oy + m.h * m.res).toFixed(2) }
})
const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a))
const yawDeg = computed({
  get: () => Math.round(((cross.yaw ?? 0) * 180) / Math.PI),
  set: (v) => { cross.yaw = (v * Math.PI) / 180 },
})
function pickTool(t) {
  if (navTool.value === t) { navTool.value = ''; return }
  const p = robot.state?.pose || { x: 0, y: 0, yaw: 0 }
  cross.x = +p.x.toFixed(2); cross.y = +p.y.toFixed(2)
  cross.yaw = t === 'pose' ? p.yaw : null
  navTool.value = t
  radarBig.value = true
}
function nudge(k, d) {
  if (k === 'yaw') { cross.yaw = wrap((cross.yaw ?? 0) + (d * 5 * Math.PI) / 180); return }
  const lo = ext.value[k + '0'], hi = ext.value[k + '1']
  cross[k] = Math.min(hi, Math.max(lo, +(cross[k] + d * 0.1).toFixed(2)))
}
function onPick({ x, y, yaw }) {
  if (!navTool.value) return
  cross.x = +x.toFixed(2); cross.y = +y.toFixed(2)
  if (navTool.value === 'pose' && yaw != null) cross.yaw = yaw
}
function onErase(strokes) {
  if (navTool.value === 'erase') send({ t: 'map_erase', strokes: [strokes] })
}
function restoreMap() {
  if (window.confirm('撤销所有擦除，恢复建图时的原图？')) send({ t: 'map_restore' })
}
function confirmPose() {
  send({ t: 'init_pose', x: cross.x, y: cross.y, yaw: cross.yaw ?? 0 })
  navTool.value = ''
}
// goal preview: re-plan (no motion) a moment after the crosshair stops moving
const plan = ref({})
const preview = ref(null)
let planId = 0, planTimer = 0
watch(() => [navTool.value, cross.x, cross.y], () => {
  clearTimeout(planTimer)
  if (navTool.value === 'route') return       // the patrol preview only changes with its points
  preview.value = null
  if (navTool.value !== 'goal') return
  plan.value = { msg: '正在规划路线…' }
  planTimer = setTimeout(() => send({ t: 'nav_plan', id: ++planId, x: cross.x, y: cross.y }), 400)
})
watch(() => robot.acks.nav_plan, (a) => {
  if (!a || a.id !== planId || navTool.value !== 'goal') return
  plan.value = { ok: a.ok, msg: a.ok ? a.msg : a.msg }
  preview.value = a.ok ? a.path : null
})
function go() {
  send({ t: 'nav_goal', x: cross.x, y: cross.y, yaw: null })
  navTool.value = ''
}
// ---- patrol route (kept per map on this device) ----
const routeKey = () => `c21.route.${navMap.value}`
const routePts = ref([])
const routeLoop = ref(false)
watch(navMap, () => {
  try { routePts.value = JSON.parse(localStorage.getItem(routeKey()) || '[]') } catch { routePts.value = [] }
}, { immediate: true })
watch(routePts, (v) => localStorage.setItem(routeKey(), JSON.stringify(v)), { deep: true })
function addPoint() {
  if (routePts.value.length < 30) routePts.value.push([cross.x, cross.y])
}
function addHome() {
  const h = robot.state?.nav?.home
  if (h && routePts.value.length < 30) routePts.value.push([h.x, h.y])
}
const routePlan = ref({})
let routeId = 0, routeTimer = 0
watch(() => [navTool.value, routeLoop.value, JSON.stringify(routePts.value)], () => {
  clearTimeout(routeTimer)
  if (navTool.value !== 'route') return
  preview.value = null
  if (!routePts.value.length) { routePlan.value = {}; return }
  routePlan.value = { msg: '正在检查路线…' }
  routeTimer = setTimeout(() => send({ t: 'nav_route_check', id: ++routeId, pts: routePts.value, loop: routeLoop.value }), 400)
})
watch(() => robot.acks.nav_route_check, (a) => {
  if (!a || a.id !== routeId || navTool.value !== 'route') return
  routePlan.value = { ok: a.ok, msg: a.ok ? `${a.msg}，确认虚线没问题再点“开始巡航”` : a.msg }
  preview.value = a.ok ? a.path : null
})
function startRoute() {
  send({ t: 'nav_route', pts: routePts.value, loop: routeLoop.value })
  navTool.value = ''
}
function goHome() {
  navTool.value = ''
  send({ t: 'nav_home' })
}
function setHome() {
  if (window.confirm('把小车现在的位置和朝向设为起点？')) send({ t: 'home_set' })
}
const navAck = ref(null)
watch(() => [robot.acks.nav_goal, robot.acks.init_pose, robot.acks.home_set, robot.acks.map_erase], (now, old) => {
  const k = now.findIndex((a, i) => a !== old[i])
  if (k >= 0) navAck.value = now[k]
})
const NAV_TEXT = { idle: '空闲', pending: '正在规划…', active: '导航中', succeeded: '已到达目标', canceled: '已取消',
  failed: '导航失败（等了一会儿路还是不通）', rejected: '目标被拒绝', waypoint: '到达途经点',
  paused: '网络中断，已停车（15 秒内连回会自动继续）' }
const navText = computed(() => {
  const n = robot.state?.nav
  if (!n) return ''
  const r = n.route
  let t = '状态：' + (n.status === 'active' && n.kind === 'home' ? '回起点中' : NAV_TEXT[n.status] || n.status)
  if (r) t += ` · 巡航 第 ${r.i + 1}/${r.n} 点` + (r.loop ? ` · 第 ${r.lap} 圈` : '') + (r.waiting ? '（稍停）' : '')
  if (n.status === 'active' && n.dist != null) t += ` · 剩余 ${n.dist.toFixed(1)} m`
  if (n.status === 'active' && n.retries > 0) t += ` · 遇阻，等待后重新规划（第 ${n.retries} 次）`
  if (n.route_end) t += ` · ${n.route_end}`
  return t
})
const hostEdit = ref(robot.host)
// eslint-disable-next-line no-undef
const BUILD = __BUILD__
</script>

<style>
* { box-sizing: border-box; }
html, body, #app { margin: 0; height: 100%; background: #0e1117; color: #d6dae3; font-family: system-ui, sans-serif;
  overscroll-behavior: none; -webkit-user-select: none; user-select: none; }
button { background: #232a37; color: #d6dae3; border: 1px solid #323b4c; border-radius: 8px; padding: 8px 14px; font-size: 14px; }
button.on { background: #2563eb; border-color: #3b82f6; color: #fff; }
input[type=range] { width: 100%; margin: 6px 0 14px; }
</style>

<style scoped>
.app { display: flex; flex-direction: column; height: 100%; }
.bar { display: flex; align-items: center; gap: 10px; padding: 8px 14px; background: #151a23; border-bottom: 1px solid #222a37; font-size: 14px; }
.dot { width: 12px; height: 12px; border-radius: 50%; }
.dot.ok { background: #22c55e; } .dot.warn { background: #f59e0b; } .dot.bad { background: #ef4444; }
.chip { background: #202735; padding: 3px 10px; border-radius: 12px; font-size: 13px; }
.chip.low { background: #7c2d12; }
.chip.warn { background: #78350f; }
.grow { flex: 1; }
.mono { font-family: ui-monospace, Consolas, monospace; }
.small { font-size: 12px; line-height: 1.6; }
.body { flex: 1; display: flex; min-height: 0; }
.view { flex: 1; position: relative; overflow: hidden; background: #000; }
.video { width: 100%; height: 100%; object-fit: contain; display: block; }
.placeholder { display: flex; align-items: center; justify-content: center; color: #555e70; }
.vsel { position: absolute; top: 10px; left: 10px; display: flex; gap: 6px; }
.vsel button { padding: 5px 10px; background: #000a; }
.radar { position: absolute; top: 10px; right: 10px; width: 200px; height: 200px; background: #0b0e14cc; border: 1px solid #2a3140; border-radius: 10px; }
.radar.big { width: 420px; height: 420px; }
.radar.map { width: 260px; height: 260px; overflow: hidden; }
.radar.map.big { width: 58%; height: 68%; }
.mapbox { padding: 12px 0; }
.mapsize { position: absolute; top: 6px; right: 6px; padding: 4px 10px; font-size: 12px; background: #000a; }
.maphint { position: absolute; left: 0; right: 0; top: 6px; margin: 0 auto; width: fit-content; max-width: 70%; padding: 4px 10px;
  background: #000c; color: #fde68a; border-radius: 8px; font-size: 13px; pointer-events: none; }
select.text { margin-top: 6px; }
.crossbox { margin: 10px 0; padding: 10px; border: 1px solid #2a3140; border-radius: 10px; }
.nudge { display: flex; align-items: center; gap: 8px; }
.nudge input[type=range] { flex: 1; margin: 6px 0; }
.nudge button { padding: 6px 12px; }
button.go { background: #15803d; border-color: #22c55e; color: #fff; }
button:disabled { opacity: 0.4; }
label.chk { display: flex; align-items: center; gap: 6px; margin: 6px 0; }
.stick { position: absolute; bottom: 34px; }
.stick.left { left: 28px; } .stick.right { right: 28px; }
.estop { position: absolute; bottom: 40px; left: 50%; transform: translateX(-50%); width: 120px; height: 120px; border-radius: 50%;
  background: #b91c1c; border: 4px solid #fca5a5; color: #fff; font-size: 20px; font-weight: 700; box-shadow: 0 6px 20px #000a; }
.estop.on { background: #f59e0b; border-color: #fde68a; color: #111; font-size: 16px; }
.side { width: 300px; background: #121722; border-left: 1px solid #222a37; display: flex; flex-direction: column; }
.tabs { display: flex; }
.tabs button { flex: 1; border-radius: 0; border: none; border-bottom: 2px solid transparent; background: transparent; padding: 12px 0; }
.tabs button.on { border-bottom-color: #3b82f6; color: #fff; background: transparent; }
.pane { padding: 14px; overflow-y: auto; flex: 1; font-size: 14px; }
.pane label { display: block; color: #9aa3b5; }
.pane label.row { display: flex; gap: 8px; align-items: center; margin-bottom: 14px; }
.btns { display: flex; gap: 8px; flex-wrap: wrap; margin-bottom: 14px; }
.text { width: 100%; padding: 8px; margin: 6px 0 10px; background: #0e1117; color: #fff; border: 1px solid #323b4c; border-radius: 6px; font-size: 15px; }
.muted { color: #6b7385; }
.mode { display: flex; flex-wrap: wrap; align-items: center; justify-content: space-between; padding: 10px 0; border-bottom: 1px solid #222a37; }
.mode small { color: #f59e0b; }
.mode.off { opacity: .75; }
.mode small.muted { color: #8b95a7; }
.err { width: 100%; color: #f87171; font-size: 12px; }
h4 { margin: 12px 0 4px; color: #9aa3b5; }
button { white-space: nowrap; }
@media (max-width: 900px) {
  .body { flex-direction: column; }
  .side { width: 100%; height: 42%; border-left: none; border-top: 1px solid #222a37; }
  .radar { width: 140px; height: 140px; }
  .estop { width: 90px; height: 90px; font-size: 16px; }
  .stick { transform: scale(0.8); transform-origin: bottom center; bottom: 20px; }
  .stick.left { left: 6px; } .stick.right { right: 6px; }
}
</style>
