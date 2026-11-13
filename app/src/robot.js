// Connection to the C21 gateway: WebSocket JSON + heartbeat lease + command stream.
import { reactive } from 'vue'

const DEFAULT_HOST = '10.34.97.79:8080'
const HB_MS = 100          // gateway lease is 500 ms
const CMD_MS = 50          // 20 Hz while a stick is held

function defaultHost() {
  const saved = localStorage.getItem('c21.host')
  if (saved) return saved
  // When served by the gateway itself, talk to the same origin.
  if (location.protocol.startsWith('http') && location.port === '8080') return location.host
  return DEFAULT_HOST
}

export const robot = reactive({
  host: defaultHost(),
  connected: false,
  info: null,
  state: null,
  scan: [],
  lastMsg: 0,
  log: [],
  acks: {},
})

let ws = null
let hbTimer = null
let cmdTimer = null
let retryTimer = null
let want = { vx: 0, vy: 0, wz: 0 }
let active = false
let zeroBurst = 0

function addLog(text) {
  robot.log.unshift(`${new Date().toLocaleTimeString()}  ${text}`)
  if (robot.log.length > 80) robot.log.length = 80
}

export function httpBase() {
  return `http://${robot.host}`
}

export function setHost(h) {
  robot.host = h.trim()
  localStorage.setItem('c21.host', robot.host)
  disconnect()
  connect()
}

export function connect() {
  clearTimeout(retryTimer)
  if (ws) return
  let sock
  try {
    sock = new WebSocket(`ws://${robot.host}/ws`)
  } catch (e) {
    addLog(`地址无效: ${e}`)
    return
  }
  ws = sock
  sock.onopen = () => {
    robot.connected = true
    addLog(`已连接 ${robot.host}`)
    hbTimer = setInterval(() => send({ t: 'hb' }), HB_MS)
  }
  sock.onmessage = (ev) => {
    let m
    try { m = JSON.parse(ev.data) } catch { return }
    robot.lastMsg = Date.now()
    if (m.t === 'state') robot.state = m
    else if (m.t === 'scan') robot.scan = m.pts
    else if (m.t === 'hello') robot.info = m.info
    else if (m.t === 'ack') {
      robot.acks[m.req + (m.name ? ':' + m.name : '')] = m
      if (m.req !== 'nav_plan' && m.req !== 'nav_route_check') addLog(`${m.req}${m.name ? ' ' + m.name : ''}: ${m.ok ? '成功' : '失败'} ${m.msg || ''}`)
    }
  }
  sock.onclose = () => {
    if (ws === sock) ws = null
    robot.connected = false
    clearInterval(hbTimer)
    stopStream()
    retryTimer = setTimeout(connect, 1500)
  }
  sock.onerror = () => {}
}

export function disconnect() {
  clearTimeout(retryTimer)
  if (ws) {
    const s = ws
    ws = null
    s.onclose = null
    s.close()
  }
  robot.connected = false
  clearInterval(hbTimer)
  stopStream()
}

export function send(obj) {
  if (ws && ws.readyState === 1) {
    ws.send(JSON.stringify(obj))
    return true
  }
  return false
}

// ---- motion ----
function tick() {
  if (active) {
    send({ t: 'cmd', ...want })
  } else if (zeroBurst > 0) {
    send({ t: 'cmd', vx: 0, vy: 0, wz: 0 })
    zeroBurst--
  } else {
    stopStream()
  }
}

function stopStream() {
  clearInterval(cmdTimer)
  cmdTimer = null
}

export function setVelocity(vx, vy, wz) {
  want = { vx, vy, wz }
  const moving = Math.abs(vx) > 1e-3 || Math.abs(vy) > 1e-3 || Math.abs(wz) > 1e-3
  if (moving) {
    active = true
    if (!cmdTimer) { tick(); cmdTimer = setInterval(tick, CMD_MS) }
  } else if (active) {
    active = false
    zeroBurst = 4
    if (!cmdTimer) cmdTimer = setInterval(tick, CMD_MS)
    tick()
  }
}

export function estop(on = true) {
  if (on) { active = false; want = { vx: 0, vy: 0, wz: 0 } }
  send({ t: 'estop', on })
}

export const gimbal = (angle) => send({ t: 'gimbal', angle })
export const beep = (n = 1) => send({ t: 'beep', n })
export const setLimits = (max_linear, max_angular) => send({ t: 'limits', max_linear, max_angular })
export const setMode = (name, on, args) => send({ t: 'mode', name, on, args })

// Losing focus (app switched, screen off) must never leave the car moving.
document.addEventListener('visibilitychange', () => {
  if (document.hidden) setVelocity(0, 0, 0)
})
window.addEventListener('blur', () => setVelocity(0, 0, 0))
