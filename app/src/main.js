import { createApp } from 'vue'
import App from './App.vue'
import { connect } from './robot.js'

createApp(App).mount('#app')
connect()
