import { defineConfig } from 'vite'
import vue from '@vitejs/plugin-vue'

export default defineConfig({
  plugins: [vue()],
  base: './',
  build: { outDir: 'dist', assetsInlineLimit: 0 },
  define: { __BUILD__: JSON.stringify(new Date().toLocaleString('zh-CN', { hour12: false })) },
})
