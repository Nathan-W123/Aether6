import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

// The dashboard is served from the FastAPI process in production, so the dev server proxies
// the API to a locally running backend instead of duplicating any of its logic.
export default defineConfig({
  plugins: [react()],
  server: {
    port: 5173,
    proxy: {
      '/api': { target: 'http://127.0.0.1:8000', changeOrigin: true },
      '/health': { target: 'http://127.0.0.1:8000', changeOrigin: true },
    },
  },
  build: {
    outDir: 'dist',
    sourcemap: false,
    chunkSizeWarningLimit: 900,
    rollupOptions: {
      output: {
        // Three.js is ~600 kB of the bundle and never changes between deploys; splitting it
        // out keeps the app chunk small and cacheable.
        manualChunks: { three: ['three'] },
      },
    },
  },
})
