import resolve from 'rollup-plugin-node-resolve'
import commonjs from 'rollup-plugin-commonjs'
import svelte from 'rollup-plugin-svelte'
import copy from 'rollup-plugin-copy'
import buble from 'rollup-plugin-buble'
import includePaths from 'rollup-plugin-includepaths'

export default {
  input: 'src/main.js',
  output: {
    file: 'dist/bundle.js',
    format: 'iife',
    sourcemap: true
  },
  plugins: [
    resolve(),
    includePaths({
      paths: ['deps/lcm_bridge_client/dist'],
      extensions: ['.js']
    }),
    commonjs(),
    svelte({
      include: 'src/components/**/*.html',
      cascade: false,
      css: (css) => {
        css.write('dist/site.css')
      }
    }),
    copy({
      'src/index.html': 'dist/index.html',
      'src/cam.html': 'dist/cam.html',
      'src/pidTune.html': 'dist/pidTune.html',
      'src/diagnostics.html': 'dist/diagnostics.html',
      'src/static': 'dist/static',
      'node_modules/leaflet/dist/leaflet.css': 'dist/leaflet.css'
    }),
    buble()
  ]
}
