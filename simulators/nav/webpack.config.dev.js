/* webpack.config.dev.js */

const CopyWebpackPlugin = require('copy-webpack-plugin');
const { VueLoaderPlugin } = require('vue-loader');
const path = require('path');
module.exports = {
  mode: 'development',
  entry: [
    './src/app.ts'
  ],
  module: {
    rules: [
      {
        enforce: 'pre',
        test: [/\.ts$/, /\.vue$/],
        exclude: /node_modules/,
        loader: 'eslint-loader',
      },
      {
        test: /\.vue$/,
        use: 'vue-loader'
      },
      {
        test: /\.css$/,
        use: [
          'vue-style-loader',
          'css-loader'
        ]
      },
      {
        test: /\.ts$/,
        exclude: /node_modules|vue\/src/,
        loader: "ts-loader",
        options: {
          appendTsSuffixTo: [/\.vue$/]
        },
      }
    ]
  },

  plugins: [
    new VueLoaderPlugin(),
    new CopyWebpackPlugin({
      patterns: [
        { from: 'src' }
      ]
    })
  ],

  resolve: {
    extensions: ['.ts', '.js', '.vue', '.json'],
    alias: {
        'vue$': 'vue/dist/vue.esm.js'
    },
    modules: [
      'deps',
      'node_modules'
    ]
  }
}
