const path = require('path');
const webpack = require('webpack');
const HtmlWebpackPlugin = require('html-webpack-plugin');
const HtmlWebpackInlineSourcePlugin = require('html-webpack-inline-source-plugin');
const ExtractTextPlugin = require("extract-text-webpack-plugin");
const CompressionPlugin = require('compression-webpack-plugin');
const FilterChunkWebpackPlugin = require('filter-chunk-webpack-plugin');

const PATHS = {
  src: path.join(__dirname, 'src', 'index.js'),
  build: path.join(__dirname, 'build'),
  template: path.join(__dirname, 'src', 'template.html')
};

module.exports = {
  entry: {
    app: PATHS.src
  },
  output: {
    path: PATHS.build,
    filename: '[name].js',
  },
  devServer: {
    stats: 'errors-only',
    host: '0.0.0.0',
    port: 5000
  },
  module: {
    rules: [
      {
        test: /\.jsx?$/, loader: 'babel-loader', exclude: /node_modules/
      },
      {
        test: /\.css$/, use: ExtractTextPlugin.extract({
          fallback: 'style-loader',
          use: 'css-loader'
        }), exclude: /node_modules/
      }
    ]
  },
  plugins: [
    new ExtractTextPlugin({
      allChunks: true,
      filename: '[name].css'
    }),
    new webpack.optimize.UglifyJsPlugin(),
    new HtmlWebpackPlugin({
      title: 'Welcome to the LipSync Omni Page',
      template: PATHS.template,
      minify: {
        removeComments: true,
        minifyCSS: true,
        minifyJS: true,
        preserveLineBreaks: true,
        collapseWhitespace: true
      },
      inlineSource: '.(js|css)$'

    }),
    new HtmlWebpackInlineSourcePlugin(),
    new CompressionPlugin({
      test: /\.html$/
    }),
    new FilterChunkWebpackPlugin({
      patterns: [
        '*.js',
        '*.css'
      ]
    })
  ],
};
