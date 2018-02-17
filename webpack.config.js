const path = require("path");
const webpack = require('webpack');
const HtmlWebpackPlugin = require("html-webpack-plugin");
// const InlineChunkWebpackPlugin = require('html-webpack-inline-chunk-plugin');

const PATHS = {
  src: path.join(__dirname, "src"),
  build: path.join(__dirname, "build"),
};

module.exports = {
  // Entries have to resolve to files! They rely on Node
  // convention by default so if a directory contains *index.js*,
  // it resolves to that.
  entry: {
    app: PATHS.src
  },
  output: {
    path: PATHS.build,
    filename: "[name].js",
  },
  devServer: {
    stats: 'errors-only',
    host: '0.0.0.0',
    port: 5000
  },
  plugins: [
    new webpack.optimize.CommonsChunkPlugin({
      name: 'vendor',
      minChunks: Infinity,
    }),
    new HtmlWebpackPlugin({
      title: "Webpack demo",
      minify: {
        minifyCSS: true,
        removeComments: true
      }
    })
  ],
};
