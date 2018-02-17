const path = require("path");
const webpack = require('webpack');
const HtmlWebpackPlugin = require("html-webpack-plugin");

const PATHS = {
  src: path.join(__dirname, "src"),
  build: path.join(__dirname, "build"),
};

module.exports = {
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
  module: {
    rules: [
      {
        test: /\.html$/,
        loader: 'file-loader',
        options: {
          name: '[name].[ext]'
        }
      }
    ]
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
