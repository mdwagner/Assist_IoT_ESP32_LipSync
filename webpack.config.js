const path = require("path");
const HtmlWebpackPlugin = require("html-webpack-plugin");
const ExtractTextPlugin = require("extract-text-webpack-plugin");

const PATHS = {
  src: path.join(__dirname, 'src', 'index.js'),
  build: path.join(__dirname, 'build'),
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
    new HtmlWebpackPlugin({
      title: 'Welcome to the LipSync Omni Page',
      template: 'src/template.html',
      minify: {
        removeComments: true
      }
    })
  ],
};
