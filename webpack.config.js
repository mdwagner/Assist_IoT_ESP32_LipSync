const path = require("path");
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
        test: /\.jsx?$/, loader: 'babel-loader', exclude: /node_modules/
      },
      {
        test: /\.s?css$/, loader: 'css-loader'
      }
    ]
  },
  plugins: [
    new HtmlWebpackPlugin({
      template: 'src/index.ejs',
      minify: {
        removeComments: true
      }
    })
  ],
};
