import React, { Component } from 'react';
import ReactDOM from 'react-dom';
import './App.css';

class App extends Component {

  constructor(props) {
    super(props);
    this.state = {
      inputValue: 0
    };
    // var connection = new WebSocket('ws://'+location.hostname+':81/', ['arduino']);
    // var connection = new WebSocket('ws://192.168.4.1:81/', ['arduino']);
    this.connection = new WebSocket('ws://0.0.0.0:5001');
  }

  touchStart = (id) => {
    // this.connection.send(`${id}`);
    console.log(`${id}`);
  }

  inputChange = (e) => {
    if (e) {
      e.persist();
      this.setState(prevState => Object.assign({}, prevState, { inputValue: e.target.value }),
        this.touchStart(e.target.value));
    }
  }

  render() {
    return (
      <div>
        <div id={"centerapp"}>
          <h1>LipSync Omni Contoller</h1>
          <div className={"container containerPlacement"}>
            <div id={"g1"} className={"gauge"}></div>
            <button id={"I"} className={"button"} type={"button"} onClick={() => this.touchStart('I')}>Mouse</button>
            <button id={"M"} className={"button"} type={"button"} onClick={() => this.touchStart('M')}>JoyStick</button>
            <button id={"L"} className={"button"} type={"button"} onClick={() => this.touchStart('L')}>TV Remote</button>
            <button id={"R"} className={"button"} type={"button"} onClick={() => this.touchStart('R')}>Playstation</button>
            <button id={"T"} className={"button"} type={"button"} onClick={() => this.touchStart('T')}>Test Comms</button>
          </div>
          <div className={"speedControl"}>
            <label htmlFor={"points"}>SPEED CONTROL</label>
            <input type={"range"} id={"speed_id"} value={this.state.inputValue} min={"0"} max={"1023"} onChange={this.inputChange} />
          </div>
        </div>
      </div>
    );
  }
}

ReactDOM.render(<App />, document.querySelector('#root'));
