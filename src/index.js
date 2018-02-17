import React, { Component } from 'react';
import ReactDOM from 'react-dom';
import './App.css';

class App extends Component {
  render() {
    return (
      <div>
        <div id="centerapp">
          <b>LipSync Omni Contoller</b>
          <div className="container containerPlacement" >
            <div id="g1" className="gauge"></div>
            <button id="I" className="button" type="button">Mouse</button>
            <button id="M" className="button" type="button">JoyStick</button>
            <button id="L" className="button" type="button">TV Remote</button>
            <button id="R" className="button" type="button">Playstation</button>
            <button id="T" className="button" type="button">Test Comms</button>
          </div>
          <div className="speedControl">
            <label htmlFor="points">SPEED CONTROL</label>
            <input type="range" id="speed_id" value="0" min="0" max="1023" onChange="connection.send(this.value)" />
          </div>
        </div>
      </div>
    );
  }
}

ReactDOM.render(<App />, document.querySelector('#root'));
