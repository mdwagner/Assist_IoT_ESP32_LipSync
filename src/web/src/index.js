import { render, h, Component } from 'preact';
import './App.css';

class App extends Component {

  constructor(props) {
    super(props);
    this.state = {
      inputValue: 0
    };
    // this.connection = new WebSocket('ws://0.0.0.0:5001'); // WebSocket Connection configuration
  }

  touchStart = (id) => {
    // this.connection.send(`${id}`); // uncomment when `this.connection` is set correctly
    console.log(`${id}`); // read out of text sent to WebSocket
  }

  inputChange = (e) => {
    const newValue = e.target.value;
    this.setState(prevState => {
      const newState = {
        inputValue: newValue
      };
      return Object.assign({}, prevState, newState);
    }, this.touchStart(newValue));
  }

  render(props, state) {
    return (
      <div>
        <div id="centerapp">
          <h1>LipSync Omni Contoller</h1>
          <div className="container containerPlacement">
            <div id="g1" className="gauge"></div>
            <button id="I" className="button" type="button" onClick={() => this.touchStart('I')}>Mouse</button>
            <button id="M" className="button" type="button" onClick={() => this.touchStart('M')}>JoyStick</button>
            <button id="L" className="button" type="button" onClick={() => this.touchStart('L')}>TV Remote</button>
            <button id="R" className="button" type="button" onClick={() => this.touchStart('R')}>Playstation</button>
            <button id="T" className="button" type="button" onClick={() => this.touchStart('T')}>Test Comms</button>
          </div>
          <div className="speedControl">
            <label htmlFor="points">SPEED CONTROL</label>
            <input
              type="range"
              id="speed_id"
              value={this.state.inputValue}
              min="0"
              max="1023"
              onChange={this.inputChange} />
          </div>
        </div>
      </div>
    );
  }
}

render(<App />, document.getElementById('root'));
