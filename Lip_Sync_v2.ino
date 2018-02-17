#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include "BLE2902.h"
#include "BLEHIDDevice.h"
#include "HIDTypes.h"
#include <driver/adc.h>
#include <EEPROM.h>
//#include <Mouse.h>
#include <math.h>

#include <WiFi.h>
#//nclude <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
//#include <FS.h>
#include <SPIFFS.h>


#define LED_1 23                                   // LipSync LED Color1 : GREEN - digital output pin 4
#define LED_2 18                                   // LipSync LED Color2 : RED - digital outputpin 5
#define CONNECT_LED 5                              // LED for visual confirmation that Peripheral is connected Central
#define PUSH_BUTTON_CALI_MODE 14                   // JUMPER on the bread board.  This is being used temporarily until the final design work and testing is completed
 
#define X_DIR_HIGH 36
#define X_DIR_LOW 37
#define Y_DIR_HIGH 38
#define Y_DIR_LOW 39

//State Machine States
// state machine states
unsigned int state;
#define SEQUENCE_IDLE 0x00
#define GET_SAMPLE 0x10

#define GET_SAMPLE__WAITING 0x12

//Serial stuff
#define USE_SERIAL Serial
#define DBG_OUTPUT_PORT Serial

//***VARIABLE DECLARATIONS FOR WIFI***//
const char WiFiAPPSK[] = "";

const char* host = "LipSync_v2";

String Argument_Name, Clients_Response1, Clients_Response2;

const char* ssid = "Black";
const char* password = "";
const char* accesspoint;

//--------------------------------------


//***VARIABLE DECLARATIONS FOR THE LIPSYNC***//
int xh, yh, xl, yl;                               // xh: x-high, yh: y-high, xl: x-low, yl: y-low
int x_right, x_left, y_up, y_down;                // individual neutral starting positions for each FSR

int xh_max, xl_max, yh_max, yl_max;               // may just declare these variables but not initialize them because
// these values will be pulled from the EEPROM

float constant_radius = 30.0;                     // constant radius is initialized to 30.0 but may be changed in joystick initialization
float xh_yh_radius, xh_yl_radius, xl_yl_radius, xl_yh_radius;
float xh_yh, xh_yl, xl_yl, xl_yh;
int box_delta;                                    // the delta value for the boundary range in all 4 directions about the x,y center
int cursor_delta;                                 // amount cursor moves in some single or combined direction
int speed_counter = 4;                            // cursor speed counter
int cursor_click_status = 0;                      // value indicator for click status, ie. tap, back and drag
int comm_mode = 0;                                // 0 == USB Communications or 1 == Bluetooth Communications
int config_done;                                  // Binary check of completed Bluetooth configuration
unsigned int puff_count, sip_count;               // int puff and long sip incremental counter :: changed from unsigned long to unsigned int

int poll_counter = 0;                             // cursor poll counter
int init_counter_A = 0;                           // serial port initialization counter
int init_counter_B = 0;                           // serial port initialization counter

int default_cursor_speed = 30;
int delta_cursor_speed = 5;

int cursor_delay;
float cursor_factor;
int cursor_max_speed;

float yh_comp = 1.0;
float yl_comp = 1.0;
float xh_comp = 1.0;
float xl_comp = 1.0;

float yh_check, yl_check, xh_check, xl_check;
int xhm_check, xlm_check, yhm_check, ylm_check;
float sip_threshold, puff_threshold, cursor_click, cursor_back;

typedef struct {
  int _delay;
  float _factor;
  int _max_speed;
} _cursor;

_cursor setting1 = {5, -1.1, default_cursor_speed - (4 * delta_cursor_speed)}; // 5,-1.0,10
_cursor setting2 = {5, -1.1, default_cursor_speed - (3 * delta_cursor_speed)}; // 5,-1.2,10
_cursor setting3 = {5, -1.1, default_cursor_speed - (2 * delta_cursor_speed)};
_cursor setting4 = {5, -1.1, default_cursor_speed - (delta_cursor_speed)};
_cursor setting5 = {5, -1.1, default_cursor_speed};
_cursor setting6 = {5, -1.1, default_cursor_speed + (delta_cursor_speed)};
_cursor setting7 = {5, -1.1, default_cursor_speed + (2 * delta_cursor_speed)};
_cursor setting8 = {5, -1.1, default_cursor_speed + (3 * delta_cursor_speed)};
_cursor setting9 = {5, -1.1, default_cursor_speed + (4 * delta_cursor_speed)};

_cursor cursor_params[9] = {setting1, setting2, setting3, setting4, setting5, setting6, setting7, setting8, setting9};

int single = 0;
int puff1, puff2;
//-----------------------------------------------------------------------------------------------------------------------------------

//Webserver instance
WebServer server(80);

static BLEHIDDevice* hid;
BLECharacteristic* input;
uint8_t buttons = 0;
bool connected = false;

class MyCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer){
    connected = true;
  }

  void onDisconnect(BLEServer* pServer){
    connected = false;
  }
};

void taskServer(void*){


    BLEDevice::init("ESP32_Omni_Device");
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyCallbacks());

    hid = new BLEHIDDevice(pServer);

    input = hid->inputReport(1); // <-- input REPORTID from report map
    std::string name = "Assist_IoT";
    hid->manufacturer()->setValue(name);

    hid->pnp(0x02, 0xe502, 0xa111, 0x0210);
    hid->hidInfo(0x00,0x01);

  BLESecurity *pSecurity = new BLESecurity();
  pSecurity->setKeySize();
  pSecurity->setAuthenticationMode(ESP_LE_AUTH_BOND);

const uint8_t mouse1_report[] = {

0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
0x09, 0x02,        // Usage (Mouse)
0xA1, 0x01,        // Collection (Application)
0x85, 0x01,        //   Report ID (1)
0x09, 0x01,        //   Usage (Pointer)
0xA1, 0x00,        //   Collection (Physical)
0x05, 0x09,        //     Usage Page (Button)
0x19, 0x01,        //     Usage Minimum (0x01)
0x29, 0x03,        //     Usage Maximum (0x03)
0x25, 0x01,        //     Logical Maximum (1)
0x75, 0x01,        //     Report Size (1)
0x95, 0x03,        //     Report Count (3)
0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
0x75, 0x05,        //     Report Size (5)
0x95, 0x01,        //     Report Count (1)
0x81, 0x01,        //     Input (Const,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
0x05, 0x01,        //     Usage Page (Generic Desktop Ctrls)
0x09, 0x30,        //     Usage (X)
0x26, 0x05,0x56,        //     Logical Maximum (127)
0x09, 0x31,        //     Usage (Y)
0x26, 0x03,0x00,        //     Logical Maximum (127)
0x09, 0x38,        //     Usage (Wheel)
0x15, 0x00,        //     Logical Minimum (-127)
0x25, 0x7f,        //     Logical Maximum (127)
0x75, 0x08,        //     Report Size (8)
0x95, 0x03,        //     Report Count (3)
0x81, 0x02,        //     Input (Data,Var,Rel,No Wrap,Linear,Preferred State,No Null Position)
0xC0,              //   End Collection
0xC0,              // End Collection

// 52 bytes
    };

const uint8_t mouse2_report[] = {

0x05, 0x01,     // Usage Page (Generic Desktop Ctrls)
0x09, 0x02,     // Usage (Mouse)   << CHANGE THIS VALUE TO SET THE TYPE OF DEVICE FOR COMMUNICATION PURPOSES
0xA1, 0x01,     // Collection (Physical)
0x09, 0x01,         // Usage (Pointer)
0xA1, 0x00,          // Collection (Physical)
0x85, 0x01,         //   Report ID (1)
0x05, 0x09,         //   Usage Page (Button)
0x19, 0x01,         //   Usage Minimum (0x01)
0x29, 0x03,         //   Usage Maximum (0x03)
0x15, 0x00,         //   Logical Minimum (0)
0x25, 0x01,         //   Logical Maximum (1)
0x95, 0x03,         //   Report Count (3)
0x75, 0x01,     //   Report Size (1)
0x81, 0x02,     //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
0x95, 0x01,     //   Report Count (1)
0x75, 0x05,     //   Report Size (5)
0x81, 0x03,     //   Input (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
0x05, 0x01,     //   Usage Page (Generic Desktop Ctrls)
0x09, 0x30,     //   Usage (X)
0x09, 0x31,     //   Usage (Y)
0x09, 0x38,     //   Usage (Wheel)
0x15, 0x81,     //   Logical Minimum (-127)
0x25, 0x7F,     //   Logical Maximum (127)
0x75, 0x08,     //   Report Size (8)
0x95, 0x03,     //   Report Count (3)
0x81, 0x06,     //   Input (Data,Var,Rel,No Wrap,Linear,Preferred State,No Null Position)
0xC0,           // End Collection
0xC0,           // End Collection
  
};


    hid->reportMap((uint8_t*)mouse2_report, sizeof(mouse2_report));
    hid->startServices();

    BLEAdvertising *pAdvertising = pServer->getAdvertising();

    //  The setAppearance function only changes what is passed in the adv packet...it does not set the communication protocol
    //pAdvertising->setAppearance(HID_KEYBOARD);
    //pAdvertising->setAppearance(HID_JOYSTICK);
    pAdvertising->setAppearance(HID_MOUSE);
    pAdvertising->addServiceUUID(hid->hidService()->getUUID());
    pAdvertising->start();

    hid->setBatteryLevel(5);

    Serial.print("Advertising started!");
    delay(portMAX_DELAY);
  
};


// THIS IS TOO FANCY FOR OUR NEEDS BUT IT MIGHT BE USEFUL LATER ON....
String getContentType(String filename){
  if(server.hasArg("download")) return "application/octet-stream";
  else if(filename.endsWith(".htm")) return "text/html";
  else if(filename.endsWith(".html")) return "text/html";
  else if(filename.endsWith(".css")) return "text/css";
  else if(filename.endsWith(".js")) return "application/javascript";
  else if(filename.endsWith(".png")) return "image/png";
  else if(filename.endsWith(".gif")) return "image/gif";
  else if(filename.endsWith(".jpg")) return "image/jpeg";
  else if(filename.endsWith(".ico")) return "image/x-icon";
  else if(filename.endsWith(".xml")) return "text/xml";
  else if(filename.endsWith(".pdf")) return "application/x-pdf";
  else if(filename.endsWith(".zip")) return "application/x-zip";
  else if(filename.endsWith(".gz")) return "application/x-gzip";
  else if(filename.endsWith(".svg")) return "image/svg+xml";
  return "text/plain";
}

bool handleFileRead(String path){
  DBG_OUTPUT_PORT.println("handleFileRead: " + path);
  if(path.endsWith("/"))
    {
      path += "relay.html";
      state = SEQUENCE_IDLE;
    }
  String contentType = getContentType(path);
  DBG_OUTPUT_PORT.println("contenType = " + contentType);
  String pathWithGz = path + ".gz";
  DBG_OUTPUT_PORT.println("pathWithGz: = " + pathWithGz);
  DBG_OUTPUT_PORT.println("PathFile: " + pathWithGz);
  if(SPIFFS.exists(pathWithGz) || SPIFFS.exists(path)){
    if(SPIFFS.exists(pathWithGz))
      path += ".gz";
    File file = SPIFFS.open(path, "r");
    DBG_OUTPUT_PORT.println("Path after SPIFF open = " + path);
    size_t sent = server.streamFile(file, contentType);
    file.close();
    return true;
  }
  return false;
}

void setupWiFi()
{
  /*WiFi.mode(WIFI_STA);
  
  String AP_NameString = "Black";

  char AP_NameChar[AP_NameString.length() + 1];
  memset(AP_NameChar, 0, AP_NameString.length() + 1);

  for (int i=0; i<AP_NameString.length(); i++)
    AP_NameChar[i] = AP_NameString.charAt(i);

  WiFi.softAP(AP_NameChar, WiFiAPPSK);*/
  WiFi.begin("Black", "");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
     Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  }

void writeEEPROM(String buffer, int N) {
  EEPROM.begin(512); delay(10);
  for (int L = 0; L < 32; ++L) {
    EEPROM.write(N + L, buffer[L]);
  }
  EEPROM.commit();
}
//
String readEEPROM(int min, int max) {
  EEPROM.begin(512); delay(10); String buffer;
  for (int L = min; L < max; ++L)
    //if (isAlphaNumeric(EEPROM.read(L)))
      buffer += char(EEPROM.read(L));
  return buffer;
}

void scanWifi()
{
  // Direct Serial is allowed here, since this function will only be called from serial input.
  Serial.println(F("WIFI : SSID Scan start"));
  int n = WiFi.scanNetworks();
  if (n == 0)
    Serial.println(F("WIFI : No networks found"));
  else
  {
    Serial.print(F("WIFI : "));
    Serial.print(n);
    Serial.println(F(" networks found"));
    for (int i = 0; i < n; ++i)
    {
      // Print SSID and RSSI for each network found
      Serial.print(F("WIFI : "));
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(WiFi.SSID(i));
      Serial.print(" (");
      Serial.print(WiFi.RSSI(i));
      Serial.print(")");
      Serial.println("");
      delay(10);
    }
  }
  Serial.println("");
}


//********************************************************************************
// Web response to list APs and get users choice.  Create HTML with string for now use saas later on
//********************************************************************************
void handle_clientresponse()
{
  char *TempString = (char*)malloc(80);
  String webpage;
  webpage =  "<html>";
   webpage += "<head><title>ESP8266 Input Example</title>";
    webpage += "<style>";
     webpage += "body { background-color: #E6E6FA; font-family: Arial, Helvetica, Sans-Serif; Color: blue;}";
      webpage += "</style>";
        webpage += "</head>";
          webpage += "<body>";
          webpage += ("<table><TR><TH>Access Points:<TH>RSSI");

          int n = WiFi.scanNetworks();
          if (n == 0)
            webpage += ("No Access Points found");
          else
          {
            for (int i = 0; i < n; ++i)
            {
              webpage += ("<TR><TD>");
              webpage += WiFi.SSID(i);
              webpage += "<TD>";
              webpage += WiFi.RSSI(i);
              webpage += "<TR><TD>";
            }
          }
          webpage += "<h1><br>ESP8266 Server - Getting input from a client</h1>";  
            String IPaddress = WiFi.localIP().toString();
            webpage += "<form action='http://"+IPaddress+"' method='POST'>";
            webpage += "&nbsp;&nbsp;&nbsp;&nbsp;Please enter the Access Point you would like to connect to:<input type='text' name='access_point'>&nbsp;<input type='submit' value='Enter'>";
            webpage += "</form>";
          webpage += "</body>";
         webpage += "</html>";
  server.send(200, "text/html", webpage); // Send a response to the client asking for input
  if (server.args() > 0 ) { // Arguments were received
    for ( uint8_t i = 0; i < server.args(); i++ ) {
      Serial.print(server.argName(i)); // Display the argument
      Argument_Name = server.argName(i);
      if (server.argName(i) == "access_point") {
        Serial.print(" Input received was: ");
        Serial.println(server.arg(i));
        Clients_Response1 = server.arg(i);
        // e.g. range_maximum = server.arg(i).toInt();   // use string.toInt()   if you wanted to convert the input to an integer number
        // e.g. range_maximum = server.arg(i).toFloat(); // use string.toFloat() if you wanted to convert the input to a floating point number
      }
    }
  }
  free(TempString);
}


//********************************************************************************
// Web repsonse with users choice
//********************************************************************************
void handle_showclientresponse() {
  String webpage;
  webpage =  "<html>";
   webpage += "<head><title>ESP8266 Input Example</title>";
    webpage += "<style>";
     webpage += "body { background-color: #E6E6FA; font-family: Arial, Helvetica, Sans-Serif; Color: blue;}";
    webpage += "</style>";
   webpage += "</head>";
   webpage += "<body>";
    webpage += "<h1><br>ESP8266 Server - This was what the client sent</h1>";
    webpage += "<p>Name received was: " + Clients_Response1 + "</p>";
    webpage += "<p>Address received was: " + Clients_Response2 + "</p>";
   webpage += "</body>";
  webpage += "</html>";
  server.send(200, "text/html", webpage); // Send a response to the client asking for input
}

void setup() {
  Serial.begin(115200);


  //***********************  WIP BEGIN  *******************
  /*
  //https://arduino.stackexchange.com/questions/25945/how-to-read-and-write-eeprom-in-esp8266
  //writeEEPROM(ssid, 32); 
  //writeEEPROM(password, 64); 

  Serial.println();
  Serial.println(readEEPROM(32, 64));
  Serial.println(readEEPROM(64, 96));

  if (readEEPROM(0,32) == NULL ){
    Serial.println("Setting AP....");
    server.on("/", handle_clientresponse);
  }else{
    Serial.println("Starting wifi....");
    setupWiFi(readEEPROM(32,64).c_str(), readEEPROM(64,96).c_str());
  }
  
  */
  setupWiFi();

  Serial.println(WiFi.localIP());
  //https://github.com/G6EJD/ESP8266-WebServer-Getting-Client-Data
  server.on("/", HTTP_GET, [](){
    handleFileRead("/");
  });
  
  //server.on("/result", handle_showclientresponse);
  
  //Handle when user requests a file that does not exist
  server.onNotFound([](){
    //if(!handleFileRead(server.uri()))
  server.send(404, "text/plain", "FileNotFound");
  });
  
  server.begin();
  Serial.println("HTTP server started");
  SPIFFS.begin();

  //NOT SURE MDNS IS NEEDED COMMENTING OUT FOR NOW
  /*MDNS.begin(host);
  Serial.print("Open http://");
  Serial.print(host);
  Serial.println(".local/edit to see the file browser");
  
  server.on("/", HTTP_GET, [](){
    handleFileRead("/");
  });*/

  // ***************************  WIP END ***********************
  
  Serial.println("Starting BLE work!");
  adc1_config_width(ADC_WIDTH_BIT_11);
  
  adc1_config_channel_atten((adc1_channel_t)36, ADC_ATTEN_DB_11);
  adc1_config_channel_atten((adc1_channel_t)37, ADC_ATTEN_DB_11);
  adc1_config_channel_atten((adc1_channel_t)38, ADC_ATTEN_DB_11);
  adc1_config_channel_atten((adc1_channel_t)39, ADC_ATTEN_DB_11);

  pinMode(CONNECT_LED, OUTPUT);
  pinMode(LED_1, OUTPUT);                         // visual feedback for init and calibration
  pinMode(LED_2, OUTPUT);                         // visual feedback for init and calibration
  pinMode(PUSH_BUTTON_CALI_MODE, INPUT_PULLUP);          // Connect Pin 11 to GND to put LipSync in calibration mode
  
//NOTE: Interrupt architecture on the ESP maybe different.  Strategy is to get mouse movement working then working the click events
  /*pinMode(12, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(12), click, CHANGE);
  pinMode(13, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(13), click, CHANGE);
  pinMode(32, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(32), click, CHANGE);*/

  xTaskCreate(taskServer, "server", 20000, NULL, 5, NULL);

  blink(4, 250, 3);
}

void loop() {
  //Uncomment to work on web page responses
  //server.handleClient();
  if(connected){
      digitalWrite(CONNECT_LED, HIGH); //When the connect event comes this LED goes high(ON)
      if (digitalRead(PUSH_BUTTON_CALI_MODE) == LOW) {
        Joystick_Calibration();
      } else {
  
      
      uint16_t xh = analogRead(X_DIR_HIGH);  //A0 on the Aarduino Micro
      uint16_t xl = analogRead(X_DIR_LOW);  //A1 on the Aarduino Micro
      uint16_t yh = analogRead(Y_DIR_HIGH);  //A2 on the Aarduino Micro
      uint16_t yl = analogRead(Y_DIR_LOW);  //D10/A10 on the Aarduino Micro
                                                                                                                                    //These notes are from the original developer.......
      xh_yh = sqrt(sq(((xh - x_right) > 0) ? (float)(xh - x_right) : 0.0) + sq(((yh - y_up) > 0) ? (float)(yh - y_up) : 0.0));     // sq() function raises input to power of 2, returning the same data type int->int ...
      xh_yl = sqrt(sq(((xh - x_right) > 0) ? (float)(xh - x_right) : 0.0) + sq(((yl - y_down) > 0) ? (float)(yl - y_down) : 0.0));   // the sqrt() function raises input to power 1/2, returning a float type
      xl_yh = sqrt(sq(((xl - x_left) > 0) ? (float)(xl - x_left) : 0.0) + sq(((yh - y_up) > 0) ? (float)(yh - y_up) : 0.0));      // These are the vector magnitudes of each quadrant 1-4. Since the FSRs all register
      xl_yl = sqrt(sq(((xl - x_left) > 0) ? (float)(xl - x_left) : 0.0) + sq(((yl - y_down) > 0) ? (float)(yl - y_down) : 0.0));    // a larger digital value with a positive application force, a large negative difference
      
      if ((xh_yh > xh_yh_radius) || (xh_yl > xh_yl_radius) || (xl_yl > xl_yl_radius) || (xl_yh > xl_yh_radius)) {

      poll_counter++;

      delay(20);    // originally 15 ms

      if (poll_counter >= 3) {

      //NOTE: Testing shows that only quad 1 and quad 3 are being executed.....need help understanding why
      //but it maybe due to the fact that only one FSR can be pressed under the current hardware setup and
      //the calibration routine needs to be executed.
      
        if ((xh_yh >= xh_yl) && (xh_yh >= xl_yh) && (xh_yh >= xl_yl)) {
          Serial.println("quad1");
          uint8_t val[] = {buttons, xh>>5, yh>>5, 0};
          input->setValue(val, 4);
          input->notify();
          Serial.print("FSR (x) = ");
          Serial.println(xh);
          Serial.print("FSR (y) = ");
           Serial.println(xl);
          vTaskDelay(3);
          delay(cursor_delay);
          poll_counter = 0;
        } else if ((xh_yl > xh_yh) && (xh_yl > xl_yl) && (xh_yl > xl_yh)) {
          Serial.println("quad4");
          uint8_t val[] = {buttons, xh>>5, yl>>5, 0};
          input->setValue(val, 4);
          input->notify();
          Serial.print("FSR (x) = ");
          Serial.println(xh);
          Serial.print("FSR (y) = ");
           Serial.println(xl);
          vTaskDelay(3);
          delay(cursor_delay);
          poll_counter = 0;
        } else if ((xl_yl >= xh_yh) && (xl_yl >= xh_yl) && (xl_yl >= xl_yh)) {
          Serial.println("quad3");
          uint8_t val[] = {buttons, xl>>5, yl>>5, 0};
          input->setValue(val, 4);
          input->notify();
          Serial.print("FSR (x) = ");
          Serial.println(xh);
          Serial.print("FSR (y) = ");
           Serial.println(xl);
          vTaskDelay(3);
          delay(cursor_delay);
          poll_counter = 0;
        } else if ((xl_yh > xh_yh) && (xl_yh >= xh_yl) && (xl_yh >= xl_yl)) {
          Serial.println("quad2");
          uint8_t val[] = {buttons, xl>>5, yh>>5, 0};
          input->setValue(val, 4);
          input->notify();
          Serial.print("FSR (x) = ");
          Serial.println(xh);
          Serial.print("FSR (y) = ");
           Serial.println(xl);
          vTaskDelay(3);
          delay(cursor_delay);
          poll_counter = 0;
        }
        delay(50);
      }
     }
    }
  }
  else digitalWrite(CONNECT_LED, LOW);
}

   

//***LED BLINK FUNCTIONS***//

void blink(int num_Blinks, int delay_Blinks, int LED_number ) {
  if (num_Blinks < 0) num_Blinks *= -1;

  switch (LED_number) {
    case 1: {
        for (int i = 0; i < num_Blinks; i++) {
          digitalWrite(LED_1, HIGH);
          delay(delay_Blinks);
          digitalWrite(LED_1, LOW);
          delay(delay_Blinks);
        }
        break;
      }
    case 2: {
        for (int i = 0; i < num_Blinks; i++) {
          digitalWrite(LED_2, HIGH);
          delay(delay_Blinks);
          digitalWrite(LED_2, LOW);
          delay(delay_Blinks);
        }
        break;
      }
    case 3: {
        for (int i = 0; i < num_Blinks; i++) {
          digitalWrite(LED_1, HIGH);
          delay(delay_Blinks);
          digitalWrite(LED_1, LOW);
          delay(delay_Blinks);
          digitalWrite(LED_2, HIGH);
          delay(delay_Blinks);
          digitalWrite(LED_2, LOW);
          delay(delay_Blinks);
        }
        break;
      }
  }
}

//***JOYSTICK INITIALIZATION FUNCTION***//

void Joystick_Initialization(void) {
  xh = analogRead(X_DIR_HIGH);            // Initial neutral x-high value of joystick
  delay(10);

  xl = analogRead(X_DIR_LOW);             // Initial neutral x-low value of joystick
  delay(10);

  yh = analogRead(Y_DIR_HIGH);            // Initial neutral y-high value of joystick
  delay(10);

  yl = analogRead(Y_DIR_LOW);             // Initial neutral y-low value of joystick
  delay(10);

  x_right = xh;
  x_left = xl;
  y_up = yh;
  y_down = yl;

  EEPROM.get(6, yh_comp);
  delay(10);
  EEPROM.get(10, yl_comp);
  delay(10);
  EEPROM.get(14, xh_comp);
  delay(10);
  EEPROM.get(18, xl_comp);
  delay(10);
  EEPROM.get(22, xh_max);
  delay(10);
  EEPROM.get(24, xl_max);
  delay(10);
  EEPROM.get(26, yh_max);
  delay(10);
  EEPROM.get(28, yl_max);
  delay(10);

  constant_radius = 30.0;                       //40.0 works well for a constant radius

  xh_yh_radius = constant_radius;
  xh_yl_radius = constant_radius;
  xl_yl_radius = constant_radius;
  xl_yh_radius = constant_radius;

}

void Joystick_Calibration(void) {

  Serial.println("Prepare for joystick calibration!");
  Serial.println(" ");
  blink(4, 300, 3);

  Serial.println("Move mouthpiece to the furthest vertical up position and hold it there until the LED turns SOLID RED, then release the mouthpiece.");
  blink(6, 500, 1);
  yh_max = analogRead(Y_DIR_HIGH);
  blink(1, 1000, 2);
  Serial.println(yh_max);

  Serial.println("Move mouthpiece to the furthest horizontal right position and hold it there until the LED turns SOLID RED, then release the mouthpiece.");
  blink(6, 500, 1);
  xh_max = analogRead(X_DIR_HIGH);
  blink(1, 1000, 2);
  Serial.println(xh_max);

  Serial.println("Move mouthpiece to the furthest vertical down position and hold it there until the LED turns SOLID RED, then release the mouthpiece.");
  blink(6, 500, 1);
  yl_max = analogRead(Y_DIR_LOW);
  blink(1, 1000, 2);
  Serial.println(yl_max);

  Serial.println("Move mouthpiece to the furthest horizontal left position and hold it there until the LED turns SOLID RED, then release the mouthpiece.");
  blink(6, 500, 1);
  xl_max = analogRead(X_DIR_LOW);
  blink(1, 1000, 2);
  Serial.println(xl_max);

  int max1 = (xh_max > xl_max) ? xh_max : xl_max;
  int max2 = (yh_max > yl_max) ? yh_max : yl_max;
  float max_final = (max1 > max2) ? (float)max1 : (float)max2;

  //int delta_max_total = (yh_max - y_up) + (yl_max - y_down) + (xh_max - x_right) + (xl_max - x_left);

  Serial.print("max_final: ");
  Serial.println(max_final);
  Serial.println(" ");
  Serial.println("REMOVE THE JUMPER QUICKLY WHILE THE LEDS ARE FLASHING!!!!!...");
 
  //float avg_delta_max = ((float)(delta_max_total)) / 4;

  //Serial.print("avg_delta_max: ");
  //Serial.println(avg_delta_max);

  yh_comp = (max_final - y_up) / (yh_max - y_up);
  yl_comp = (max_final - y_down) / (yl_max - y_down);
  xh_comp = (max_final - x_right) / (xh_max - x_right);
  xl_comp = (max_final - x_left) / (xl_max - x_left);

  EEPROM.put(6, yh_comp);
  delay(10);
  EEPROM.put(10, yl_comp);
  delay(10);
  EEPROM.put(14, xh_comp);
  delay(10);
  EEPROM.put(18, xl_comp);
  delay(10);
  EEPROM.put(22, xh_max);
  delay(10);
  EEPROM.put(24, xl_max);
  delay(10);
  EEPROM.put(26, yh_max);
  delay(10);
  EEPROM.put(28, yl_max);
  delay(10);

  blink(5, 250, 3);

  Serial.println(" ");
  Serial.println("Joystick speed calibration procedure is complete....");
}

IRAM_ATTR void click(){
  buttons = digitalRead(12)<<0 | digitalRead(13)<<1 | digitalRead(32)<<2;  
}
