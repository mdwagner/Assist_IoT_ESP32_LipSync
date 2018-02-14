This is a readme file for the LipSync_v2 code.

Pull or unzip the code to your Arduino sketch directory

Pull or unzip the ESP32 snippet code from chegewara's repo.  Chegewara has created a custom BLE library specifically for HID BLE devices.  The Arduino code is based on this.

https://github.com/chegewara/esp32-snippets/releases

For perspective below us the directory layout for the .ino code, data files, and the library:

User Directory.../Arduino/Lip_Sync_v2/Lip_Sync_v2.ino   <- Arduino Code

User Directory.../Arduino/Lip_Sync_v2/data/...(Web Server pages and JS)   <- Pages and JS Uploaded to SPIFFS location

User Directory.../Arduino/libraries/ESP32_BLE   <- Library from Chegewara
