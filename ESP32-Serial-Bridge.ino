// UART to WiFi TCP-IP bridge for telnet connection to the ESP32 serial UART
// Designed for the Seeed Studio Xiao ESP32 devices (tested on ESP32S3).
// Uses the default Serial0 device (pins 43/44 on the Xiao ESP32S3/
//
// Code Based upon the project by by AlphaLima
// ESP32 WiFi <-> 3x UART Bridge www.LK8000.com
//
// Simplified to address just the one UART, the default Serial0
// Bluetooth disabled, as ESP32S3 only support BLE, for which the BluetoothSerial libaray doesn't work
// Echos all traffic out the default Serial line (USB connection)
//
// Note the OTA handler option has not been touched - not tested / don't know if it is appropriate to use.

#include "config.h"
#include <esp_wifi.h>
#include <WiFi.h>

#ifdef OTA_HANDLER  
#include <ArduinoOTA.h> 
#endif // OTA_HANDLER

#ifdef PROTOCOL_TCP
#include <WiFiClient.h>
WiFiServer server0(SERIAL0_TCP_PORT);
WiFiClient TCPClient;
#endif

uint8_t buf1[bufferSize];
uint16_t i1 = 0;

uint8_t buf2[bufferSize];
uint16_t i2 = 0;

void setup() {
  Serial.begin(BAUD_SERIAL);
  Serial.println("Initialising EPS32");
  
  delay(500);

  Serial.println("\nWiFi serial bridge V1.00");
  Serial.println("Starting UART0");
  Serial0.begin(UART_BAUD0);
  
#ifdef MODE_AP 
  //AP mode (client connects directly to ESP) (no router)

  Serial.print("Open ESP Access Point mode: ");
  Serial.println(ssid);
  Serial.println("Client can connect directly to ESP, no router required");

  WiFi.mode(WIFI_AP);
   
  Serial.print("Setting soft-AP configuration ... ");
  Serial.println(WiFi.softAPConfig(ip, gateway, netmask) ? "Ready" : "Failed!");

  Serial.print("Setting soft-AP ... ");
  Serial.println(WiFi.softAP(ssid, pw) ? "Ready" : "Failed!");

  Serial.print("Soft-AP IP address = ");
  Serial.println(WiFi.softAPIP());

#endif  // MODE_AP

#ifdef MODE_STA
    Serial.println("Open ESP Station mode");
  // STATION mode (ESP connects to router and gets an IP)
  // Assuming phone is also connected to that router
  // from RoboRemo you must connect to the IP of the ESP
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pw);

  Serial.print("try to Connect to Wireless network: ");
  Serial.println(ssid);
    
  while (WiFi.status() != WL_CONNECTED) {   
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
#endif // MODE_STA

#ifdef OTA_HANDLER  
  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
  // if DNSServer is started with "*" for domain name, it will reply with
  // provided IP to all DNS request

  ArduinoOTA.begin();
#endif // OTA_HANDLER    

#ifdef PROTOCOL_TCP
  
  Serial.println("Starting TCP Server0");  
  server0.begin(); // start TCP server 
  server0.setNoDelay(true);


#endif

 // esp_err_t esp_wifi_set_max_tx_power(50);  //lower WiFi Power
}

// keep track of number of stations connected, so that changes can be reported to the
// Serial terminal.
int numStationsOld = 0;
int numStationsNew = 0;

void loop() 
{  

#ifdef MODE_AP
numStationsNew = WiFi.softAPgetStationNum();
if( numStationsNew != numStationsOld ){
  Serial.print("Number of connected stations changed from ");
  Serial.print( numStationsOld );
  Serial.print( " to ");
  Serial.println( numStationsNew );
  numStationsOld = numStationsNew;
}
#endif

#ifdef OTA_HANDLER  
ArduinoOTA.handle();
#endif // OTA_HANDLER
  
#ifdef PROTOCOL_TCP
if (server0.hasClient()) {  // new clieant available.
  if(TCPClient)   // stop any existing client
    TCPClient.stop();

  TCPClient = server0.available();
  TCPClient.flush();
  Serial.print("New client for Serial0 "); 
}
#endif
               
if(TCPClient) 
{
  if( TCPClient.available()){  
    // if there is data avaialble from the TCP client, 
    // then for as long as there is still data, accumulate into a buffer and then send.
    while(TCPClient.available())
    {
      buf1[i1] = TCPClient.read(); // read char from client (LK8000 app)
      if(i1<bufferSize-1) i1++;
    } 
    // echo to Serial
    Serial.write(buf1,i1);
    Serial0.write(buf1, i1); // now send to UART(num):
    i1 = 0;
  }    
}

if(Serial0.available()){
  // if there is data avaialble from the Serial client, 
  // then for as long as there is still data, accumulate into a buffer and then send.
  while(Serial0.available())
  {     
    buf2[i2] = Serial0.read(); // read char from UART(num)
      if(i2<bufferSize-1) 
        i2++;
  }

  // echo to Serial
  Serial.write(buf2,i2);
  // Serial.print(">");
  // now send to WiFi:
  if(TCPClient)                     
    TCPClient.write(buf2, i2);

  i2 = 0;
}

}  // end of loop()

