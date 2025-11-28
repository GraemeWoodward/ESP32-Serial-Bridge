// ESP32 WiFi <-> 3x UART Bridge
// by AlphaLima
// www.LK8000.com

// Disclaimer: Don't use  for life support systems
// or any other situations where system failure may affect
// user or environmental safety.

#include "config.h"
#include <esp_wifi.h>
#include <WiFi.h>




#ifdef BLUETOOTH
#include <BluetoothSerial.h>
BluetoothSerial SerialBT; 
#endif

#ifdef OTA_HANDLER  
#include <ArduinoOTA.h> 

#endif // OTA_HANDLER

//HardwareSerial Serial_one(1);
//HardwareSerial Serial_two(2);
//HardwareSerial* COM[NUM_COM] = {&Serial, &Serial_one , &Serial_two};

#define MAX_NMEA_CLIENTS 1
#ifdef PROTOCOL_TCP
#include <WiFiClient.h>

WiFiServer server0(SERIAL0_TCP_PORT);
//WiFiServer server_1(SERIAL1_TCP_PORT);
//WiFiServer server_2(SERIAL2_TCP_PORT);
//WiFiServer *server[NUM_COM]={&server_0};  // just the one COM port being used in this configuration.

WiFiClient TCPClient;
#endif


uint8_t buf1[bufferSize];
//uint16_t i1[NUM_COM]={0,0,0};
uint16_t i1 = 0;

uint8_t buf2[bufferSize];
//uint16_t i2[NUM_COM]={0,0,0};
uint16_t i2 = 0;


uint8_t BTbuf[bufferSize];
uint16_t iBT =0;


void setup() {
  Serial.begin(BAUD_SERIAL);
  Serial.println("Initialising EPS32");
  
  delay(500);

  Serial.println("\nWiFi serial bridge V1.00");
  Serial.println("Starting UART0");
  Serial0.begin(UART_BAUD0);

  //COM[0]->begin(UART_BAUD0, SERIAL_PARAM0, SERIAL0_RXPIN, SERIAL0_TXPIN);
  //COM[1]->begin(UART_BAUD1, SERIAL_PARAM1, SERIAL1_RXPIN, SERIAL1_TXPIN);
  //COM[2]->begin(UART_BAUD2, SERIAL_PARAM2, SERIAL2_RXPIN, SERIAL2_TXPIN);
  
  #ifdef MODE_AP 

  Serial.print("Open ESP Access Point mode: ");
  Serial.println(ssid);
  Serial.println("Client can connect directly to ESP, no router required");

  //AP mode (client connects directly to ESP) (no router)
  WiFi.mode(WIFI_AP);
   
  //WiFi.softAP(ssid, pw); // configure ssid and password for softAP
  //delay(2000); // VERY IMPORTANT
  //WiFi.softAPConfig(ip, ip, netmask); // configure ip address for softAP

  Serial.print("Setting soft-AP configuration ... ");
  Serial.println(WiFi.softAPConfig(ip, gateway, netmask) ? "Ready" : "Failed!");

  Serial.print("Setting soft-AP ... ");
  Serial.println(WiFi.softAP(ssid, pw) ? "Ready" : "Failed!");

  Serial.print("Soft-AP IP address = ");
  Serial.println(WiFi.softAPIP());

  #endif


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
  #endif

#ifdef BLUETOOTH
    Serial.println("Open Bluetooth Server");  
  SerialBT.begin(ssid); //Bluetooth device name
#endif

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

  /*
  COM[1]->println("Starting TCP Server 2");
  if(debug) COM[DEBUG_COM]->println("Starting TCP Server 2");  
  server[1]->begin(); // start TCP server 
  server[1]->setNoDelay(true);
  COM[2]->println("Starting TCP Server 3");
  if(debug) COM[DEBUG_COM]->println("Starting TCP Server 3");  
  server[2]->begin(); // start TCP server   
  server[2]->setNoDelay(true);
  */

  #endif

 // esp_err_t esp_wifi_set_max_tx_power(50);  //lower WiFi Power
}

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
  
#ifdef BLUETOOTH
  // receive from Bluetooth:
  if(SerialBT.hasClient()) 
  {
    while(SerialBT.available())
    {
      BTbuf[iBT] = SerialBT.read(); // read char from client (LK8000 app)
      if(iBT <bufferSize-1) iBT++;
    }          
    for(int num= 0; num < NUM_COM ; num++)
      Serial0->write(BTbuf,iBT); // now send to UART(num):          
    iBT = 0;
  }  
#endif  

#ifdef PROTOCOL_TCP

if (server0.hasClient()) {
  if(TCPClient)   // stop any existing client
    TCPClient.stop();

  TCPClient = server0.available();
  TCPClient.flush();
  Serial.print("New client for COM0 "); 
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
//      Serial.print("<");
      Serial.write(buf1,i1);
//      Serial.print(",");
//      Serial.print(i1);
 //     Serial.print(">");

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
//  Serial.print("<");
 Serial.write(buf2,i2);
 // Serial.print(">");

  // now send to WiFi:
 
      if(TCPClient)                     
        TCPClient.write(buf2, i2);

#ifdef BLUETOOTH        
  // now send to Bluetooth:
  if(SerialBT.hasClient())      
    SerialBT.write(buf2, i2);               
#endif  
  i2 = 0;
}

}

