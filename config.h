// config: ////////////////////////////////////////////////////////////
// 

//#define OTA_HANDLER 


// Only *ONE* of the following modes can be enabled - pick one, comment out the other.
//#define MODE_AP         // Access point mode: terminal connects directly to the ESP
#define MODE_STA  // Station mode - both radar and terminal are stations, connected via another access point.

#define PROTOCOL_TCP


#define VERSION "1.10"

// For AP mode:
#ifdef MODE_AP
const char *ssid = "InsectRadar";  // You will connect your phone to this Access Point
const char *pw = "InsectRadar"; // and this is the password
#endif

#ifdef MODE_STA
const char *ssid = "GKW";  // You will connect your phone to this Access Point
const char *pw = "GKW2025!"; // and this is the password
#endif

IPAddress ip(192, 168, 4, 1); // Connect to this IP.  NOte - port is configured below
IPAddress gateway(192, 168, 4, 1);
IPAddress netmask(255, 255, 255, 0);


/* Use default Seeed Studio default Xiao ESP32S3 Serial0 device only - 
don't worry about seeting up COM1 and COM2, or manually assigning pins
(legacy code below commented out).
*/

#define BAUD_SERIAL 115200  // Baud rate of Serial port (USB)

/*************************  COM Port 0 *******************************/
#define UART_BAUD0 230400            // Baudrate UART0
#define SERIAL_PARAM0 SERIAL_8N1    // Data/Parity/Stop UART0
//#define SERIAL0_RXPIN 44            // receive Pin UART0
//#define SERIAL0_TXPIN 43             // transmit Pin UART0
#define SERIAL0_TCP_PORT 8880       // Wifi Port UART0
/*************************  COM Port 1 *******************************/
/*#define UART_BAUD1 19200            // Baudrate UART1
#define SERIAL_PARAM1 SERIAL_8N1    // Data/Parity/Stop UART1
#define SERIAL1_RXPIN 16            // receive Pin UART1
#define SERIAL1_TXPIN 17            // transmit Pin UART1
#define SERIAL1_TCP_PORT 8881       // Wifi Port UART1
*/
/*************************  COM Port 2 *******************************/
/*#define UART_BAUD2 19200            // Baudrate UART2
#define SERIAL_PARAM2 SERIAL_8N1    // Data/Parity/Stop UART2
#define SERIAL2_RXPIN 15            // receive Pin UART2
#define SERIAL2_TXPIN 4             // transmit Pin UART2
#define SERIAL2_TCP_PORT 8882       // Wifi Port UART2
*/

#define bufferSize 1024

//////////////////////////////////////////////////////////////////////////

