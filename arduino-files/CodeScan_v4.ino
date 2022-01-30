/*
 MG7101 QR Code Scanning Kiosk Prototype V4
 Sync time of DS3231 RTC with Network Time Protocol (NTP) time server.
 Scan a 1D barcode, 2D barcode or QR code and upload date/time and scanned value to AWS IoT Core server.
 created 4 Sep 2021
 by Stephen Julian
 modified 11 Nov 2021
 by Stephen Julian
 Includes some code snippets adapted from the example sketches bundled with include libraries.
 These libraries are included below and described in the project Final Report document.
 This work is published under the MIT licence.
*/

int SenseFromPIRPin = 4;
int TriggerToQRScanPin = 20;
int StatusFromQRScanPin = 21;
int RxFromQRScanPin = 3; //mySerial1 (QR Code Scanner)
int TxToQRScanPin = 2; //mySerial1 (QR Code Scanner)
int RxFromLcdPin = 1; //mySerial2 (LCD Display)
int TxToLcdPin = 0; //mySerial2 (LCD Display)
int LEDredPin = 7;
int LEDbluePin = 6;
int LEDgreenPin = 5;
bool ledIsFlashing = false;
bool ledIsOn = false;
int val = 0;
bool result = false;
bool modeScan = false;
bool debugMode = false;
bool firstNTPSyncDone = false;

#include <Arduino.h>
#include <wiring_private.h>
#include "SparkFun_DE2120_Arduino_Library.h" //Click here to get the library: http://librarymanager/All#SparkFun_DE2120
#include <stdio.h> // used for basic concatenating of strings in C
#include <string.h> // used for basic concatenating of strings in C
#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <RTClib.h>
#include <ArduinoBearSSL.h>
#include <ArduinoECCX08.h>
#include <ArduinoMqttClient.h>
#include "arduino_secrets.h"

// UARTs with SERCOM configs compatible with Nano 33 IoT. See docs:
// https://github.com/arduino/ArduinoCore-samd/blob/master/cores/arduino/SERCOM.h
// https://github.com/arduino/ArduinoCore-samd/blob/master/variants/nano_33_iot/variant.h
// https://github.com/arduino/ArduinoCore-samd/blob/master/variants/nano_33_iot/variant.cpp
//Uart mySerial1(&sercom0, 5, 6, SERCOM_RX_PAD_1, UART_TX_PAD_0); // QR Code Reader
//Uart mySerial2(&sercom1, RxFromLcdPin, TxToLcdPin, SERCOM_RX_PAD_1, UART_TX_PAD_2); // LCD Module serial interface.
// Spare serial interface. Create the new UART instance assigning it to pin 1, 0:
//Uart Serial1 (&sercom5, PIN_SERIAL1_RX, PIN_SERIAL1_TX, PAD_SERIAL1_RX, PAD_SERIAL1_TX);

// UARTs with SERCOM configs compatible with both MKR WiFi 1010 and MKR GSM 1400. See docs:
// https://github.com/arduino/ArduinoCore-samd/blob/master/cores/arduino/SERCOM.h
// https://github.com/arduino/ArduinoCore-samd/blob/master/variants/mkrwifi1010/variant.h
// https://github.com/arduino/ArduinoCore-samd/blob/master/variants/mkrwifi1010/variant.cpp
// TX pad is always either UART_TX_PAD_0 or UART_TX_PAD_2
Uart mySerial0 (&sercom0, 3, 2, SERCOM_RX_PAD_3, UART_TX_PAD_2); // RX=3, TX=2 // QR Code Reader
//Uart mySerial3 (&sercom3, 1, 0, SERCOM_RX_PAD_1, UART_TX_PAD_0); // RX=1, TX=0 // LCD Module
//Uart Serial1 (&sercom5, PIN_SERIAL1_RX, PIN_SERIAL1_TX, PAD_SERIAL1_RX, PAD_SERIAL1_TX); // RX=13, TX=14

DE2120 scanner;
#define BUFFER_LEN 40
char scanBuffer[BUFFER_LEN];

WiFiClient    wifiClient;            // Used for the TCP socket connection.
BearSSLClient sslClient(wifiClient); // Used for SSL/TLS connection, integrates with ECC508 crypto IC.
MqttClient    mqttClient(sslClient); // Used for messaging AWS IoT Core server.

RTC_DS3231 rtc;
int status = WL_IDLE_STATUS;
int keyIndex = 0; // your network key Index number (needed only for WEP. WEP is insecure, use WPA2 instead.)
unsigned int localPort = 2390;      // local port to listen for UDP packets
IPAddress timeServer(216, 239, 35, 4); // time2.google.com NTP server
const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets

// A UDP instance to let us send and receive packets over UDP
WiFiUDP Udp;

const int LEDFlashInterval = 1000; // 500 ms or half a second.
int millisAtLastLEDwrite = 0;

const int millisInterval = 30 * 1000; // 30 * 1000 = 30000 = 30 seconds
int millisAtLastSync = 0;

///////please enter your sensitive data in the Secret tab/arduino_secrets.h
const char ssid[] = SECRET_SSID;        // your network SSID (name)
const char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
const char broker[] = SECRET_BROKER;
const char* certificate = SECRET_CERTIFICATE;
const int deviceID = 1234;
int ledStatusCode = 0;
/*
 * Possible ledStatusCode values are:
0 = Waiting for the server = Red, Solid.
1 = Server error = Red, Flashing.
2 = Systems Nominal = Green, Solid.
3 = Data is in transit = Green, Flashing.
4 = Message Sent = Blue, Solid.
5 = New Message Received = Blue, Flashing.
6 = Reserved = Orange, Solid.
7 = Reserved = Orange, Flashing.
 * 
 */

// START Attach interrupt handler(s) to SERCOM(s)
// SERCOM IRQ for QR Code Scanner
void SERCOM0_Handler()
{
  mySerial0.IrqHandler();
}

/*
  // SERCOM IRQ for (additional serial device)
  void SERCOM5_Handler()
  {
  Serial1.IrqHandler();
  }

  // SERCOM IRQ for LCD Module
  void SERCOM3_Handler()
  {
  mySerial3.IrqHandler();
  }
*/
// END Attach interrupt handler(s) to SERCOM(s)

bool syncRTCwithNTPserver() {
  bool result = sendNTPpacket(timeServer); // send an NTP packet to a time server
  // wait to see if a reply is available
  delay(1000);
  if (Udp.parsePacket()) {
    result = setLedStatusCode(5);
    Serial.println("packet received");
    // We've received a packet, read the data from it
    Udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer
    //the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, extract the two words:
    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    Serial.print("Seconds since Jan 1 1900 = ");
    Serial.println(secsSince1900);
    // now convert NTP time into everyday time:
    Serial.print("Unix time = ");
    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;
    // subtract seventy years:
    unsigned long epoch = secsSince1900 - seventyYears;
    time_t t = epoch;
    rtc.adjust(DateTime(year(t), month(t), day(t), hour(t), minute(t), second(t)));
    firstNTPSyncDone = true;
    millisAtLastSync = millis();
    // print Unix time:
    Serial.println(epoch);
    Serial.print(year(t));
    Serial.print("/");
    Serial.print(month(t));
    Serial.print("/");
    Serial.println(day(t));
    // print the hour, minute and second:
    Serial.print("The UTC time is ");       // UTC is the time at Greenwich Meridian (GMT)
    Serial.print((epoch  % 86400L) / 3600); // print the hour (86400 equals secs per day)
    Serial.print(':');
    if (((epoch % 3600) / 60) < 10) {
      // In the first 10 minutes of each hour, we'll want a leading '0'
      Serial.print('0');
    }
    Serial.print((epoch  % 3600) / 60); // print the minute (3600 equals secs per minute)
    Serial.print(':');
    if ((epoch % 60) < 10) {
      // In the first 10 seconds of each minute, we'll want a leading '0'
      Serial.print('0');
    }
    Serial.println(epoch % 60); // print the second
    delay(3000);
    result = sendEpochMessageToAWS(epoch);
    //result = makeLEDGreen();
  }
  result = setLedStatusCode(2);
  return result;
}

bool sendEpochMessageToAWS(unsigned long myEpoch) {
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }
  delay(2000);
  if (!mqttClient.connected()) {
    // MQTT client is disconnected, connect
    connectMQTT();
  }
  time_t t = myEpoch;
  DateTime rightnow = rtc.now();
  // poll for new MQTT messages and send keep alives
  mqttClient.poll();
  Serial.println("Publishing message..."); 
  // send message, the Print interface can be used to set the message contents
  mqttClient.beginMessage("arduino/outgoing");
  mqttClient.print("{");

  mqttClient.print("\"utc_datetime\":\"");
  mqttClient.print(rightnow.year(), DEC);
  mqttClient.print('/');
  mqttClient.print(rightnow.month(), DEC);
  mqttClient.print('/');
  mqttClient.print(rightnow.day(), DEC);
  mqttClient.print(" ");
  mqttClient.print(rightnow.hour(), DEC);
  mqttClient.print(':');
  mqttClient.print(rightnow.minute(), DEC);
  mqttClient.print(':');
  mqttClient.print(rightnow.second(), DEC);

  mqttClient.print(",");
  mqttClient.print("\"deviceID\":");
  mqttClient.print(deviceID);
  mqttClient.print(",");
  mqttClient.print("\"event\":");
  mqttClient.print("\"NTP server synchronised\",");
  mqttClient.print("\"epoch\":");
  mqttClient.print(myEpoch);
  mqttClient.print("}");
  mqttClient.endMessage();
  Serial.print("NTP Server synchronised. ");
  Serial.println("Message published!");
  Serial.println(" ");
  return setLedStatusCode(4);
}

bool sendScanMessageToAWS(char* myCode) {
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }
  if (!mqttClient.connected()) {
    // MQTT client is disconnected, connect
    connectMQTT();
  }
  // poll for new MQTT messages and send keep alives
  mqttClient.poll();
  
  Serial.println("Publishing message..."); 
  // send message, the Print interface can be used to set the message contents
  mqttClient.beginMessage("arduino/outgoing");
  mqttClient.print("{");
  mqttClient.print("\"utc_datetime\":\"");
  DateTime rightnow = rtc.now();
  
  mqttClient.print(rightnow.year(), DEC);
  mqttClient.print('/');
  mqttClient.print(rightnow.month(), DEC);
  mqttClient.print('/');
  mqttClient.print(rightnow.day(), DEC);
  mqttClient.print(" ");
  mqttClient.print(rightnow.hour(), DEC);
  mqttClient.print(':');
  mqttClient.print(rightnow.minute(), DEC);
  mqttClient.print(':');
  mqttClient.print(rightnow.second(), DEC);
  
  mqttClient.print("\",");
  mqttClient.print("\"deviceID\":");
  mqttClient.print(deviceID);
  mqttClient.print(",");
  mqttClient.print("\"Event\":");
  mqttClient.print("\"Code scanned.\",");
  mqttClient.print("\"Code\":\"");
  int imax = strlen(myCode);
  for (int i=0; i<imax; i++) {
    mqttClient.print(myCode[i]);
  }
  //mqttClient.print(myCode);
  mqttClient.print("\"");
  mqttClient.print("}");
  mqttClient.endMessage();
  //Serial.println(myEpoch);
  Serial.print("utc_datetime: ");
  Serial.print(rightnow.year());
  Serial.print(':');
  Serial.print(rightnow.month());
  Serial.print(':');
  Serial.print(rightnow.day());
  Serial.print(" ");
  Serial.print(rightnow.hour());
  Serial.print(':');
  Serial.print(rightnow.minute());
  Serial.print(':');
  Serial.print(rightnow.second());
  
  Serial.print(" Event: ");
  Serial.print("Code scanned. ");
  Serial.print("Code:");
  imax = strlen(myCode);
  for (int i=0; i<imax; i++) {
    Serial.print(myCode[i]);
  }
  Serial.print(". Message published!");
  Serial.println(" ");
  return setLedStatusCode(4);
}
  
// send an NTP request to the time server at the given address
unsigned long sendNTPpacket(IPAddress& address) {

  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
  return setLedStatusCode(4);
}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

unsigned long getTime() {
  // get the current time from the WiFi module  
  return WiFi.getTime();
}

void connectWiFi() {
  Serial.print("Attempting to connect to SSID: ");
  Serial.print(ssid);
  Serial.print(" ");

  while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
    // failed, retry
    Serial.print(".");
    delay(5000);
  }
  Serial.println();
  Serial.println("You're connected to the network");
}

void connectMQTT() {
  Serial.print("Attempting to MQTT broker: ");
  Serial.print(broker);
  Serial.println(" ");
  while (!mqttClient.connect(broker, 8883)) {
    // failed, retry
    Serial.print(".");
    delay(1000);
  }
  Serial.println();
  Serial.println("You're connected to the MQTT broker");
  // subscribe to a topic
  mqttClient.subscribe("arduino/incoming");
}

void onMessageReceived(int messageSize) {
  // we received a message, print out the topic and contents
  Serial.print("Received a message with topic '");
  Serial.print(mqttClient.messageTopic());
  Serial.print("', length ");
  Serial.print(messageSize);
  Serial.println(" bytes:");
  // use the Stream interface to print the contents
  while (mqttClient.available()) {
    Serial.print((char)mqttClient.read());
  }
  Serial.println();
  Serial.println();
}

bool setLedStatusCode(int statusCode) {
  ledStatusCode = statusCode;
  return true;
}

bool makeLEDOff() {
  analogWrite(LEDredPin, 0);
  analogWrite(LEDgreenPin, 0);
  analogWrite(LEDbluePin, 0);
  ledIsOn = false;
  delay(50);
  return true;
}

bool makeLEDOn() {
  result = false;
  switch(ledStatusCode) { 
    case 0: 
        //statements 
        ledIsFlashing = false;
        result = makeLEDRed();
        break; 
    case 1: 
        //statements
        ledIsFlashing = true;
        result = makeLEDRed();
        break; 
    case 2: 
        //statements
        ledIsFlashing = false;
        result = makeLEDGreen();
        break; 
    case 3: 
        //statements
        ledIsFlashing = true;
        result = makeLEDGreen();
        break; 
    case 4: 
        //statements
        ledIsFlashing = false;
        result = makeLEDBlue();
        break; 
    case 5: 
        //statements
        ledIsFlashing = true;
        result = makeLEDBlue();
        break; 
    case 6: 
        ledIsFlashing = false;
        result = makeLEDOrange();
        //statements
        break; 
    case 7: 
        //statements
        ledIsFlashing = true;
        result = makeLEDOrange();
        break; 
    default:
        ledIsFlashing = false;
        result = makeLEDRed();
  }
  ledIsOn = true;
  return result;
}

bool makeLEDGreen() {
  analogWrite(LEDredPin, 0);
  analogWrite(LEDgreenPin, 255);
  analogWrite(LEDbluePin, 0);
  delay(50);
  return true;
}

bool makeLEDRed() {
  analogWrite(LEDredPin, 255);
  analogWrite(LEDgreenPin, 0);
  analogWrite(LEDbluePin, 0);
  delay(50);
  return true;
}

bool makeLEDBlue() {
  analogWrite(LEDredPin, 0);
  analogWrite(LEDgreenPin, 0);
  analogWrite(LEDbluePin, 255);
  delay(50);
  return true;
}

bool makeLEDOrange() {
  analogWrite(LEDredPin, 128);
  analogWrite(LEDgreenPin, 128);
  analogWrite(LEDbluePin, 0);
  delay(50);
  return true;
}

void setup() {
  pinPeripheral(2, PIO_SERCOM); // QR Code Reader
  pinPeripheral(3, PIO_SERCOM); // QR Code Reader
  delay(20);
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  delay(10);
  Serial.println("Starting setup...");
  pinMode(TriggerToQRScanPin, OUTPUT);
  digitalWrite(TriggerToQRScanPin, HIGH);
  delay(300);
  pinMode(LEDredPin, OUTPUT);
  pinMode(LEDbluePin, OUTPUT);
  pinMode(LEDgreenPin, OUTPUT); 
  delay(1000);
  result = makeLEDOff();
  result = setLedStatusCode(0);
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    abort();
  }
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, let's set the time!");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }

  // When time needs to be re-set on a previously configured device, the
  // following line sets the RTC to the date & time this sketch was compiled
  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  // This line sets the RTC with an explicit date & time, for example to set
  // January 21, 2014 at 3am you would call:
  // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  
  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }
  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }
  delay(10);
  pinPeripheral(2, PIO_SERCOM); // QR Code Reader
  pinPeripheral(3, PIO_SERCOM); // QR Code Reader
  delay(20);
  pinMode(TriggerToQRScanPin, OUTPUT);
  digitalWrite(TriggerToQRScanPin, HIGH);
  delay(300);
  // initialize scanner serial communication at 9600 bits per second:
  mySerial0.begin(9600);
  if (scanner.begin(mySerial0) == false) {
    
    // MKR WiFi 1010 SERCOM0 pins:
    pinPeripheral(2, PIO_SERCOM); // QR Code Reader
    pinPeripheral(3, PIO_SERCOM); // QR Code Reader
    // MKR WiFi 1010 SERCOM3 pins:
    //pinPeripheral(0, PIO_SERCOM); // LCD Module
    //pinPeripheral(1, PIO_SERCOM); // LCD Module
    
    // Nano 33 IoT SERCOM0 pins:
    //pinPeripheral(5, PIO_SERCOM_ALT);
    //pinPeripheral(6, PIO_SERCOM_ALT);
    // Nano 33 IoT SERCOM1 pins:
    //pinPeripheral(13, PIO_SERCOM);
    //pinPeripheral(8, PIO_SERCOM);
    Serial.println("Scanner did not respond. Please check!");
  }
  digitalWrite(TriggerToQRScanPin, HIGH);
  delay(200);
  Serial.print(scanner.startScan());
  delay(1);
   Serial.print(scanner.changeBaudRate(9600));
  delay(1);
   Serial.print(scanner.enableContinuousRead(3));
  delay(1);
   Serial.println(scanner.lightOff());
  delay(1);
   Serial.print(scanner.reticleOn());
  delay(1);
   Serial.print(scanner.startScan());
  delay(1);
   Serial.println(scanner.startScan());
  delay(200);
  //digitalWrite(TriggerToQRScanPin, HIGH);
  result = makeLEDGreen();
  delay(300);
  connectWiFi();
  Serial.println("Connected to wifi");
  printWifiStatus();
  Serial.println("\nStarting connection to server...");
  Udp.begin(localPort);
  
  // Check for presence of ECCX08 crypto IC
  if (!ECCX08.begin()) {
    Serial.println("No ECCX08 present!");
    while (1);
  }

  // Set a callback to get the current time
  // used to validate the servers certificate
  ArduinoBearSSL.onGetTime(getTime);

  // Set the ECCX08 slot to use for the private key
  // and the accompanying public certificate for it
  sslClient.setEccSlot(0, certificate);

  // Optional, set the client id used for MQTT,
  // each device that is connected to the broker
  // must have a unique client id. The MQTTClient will generate
  // a client id for you based on the millis() value if not set
  //
  // mqttClient.setId("clientId");

  // Set the message callback, this function is
  // called when the MQTTClient receives a message
  mqttClient.onMessage(onMessageReceived);
  result = setLedStatusCode(2);
  Serial.println("Setup finished.");
  delay(3000);
}

void loop() {
  int millisSinceLastLEDwrite = millis() - millisAtLastLEDwrite;
  if(!firstNTPSyncDone) {
    result = syncRTCwithNTPserver(); // trigger RTC time sync with the NTP server
    delay(250);
  } else if(millisSinceLastLEDwrite > LEDFlashInterval) { // check if we are due for another RTC time sync with the NTP server
    if(ledIsOn) {
      result = makeLEDOff();
    } else {
      result = makeLEDOn();
    }
    if(millisSinceLastLEDwrite > (LEDFlashInterval*2)) {
      millisAtLastLEDwrite = millis(); // store marker for when we last completed an LED ON/OFF Flash cycle.
    }
    delay(250);
  } else {
    int millisSinceLastSync = millis() - millisAtLastSync; // find elapsed time since last RTC time sync with the NTP server
    if(millisSinceLastSync > millisInterval) { // check if we are due for another RTC time sync with the NTP server
      result = syncRTCwithNTPserver(); // trigger RTC time sync with the NTP server
      millisAtLastSync = millis(); // store marker for when we last synced the RTC time with the NTP server.
    }
    delay(250);
  }
  digitalWrite(TriggerToQRScanPin, HIGH);
  delay(200);
  Serial.println(scanner.startScan());
  delay(1);
  Serial.println(scanner.changeBaudRate(9600));
  delay(1);
  Serial.println(scanner.enableContinuousRead(3));
  delay(1);
  Serial.println(scanner.lightOff());
  delay(1);
  Serial.println(scanner.reticleOn());
  delay(1);
  Serial.println(scanner.startScan());
  delay(1);
  Serial.println(scanner.startScan());

  digitalWrite(TriggerToQRScanPin, HIGH);
  delay(300);
  int maxCount = (int)millis() + 20000;
  while ((int)millis() < maxCount)
  {
    if (scanner.readBarcode(scanBuffer, BUFFER_LEN) > 0)
    {
      Serial.print("Code scanned: ");
      int imax = strlen(scanBuffer);
      for (int i=0; i<imax; i++) {
        Serial.print(scanBuffer[i]);
      }
      Serial.println();
      result = sendScanMessageToAWS(scanBuffer);
      break;
    }
    delay(200);
  }
  scanner.stopScan();
  digitalWrite(TriggerToQRScanPin, LOW);
  delay(1500); // delay in between reads for stability
}
