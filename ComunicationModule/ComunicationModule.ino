#include <ESP8266WiFi.h>
#include "HTTPSRedirect.h"
#include "DebugMacros.h"
#include <SPI.h>

#define LED0 2

char SPI_Data[4];
bool valor;
String cont;
char incomingByte;
float datas[2];


// for stack analytics
extern "C" {
#include <cont.h>
  extern cont_t g_cont;
}

// Fill ssid and password with your network credentials
const char* ssid = "espteste";//        WiFi Network Name
const char* password = "testeesp";//    WiFi Network Password

const char* host = "script.google.com";

// Replace with your own script id to make server side changes
const char *GScriptId = "AKfycbyM7y8gvn2zgjUOxDtzBbxSy2FWM2jBAsAnyeEVZtCsLFMpag_e";
                      //AKfycbxbYcgF58SVLZ3pcT7gER6ZzPSWEZZvZKJe1JplXIMTeiEZAGDn
const int httpsPort = 443;

// Execute in a linux terminal to get a Certificate number.
// echo | openssl s_client -connect script.google.com:443 |& openssl x509 -fingerprint -noout 
const char* fingerprint = "2F 0A F9 88 7B 30 B1 D3 91 19 7E 63 48 43 A1 46 15 B3 52 CB";
// SHA1: 07 EB DC 3D D6 DC BF C2 DD 7D B1 95 6F 38 A1 09 1A 66 8F 33

// Write to Google Spreadsheet
String url = String("/macros/s/") + GScriptId + "/exec?value=0";
// Fetch Google Calendar events for 1 week ahead
String url2 = String("/macros/s/") + GScriptId + "/exec?cal";
// Read from Google Spreadsheet
String url3 = String("/macros/s/") + GScriptId + "/exec?read";

String payload_base =  "{\"command\": \"appendRow\", \
                    \"sheet_name\": \"Sheet1\", \
                    \"values\": ";
String payload_base_1 =  "{\"command\": \"appendRow\", \
                    \"sheet_name\": \"Sheet1\", \
                    \"values\": ";
String payload_base_2 =  "{\"command\": \"appendRow\", \
                    \"sheet_name\": \"Sheet1\", \
                    \"values\": ";
String payload_base_3 =  "{\"command\": \"appendRow\", \
                    \"sheet_name\": \"Sheet1\", \
                    \"values\": ";                                        
String payload_base_SF =  "{\"command\": \"SpectrumFirst\", \
                    \"sheet_name\": \"Sheet1\", \
                    \"values\": ";                                        
String payload_base_SN =  "{\"command\": \"appendRow\", \
                    \"sheet_name\": \"Sheet1\", \
                    \"values\": ";                                        

String payload = "";

HTTPSRedirect* client = nullptr;

unsigned int free_heap_before = 0;
unsigned int free_stack_before = 0;

void setup() {
  //UART Setup
  Serial.begin(115200);
  Serial.flush();

  //SPI Setup
  pinMode(SS, OUTPUT);
  digitalWrite(SS, HIGH);
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV128);
  SPI.setDataMode(SPI_MODE0);
      
  //Pin Setup
  pinMode(LED0, OUTPUT);
  digitalWrite(LED0, !LOW);
  
  free_heap_before = ESP.getFreeHeap();
  free_stack_before = cont_get_free_stack(&g_cont);
  Serial.printf("Free heap before: %u\n", free_heap_before);
  Serial.printf("unmodified stack   = %4d\n", free_stack_before);
  
  Serial.println();
  Serial.print("Connecting to wifi: ");
  Serial.println(ssid);
  // flush() is needed to print the above (connecting...) message reliably, 
  // in case the wireless connection doesn't go through
  Serial.flush();

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    digitalWrite(LED0, !LED0);
    
  }
  
  digitalWrite(LED0, !LOW);
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // Use HTTPSRedirect class to create a new TLS connection
  client = new HTTPSRedirect(httpsPort);
  client->setPrintResponseBody(true);
  client->setContentTypeHeader("application/json");
  
  Serial.print("Connecting to ");
  Serial.println(host);

  // Try to connect for a maximum of 5 times
  bool flag = false;
  for (int i=0; i<5; i++){
    int retval = client->connect(host, httpsPort);
    if (retval == 1) {
       flag = true;
       break;
    }
    else
      Serial.println("Connection failed. Retrying...");
  }

  if (!flag){
    Serial.print("Could not connect to server: ");
    Serial.println(host);
    Serial.println("Exiting...");
    return;
  }
  
  if (client->verify(fingerprint, host)) {
    Serial.println("Certificate match.");
  } else {
    Serial.println("Certificate mis-match");
  }

  // Send memory data to Google Sheets
  //payload = payload_base + "\"" + free_heap_before + "," + free_stack_before + "\"}";
  //client->POST(url2, host, payload, false);
  //payload = payload_base + "\"" + ESP.getFreeHeap() + "," + cont_get_free_stack(&g_cont) + "\"}";
  //client->POST(url2, host, payload, false);
  
  // Note: setup() must finish within approx. 1s, or the watchdog timer
  // will reset the chip. Hence don't put too many requests in setup()
  // ref: https://github.com/esp8266/Arduino/issues/34
  
  Serial.println("\nGET: Write into cell 'A1'");
  Serial.println("=========================");

  // fetch spreadsheet data
  client->GET(url, host);

  // Send memory data to Google Sheets
  payload = payload_base + "\"" + ESP.getFreeHeap() + "," + cont_get_free_stack(&g_cont) + "\"}";
  client->POST(url2, host, payload, false);
  
  Serial.println("\nGET: Fetch Google Calendar Data:");
  Serial.println("================================");

  // fetch spreadsheet data
  client->GET(url2, host);

  // Send memory data to Google Sheets
  payload = payload_base + "\"" + ESP.getFreeHeap() + "," + cont_get_free_stack(&g_cont) + "\"}";
  client->POST(url2, host, payload, false);
  
  Serial.println("\nSeries of GET and POST requests");
  Serial.println("===============================");
  
  Serial.printf("Free heap: %u\n", ESP.getFreeHeap());
  Serial.printf("unmodified stack   = %4d\n", cont_get_free_stack(&g_cont));

  // delete HTTPSRedirect object
  delete client;
  client = nullptr;
}

void loop() {

  //Local variables
  static int error_count = 0;
  static int connect_count = 0;
  const unsigned int MAX_CONNECT = 20;
  static bool flag = false;
  int j=0, i=0, Spectrum[15];
  float data[18], Voltage=0.0, Current=0.0, FP=0.0;
  
  
  //Data request
 
  digitalWrite(SS, LOW);

  for(i=0;i<2;i++){
  SPI.transfer('V');
  SPI.transfer(0);
  for(j=0;j<4;j++){
     SPI_Data[j] = SPI.transfer(0);
     Serial.println(SPI_Data[j]);
  }
  memcpy((void*)&data[i], (void*)SPI_Data,4);
  Serial.println(data[i]);
  }
  
Voltage = data[0];
Current = data[1];
FP = data[2];

 for(i=0;i<15;i++){
  Spectrum[i] = data[i+2];
 }
  digitalWrite(SS, HIGH);


 //Serial.println(a);
  //memcpy((void*)&data, (void*)SPI_Data,4);
  //Serial.println(data);

  digitalWrite (LED0 , (valor?LOW:HIGH) );
   
   if (!flag){
    //free_heap_before = ESP.getFreeHeap();
    //free_stack_before = cont_get_free_stack(&g_cont);
    client = new HTTPSRedirect(httpsPort);
    flag = true;
    client->setPrintResponseBody(true);
    client->setContentTypeHeader("application/json");
  }

  if (client != nullptr){
    if (!client->connected()){
      client->connect(host, httpsPort);
      payload = payload_base_1 + "\"" + Voltage + "," + Current + "\"}";
      client->POST(url2, host, payload, false);
    }
  }
  else{
    DPRINTLN("Error creating client object!");
    error_count = 5;
  }
  
  if (connect_count > MAX_CONNECT){
    //error_count = 5;
    connect_count = 0;
    flag = false;
    delete client;
    return;
  }

  Serial.println("GET Data from cell 'A1':");
  if (client->GET(url3, host)){
    ++connect_count;
  }
  else{
    ++error_count;
    DPRINT("Error-count while connecting: ");
    DPRINTLN(error_count);
  }

  Serial.println("POST append memory data to spreadsheet:");
//  payload = payload_base + "\"" + Voltage + "," + Current+ "," + cont + "\"}";
  payload = payload_base + "\"" + Voltage + "," + Current + "," + FP + "," + Spectrum[0] + "," + Spectrum[1] + "," + Spectrum[2] + "," + Spectrum[3] + "," + Spectrum[4] + "," + Spectrum[5] + "," + Spectrum[6] + "," + Spectrum[7] + "," + Spectrum[8] + "," + Spectrum[9] + "," + Spectrum[10] + "\"}";
  if(client->POST(url2, host, payload)){
    ;
  }
  else{
    ++error_count;
    DPRINT("Error-count while connecting: ");
    DPRINTLN(error_count);
  }

  if (error_count > 3){
    Serial.println("Halting processor..."); 
    delete client;
    client = nullptr;
    //Serial.printf("Final free heap: %u\n", ESP.getFreeHeap());
    //Serial.printf("Final unmodified stack   = %4d\n", cont_get_free_stack(&g_cont));
    Serial.flush();
    ESP.deepSleep(0);
  }
  
  // In my testing on a ESP-01, a delay of less than 1500 resulted 
  // in a crash and reboot after about 50 loop runs.
  delay(4000);

}
