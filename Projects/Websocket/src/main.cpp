#include <Arduino.h>
#include "pin_config.h"
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <HardwareSerial.h>
#include <USBCDC.h>

const char *ssid = "Involveininnovation";
const char *password= "";

const int RED_PIN=1;
const int GREEN_PIN=2;
const int BLUE_PIN=3;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

int y=0;
int getY(){
  if(y<140){
    y=y+16;
  }else{
    y=16;
  }
  return y;
}

String getPinState(int output){
  if(digitalRead(output)){
    return "checked";
  }
  else{
    return "";
  }
}

String processor(const String &var)
{
  String status = "";
  
  if (var == "RED_STATE")
  {
    return getPinState(RED_PIN);
  }
  else if (var == "GREEN_STATE")
  {
    return getPinState(GREEN_PIN);
  }
  else if (var == "BLUE_STATE")
  {
    return getPinState(BLUE_PIN);
  }
  return status;
}


void turnLightOnOff(const char *command){
  if(strcmp(command,"RED_ON")==0){
    digitalWrite(RED_PIN,HIGH);
  } else if(strcmp(command,"RED_OFF")==0){
    digitalWrite(RED_PIN,LOW);
  }
  else if(strcmp(command,"GREEN_ON")==0){
    digitalWrite(GREEN_PIN,HIGH);
  } else if(strcmp(command,"GREEN_OFF")==0){
    digitalWrite(GREEN_PIN,LOW);
  }
  else if(strcmp(command,"BLUE_ON")==0){
    digitalWrite(BLUE_PIN,HIGH);
  } else if(strcmp(command,"BLUE_OFF")==0){
    digitalWrite(BLUE_PIN,LOW);
  }
}

void onWsEvent(AsyncWebSocket *server,AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len){
  switch(type){
    case WS_EVT_CONNECT:{
      
    }
    case WS_EVT_DISCONNECT:{
      
    }
    case WS_EVT_DATA:
    {
      AwsFrameInfo *info =(AwsFrameInfo *)arg;
      if(info->final && info->index==0 && info->len ==len){
        if(info->opcode==WS_TEXT){
          data[len]=0;
          char *command = (char *)data;
          turnLightOnOff(command);
        }
      }
    }
      case WS_EVT_PONG:
      {
        
      }

      case WS_EVT_ERROR:
      {
        
      }
  }
}
void notFound(AsyncWebServerRequest *request){
  request-> send(404,"text/plain","Not found");
}
void setup() {
  // put your setup code here, to run once:
  
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
 // Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  int result = WiFi.waitForConnectResult();
  if(result == WL_CONNECTED){
    
  }else{
    
    
    WiFi.disconnect();
    int n = WiFi.scanNetworks();
     if (n == 0) {
        
    } else {
        
        for (int i = 0; i < n; ++i) {
            // Print SSID and RSSI for each network found
            
            //Serial.println((WiFi.encryptionType(i) == WIFI_AUTH_OPEN)?" ":"*");
            delay(10000);
        }
    }
  }
 
  if (!SPIFFS.begin(true))
  {
    
    return;
  }

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            {
              

              request->send(SPIFFS, "/index.html", "text/html",false,processor);
            });

  server.on("/css/style.css", HTTP_GET, [](AsyncWebServerRequest *request)
            { 
             
              request->send(SPIFFS, "/css/style.css", "text/css"); 
            });

  server.on("/script/socket.js", HTTP_GET, [](AsyncWebServerRequest *request)
            { 
             
              request->send(SPIFFS, "/script/socket.js", "text/javascript"); 
            });

  server.onNotFound(notFound);
  server.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
}