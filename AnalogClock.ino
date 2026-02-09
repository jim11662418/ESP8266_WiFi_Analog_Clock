//--------------------------------------------------------------------------
// Analog Clock
//
// This sketch uses an ESP8266 on a WEMOS D1 Mini board to retrieve the time 
// from an NTP server and pulse the Lavet motor on an inexpensive analog quartz
// clock to keep the clock in sync with local time. The ESP8266 stores the position 
// of the clock's hour, minute and second hands in I2C Serial EERAM. NTP time 
// and analog clock time are display in the serial monitor once each second.
//
// on startup:
//    RGB LED flashes YELLOW while waiting to connect to WiFi
//    RGB LED flashes PURPLE while waiting to connect to the NTP server
//    RGB LED flashes BLUE while waiting for input from the setup web page
//
// after startup:
//    RGB LED flashes GREEN during normal operation
//    RGB LED flashes RED if unable to sync with NTP server
//    RGB LED flashes WHITE while the analog clock's hands are stopped waiting for actual time to catch up
//    RGB LED shows steady GREEN during OTA update
//    RGB LED shows steady RED in event of an OTA error
//
//--------------------------------------------------------------------------

#include <TimeLib.h>                                  // https://github.com/PaulStoffregen/Time
#include <ESP8266WiFi.h>
#include <NtpClientLib.h>                             // https://github.com/gmag11/NtpClient
#include <Ticker.h>
#include <EERAM.h>                                    // https://github.com/MajenkoLibraries/EERAM/
#include <ESPAsyncWebServer.h>                        // https://github.com/me-no-dev/ESPAsyncWebServer
#include <ESPTelnet.h>                                // https://github.com/LennartHennigs/ESPTelnet
#include <ArduinoOTA.h>

#define GRAPHIC                                       // display analog clock graphic on status page
#define SVG                                           // use Scalable Vector Graphics image for the analog clock face 
#include "html_code.h"                                // HTML code for clock status and setup pages

// WEMOS D1 Mini pins
#define D0 16                                         // can't use D0 to generate interrupts
#define D1  5
#define D2  4
#define D3  0                                         // 10K pull-up, boot fails if pulled LOW
#define D4  2                                         // 10K pull-up, boot fails if pulled LOW, BUILTIN_LED,
#define D5 14
#define D6 12
#define D7 13
#define D8 15                                         // 10K pull-down, boot fails if pulled HIGH

#define SDA D1                                        // output to SDA on the EERAM
#define SCL D2                                        // output to SCL on the EERAM
#define COIL1 D3                                      // output to clock's Lavet motor coil
#define COIL2 D7                                      // output to clock's Lavet motor coil
#define REDLED D5                                     // output to red part of the RGB LED
#define GREENLED D4                                   // output to green part of the RGB LED
#define BLUELED D8                                    // output to blue part of the RGB LED
#define SWITCHPIN D6                                  // input from push button switch

#define TITLE "ESP8266 WiFi Analog Clock Version 2.9" // added ArduinoOTA

#define DEBOUNCE 50                                   // 50 milliseconds to debounce the pushbutton switch
#define PULSETIME 30                                  // 30 millisecond pulse for the clock's Lavet motor

#define WIFISSID "********"
#define PASSWORD "********"    

#define HOUR     0x0000                               // address in EERAM for analogClkHour
#define MINUTE   HOUR+1                               // address in EERAM for analogClkMinute
#define SECOND   HOUR+2                               // address in EERAM for analogClkSecond
#define TIMEZONE HOUR+3                               // address in EERAM for analogClktimeZone
#define CHECK1   HOUR+4                               // address in EERAM for 1st check byte 0xAA
#define CHECK2   HOUR+5                               // address in EERAM for 2nd check byte 0x55

//#define NTPSERVERNAME "time.aws.com"
//#define NTPSERVERNAME "time.cloudflare.com"
//#define NTPSERVERNAME "time.apple.com"
//#define NTPSERVERNAME "north-america.pool.ntp.org"
//#define NTPSERVERNAME "time.nist.gov"
  #define NTPSERVERNAME "time.windows.com"
//#define NTPSERVERNAME "time.google.com"

#define LONGINTERVAL 600                              // re-sync with the NTP server every 10 minutes
#define SHORTINTERVAL 10                              // if the time has not been set, re-try the NTP server every 10 seconds

AsyncWebServer server(80);
ESPTelnet telnet;
EERAM eeRAM;
Ticker pulseTimer,clockTimer,ledTimer;
NTPSyncEvent_t ntpEvent;
String lastSyncTime = "";
String NTPtime, clockTime;
boolean syncEventTriggered = false;                   // true if an NTP time sync event has been triggered
boolean printTime = false;                            // if true, print NTP and clock's time
byte analogClktimeZone=5;                             // GMT -5 = EST
byte analogClkHour=0;
byte analogClkMinute=0;
byte analogClkSecond=0;
byte analogClkWeekday=0;
byte analogClkDay=0;
byte analogClkMonth=0;
byte analogClkYear=0;
time_t analogClkTime;
bool NTPerror = false;
unsigned long startMillis,stopMillis;

// RGB LED colors - https://www.w3schools.com/colors/colors_picker.asp
int RGBred[]    = {255,0,0};
int RGBblue[]   = {0,255,0};
int RGBgreen[]  = {0,0,255};
int RGBcyan[]   = {0,255,255};
int RGByellow[] = {255,255,0};
int RGBpurple[] = {255,0,255};
int RGBwhite[]  = {255,255,255};

// prototypes
void ICACHE_RAM_ATTR pinInterruptISR();               // ISR functions should be defined with ICACHE_RAM_ATTR attribute
void RGBLEDoff();
void pulseOff();
void checkClock();
void processSyncEvent(NTPSyncEvent_t ntpEvent);
void setRGBLED(int* colors);
void RGBLEDoff();
void setupTelnet();

//--------------------------------------------------------------------------
// Setup
//--------------------------------------------------------------------------
void setup() {

  // configure hardware...
  eeRAM.begin(SDA,SCL);
  pinMode(SWITCHPIN,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SWITCHPIN),pinInterruptISR,FALLING); // interrupt when pushbutton is pressed   
  pinMode(COIL1,OUTPUT);
  pinMode(COIL2,OUTPUT);
  pinMode(REDLED,OUTPUT);
  pinMode(GREENLED,OUTPUT);
  pinMode(BLUELED,OUTPUT);
  pulseOff();                                         // turn off outputs to Lavet motor coil
  RGBLEDoff();                                        // turn off outputs to LEDs

  // print the banner... 
  Serial.begin(115200);  
  unsigned long waitTime = millis()+500;
  while(millis() < waitTime) yield();                 // wait 500 milliseconds for the serial port
   
  Serial.printf("\n\n%s\n",TITLE);
  Serial.printf("Chip ID: %u\n",ESP.getChipId());
  Serial.printf("CPU Frequency: %u\n",ESP.getCpuFreqMHz());    
  Serial.printf("Sketch size: %u\n",ESP.getSketchSize());
  Serial.printf("Free size: %u\n",ESP.getFreeSketchSpace());
  Serial.printf("%s Reset\n",ESP.getResetReason().c_str());

  WiFi.mode(WIFI_STA);   
  WiFi.persistent(false);
  WiFi.setAutoConnect(false);
  WiFi.setAutoReconnect(false);
  
  // configure for static IP
  IPAddress local_IP(192,168,1,151);
  IPAddress gateway(192,168,1,1);
  IPAddress subnet(255,255,255,0);
  IPAddress dns(192,168,1,1);
  WiFi.config(local_IP,gateway,subnet,dns);

  // connect to WiFi...
  Serial.printf("\nConnecting to %s",WIFISSID);
  startMillis = millis();
  WiFi.begin(WIFISSID,PASSWORD);
  byte waitCount = 60;                                // 60 seconds
  byte lastSeconds = second();
  while (WiFi.status() != WL_CONNECTED) {             // while waiting to connect to WiFi...
    yield();    
    byte seconds = second();      
    if (lastSeconds != seconds) {
      lastSeconds = seconds;
      setRGBLED(RGByellow);                           // RGB LED flashes YELLOW while waiting to connect to WiFi          
      Serial.print(".");                              // print '.' every second
      if (--waitCount==0) ESP.restart();              // if WiFi not connected after 60 seconds, restart the ESP8266      
    }
  }  
  stopMillis = millis();
  Serial.printf("\nConnected to %s in %s milliseconds\n",WIFISSID,String(stopMillis-startMillis));
  Serial.printf("\nIP Address set to: %s\n",WiFi.localIP().toString().c_str());
  
  // start telnet
  setupTelnet();
   
  // start AsyncWebServer
  server.begin();
  Serial.println("\nAsyncWebServer started.");

  // if previously saved in EERAM, read hour, minute, second and timezone values from EERAM  
  if((eeRAM.read(CHECK1)==0xAA)&&(eeRAM.read(CHECK2)==0x55)){
    Serial.println("\nReading values from EERAM.");
    analogClkHour = eeRAM.read(HOUR);
    analogClkMinute = eeRAM.read(MINUTE);
    analogClkSecond = eeRAM.read(SECOND);
    analogClktimeZone = eeRAM.read(TIMEZONE); 

    server.on("/",HTTP_GET,[](AsyncWebServerRequest *request){
      request->send(200,"text/html",statuspage);
    });

    server.on("/time",HTTP_GET,[](AsyncWebServerRequest *request){
      String timeStr = NTP.getTimeStr(analogClkTime)+" "+NTP.getTimeStr(NTP.getLastNTPSync())+" "+NTP.getUptimeString();
      request->send(200,"text/plain",timeStr.c_str());      
    });  
  }
   
  // the values stored in EERAM are invalid, get hour, minute, second and timezone from the setup web page   
  else {
    Serial.printf("\nBrowse to %s to set up the Analog Clock.\n\r",WiFi.localIP().toString().c_str());
      
    server.on("/",HTTP_GET,[](AsyncWebServerRequest *request) {
      request->send_P(200,"text/html",setuppage);
    });

    // when the 'Submit' button is clicked...
    server.on("/post",HTTP_POST,[](AsyncWebServerRequest *request){ 
      request->send_P(200,"text/plain","OK, Bye");

      // save the values from the setup page in EERAM
      byte params = request->params();
      Serial.printf("Params: %u\n",params);
      for(int i=0;i<params;i++){
        const AsyncWebParameter* p = request->getParam(i);
        eeRAM.write(i,atoi(p->value().c_str()));
        Serial.printf("%s: %s\n",p->name().c_str(),p->value().c_str());
      }
      ESP.restart();                                  // reset the ESP8266 after values from setup web page have been submitted          
    });

     // loop here flashing the blue LED until the ESP8266 is reset after the 'Submit' button on the setup web page is clicked
    lastSeconds = second();
    while(true) {                                     
      yield();        
      byte seconds = second();  
      if (lastSeconds != seconds) {
        lastSeconds = seconds;
        setRGBLED(RGBblue);                           // RGB LED flashes BLUE while waiting to for values from setup web page              
      }
    }
  }

  // connect to the NTP server...
  startMillis=millis();                               // let's see how long this takes  
  NTP.begin(NTPSERVERNAME,-analogClktimeZone,true);   // start the NTP client
  NTP.setDSTZone(DST_ZONE_USA);                       // use US rules for switching between standard and daylight saving time
  NTP.setDayLight(true);                              // yes to daylight saving time   
  NTP.onNTPSyncEvent([](NTPSyncEvent_t event){ntpEvent=event;syncEventTriggered=true;});
  NTP.setInterval(SHORTINTERVAL,LONGINTERVAL);

  waitCount = 60;                                     // 60 seconds
  Serial.print("\nWaiting for sync with NTP server");   
  while (timeStatus() != timeSet) {                   // while waiting for the time to be synced and set...
    yield();    
    byte seconds = second();      
    if (lastSeconds != seconds) {
      lastSeconds = seconds;
      setRGBLED(RGBpurple);                           // LED flashes PURPLE while waiting to connect to NTP server          
      Serial.print(".");                              // print '.' every second
      if (--waitCount==0) ESP.restart();              // if the time is not set after 60 seconds, restart the ESP8266      
    }
  }    
  Serial.println("\nSynced with "+NTP.getNtpServerName());
      
  analogClkWeekday=weekday();                         // take initial values for weekday...
  analogClkDay=day();                                 // and day... 
  analogClkMonth=month();                             // and month...
  analogClkYear=year()-1970;                          // and year from NTP
  analogClkTime = makeTime({analogClkSecond,analogClkMinute,analogClkHour,analogClkWeekday,analogClkDay,analogClkMonth,analogClkYear});         

  // lastly, start up 100 millisecond ticker callback used to advance the analog clock's second hand
  clockTimer.attach_ms(100,checkClock);
  
  setupOTA();
  
  Serial.printf("\nBrowse to %s for Analog Clock status.\n\n",WiFi.localIP().toString().c_str());                          
} // end of setup()

//--------------------------------------------------------------------------
// Main Loop
//--------------------------------------------------------------------------
void loop() {
  static byte lastSeconds=0; 
  
  telnet.loop();  
  ArduinoOTA.handle();
   
  // once each second....
  byte secs = second();  
    if (lastSeconds != secs) {
      lastSeconds = secs;   
        if (analogClkTime > now()) {
          setRGBLED(RGBwhite);                        // RGB LED flashes WHITE while waiting to connect to WiFi                     
      }
      else {
        if (NTPerror) {
          setRGBLED(RGBred);                          // RGB LED flashes RED if there's an issue with the NTP server
        }
        else {
          setRGBLED(RGBgreen);                        // RGB LED flashes GREEN
        }
      }
    }

  // if either ESP8266's internal time or analog clock's time has changed...
  if (printTime) {
    printTime = false;       
    // print ESP8266's internal time and analog clock time
    NTPtime = NTP.getTimeStr(now());
    clockTime= NTP.getTimeStr(analogClkTime);
    Serial.println(NTPtime+"\t"+clockTime);
    telnet.println(NTPtime+"\t"+clockTime);
  }
    
  // process any NTP events
  if (syncEventTriggered) {                           // if an NTP time sync event has occured...
    processSyncEvent(ntpEvent);
    syncEventTriggered = false;
  }
} // end of loop()

//------------------------------------------------------------------------
// flash the RGB LED
//-------------------------------------------------------------------------
void setRGBLED(int* colors){
  analogWrite(REDLED,colors[0]);
  analogWrite(BLUELED,colors[1]);
  analogWrite(GREENLED,colors[2]);
  ledTimer.once_ms(100,RGBLEDoff);    
}    

//------------------------------------------------------------------------
// Ticker callback that turns off the RGB LED after 100 milliseconds.
//-------------------------------------------------------------------------
void RGBLEDoff(){
  analogWrite(REDLED,0); 
  analogWrite(GREENLED,0);                          
  analogWrite(BLUELED,0);                           
}

//--------------------------------------------------------------------------
// pulse the clock's Lavet motor to advance the clock's second hand.
// The Lavet motor requires polarized control pulses. If the control pulses are inverted,
// the clock appears to run one second behind. To remedy the problem, invert the polarity
// of the control pulses. This is easily done by exchanging the wires connecting the Lavet motor.
//--------------------------------------------------------------------------
void pulseCoil() {
  if ((analogClkSecond%2)==0){                        // positive motor pulse on even seconds
    digitalWrite(COIL1,HIGH);
    digitalWrite(COIL2,LOW);
  }
  else {                                              // negative motor pulse on odd seconds
    digitalWrite(COIL1,LOW);
    digitalWrite(COIL2,HIGH);
  }
  pulseTimer.once_ms(PULSETIME,pulseOff);             // turn off pulse after 30 milliseconds...
}

//------------------------------------------------------------------------
// Ticker callback that turns off the pulse to the analog clock Lavet motor after 30 milliseconds.
//-------------------------------------------------------------------------
void pulseOff() {
  digitalWrite(COIL1,LOW);
  digitalWrite(COIL2,LOW);
}

//--------------------------------------------------------------------------
// Ticker callback runs every 100 milliseconds to check if the analog clock's 
// second hand needs to be advanced.
//--------------------------------------------------------------------------
void checkClock() {
  static byte lastSeconds = 0;  
   
  if (analogClkTime < now()) {                        // if the analog clock is behind the actual time and needs to be advanced...
    pulseCoil();                                      // pulse the motor to advance the analog clock's second hand
    if (++analogClkSecond==60){                       // since the clock motor has been pulsed, increase the seconds count
      analogClkSecond=0;                              // at 60 seconds, reset analog clock's seconds count back to zero
      if (++analogClkMinute==60) {
        analogClkMinute=0;                            // at 60 minutes, reset analog clock's minutes count back to zero
        if (++analogClkHour==24) {
          analogClkHour=0;                            // at 24 hours, reset analog clock's hours count back to zero
          analogClkWeekday=weekday();                 // update values
          analogClkDay=day();
          analogClkMonth=month();
          analogClkYear=year()-1970;   
        }
      }
    }
    // update with new values
    analogClkTime = makeTime({analogClkSecond,analogClkMinute,analogClkHour,analogClkWeekday,analogClkDay,analogClkMonth,analogClkYear});      
    eeRAM.write(HOUR,analogClkHour);                  // save the new values in eeRAM
    eeRAM.write(MINUTE,analogClkMinute);
    eeRAM.write(SECOND,analogClkSecond); 
    printTime = true;                                 // set flag to update display
  } // if (analogClkTime<now())   
 
  // this part was added so that the times are printed when the analog clock's hands are stopped waiting for the ESP8266's internal time to catch up
  byte secs = second();  
  if (lastSeconds != secs) {                          // when the ESP8266's internal time changes...
    lastSeconds = secs;                               // save for next time 
    printTime = true;                                 // set flag to print new time 
  }  
 }

//--------------------------------------------------------------------------
// NTP client event handler
//--------------------------------------------------------------------------
void processSyncEvent(NTPSyncEvent_t ntpEvent) {
   switch(ntpEvent) {
      case timeSyncd:
         stopMillis = millis();
         Serial.printf("%s responded in %s milliseconds\n",NTPSERVERNAME,String(stopMillis-startMillis));
         telnet.printf("%s responded in %s milliseconds\n\r",NTPSERVERNAME,String(stopMillis-startMillis));
         lastSyncTime = NTP.getTimeStr(NTP.getLastNTPSync());
         NTPerror = false;
         break;
      case noResponse:
         Serial.println("Time Sync error: No response from NTP server");
         telnet.println("Time Sync error: No response from NTP server");      
         NTPerror = true;
         break;
      case invalidAddress:
         Serial.println("Time Sync error: NTP server not reachable");      
         telnet.println("Time Sync error: NTP server not reachable");
         NTPerror = true;
         break;
      case requestSent:
         startMillis=millis();      
         Serial.println("NTP request sent, waiting for response...");
         telnet.println("NTP request sent, waiting for response...");      
         break;
      case errorSending:
         Serial.println("Time Sync error: An error occurred while sending the request to the NTP server");      
         telnet.println("Time Sync error: An error occurred while sending the request to the NTP server");
         NTPerror = true;
         break;
      case responseError:
         Serial.println("Time Sync error: Wrong response received from the NTP server");
         telnet.println("Time Sync error: Wrong response received from the NTP server");      
         NTPerror = true;
   }    
}

//--------------------------------------------------------------------------
// interrupt when the push button switch is pressed. clear EERAM and restart
// the ESP8266. this forces the user to re-enter the values for clock's
// hour, minute, second and timezone.
//--------------------------------------------------------------------------
void ICACHE_RAM_ATTR pinInterruptISR() {              // ISR functions should be defined with ICACHE_RAM_ATTR...
  unsigned long debounce_time = millis()+DEBOUNCE;
  while(millis() < debounce_time);                    // wait 50 milliseconds for the switch contacts to stop bouncing
  for (byte i=0;i<10;i++) {
    eeRAM.write(HOUR+i,0);                            // clear eeram     
  }   
  ESP.restart();                                      // restart the ESP8266
}

//--------------------------------------------------------------------------
// setup over-the-air updates
//--------------------------------------------------------------------------
void setupOTA(){
  ArduinoOTA.onStart([]() {                           // show steady GREEN LED while updating
    analogWrite(REDLED,0); 
    analogWrite(GREENLED,255);                          
    analogWrite(BLUELED,0);  
  });

  ArduinoOTA.onEnd([]() {                             // turn off GREEN LED
    RGBLEDoff();
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });

  ArduinoOTA.onError([](ota_error_t error) {
    analogWrite(REDLED,255);                           // show steady RED LED in event of OTA error
    analogWrite(GREENLED,0);                          
    analogWrite(BLUELED,0);  
  });  
  
  ArduinoOTA.begin();
}

//--------------------------------------------------------------------------
// setup Telnet
//--------------------------------------------------------------------------
void setupTelnet() {  
  // passing on functions for various telnet events
  telnet.onConnect(onTelnetConnect);
  telnet.onConnectionAttempt(onTelnetConnectionAttempt);
  telnet.onReconnect(onTelnetReconnect);
  telnet.onDisconnect(onTelnetDisconnect);
  telnet.onInputReceived(onTelnetInput);

  Serial.print("Telnet: ");
  if (telnet.begin(23)) {
    Serial.println("running");
  } else {
    Serial.println("error.");
  }
}

// (optional) callback functions for telnet events
void onTelnetConnect(String ip) {
  Serial.print("Telnet: ");
  Serial.print(ip);
  Serial.println(" connected");
  
  telnet.println("\nWelcome " + telnet.getIP());
  telnet.println("Type 'bye'  to disconnect.");
}

void onTelnetDisconnect(String ip) {
  Serial.print("Telnet: ");
  Serial.print(ip);
  Serial.println(" disconnected");
}

void onTelnetReconnect(String ip) {
  Serial.print("Telnet: ");
  Serial.print(ip);
  Serial.println(" reconnected");
}

void onTelnetConnectionAttempt(String ip) {
  Serial.print("Telnet: ");
  Serial.print(ip);
  Serial.println(" tried to connected");
}

void onTelnetInput(String str) {
  // check for reset command
  if (str == "reset") {
    telnet.println("\nresetting..."); 
    Serial.println("\nresetting...");
    telnet.disconnectClient();
    ESP.restart();
  }
  // disconnect the client
  else if (str == "bye") {
    telnet.println("\ndisconnecting...");
    telnet.disconnectClient();
  } 
}
