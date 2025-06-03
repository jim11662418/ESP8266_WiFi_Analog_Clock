//-------------------------------------------------------------------------------------------------------------------------------------
// Copyright 2022-2025 Jim Loos
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files
// (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge,
// publish, distribute, sub-license, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do
// so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
// OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
// LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR
// IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//-------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------
// Analog Clock
//
// This sketch uses an ESP8266 on a WEMOS D1 Mini board to retrieve the time 
// from an NTP server and pulse the Lavet motor on an inexpensive analog quartz
// clock to keep the clock in sync with local time. The ESP8266 stores the position 
// of the clock's hour, minute and second hands in I2C Serial EERAM. NTP time 
// and analog clock time are display in the serial monitor once each second.
//
// on startup:
//    YELLOW LED flashes while waiting to connect to WiFi
//    PURPLE LED flashes while waiting to connect to the NTP server
//    BLUE LED flashes while waiting for input from the setup web page
//
// after startup:
//    GREEN LED flashes during normal operation
//    RED LED flashes if unable to sync with NTP server
//    WHITE LED flashes while the analog clock's hands are stopped waiting for actual time to catch up
//
//--------------------------------------------------------------------------------------------------------------------------------------

#define VERSION "2.6"

#include <TimeLib.h>
#include <ESP8266WiFi.h>
#include <NtpClientLib.h>                       // https://github.com/gmag11/NtpClient
#include <Ticker.h>
#include <EERAM.h>                              // https://github.com/MajenkoLibraries/EERAM/
#include <ESPAsyncWebServer.h>                  // https://github.com/me-no-dev/ESPAsyncWebServer
#include <TelnetPrint.h>                        // https://github.com/JAndrassy/TelnetStream/

#define GRAPHIC                                 // display analog clock graphic on status page
#define SVG                                     // use Scalable Vector Graphics image for the analog clock face 
#include "html_code.h"                          // HTML code for clock status and setup pages

// WEMOS D1 Mini pins
#define D0 16                                   // can't use D0 to generate interrupts
#define D1  5
#define D2  4
#define D3  0                                   // 10K pull-up, boot fails if pulled LOW
#define D4  2                                   // 10K pull-up, boot fails if pulled LOW, BUILTIN_LED,
#define D5 14
#define D6 12
#define D7 13
#define D8 15                                   // 10K pull-down, boot fails if pulled HIGH

#define SDA D1                                  // output to SDA on the EERAM
#define SCL D2                                  // output to SCL on the EERAM
#define COIL1 D3                                // output to clock's lavet motor coil
#define COIL2 D7                                // output to clock's lavet motor coil
#define REDLED D5                               // output to red part of the RGB LED
#define GREENLED D4                             // output to green part of the RGB LED
#define BLUELED D8                              // output to blue part of the RGB LED
#define SWITCHPIN D6                            // input from push button switch

#define title "ESP8266 WiFi Analog Clock"

#define DEBOUNCE 50                             // 50 milliseconds to debounce the pushbutton switch
#define PULSETIME 30                            // 30 millisecond pulse for the clock's lavet motor

#define WIFISSID ".........."
#define PASSWORD ".........."    

#define HOUR     0x0000                         // address in EERAM for analogClkHour
#define MINUTE   HOUR+1                         // address in EERAM for analogClkMinute
#define SECOND   HOUR+2                         // address in EERAM for analogClkSecond
#define TIMEZONE HOUR+3                         // address in EERAM for analogClktimeZone
#define CHECK1   HOUR+4                         // address in EERAM for 1st check byte 0xAA
#define CHECK2   HOUR+5                         // address in EERAM for 2nd check byte 0x55

#define NTPSERVERNAME "time.nist.gov"
//#define NTPSERVERNAME "0.us.pool.ntp.org"
//#define NTPSERVERNAME "time.windows.com"
//#define NTPSERVERNAME "time.google.com"
//#define NTPSERVERNAME "time-a-g.nist.gov"
#define LONGINTERVAL 600                        // re-sync with the NTP server every 600 seconds
#define SHORTINTERVAL 10                        // if the time has not been set, re-try the NTP server every 10 seconds

AsyncWebServer server(80);
EERAM eeRAM;
time_t analogClkTime;
IPAddress ip;
Ticker pulseTimer,clockTimer,ledTimer;
NTPSyncEvent_t ntpEvent;
String lastSyncTime = "";
String NTPtime, clockTime;
boolean syncEventTriggered = false;             // true if an NTP time sync event has been triggered
boolean printTime = false;                      // if true, print NTP and clock's time
byte analogClktimeZone=5;                       // GMT -5 = EST
byte analogClkHour=0;
byte analogClkMinute=0;
byte analogClkSecond=0;
byte analogClkWeekday=0;
byte analogClkDay=0;
byte analogClkMonth=0;
byte analogClkYear=0;
int activeLED = GREENLED;

void ICACHE_RAM_ATTR pinInterruptISR();         // ISR functions should be defined with ICACHE_RAM_ATTR attribute
void LEDsoff();
void pulseOff();
void checkClock();
void processSyncEvent(NTPSyncEvent_t ntpEvent);

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
  digitalWrite(COIL1,LOW);
  digitalWrite(COIL2,LOW);
  digitalWrite(REDLED,LOW);
  digitalWrite(GREENLED,LOW);
  digitalWrite(BLUELED,LOW);

  // print the banner... 
  Serial.begin(115200);  
  unsigned long waitTime = millis()+500;
  while(millis() < waitTime) yield();           // wait 500 milliseconds for the serial port
   
  Serial.printf("\n\n%s\n",title);
  Serial.printf("Sketch size: %u\n",ESP.getSketchSize());
  Serial.printf("Free size: %u\n",ESP.getFreeSketchSpace());
  Serial.printf("%s Reset\n",ESP.getResetReason().c_str());

  // connect to WiFi...
  Serial.printf("\nConnecting to %s",WIFISSID);
  WiFi.begin(WIFISSID,PASSWORD);
  byte waitCount = 60;                          // 60 seconds
  byte lastSeconds = second();
  while (WiFi.status() != WL_CONNECTED) {       // while waiting to connect to WiFi...
    yield();    
    byte seconds = second();      
    if (lastSeconds != seconds) {
      lastSeconds = seconds;
      digitalWrite(REDLED,HIGH);                // flash the red LED once each second while waiting to connect to WiFi
      ledTimer.once_ms(100,LEDsoff);
      Serial.print(".");                        // print '.' every second
      if (--waitCount==0) ESP.restart();        // if WiFi not connected after 60 seconds, restart the ESP8266      
    }
  }  
  Serial.println("\nConnected.");
  digitalWrite(REDLED,LOW);                     // turn off the red LED

  // start TelnetPrint
  TelnetPrint.begin();
  Serial.println("\nTelnetPrint started.");
   
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
   
  // else, get hour, minute, second and timezone values from the setup web page   
  else {
    Serial.printf("\nBrowse to %s to set up the Analog Clock.\n\r",WiFi.localIP().toString().c_str());
      
    server.on("/",HTTP_GET,[](AsyncWebServerRequest *request){
      request->send_P(200,"text/html",setuppage);
    });

    server.on("/post",HTTP_POST,[](AsyncWebServerRequest * request) {
      request->send_P(200,"text/plain","OK, Bye");

      // save the values from the setup page in EERAM
      byte params = request->params();
        for(int i=0;i<params;i++){
        const AsyncWebParameter* p = request->getParam(i);
        eeRAM.write(i,atoi(p->value().c_str()));
        Serial.printf("%s: %s\n",p->name().c_str(),p->value().c_str());
      }

      ESP.restart();                 
    });

    lastSeconds = second();
    while(true) {                               // loop here until initial values are entered into the web page
      yield();        
      byte seconds = second();  
      if (lastSeconds != seconds) {
        lastSeconds = seconds;
        digitalWrite(BLUELED,HIGH);             // flash the blue LED once each second while waiting for input to the web page
        ledTimer.once_ms(100,LEDsoff);
      }
    }
  }

  // connect to the NTP server...
  NTP.begin(NTPSERVERNAME,-analogClktimeZone,true);  // start the NTP client
  NTP.setDSTZone(DST_ZONE_USA);                      // use US rules for switching between standard and daylight saving time
  NTP.setDayLight(true);                             // yes to daylight saving time   
  NTP.onNTPSyncEvent([](NTPSyncEvent_t event){ntpEvent=event;syncEventTriggered=true;});
  NTP.setInterval(SHORTINTERVAL,LONGINTERVAL);

  waitCount = 60;                                    // 60 seconds
  Serial.print("\nWaiting for sync with NTP server");   
  while (timeStatus() != timeSet) {                  // while waiting for the time to be synced and set...
    yield();    
    byte seconds = second();      
    if (lastSeconds != seconds) {
      lastSeconds = seconds;
      digitalWrite(REDLED,HIGH);                     // LED flashes PURPLE while waiting to connect to the NTP server
      digitalWrite(BLUELED,HIGH);
      ledTimer.once_ms(100,LEDsoff);
      Serial.print(".");                             // print '.' every second
      if (--waitCount==0) ESP.restart();             // if the time is not set after 60 seconds, restart the ESP8266      
    }
  }    
  Serial.println("\nSynced with "+NTP.getNtpServerName());
      
  analogClkWeekday=weekday();                        // take initial values for weekday...
  analogClkDay=day();                                // and day... 
  analogClkMonth=month();                            // and month...
  analogClkYear=year()-1970;                         // and year from NTP
  analogClkTime = makeTime({analogClkSecond,analogClkMinute,analogClkHour,analogClkWeekday,analogClkDay,analogClkMonth,analogClkYear});         

  // lastly, start up 100 millisecond ticker callback used to advance the analog clock's second hand
  clockTimer.attach_ms(100,checkClock);

  Serial.printf("\nBrowse to %s for Analog Clock status.\n\n",WiFi.localIP().toString().c_str());                          
} // end of setup()

//--------------------------------------------------------------------------
// Main Loop
//--------------------------------------------------------------------------
void loop() {
  static byte lastSeconds=0; 
   
  // once each second....
  byte secs = second();  
    if (lastSeconds != secs) {
      lastSeconds = secs;   
        if (analogClkTime > now()) {
          digitalWrite(BLUELED,HIGH);           // LED flashes WHITE if waiting for actual time to catch up to analog clock time                 
          digitalWrite(GREENLED,HIGH);                          
          digitalWrite(REDLED,HIGH);             
      }
      else {
        digitalWrite(activeLED,HIGH);           // else, LED flashes GREEN during normal operation
      }

      ledTimer.once_ms(100,LEDsoff);
    }

  // if either ESP8266's internal time or analog clock's time has changed...
  if (printTime) {
    printTime = false;       
    // print ESP8266's internal time and analog clock time
    NTPtime = NTP.getTimeStr(now());
    clockTime= NTP.getTimeStr(analogClkTime);
    Serial.println(NTPtime+"\t"+clockTime);
    TelnetPrint.println(NTPtime+"\t"+clockTime);
  }
    
  // process any NTP events
  if (syncEventTriggered) {                     // if an NTP time sync event has occured...
    processSyncEvent(ntpEvent);
    syncEventTriggered = false;
  }
} // end of loop()

//------------------------------------------------------------------------
// Ticker callback that turns off the LEDs after 100 milliseconds.
//-------------------------------------------------------------------------
void LEDsoff(){
   digitalWrite(BLUELED,LOW);                          
   digitalWrite(GREENLED,LOW);                          
   digitalWrite(REDLED,LOW);                          
}

//--------------------------------------------------------------------------
// pulse the clock's Lavet motor to advance the clock's second hand.
// The Lavet motor requires polarized control pulses. If the control pulses are inverted,
// the clock appears to run one second behind. To remedy the problem, invert the polarity
// of the control pulses. This is easily done by exchanging the wires connecting the Lavet motor.
//--------------------------------------------------------------------------
void pulseCoil() {
  if ((analogClkSecond%2)==0){                  // positive motor pulse on even seconds
    digitalWrite(COIL1,HIGH);
    digitalWrite(COIL2,LOW);
  }
  else {                                        // negative motor pulse on odd seconds
    digitalWrite(COIL1,LOW);
    digitalWrite(COIL2,HIGH);
  }
  pulseTimer.once_ms(PULSETIME,pulseOff);       // turn off pulse after 30 milliseconds...
}

//------------------------------------------------------------------------
// Ticker callback that turns off the pulse to the analog clock Lavet motor
// after 30 milliseconds.
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
   
  if (analogClkTime < now()) {                  // if the analog clock is behind the actual time and needs to be advanced...
    pulseCoil();                                // pulse the motor to advance the analog clock's second hand
    if (++analogClkSecond==60){                 // since the clock motor has been pulsed, increase the seconds count
      analogClkSecond=0;                        // at 60 seconds, reset analog clock's seconds count back to zero
      if (++analogClkMinute==60) {
        analogClkMinute=0;                      // at 60 minutes, reset analog clock's minutes count back to zero
        if (++analogClkHour==24) {
          analogClkHour=0;                      // at 24 hours, reset analog clock's hours count back to zero
          analogClkWeekday=weekday();           // update values
          analogClkDay=day();
          analogClkMonth=month();
          analogClkYear=year()-1970;   
        }
      }
    }
    // update with new values
    analogClkTime = makeTime({analogClkSecond,analogClkMinute,analogClkHour,analogClkWeekday,analogClkDay,analogClkMonth,analogClkYear});      
    eeRAM.write(HOUR,analogClkHour);            // save the new values in eeRAM
    eeRAM.write(MINUTE,analogClkMinute);
    eeRAM.write(SECOND,analogClkSecond); 
    printTime = true;                           // set flag to update display
  } // if (analogClkTime<now())   
 
  // this part was added so that the times are printed when the analog clock's hands are stopped waiting for the ESP8266's internal time to catch up
  byte secs = second();  
  if (lastSeconds != secs) {                    // when the ESP8266's internal time changes...
    lastSeconds = secs;                         // save for next time 
    printTime = true;                           // set flag to print new time 
  }  
 }

//--------------------------------------------------------------------------
// NTP client event handler
//--------------------------------------------------------------------------
void processSyncEvent(NTPSyncEvent_t ntpEvent) {
  static unsigned long startMillis,stopMillis;
   switch(ntpEvent) {
      case timeSyncd:
         stopMillis = millis();
         Serial.printf("%s responded in %s milliseconds\n",NTPSERVERNAME,String(stopMillis-startMillis));
         TelnetPrint.printf("%s responded in %s milliseconds\n",NTPSERVERNAME,String(stopMillis-startMillis));
         lastSyncTime = NTP.getTimeStr(NTP.getLastNTPSync());
         activeLED = GREENLED;
         break;
      case noResponse:
         Serial.println("Time Sync error: No response from NTP server");
         TelnetPrint.println("Time Sync error: No response from NTP server");      
         activeLED = REDLED;   
         break;
      case invalidAddress:
         Serial.println("Time Sync error: NTP server not reachable");      
         TelnetPrint.println("Time Sync error: NTP server not reachable");
         activeLED = REDLED;   
         break;
      case requestSent:
         startMillis=millis();      
         Serial.println("NTP request sent, waiting for response...");
         TelnetPrint.println("NTP request sent, waiting for response...");      
         break;
      case errorSending:
         Serial.println("Time Sync error: An error occurred while sending the request to the NTP server");      
         TelnetPrint.println("Time Sync error: An error occurred while sending the request to the NTP server");
         activeLED = REDLED;   
         break;
      case responseError:
         Serial.println("Time Sync error: Wrong response received from the NTP server");
         TelnetPrint.println("Time Sync error: Wrong response received from the NTP server");      
         activeLED = REDLED;   
   }    
}

//--------------------------------------------------------------------------
// interrupt when the push button switch is pressed. clear EERAM and restart
// the ESP8266. this forces the user to re-enter the values for clock's
// hour, minute, second and timezone.
//--------------------------------------------------------------------------
void ICACHE_RAM_ATTR pinInterruptISR() {        // ISR functions should be defined with ICACHE_RAM_ATTR...
  unsigned long debounce_time = millis()+DEBOUNCE;
  while(millis() < debounce_time);              // wait 50 milliseconds for the switch contacts to stop bouncing
  for (byte i=0;i<10;i++) {
    eeRAM.write(HOUR+i,0);                      // clear eeram     
  }   
  ESP.restart();                                // restart the ESP8266
}
