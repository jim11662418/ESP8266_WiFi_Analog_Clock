# ESP8266 WiFi Analog Clock
## Introduction
This project uses an WEMOS D1 Mini ESP8266 module and an Arduino sketch to connect to a NTP (Network Time Protocol) server to automatically retrieve and display the local time on a inexpensive analog quartz clock. The ESP8266 reconnects to the NTP server every 15 minutes which keeps the clock accurate. The clock also automatically adjusts for daylight savings time.

<p align="center"><img src="/images/Clock-face.JPEG"/>
<br />
<br />  
<p align="center"><img src="/images/Clock-back.JPEG"/>
<p align="center">WEMOS D1 Mini ESP8266 Module with EERAM IC and Components on a Piece of Perfboard</p>
<br />
<br />
<p align="center"><img src="/images/Schematic.jpg"/>
<p align="center">Schematic</p>
<p align="center"><img src="/images/AnalogClock.gif"/>

## Hardware
I'm using an analog clock with a quartz movement I found at my local Walmart for $3.88. Whatever analog clock you decide to use, its quartz movement will need to be modified so that it can be controlled by the ESP8266 module. Open up the movement (most of them snap together without any fasteners), disconnect the internal coil of the Lavet stepping motor from its quartz oscillator and then solder a wire to each of the coil's leads to make connections for the ESP8266. If you search around on the web you'll find articles showing how others have done it. Be careful when working with the coil. The coil's wires are typically thinner than a human hair and extremely fragile.
<p align="center"><img src="/images/Clock%20Movement.jpeg"/>
<p align="center">Modified Clock Movement</p>

## Software
The sketch: AnalogClock.ino should be (I hope) clear enough, but here, in brief, is a summary of how it operates. Ten times each second the ESP8266 compares the time displayed on the analog clock to the actual time retrieved from an NTP server. If the analog clock lags behind the actual time, the ESP8266 advances the clock's second hand until the clock agrees with the actual time. If the time displayed on the analog clock time is ahead of the actual time, the ESP8266 simply waits until the actual time catches up with the analog clock since it can't move the clock's hands backwards. 

The ESP8266 advances the analog clock's second hand by generating bipolar pulses, alternately positive and negative to the clock's Lavet motor coil. Because of differences in clock mechanisms, you may need to increase or decrease the "PULSETIME" constant in the sketch by few milliseconds to make your mechanism step reliably. Experimentally, I found that 30 milliseconds works best for my movement.

The biggest problem with using these cheap analog clocks for a project like this is that the clocks don't provide any type of feedback to indicate the position of the clock's hands. Thus if power is interrupted to the ESP8266 controlling the clock, the ESP8266 "forgets" where the clock's hands are positioned.  To get around this problem, the positions of the hour, minute and second hands are stored in a [Microchip 47L04 Serial EERAM](https://www.microchip.com/wwwproducts/en/47L04) (4Kbit SRAM with EEPROM backup) and updated each second as the clock's hands positions change. If power is interrupted, the ESP8266 can retrieve the last position of the clock's hands from the EERAM when power is reapplied. 

The very first time that the sketch is run, the user will be directed to a simple web page (see below) served by the ESP8266 which is used to tell it where the analog clock's hands are initially positioned. From that point on, the ESP8266 will use the data stored in the EERAM to "remember" the positions of the clock's hands.
<br />
<br />
<p align="center"><img src="/images/screen1.jpg"/>
<p align="center">Analog Clock Setup Page</p>
<br />
<br />
<p align="center"><img src="/images/screen2.jpg"/>
<p align="center">Arduino Serial Monitor During Analog Clock Startup</p>
<br />
<br />
Once the ESP8266 finishes its initialization and starts operation, it serves a simple web page showing the clock's status. The status page can optionally show a graphic image representing the clock's Face drawn using Scalable Vector Graphics, or HTML Canvas, or no image at all.
<br />
<br />
<p align="center"><img src="/images/screen3.jpg"/>
<p align="center">Analog Clock Status Page Using Scalable Vector Graphics to Draw the Clock Face</p>
<br />
<br />
<p align="center"><img src="/images/screen4.jpg"/>
<p align="center">Analog Clock Status Page Using the HTML Canvas Element to Draw the Clock Face</p>
<br />
<br />
<p align="center"><img src="/images/screen5.jpg"/>
<p align="center">Analog Clock Status Page Displaying Text Only</p>
