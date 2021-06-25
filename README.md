BellioT - IoT Bell
==================

Contents
--------

- [Intro](#intro)
    - [What is it?](#what-is-it)
    - [Why do I need it?](#why-do-i-need-it)
    - [What are the features?](#what-are-the-features)
- [Details - TBA](#details)
    - [Components](#components)
    - [Wiring](#wiring)
    - [Description](#description)
    - [Demo Images](#demo-images)
- [Progress](#progress)
- [Credits and References - TBA](#credits-and-references)
    - [MCP3202 Usage](#mcp3202-usage)
    - [ESP8266 I2S Usage](#esp8266-i2s-usage)
    - [ESP32 Timer Interrupt Usage](#esp32-timer-interrupt-usage)
    - [PCM5102 Module Wiring](#pcm5102-module-wiring)
    - [Libraries](#libraries)

Intro
-----  

### What is it?
An IoT Door Bell that enables indoor people to see and talk easily with outdoor guests.

### Why do I need it?
In my house, my room is quite high from the ground. So whenever a guest presses my house's bell, I have to go to the first floor to ask who that is. That's very inconvenient. Of course I can just shout out from upstairs, but that is not very nice. So this device may make my life easier.

### What are the features?
The bell is connected to a master device via Wifi and MQTT, and the two devices behave like two telephones. I can talk easily to the guest (and may even see his/her face with a camera --- hopefully, not done yet). I am also thinking about extending it so that I can use a mobile app to communicate with my guest when I'm not at home.

Details
-------

### Components

- ESP32/ESP8266 x2
- PCM5102 Module x2 (the purple one)
- MCP3202 12-bit ADC x2 (BI/P is the one comaptible with breadboard)
- MAX9814 module with AGC x2
- A Raspberry Pi to run Mosquitto MQTT Broker

### Wiring

For Master device (ESP8266 NodeMCU):  
![image](https://user-images.githubusercontent.com/46307950/123385488-c8d7fc80-d5bf-11eb-90a3-92808af9fbdc.png)

For Bell device (ESP32-CAM):  
![image](https://user-images.githubusercontent.com/46307950/123391716-7fd77680-d5c6-11eb-83aa-0279f06460c9.png)

### Description

Note: the reason I use an ESP8266 for one device and ESP32 for another is that I don't have one more ESP32. I would prefer two ESP32's.

MAX9814 is the sensor to record audio from the environment. The external ADC MCP3202 with read the values (frequency = 8kHz) and send to the ESP via SPI. The ESP buffers 80 16-bit samples of data, then it sends the audio data (8-bit, so the payload has 160 values) to its designated MQTT topic. The other ESP reads from that topic and uses I2S to play the audio via the external DAC PCM5102.  

Note that each device acts as both the transmitter and receiver, just like telephones.

If you want to try my code, do the following steps:  

- Remember to change Wifi credentials and server IP in the code to your own
- First run a MQTT broker on Raspberry Pi:  `mosquitto -p 2883`  
- Then power on two devices. They will publish data in topics "audioMaster" and "audioBell".

### Demo Images

Progress
--------

For now audio transmission is almost completed. Voice can be heard clearly, but there is still quite some noise. An observation is that the voice heard from ESP32 device has less noise.  

These are the things I want to add in the future:  

- Camera stream to a NodeJS server using WebSocket
- Integrate LCD to see the camera directly from the Master device
- Provide an UI to set up SSID and Password easily
- Reduce noise, maybe by implementing some kind of codec
- Integrate with a smart home network
- Finally, design a case so that it becomes a complete product

Credits and References
----------------------

- MCP3202 Usage: https://github.com/souviksaha97/MCP3202
- ESP8266 I2S Usage: TBA
- ESP32 Timer Interrupt Usage: https://diyprojects.io/esp32-timers-alarms-interrupts-arduino-code/
- PCM5102 Module Wiring: https://github.com/earlephilhower/ESP8266Audio
- Libraries:
    - PubSubClient: https://github.com/knolleary/pubsubclient
    - ESP8266TimerInterrupt: https://github.com/khoih-prog/ESP8266TimerInterrupt