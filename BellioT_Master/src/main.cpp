#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <i2s.h>
#include <i2s_reg.h>
#include <SPI.h>

#define TIMER_INTERRUPT_DEBUG 0
#define _TIMERINTERRUPT_LOGLEVEL_ 0
#include "ESP8266TimerInterrupt.h"

#define BUTTON_CH 1
#define CALL_TIMEOUT 600000
#define IDLE_TIMEOUT 5000
#define TERM_TIMEOUT 5000

enum StateEnum {SLEEP, CALL, TERM};
typedef enum StateEnum State;

State curState = SLEEP;
unsigned long lastData_ms = 0;
unsigned long callBegin_ms = 0;
unsigned long termBegin_ms = 0;

bool isTimeout(unsigned long ms, unsigned long timeout);

bool prevButtonPressed = false;
bool curButtonPressed = false;
void updateButtonStatus();
bool isButtonPressed();

#define I2S_BCK  15
#define I2S_WS   2
#define I2S_DATA 3

bool IRAM_ATTR i2s_write_lr_nb(int16_t left, int16_t right){
    uint32_t sample = right & 0xFFFF;
    sample = sample << 16;
    sample |= left & 0xFFFF;
    return i2s_write_sample_nb(sample);
}

#define ADC_SPI_CS   5
#define ADC_SPI_MISO 12
#define ADC_SPI_MOSI 13
#define ADC_SPI_CLK  14
#define ADC_CH     0
#define ADC_BUFFER_LENGTH 160      //so 80 16-bit samples each publish

byte adc[ADC_BUFFER_LENGTH];
// volatile bool bufferFull = false;
bool bufferFull = false;
size_t bufferN = 0;

uint16_t readADC(byte ch, byte *e1, byte *e2);

const char *ssid = "DESKTOP-H1E7PQR 5840";
const char *password = "qazwsxedc";
const uint16_t mqtt_port = 2883;
IPAddress mqtt_server(192, 168, 0, 101);

WiFiClient espClient;
PubSubClient client(espClient);

void setup_wifi();
void callback(char* topic, byte* payload, unsigned int length);
void reconnect();

ESP8266Timer ITimer;
#define ADC_FREQ 8000 // hz

volatile bool bReadADC = false;
void IRAM_ATTR TimerHandler()
{
    bReadADC = true;
}

void setup()
{
    Serial.begin(115200);

    /* Wifi and MQTT setup */
    setup_wifi();
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callback);
    if (!client.setBufferSize(ADC_BUFFER_LENGTH + 100)) {
        Serial.println("Unable to set buffer size");
    }
    Serial.println("WiFi and MQTT setup successfully");

    /* ADC SPI setup */
    pinMode(ADC_SPI_CS,OUTPUT);
    digitalWrite(ADC_SPI_CS, HIGH);
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE3);
    SPI.pins(ADC_SPI_CLK, ADC_SPI_MISO, ADC_SPI_MOSI, ADC_SPI_CS);
    SPI.begin();

    /* Timer setup */
    if (ITimer.attachInterrupt(ADC_FREQ, TimerHandler))
    {
        Serial.print(F("Starting ITimer OK"));
    }
    else
        Serial.println(F("Can't set ITimer correctly. Select another freq. or interval"));

    /* I2S init */
    i2s_begin();
    i2s_set_rate(8000);
    
}

void loop()
{
    // manage MQTT connection. MUST-HAVE
    if (!client.connected())
    {
        reconnect();
    }
    client.loop();

    // manage device's state
    updateButtonStatus();
    // if (isButtonPressed())
    //     Serial.println("Button pressed!");
    if (curState == SLEEP) {
        if (isButtonPressed()) {
            curState = CALL;
            callBegin_ms = millis();
            lastData_ms = millis();
        }
    } else if (curState == CALL) {
        // send data
        if (bReadADC) {
            readADC(ADC_CH, &adc[bufferN], &adc[bufferN+1]);
            bufferN += 2;
            if (bufferN >= ADC_BUFFER_LENGTH) {
                bufferN = 0;
                if (!client.publish("audioMaster", adc, ADC_BUFFER_LENGTH, false)) {
                    Serial.println("Publish failed");
                }
            }
            bReadADC = false;
        }

        if (isButtonPressed() || isTimeout(callBegin_ms, CALL_TIMEOUT)) {
            curState = TERM;
            termBegin_ms = millis();
        }
        else if (isTimeout(lastData_ms, IDLE_TIMEOUT))
            curState = SLEEP;

    } else if (curState == TERM) {
        if (isTimeout(termBegin_ms, TERM_TIMEOUT))
            curState = SLEEP;
    } else {
        Serial.println("???INVALID STATE???");
    }
}

uint16_t readADC(byte ch, byte *e1, byte *e2)
{
    unsigned int dataIn = 0;
    unsigned int result = 0;
    digitalWrite(ADC_SPI_CS, LOW);
    uint8_t dataOut = 0b00000001;
    dataIn = SPI.transfer(dataOut);
    dataOut = (ch == 0) ? 0b10100000 : 0b11100000;
    dataIn = SPI.transfer(dataOut);
    result = dataIn & 0x0f;
    dataIn = SPI.transfer(0x00);
    result = result << 8;

    result = result | dataIn; //12-bit value
    result = ((result << 4) & (uint16_t)0xfff0) | ((result >> 8) & (uint16_t)0x000f); //16-bit value
    
    *e1 = result >> 8;
    *e2 = result & 0x00ff;

    digitalWrite(ADC_SPI_CS, HIGH);
    return result;        
}

void setup_wifi()
{

    delay(10);
    // We start by connecting to a WiFi network
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }

    randomSeed(micros());

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
    if (strcmp(topic, "audioBell") == 0) {
        if (curState == CALL) {
            for (unsigned int i = 0; i < length; i += 2) {
                int16_t d = payload[i];
                d <<= 8;
                d |= payload[i+1]; 
                i2s_write_lr_nb(d, d);
            }
            // very important: update time of last packet received
            lastData_ms = millis();
        }
    }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "BellIot_MASTER-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
        Serial.println("connected");
        // Once connected, publish an announcement...
        client.publish("status", "connected");
        // ... and resubscribe
        client.subscribe("audioBell");
    } else {
        Serial.print("failed, rc=");
        Serial.print(client.state());
        Serial.println(" try again in 5 seconds");
        // Wait 5 seconds before retrying
        delay(5000);
    }
  }
} 

bool isTimeout(unsigned long ms, unsigned long timeout) {
    return millis() - ms >= timeout;
}

void updateButtonStatus() {
    byte tmp1, tmp2;
    prevButtonPressed = curButtonPressed;
    curButtonPressed = readADC(BUTTON_CH, &tmp1, &tmp2) > 65000;
}

bool isButtonPressed() {
    return curButtonPressed && !prevButtonPressed;
}