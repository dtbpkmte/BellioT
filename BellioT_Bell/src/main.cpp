#include <Arduino.h>
#include <driver/i2s.h>
#include <SPI.h>
#include "PubSubClient.h"
#include <WiFi.h>

#define TIMER_INTERRUPT_DEBUG      0
#include "ESP32TimerInterrupt.h"

#define BUTTON_CH 1
#define CALL_TIMEOUT 600000
#define IDLE_TIMEOUT 40000
#define TERM_TIMEOUT 5000

enum StateEnum {SLEEP, CALL, TERM};
typedef enum StateEnum State;

State curState = CALL;
unsigned long lastData_ms = 0;
unsigned long callBegin_ms = 0;
unsigned long termBegin_ms = 0;

bool isTimeout(unsigned long ms, unsigned long timeout);

bool prevButtonPressed = false;
bool curButtonPressed = false;
void updateButtonStatus();
bool isButtonPressed();

#define I2S_BCK  16
#define I2S_WS   15
#define I2S_DATA 13

const i2s_port_t I2S_PORT = I2S_NUM_1;

#define ADC_SPI_CS   12
#define ADC_SPI_MISO 2
#define ADC_SPI_MOSI 0
#define ADC_SPI_CLK  14
#define ADC_CH     0
#define ADC_BUFFER_LENGTH 160      //so 80 16-bit samples each publish

byte adc[ADC_BUFFER_LENGTH] = {0};
int bufferN = 0;
volatile bool bReadADC = false;

// hw_timer_t * timer = NULL;
// portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

uint16_t readADC(byte ch, byte *e1, byte *e2);

ESP32Timer ITimer1(1);
void IRAM_ATTR onTimer() {
    bReadADC = true;
}

const char* ssid = "DESKTOP-H1E7PQR 5840";
const char* password = "qazwsxedc";
const uint16_t mqtt_port = 2883;
IPAddress mqtt_server(192, 168, 0, 101);

WiFiClient espClient;
PubSubClient client(espClient);

void setup_wifi(); 
void callback(char* topic, byte* payload, unsigned int length); 
void reconnect(); 

void setup()
{   
    Serial.begin(115200);

    /* WiFi and MQTT init */
    setup_wifi();
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callback);
    if (!client.setBufferSize(ADC_BUFFER_LENGTH + 100)) {
        Serial.println("Unable to set buffer size");
    }

    /* SPI init */
    pinMode(ADC_SPI_CS,OUTPUT);
    digitalWrite(ADC_SPI_CS, HIGH);
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE3);
    SPI.begin(ADC_SPI_CLK, ADC_SPI_MISO, ADC_SPI_MOSI, ADC_SPI_CS);

    esp_err_t err;

    const i2s_config_t i2s_config = {
        .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = 8000,                         // 8KHz
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, 
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT, 
        .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,     
        .dma_buf_count = 8,                           // number of buffers
        .dma_buf_len = ADC_BUFFER_LENGTH/2,
        .use_apll = true,
        .tx_desc_auto_clear = true                              
    };

    // The pin config as per the setup
    const i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_BCK,
        .ws_io_num = I2S_WS,    
        .data_out_num = I2S_DATA, 
        .data_in_num = I2S_PIN_NO_CHANGE   
    };

    // Configuring the I2S driver and pins.
    // This function must be called before any I2S driver read/write operations.
    err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    if (err != ESP_OK) {
        Serial.printf("Failed installing driver: %d\n", err);
        while (true);
    }

    err = i2s_set_pin(I2S_PORT, &pin_config);
    if (err != ESP_OK) {
        Serial.printf("Failed setting pin: %d\n", err);
        while (true);
    }

    Serial.println("I2S driver installed.");

    if (ITimer1.attachInterrupt(8000, onTimer))
        Serial.println("Starting  ITimer1 OK, millis() = " + String(millis()));
    else
        Serial.println("Can't set ITimer1. Select another freq. or timer");
        
}

void loop()
{
    // manage MQTT connection. MUST-HAVE
    if (!client.connected()) {
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
                if (!client.publish("audioBell", adc, ADC_BUFFER_LENGTH, false)) {
                    Serial.println("Publish failed");
                }
            }
            bReadADC = false;
        }

        if (isTimeout(callBegin_ms, CALL_TIMEOUT)) {
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

void setup_wifi() {
    delay(10);
    // We start by connecting to a WiFi network
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
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
    if (strcmp(topic, "audioMaster") == 0) {
        if (curState == SLEEP) {
            curState = CALL;
            callBegin_ms = millis();
        }
        if (curState == CALL) {
            uint32_t d; size_t bw;
            for (unsigned int i = 0; i < length; i += 2) {
                d = payload[i];
                d <<= 8;
                d |= payload[i+1];
                d = (d << 16) | (d & 0xffff);
                // i2s_write(I2S_PORT, &d, 4, &bw, portMAX_DELAY);
                i2s_write(I2S_PORT, &d, 4, &bw, 100);
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
    String clientId = "BellIot_BELL-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
        Serial.println("connected");
        // Once connected, publish an announcement...
        client.publish("status", "connected");
        // ... and resubscribe
        client.subscribe("audioMaster");
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