#include <Arduino.h>
#include <driver/i2s.h>
#include <SPI.h>
#include "PubSubClient.h"
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>

#define TIMER_INTERRUPT_DEBUG      0
#include "ESP32TimerInterrupt.h"

enum ProgramModeEnum {SETUP, OPERATE};
typedef enum ProgramModeEnum ProgramMode;
ProgramMode progMode = SETUP;

#define CONNECT_TIMEOUT 15000
IPAddress local_ip(192,168,1,1);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);
WebServer setupServer(80);

void handleRoot();
void handleSuccess();
void handleFailure();
void handleNotFound();

const char INDEX_HTML[] =
"<!DOCTYPE HTML>"
"<html>"
"<head>"
"<meta name = \"viewport\" content = \"width = device-width, initial-scale = 1.0, maximum-scale = 1.0, user-scalable=0\">"
"<title>BellioT Setup Page</title>"
"<style>"
"\"body { background-color: #808080; font-family: Arial, Helvetica, Sans-Serif; Color: #000000; }\""
"</style>"
"</head>"
"<body>"
"<h1>BellioT Setup Page</h1>"
"<form action=\"/\" method=\"post\">"
"<label for=\"ssid\">Enter SSID:</label><br>"
"<input type=\"text\" id=\"ssid\" name=\"ssid\"><br>"
"<label for=\"ssid\">Enter Password:</label><br>"
"<input type=\"text\" id=\"pw\" name=\"pw\"><br>"
"<label for=\"ssid\">Enter IP address of MQTT Server:</label><br>"
"<input type=\"text\" id=\"ip3\" name=\"ip3\">."
"<input type=\"text\" id=\"ip2\" name=\"ip2\">."
"<input type=\"text\" id=\"ip1\" name=\"ip1\">."
"<input type=\"text\" id=\"ip0\" name=\"ip0\"><br>"
"<label for=\"ssid\">Enter port of MQTT Server:</label><br>"
"<input type=\"text\" id=\"port\" name=\"port\"><br>"
"<input type=\"submit\">"
"</form>"
"</body>"
"</html>";

const char FAILURE_HTML[] =
"<!DOCTYPE HTML>"
"<html>"
"<head>"
"<title>Setup Failed</title>"
"</head>"
"<body>"
"<h3>Wrong SSID or Password.</h3>"
"</body>"
"</html>";

const char SUCCESS_HTML[] =
"<!DOCTYPE HTML>"
"<html>"
"<head>"
"<title>Setup Success</title>"
"</head>"
"<body>"
"<h3>WiFi credentials set. Please restart the device by power off then power on again.</h3>"
"<p>Note: MQTT server config is not checked.</p>"
"</body>"
"</html>";

Preferences pref;

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

uint16_t readADC(byte ch, byte *e1, byte *e2);

ESP32Timer ITimer1(1);
void IRAM_ATTR onTimer() {
    bReadADC = true;
}

// const char* ssid = "DESKTOP-H1E7PQR 5840";
// const char* password = "qazwsxedc";
// const uint16_t mqtt_port = 2883;
// IPAddress mqtt_server(192, 168, 0, 101);
String ssid = "";
String password = "";
uint16_t mqtt_port = 2883;
IPAddress mqtt_server;

WiFiClient espClient;
PubSubClient client(espClient);

void setup_wifi(); 
void callback(char* topic, byte* payload, unsigned int length); 
void reconnect(); 

#define FORCE_SETUP 0
void setup()
{   
    Serial.begin(115200);

    // first check if the device has already been configured
    
    if (FORCE_SETUP || !pref.begin("BellioT", true) || !pref.getBool("credsSet", false)) {
        progMode = SETUP;
    } else {
        progMode = OPERATE;
        ssid = pref.getString("ssid", "");
        password = pref.getString("pw", "");
        mqtt_server = IPAddress(pref.getUInt("ip"));
        mqtt_port = pref.getUShort("port");

        // Serial.println("Received configuration:");
        // Serial.println("SSID: " + ssid);
        // Serial.println("PW: " + password);
        // Serial.print("IP: "); Serial.println(mqtt_server.toString());
        // Serial.print("port: "); Serial.println(mqtt_port);
    }
    pref.end();

    if (progMode == SETUP) {

        WiFi.mode(WIFI_AP_STA);
        WiFi.softAP("BellioT Bell AP");
        WiFi.softAPConfig(local_ip, gateway, subnet);

        setupServer.on("/", handleRoot);
        setupServer.onNotFound(handleNotFound);        

        setupServer.begin();

    } else if (progMode == OPERATE) {

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
            Serial.println("Starting  ITimer1 OK");
        else
            Serial.println("Can't set ITimer1. Select another freq. or timer");

    } else {
        Serial.println("Wrong Mode");
        while (1);
    }        
}

void loop()
{
    if (progMode == SETUP) {

        setupServer.handleClient();

    } else {

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
}

void returnFail(String msg)
{
    setupServer.sendHeader("Connection", "close");
    setupServer.sendHeader("Access-Control-Allow-Origin", "*");
    setupServer.send(500, "text/plain", msg + "\r\n");
}

void handleSubmit()
{
    String t_ssid, t_pw;

    t_ssid = setupServer.arg("ssid");
    t_pw = setupServer.arg("pw");

    // Serial.println("Received configuration:");
    // Serial.println("SSID: " + value1);
    // Serial.println("PW: " + value2);

    uint32_t t_ip;
    t_ip = setupServer.arg("ip3").toInt();
    t_ip |= setupServer.arg("ip2").toInt() << 8;
    t_ip |= setupServer.arg("ip1").toInt() << 16;
    t_ip |= setupServer.arg("ip0").toInt() << 24;

    // Serial.print("IP: "); Serial.println(ip);
    // Serial.print("port: "); Serial.println((uint16_t) setupServer.arg("port").toInt());

    // try if the credentials are correct
    WiFi.begin(t_ssid.c_str(), t_pw.c_str());
    unsigned long beginConn = millis();
    while (WiFi.status() != WL_CONNECTED) {
        if (isTimeout(beginConn, CONNECT_TIMEOUT)) {
            setupServer.send(200, "text/html", FAILURE_HTML);
            return;
        }
    }

    // save configurations
    pref.begin("BellioT", false);
    pref.putString("ssid", t_ssid);
    pref.putString("pw", t_pw);
    pref.putUInt("ip", t_ip);
    pref.putUShort("port", setupServer.arg("port").toInt());
    pref.putBool("credsSet", true);
    pref.end();

    setupServer.send(200, "text/html", SUCCESS_HTML);
}

void returnOK()
{
    setupServer.sendHeader("Connection", "close");
    setupServer.sendHeader("Access-Control-Allow-Origin", "*");
    setupServer.send(200, "text/plain", "OK\r\n");
}

void handleRoot()
{
    if (setupServer.hasArg("ssid") && 
        setupServer.hasArg("pw") && 
        setupServer.hasArg("ip3") && setupServer.hasArg("ip2") && setupServer.hasArg("ip1") && setupServer.hasArg("ip0") && 
        setupServer.hasArg("port")) {

        handleSubmit();
    }
    else {
        setupServer.send(200, "text/html", INDEX_HTML);
    }
}

void handleNotFound()
{
    String message = "File Not Found\n\n";
    message += "URI: ";
    message += setupServer.uri();
    message += "\nMethod: ";
    message += (setupServer.method() == HTTP_GET)?"GET":"POST";
    message += "\nArguments: ";
    message += setupServer.args();
    message += "\n";
    for (uint8_t i=0; i<setupServer.args(); i++){
        message += " " + setupServer.argName(i) + ": " + setupServer.arg(i) + "\n";
    }
    setupServer.send(404, "text/plain", message);
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
    WiFi.begin(ssid.c_str(), password.c_str());

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