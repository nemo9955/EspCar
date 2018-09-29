
// #define DEBUG_ESP_PORT Serial
// #define DEBUG_WIFI_MULTI
// #define DEBUG_ESP_WIFI

#include "L298N.h"
// #include <WiFiMulti.h>

#include <SPIFFS.h>
#include <FS.h>

#include <WiFiClientSecure.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

#include <WebSocketsServer.h>

#define AP_SSID "esp32car"
#define AP_PASS "pass12345"

#define ssid = "SOMNE SSID";
#define password = "ssid.pass";

#define CAR_STEP_MAX 99
#define MIN_MOTOR_SPEED 150
#define MAX_MOTOR_SPEED 256

#define LEFT_ENC_PIN 34
#define RIGHT_ENC_PIN 35

#define DISTANCE_TIMEOUT 100000
#define SEND_DATA_INTERVAL 1000

#define DISTANCE_TRIG_LEFT 23
#define DISTANCE_ECHO_LEFT 38
#define DISTANCE_TRIG_CENTER 19
#define DISTANCE_ECHO_CENTER 37
#define DISTANCE_TRIG_RIGHT 22
#define DISTANCE_ECHO_RIGHT 36

// #define ADD_ WIFIM ULTI_AP        wifiMulti.addAP("ssid_from_AP_1", "your_password_for_AP_1"); wifiMulti.addAP("ssid_from_AP_2", "your_password_for_AP_2"); wifiMulti.addAP("ssid_from_AP_3", "your_password_for_AP_3");

// WiFiMulti wifiMulti;
IPAddress local_IP(192, 168, 1, 132);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

AsyncWebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

#define MOTOR_ENB 13
#define MOTOR_IN1 12
#define MOTOR_ENA 14
#define MOTOR_IN2 27
#define MOTOR_IN3 26
#define MOTOR_IN4 25

L298N ml(MOTOR_ENA, MOTOR_IN1, MOTOR_IN2);
L298N mr(MOTOR_ENB, MOTOR_IN3, MOTOR_IN4);

bool ld;
bool rd;
int lv;
int rv;
IPAddress apIP(192, 168, 4, 1);

volatile int leftCount = 0;
volatile int rightCount = 0;
unsigned long previousMillis = 0;

float getDistance(int trig, int echo)
{
    // Clears the trig
    digitalWrite(trig, LOW);
    delayMicroseconds(5);

    // Sets the trig on HIGH state for 10 micro seconds
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);

    // Reads the echo, returns the sound wave travel time in microseconds
    int duration = pulseIn(echo, HIGH, DISTANCE_TIMEOUT);
    // Serial.print("Duration: ");
    // Serial.println(duration);

    // Calculating the distance
    float distance = duration * 0.034 / 2.0;
    // Prints the distance on the Serial Monitor
    // Serial.print("Distance: ");
    // Serial.println(distance);

    return distance;
}

void leftEncoder()
{
    leftCount++;
}

void rightEncoder()
{
    rightCount++;
}

void listDir(fs::FS &fs, const char *dirname, uint8_t levels)
{
    Serial.printf("Listing directory: %s\r\n", dirname);

    File root = fs.open(dirname);
    if (!root)
    {
        Serial.println("- failed to open directory");
        return;
    }
    if (!root.isDirectory())
    {
        Serial.println(" - not a directory");
        return;
    }

    File file = root.openNextFile();
    while (file)
    {
        if (file.isDirectory())
        {
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if (levels)
            {
                listDir(fs, file.name(), levels - 1);
            }
        }
        else
        {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("\tSIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}

void setWheels(int lw, int rw)
{
    ld = lw > 0 ? false : true;
    rd = rw > 0 ? true : false;
    lv = map(abs(lw), 0, CAR_STEP_MAX, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
    rv = map(abs(rw), 0, CAR_STEP_MAX, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);

    if (lv == MIN_MOTOR_SPEED)
        lv = 0;
    if (rv == MIN_MOTOR_SPEED)
        rv = 0;

    ml.setSpeed(lv);
    mr.setSpeed(rv);

    Serial.printf("Left  %u %u \n", ld, lv);
    Serial.printf("Right %u %u \n", rd, rv);
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length)
{

    switch (type)
    {
    case WStype_DISCONNECTED:
    {
        Serial.printf("[%u] Disconnected!\n", num);
    }
    break;
    case WStype_CONNECTED:
    {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);

        // send message to client
        webSocket.sendTXT(num, "Connected");
    }
    break;
    case WStype_TEXT:
    {
        Serial.printf("[%u] get Text: %s\n", num, payload);

        String payloadString = (const char *)payload;
        byte separator = payloadString.indexOf('_');
        String ls = payloadString.substring(0, separator);
        String rs = payloadString.substring(separator + 1);
        setWheels(ls.toInt(), rs.toInt());

        // send message to client
        // webSocket.sendTXT(num, "message here");

        // send data to all connected clients
        // webSocket.broadcastTXT("message here");
    }
    break;
    }
}

void setup()
{

    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, LOW);
    digitalWrite(MOTOR_IN3, LOW);
    digitalWrite(MOTOR_IN4, LOW);

    Serial.begin(115200);
    delay(100);

    ml.setSpeed(0);
    mr.setSpeed(0);

    if (!SPIFFS.begin())
    {
        Serial.println("SPIFFS Mount Failed");
    }
    listDir(SPIFFS, "/", 0);
    // SPIFFS.end();

    // WiFi.disconnect();
    //WiFi.disconnect(true);
    delay(100);
    delay(100);
    WiFi.setAutoConnect(true);
    WiFi.setAutoReconnect(true);
    Serial.print("ESP32 SDK: ");
    Serial.println(ESP.getSdkVersion());

    WiFi.mode(WIFI_AP);
    Serial.println("** Starting AP");
    WiFi.softAP(AP_SSID, AP_PASS);
    WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
    Serial.println("AP IP address: ");
    Serial.println(WiFi.softAPIP());

    ///////////////////////////////////////////////////

    // WiFi.mode(WIFI_AP_STA);
    //     WiFi.begin(ssid, password);
    // int i=0;
    // while (WiFi.status() != WL_CONNECTED && i<30)
    // {
    // delay(500);
    // i++;
    // Serial.println("Connecting to WiFi..");
    // }
    //  Serial.println("Connected to the WiFi network");
    //  Serial.println("IP address: ");
    //  Serial.println(WiFi.localIP());

    server.on("/hello", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "text/plain", "Hello World");
    });

    server.on("/heap", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "text/plain", String(ESP.getFreeHeap()));
    });

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(SPIFFS, "/index.html");
    });
    server.on("/virtualjoystick.js", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(SPIFFS, "/virtualjoystick.js");
    });

    server.begin();
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);

    pinMode(LEFT_ENC_PIN, INPUT);
    pinMode(RIGHT_ENC_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN), leftEncoder, HIGH);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN), rightEncoder, HIGH);

    pinMode(DISTANCE_ECHO_LEFT, INPUT);
    pinMode(DISTANCE_ECHO_CENTER, INPUT);
    pinMode(DISTANCE_ECHO_RIGHT, INPUT);
    pinMode(DISTANCE_TRIG_LEFT, OUTPUT);
    pinMode(DISTANCE_TRIG_CENTER, OUTPUT);
    pinMode(DISTANCE_TRIG_RIGHT, OUTPUT);

    digitalWrite(DISTANCE_TRIG_LEFT, LOW);
    digitalWrite(DISTANCE_TRIG_CENTER, LOW);
    digitalWrite(DISTANCE_TRIG_RIGHT, LOW);
    ml.setSpeed(255);
    mr.setSpeed(255);

    // ADD_WIFIMULTI_AP
    // if (!WiFi.config(local_IP, gateway, subnet))
    //     Serial.println("STA Failed to configure");
    // Serial.println("Connecting Wifi...");
    // if (wifiMulti.run() == WL_CONNECTED)
    // {
    //     Serial.println("");
    //     Serial.println("WiFi connected");
    //     Serial.println("IP address: ");
    //     Serial.println(WiFi.localIP());
    // }
}
bool did = true;
void loop()
{
    if (did)
    {
        Serial.print("STARTED LOOP!!");
        did = false;
    }

    webSocket.loop();

    if (lv == 0)
        ml.stop();
    else if (ld)
        ml.backward();
    else
        ml.forward();
    // ml.setmotor(ld ? _CW : _CCW, lv);

    if (rv == 0)
        mr.stop();
    else if (rd)
        mr.backward();
    else
        mr.forward();
    // ml.setmotor(ld ? _CW : _CCW, lv);

    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= SEND_DATA_INTERVAL)
    {
        previousMillis = currentMillis;

        float disL = getDistance(DISTANCE_TRIG_LEFT, DISTANCE_ECHO_LEFT);
        float disC = getDistance(DISTANCE_TRIG_CENTER, DISTANCE_ECHO_CENTER);
        float disR = getDistance(DISTANCE_TRIG_RIGHT, DISTANCE_ECHO_RIGHT);

        // Serial.println("disL\tdisC\tdisR");
        // Serial.printf("%f\t%f\t%f\n", disL, disC, disR);
        String dist = "D_";
        dist += disL;
        dist += "_";
        dist += disC;
        dist += "_";
        dist += disR;
        // Serial.println(dist);
        webSocket.broadcastTXT(dist);

        // Serial.println("Encoder left right ");
        // Serial.printf("\t%d\t%d\n", leftCount, rightCount);
        String odom = "O_";
        odom += leftCount;
        odom += "_";
        odom += rightCount;
        // Serial.println(odom);
        webSocket.broadcastTXT(odom);

        rightCount = 0;
        leftCount = 0;

        // if (wifiMulti.run() != WL_CONNECTED)
        // {
        //     Serial.println("WiFi not connected!");
        // }
    }
}
