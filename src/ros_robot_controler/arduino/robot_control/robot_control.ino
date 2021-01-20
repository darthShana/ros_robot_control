/******************************************************************************
MQTT_Light_Example.ino
Example for controlling a light using MQTT
by: Alex Wende, SparkFun Electronics

This sketch connects the ESP8266 to a MQTT broker and subcribes to the topic
room/light. When "on" is recieved, the pin LIGHT_PIN is toggled HIGH.
When "off" is recieved, the pin LIGHT_PIN is toggled LOW.
******************************************************************************/

#include <WiFi.h>
#include <Servo.h>
#include <ArduinoJson.h>
#include <WebServer.h>
#include <Adafruit_GPS.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>

Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;
#include "LSM6DS_LIS3MDL.h"  // can adjust to LSM6DS33, LSM6DS3U, LSM6DSOX...

Adafruit_Mahony filter;  // fastest/smalleset

Adafruit_Sensor_Calibration_EEPROM cal;
#define FILTER_UPDATE_RATE_HZ 100

const char *ssid     = "ESP32-Access-Point";
const char *password = "Yztt1181!"; // password of the WiFi network
const char* mqtt_server = "192.168.4.2";

WiFiClient wclient;
WebServer server(80);

Servo steering_servo;  // create servo object to control a servo
Servo thrust_servo;  // create servo object to control a servo

#define GPSSerial Serial1
Adafruit_GPS GPS(&GPSSerial);
uint32_t timer_gps = millis();
uint32_t timer_imu = millis();

PubSubClient client(wclient);

// Serving Hello world
void cmd_vel() {
  
  String postBody = server.arg("plain");
  DynamicJsonDocument doc(512);
  DeserializationError error = deserializeJson(doc, postBody);

  double velocity = doc["velocity"]; 
  double steering = doc["steering"]; 
  Serial.print("velocity: ");
  Serial.print(velocity);
  Serial.print(" steering: ");
  Serial.println(steering);

  double vel = 1500 + (500 * velocity);
  
  steering_servo.write(90+(steering*2));          // tell servo to go to position in variable 'pos'
  thrust_servo.writeMicroseconds(vel);
  delay(15);                                      // waits 15ms for the servo to reach the position
  server.send(201, F("application/json"), "");
}
 
// Define routing
void restServerRouting() {
    server.on("/", HTTP_GET, []() {
        server.send(200, F("text/html"),
            F("Welcome to the REST Web Server"));
    });
    server.on(F("/cmdVel"), HTTP_POST, cmd_vel);
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.println("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void WiFiStationConnected(WiFiEvent_t event, WiFiEventInfo_t info){
  Serial.println("Station connected");
  for(int i = 0; i< 6; i++){
    Serial.printf("%02X", info.sta_connected.mac[i]);  
    if(i<5)Serial.print(":");
  }
  Serial.println();
}
  
// Connect to WiFi network
void setup_wifi() {
  Serial.print("Setting AP (Access Point)…");
  Serial.println(ssid);

  WiFi.softAP(ssid, password);

  IPAddress IP = WiFi.softAPIP();
  WiFi.onEvent(WiFiStationConnected, SYSTEM_EVENT_AP_STACONNECTED);

  Serial.print("AP IP address: ");
  Serial.println(IP);

}

//read GPS
void read_gps() {
  char c = GPS.read();

  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }
  
  if (millis() - timer_gps > 1000) {
    if (GPS.fix) {
      timer_gps = millis(); // reset the timer
      
      StaticJsonDocument<256> doc;
      doc["latitude"] = String(GPS.latitude, 4);
      doc["lat"] = String(GPS.lat);
      doc["longitude"] = String(GPS.longitude, 4);
      doc["lon"] = String(GPS.lon);      
      doc["altitude"] = GPS.altitude;
      
      String output;
      serializeJson(doc, output);
      char copy[256];
      output.toCharArray(copy, 256);
      client.publish("sensors/gps", copy);
    }
  }
  
}

//read IMU
void read_imu() {
  StaticJsonDocument<256> doc;
  float gx, gy, gz;
  static uint8_t counter = 0;

  if ((millis() - timer_imu) < (1000 / FILTER_UPDATE_RATE_HZ)) {
    return;
  }
  timer_imu = millis();
  // Read the motion sensors
  sensors_event_t accel, gyro, mag;
  accelerometer->getEvent(&accel);
  gyroscope->getEvent(&gyro);
  magnetometer->getEvent(&mag);

  cal.calibrate(mag);
  cal.calibrate(accel);
  cal.calibrate(gyro);
  // Gyroscope needs to be converted from Rad/s to Degree/s
  // the rest are not unit-important
  gx = gyro.gyro.x * SENSORS_RADS_TO_DPS;
  gy = gyro.gyro.y * SENSORS_RADS_TO_DPS;
  gz = gyro.gyro.z * SENSORS_RADS_TO_DPS;

  // Update the SensorFusion filter
  filter.update(gx, gy, gz, 
                accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, 
                mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);


  doc["a.x"] = String(accel.acceleration.x, 4);
  doc["a.y"] = String(accel.acceleration.y, 4);
  doc["a.z"] = String(accel.acceleration.z, 4);

  doc["v.x"] = String(gyro.gyro.x, 4);
  doc["v.y"] = String(gyro.gyro.y, 4);
  doc["v.z"] = String(gyro.gyro.z, 4);

  float qw, qx, qy, qz;
  filter.getQuaternion(&qw, &qx, &qy, &qz);
  doc["o.w"] = String(qw, 4);
  doc["o.x"] = String(qx, 4);
  doc["o.y"] = String(qy, 4);
  doc["o.z"] = String(qz, 4);

  String output;
  serializeJson(doc, output);
  char copy[256];
  output.toCharArray(copy, 256);
  client.publish("sensors/imu", copy);

}

void setup() {
  while (!Serial);  // uncomment to have the sketch wait until Serial is ready

  Serial.begin(115200); // Start serial communication at 115200 baud
  delay(100);
  setup_wifi(); // Connect to network

  client.setServer(mqtt_server, 1883);
  
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
  GPSSerial.println(PMTK_Q_RELEASE);

  if (!cal.begin()) {
    Serial.println("Failed to initialize calibration helper");
  } else if (! cal.loadCalibration()) {
    Serial.println("No calibration loaded/found");
  }
  if (!init_sensors()) {
    Serial.println("Failed to find sensors");
    while (1) delay(10);
  }
  accelerometer->printSensorDetails();
  gyroscope->printSensorDetails();
  magnetometer->printSensorDetails();
  setup_sensors();
  filter.begin(FILTER_UPDATE_RATE_HZ);
  timer_imu = millis();
  Wire.setClock(400000); // 400KHz
  
  steering_servo.attach(27);  // attaches the servo on pin 9 to the servo object
  thrust_servo.attach(33);  // attaches the servo on pin 9 to the servo object

  steering_servo.write(90);              // tell servo to go to position in variable 'pos'
  thrust_servo.writeMicroseconds(1500);

  // Set server routing
  restServerRouting();
  // Start server
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
  read_gps();
  read_imu();
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}
