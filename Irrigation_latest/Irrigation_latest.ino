#include <WiFi.h>
#include <PubSubClient.h>
#include <RTClib.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>
#include <SPI.h>
#include <SD.h>
#include <SoftwareSerial.h>
#include "PCF8574.h"

#define LED_BUILTIN 12
#define ONE_WIRE_BUS 14  // GPIO13 for DS18B20 data pin
#define DHT_PIN 13       // GPIO12 for DHT21 data pin
#define AOUT_PIN_1 36
#define AOUT_PIN_2 39
#define Valve_1 25
#define Valve_2 33
#define Motor_1 13

PCF8574 IO_0(0x20);//IO_Expander for relay_1

SoftwareSerial mySerial(17, 16);  // Assuming you're using pins  (RX) and  (TX) for SoftwareSerial

const byte nitrogenAddress[] = {0x01, 0x03, 0x00, 0x1E, 0x00, 0x01, 0xE4, 0x0C};
const byte phosphorusAddress[] = {0x01, 0x03, 0x00, 0x1F, 0x00, 0x01, 0xB5, 0xCC};
const byte potassiumAddress[] = {0x01, 0x03, 0x00, 0x20, 0x00, 0x01, 0x85, 0xC0};
const byte pHAddress[] = {0x01, 0x03, 0x00, 0x06, 0x00, 0x01, 0x64, 0x0B};
const byte eCAddress[] = {0x01, 0x03, 0x00, 0x15, 0x00, 0x01, 0x95, 0xCE};
const byte tempAddress[] = {0x01, 0x03, 0x00, 0x13, 0x00, 0x01, 0x75, 0xCF};
const byte humidityAddress[] = {0x01, 0x03, 0x00, 0x12, 0x00, 0x01, 0x24, 0x0F};

byte values[8];

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

DHT dht(DHT_PIN, DHT21);  // Change to DHT22 if using DHT22

// RTC_DS1307 rtc;
RTC_DS3231 rtc;
// Define the SD card chip select pin
const int chipSelect = 5; // Change this to your chosen CS pin

unsigned long previousMillis_1 = 0;
unsigned long previousMillis_2 = 0;
unsigned long previousMillis_3 = 0;
unsigned long previousMillis_4 = 0;
unsigned long previousMillis_5 = 0;

bool codeEnabled = false;     // Flag to control whether to execute the code

float dhtTempC;
float ds18b20TempC;
float humidity;

const int EEPROM_ADDRESS_SENSOR1 = 0;
const int EEPROM_ADDRESS_SENSOR2 = EEPROM_ADDRESS_SENSOR1 + sizeof(float) * 4; 
const int EEPROM_ADDRESS_SENSOR3 = EEPROM_ADDRESS_SENSOR2 + sizeof(float) * 4; 
const int EEPROM_ADDRESS_SENSOR4 = EEPROM_ADDRESS_SENSOR3 + sizeof(float) * 4; 
const int EEPROM_ADDRESS_SENSOR5 = EEPROM_ADDRESS_SENSOR4 + sizeof(float) * 4; 
const int EEPROM_ADDRESS_SENSOR6 = EEPROM_ADDRESS_SENSOR5 + sizeof(float) * 4; // Adjust based on the number of values

const int EEPROM_ADDRESS_SENSOR7 = EEPROM_ADDRESS_SENSOR6 + sizeof(float) * 4; 
const int EEPROM_ADDRESS_SENSOR8 = EEPROM_ADDRESS_SENSOR7 + sizeof(float) * 4; 
const int EEPROM_ADDRESS_SENSOR9 = EEPROM_ADDRESS_SENSOR8 + sizeof(float) * 4; 
const int EEPROM_ADDRESS_SENSOR10 = EEPROM_ADDRESS_SENSOR9 + sizeof(float) * 4; 
const int EEPROM_ADDRESS_SENSOR11 = EEPROM_ADDRESS_SENSOR10 + sizeof(float) * 4; 

const int EEPROM_ADDRESS_SENSOR12 = EEPROM_ADDRESS_SENSOR11 + sizeof(float) * 4; 
const int EEPROM_ADDRESS_SENSOR13 = EEPROM_ADDRESS_SENSOR12 + sizeof(float) * 4; 
const int EEPROM_ADDRESS_SENSOR14 = EEPROM_ADDRESS_SENSOR13 + sizeof(float) * 4; 
const int EEPROM_ADDRESS_SENSOR15 = EEPROM_ADDRESS_SENSOR14 + sizeof(float) * 4;
const int EEPROM_ADDRESS_SENSOR16 = EEPROM_ADDRESS_SENSOR15 + sizeof(float) * 4; 
const int EEPROM_ADDRESS_SENSOR17 = EEPROM_ADDRESS_SENSOR16 + sizeof(float) * 4; 
const int EEPROM_ADDRESS_SENSOR18 = EEPROM_ADDRESS_SENSOR17 + sizeof(float) * 4; 

float retrievedInputLow, retrievedInputHigh, retrievedOutputLow, retrievedOutputHigh;
float retrievedLowLimit, retrievedHighLimit, retrievedValveStatus;
char readValveString[3];

float soilmoisturepercent_01,soilmoisturepercent_02;
const int numReadings = 20; // Number of readings to take for smoothing
int readings1[numReadings]; // Array to store the readings for sensor 1
int readings2[numReadings]; // Array to store the readings for sensor 2
int currentIndex = 0;        // Index for the current reading
int total1 = 0;              // Running total of readings for smoothing for sensor 1
int total2 = 0;              // Running total of readings for smoothing for sensor 2

String getMacAddress() {
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  String macAddress = "";
  for (int i = 0; i < 6; i++) {
    macAddress += String(mac[i], HEX);
    if (i < 5) macAddress += ":";
  }
  return macAddress;
}
String macAddress = getMacAddress();
const char* clientId = macAddress.c_str();

const char* ssid = "";
const char* password = "";
const char* mqtt_server = "";
const int mqtt_port = 1883;
const char* mqtt_topic = "esp32";

WiFiClient espClient;
PubSubClient client(espClient);

TaskHandle_t mqttTask;
TaskHandle_t npkTask;

void connectToWiFi() {
  Serial.println("Connecting to WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(ONE_WIRE_BUS, INPUT);
  pinMode(DHT_PIN, INPUT);
  pinMode(AOUT_PIN_1,INPUT);
  pinMode(AOUT_PIN_2,INPUT);
  IO_0.pinMode(P0, OUTPUT);//Valve-1
  IO_0.pinMode(P1, OUTPUT);//Valve-2
  IO_0.pinMode(P2, OUTPUT);//Motor-1
  IO_0.digitalWrite(P0,LOW);
  IO_0.digitalWrite(P1,LOW);
  IO_0.digitalWrite(P2,LOW);
  // Connect to WiFi
  connectToWiFi();

  // Set up MQTT client
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  // Create MQTT task
  xTaskCreatePinnedToCore(
    mqttTaskFunction,    // function to execute
    "mqttTask",          // task name
    8192,                // stack size
    NULL,                // parameter to pass to the function
    1,                   // task priority
    &mqttTask,           // task handle
    0                    // core (0 or 1)
  );
  xTaskCreatePinnedToCore (npkTaskFuction,"NPK Task",8192,NULL,1,&npkTask,1);

  EEPROM.begin(512); //Initialasing EEPROM
  dht.begin();
  sensors.begin();
  rtc.begin();
  mySerial.begin(9600);

  // Uncomment the following line to set the RTC to the specified date and time
  // rtc.adjust(DateTime(2023, 12, 22, 11, 51, 50));
  
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  // Initialize the SD card
  if (!SD.begin(chipSelect)) {
    Serial.println("SD card initialization failed!");
    return;
  }

  DateTime now = rtc.now();
  String time = String(now.hour())+":"+String(now.minute())+":"+String(now.second());
  String fileName = "/Rain_Fall_Data("+String(now.year()) + "-" + String(now.month()) + "-" + String(now.day()) + ").txt";

  // Open the file in "append" mode
  File file = SD.open(fileName, FILE_APPEND);
  if (!file) {
    Serial.println("Error opening file for appending.");
  } else {
    Serial.println("Appending data to the file...");
  }
}

struct ScheduleData {
  char StartTime[10]; 
  char EndTime[10]; 
  char valveName[10];
  int valveStatus;
  char day[30];
};

void callback(char* topic, byte* payload, unsigned int length) {
  String macAddress = getMacAddress();
  String value = "";
  for (int i = 0; i < length; i++) {
    value += (char)payload[i];
  }
  if (strcmp(topic, ("Device/" + macAddress + "/soilSensor01/Settings").c_str()) == 0) {
    int lowLimitIndex = value.indexOf("lowLimit:") + 9;String lowLimit = value.substring(lowLimitIndex, value.indexOf("highLimit:"));float lowLimitValue = lowLimit.toFloat();
    int highLimitIndex = value.indexOf("highLimit:") + 10;String highLimit = value.substring(highLimitIndex, value.indexOf("ValveNo:"));float highLimitValue = highLimit.toFloat();
    int valveNoIndex = value.indexOf("ValveNo:") + 8;String valveNo = value.substring(valveNoIndex, value.indexOf("ValveStatus:"));char valveString[3];valveNo.toCharArray(valveString, sizeof(valveString));
    int valveStatusIndex = value.indexOf("ValveStatus:") + 12;String valveStatus = value.substring(valveStatusIndex);float valveStatusValue = valveStatus.toFloat();

    // Print individual values
    Serial.println("Low Limit: " + lowLimit);
    Serial.println("High Limit: " + highLimit);
    Serial.println("Valve No: " + valveNo);
    Serial.println("Valve Status: " + valveStatus);

    EEPROM.put(EEPROM_ADDRESS_SENSOR11, lowLimitValue);
    EEPROM.put(EEPROM_ADDRESS_SENSOR11 + sizeof(float), highLimitValue);
    EEPROM.put(EEPROM_ADDRESS_SENSOR11 + sizeof(float) * 2, valveString);
    EEPROM.put(EEPROM_ADDRESS_SENSOR11 + sizeof(float) * 3, valveStatusValue);
    EEPROM.commit();
  }
  if (strcmp(topic, ("Device/" + macAddress + "/soilSensor02/Settings").c_str()) == 0) {
    int lowLimitIndex = value.indexOf("lowLimit:") + 9;String lowLimit = value.substring(lowLimitIndex, value.indexOf("highLimit:"));float lowLimitValue = lowLimit.toFloat();
    int highLimitIndex = value.indexOf("highLimit:") + 10;String highLimit = value.substring(highLimitIndex, value.indexOf("ValveNo:"));float highLimitValue = highLimit.toFloat();
    int valveNoIndex = value.indexOf("ValveNo:") + 8;String valveNo = value.substring(valveNoIndex, value.indexOf("ValveStatus:"));char valveString[3];valveNo.toCharArray(valveString, sizeof(valveString));
    int valveStatusIndex = value.indexOf("ValveStatus:") + 12;String valveStatus = value.substring(valveStatusIndex);float valveStatusValue = valveStatus.toFloat();
    
    // Print individual values
    Serial.println("Low Limit: " + lowLimit);
    Serial.println("High Limit: " + highLimit);
    Serial.println("Valve No: " + valveNo);
    Serial.println("Valve Status: " + valveStatus);

    EEPROM.put(EEPROM_ADDRESS_SENSOR12, lowLimitValue);
    EEPROM.put(EEPROM_ADDRESS_SENSOR12 + sizeof(float), highLimitValue);
    EEPROM.put(EEPROM_ADDRESS_SENSOR12 + sizeof(float) * 2, valveString);
    EEPROM.put(EEPROM_ADDRESS_SENSOR12 + sizeof(float) * 3, valveStatusValue);
    EEPROM.commit();
  }
  if (strcmp(topic, ("Device/" + macAddress + "/tempSensor/Settings").c_str()) == 0) {
    int lowLimitIndex = value.indexOf("lowLimit:") + 9;String lowLimit = value.substring(lowLimitIndex, value.indexOf("highLimit:"));float lowLimitValue = lowLimit.toFloat();
    int highLimitIndex = value.indexOf("highLimit:") + 10;String highLimit = value.substring(highLimitIndex, value.indexOf("ValveNo:"));float highLimitValue = highLimit.toFloat();
    int valveNoIndex = value.indexOf("ValveNo:") + 8;String valveNo = value.substring(valveNoIndex, value.indexOf("ValveStatus:"));char valveString[3];valveNo.toCharArray(valveString, sizeof(valveString));
    int valveStatusIndex = value.indexOf("ValveStatus:") + 12;String valveStatus = value.substring(valveStatusIndex);float valveStatusValue = valveStatus.toFloat();
    
    // Print individual values
    Serial.println("Low Limit: " + lowLimit);
    Serial.println("High Limit: " + highLimit);
    Serial.println("Valve No: " + valveNo);
    Serial.println("Valve Status: " + valveStatus);

    EEPROM.put(EEPROM_ADDRESS_SENSOR13, lowLimitValue);
    EEPROM.put(EEPROM_ADDRESS_SENSOR13 + sizeof(float), highLimitValue);
    EEPROM.put(EEPROM_ADDRESS_SENSOR13 + sizeof(float) * 2, valveString);
    EEPROM.put(EEPROM_ADDRESS_SENSOR13 + sizeof(float) * 3, valveStatusValue);
    EEPROM.commit();
  }
  if (strcmp(topic, ("Device/" + macAddress + "/humiditySensor/Settings").c_str()) == 0) {
    int lowLimitIndex = value.indexOf("lowLimit:") + 9;String lowLimit = value.substring(lowLimitIndex, value.indexOf("highLimit:"));float lowLimitValue = lowLimit.toFloat();
    int highLimitIndex = value.indexOf("highLimit:") + 10;String highLimit = value.substring(highLimitIndex, value.indexOf("ValveNo:"));float highLimitValue = highLimit.toFloat();
    int valveNoIndex = value.indexOf("ValveNo:") + 8;String valveNo = value.substring(valveNoIndex, value.indexOf("ValveStatus:"));char valveString[3];valveNo.toCharArray(valveString, sizeof(valveString));
    int valveStatusIndex = value.indexOf("ValveStatus:") + 12;String valveStatus = value.substring(valveStatusIndex);float valveStatusValue = valveStatus.toFloat();
    
    // Print individual values
    Serial.println("Low Limit: " + lowLimit);
    Serial.println("High Limit: " + highLimit);
    Serial.println("Valve No: " + valveNo);
    Serial.println("Valve Status: " + valveStatus);

    EEPROM.put(EEPROM_ADDRESS_SENSOR17, lowLimitValue);
    EEPROM.put(EEPROM_ADDRESS_SENSOR17 + sizeof(float), highLimitValue);
    EEPROM.put(EEPROM_ADDRESS_SENSOR17 + sizeof(float) * 2, valveString);
    EEPROM.put(EEPROM_ADDRESS_SENSOR17 + sizeof(float) * 3, valveStatusValue);
    EEPROM.commit();
  }
  if (strcmp(topic, ("Device/" + macAddress + "/SoilTemp/Settings").c_str()) == 0) {
    int lowLimitIndex = value.indexOf("lowLimit:") + 9;String lowLimit = value.substring(lowLimitIndex, value.indexOf("highLimit:"));float lowLimitValue = lowLimit.toFloat();
    int highLimitIndex = value.indexOf("highLimit:") + 10;String highLimit = value.substring(highLimitIndex, value.indexOf("ValveNo:"));float highLimitValue = highLimit.toFloat();
    int valveNoIndex = value.indexOf("ValveNo:") + 8;String valveNo = value.substring(valveNoIndex, value.indexOf("ValveStatus:"));char valveString[3];valveNo.toCharArray(valveString, sizeof(valveString));
    int valveStatusIndex = value.indexOf("ValveStatus:") + 12;String valveStatus = value.substring(valveStatusIndex);float valveStatusValue = valveStatus.toFloat();
    
    // Print individual values
    Serial.println("Low Limit: " + lowLimit);
    Serial.println("High Limit: " + highLimit);
    Serial.println("Valve No: " + valveNo);
    Serial.println("Valve Status: " + valveStatus);

    EEPROM.put(EEPROM_ADDRESS_SENSOR15, lowLimitValue);
    EEPROM.put(EEPROM_ADDRESS_SENSOR15 + sizeof(float), highLimitValue);
    EEPROM.put(EEPROM_ADDRESS_SENSOR15 + sizeof(float) * 2, valveString);
    EEPROM.put(EEPROM_ADDRESS_SENSOR15 + sizeof(float) * 3, valveStatusValue);
    EEPROM.commit();
  }

  if (strcmp(topic, ("Device/" + macAddress + "/soilSensor01/Calibration").c_str()) == 0) {
    float calibrationInputLow = value.substring(value.indexOf("inputLowValue:") + 14, value.indexOf("inputHighValue:")).toFloat();
    float calibrationInputHigh = value.substring(value.indexOf("inputHighValue:") + 15, value.indexOf("outputLowValue:")).toFloat();
    float calibrationOutputLow = value.substring(value.indexOf("outputLowValue:") + 14, value.indexOf("outputHighValue:")).toFloat();
    float calibrationOutputHigh = value.substring(value.indexOf("outputHighValue") + 15).toFloat();  // Adjusted this line

    Serial.println("soilSensor01 Calibration Values");
    Serial.print("inputLowValue: ");Serial.print(calibrationInputLow);Serial.print("\tinputHighValue: ");Serial.print(calibrationInputHigh);
    Serial.print("\toutputLowValue: ");Serial.print(calibrationOutputLow);Serial.print("\toutputHighValue: ");Serial.println(calibrationOutputHigh);

    // Store calibration values in EEPROM with specific addresses
    EEPROM.put(EEPROM_ADDRESS_SENSOR1, calibrationInputLow);
    EEPROM.put(EEPROM_ADDRESS_SENSOR1 + sizeof(float), calibrationInputHigh);
    EEPROM.put(EEPROM_ADDRESS_SENSOR1 + sizeof(float) * 2, calibrationOutputLow);
    EEPROM.put(EEPROM_ADDRESS_SENSOR1 + sizeof(float) * 3, calibrationOutputHigh);
    EEPROM.commit();
  }
  if (strcmp(topic, ("Device/" + macAddress + "/soilSensor02/Calibration").c_str()) == 0) {
    float calibrationInputLow = value.substring(value.indexOf("inputLowValue:") + 14, value.indexOf("inputHighValue:")).toFloat();
    float calibrationInputHigh = value.substring(value.indexOf("inputHighValue:") + 15, value.indexOf("outputLowValue:")).toFloat();
    float calibrationOutputLow = value.substring(value.indexOf("outputLowValue:") + 14, value.indexOf("outputHighValue:")).toFloat();
    float calibrationOutputHigh = value.substring(value.indexOf("outputHighValue") + 15).toFloat();  // Adjusted this line

    Serial.println("soilSensor02 Calibration Values");
    Serial.print("inputLowValue: ");Serial.print(calibrationInputLow);Serial.print("\tinputHighValue: ");Serial.print(calibrationInputHigh);
    Serial.print("\toutputLowValue: ");Serial.print(calibrationOutputLow);Serial.print("\toutputHighValue: ");Serial.println(calibrationOutputHigh);

    // Store calibration values in EEPROM with specific addresses
    EEPROM.put(EEPROM_ADDRESS_SENSOR2, calibrationInputLow);
    EEPROM.put(EEPROM_ADDRESS_SENSOR2 + sizeof(float), calibrationInputHigh);
    EEPROM.put(EEPROM_ADDRESS_SENSOR2 + sizeof(float) * 2, calibrationOutputLow);
    EEPROM.put(EEPROM_ADDRESS_SENSOR2 + sizeof(float) * 3, calibrationOutputHigh);
    EEPROM.commit();
  }

  if (strcmp(topic, ("Device/" + macAddress + "/humiditySensor/Calibration").c_str()) == 0){
    Serial.println("Calibration Reading For Humidty Sensor");
    float calibrationInputLow = value.substring(value.indexOf("inputLowValue:") + 14, value.indexOf("inputHighValue:")).toFloat();
    float calibrationInputHigh = value.substring(value.indexOf("inputHighValue:") + 15, value.indexOf("outputLowValue:")).toFloat();
    float calibrationOutputLow = value.substring(value.indexOf("outputLowValue:") + 14, value.indexOf("outputHighValue:")).toFloat();
    float calibrationOutputHigh = value.substring(value.indexOf("outputHighValue") + 15).toFloat();  // Adjusted this line

    Serial.println("Calibration Values");
    Serial.print("inputLowValue: ");Serial.print(calibrationInputLow);Serial.print("\tinputHighValue: ");Serial.print(calibrationInputHigh);
    Serial.print("\toutputLowValue: ");Serial.print(calibrationOutputLow);Serial.print("\toutputHighValue: ");Serial.println(calibrationOutputHigh);

    // Store calibration values in EEPROM with specific addresses
    EEPROM.put(EEPROM_ADDRESS_SENSOR9, calibrationInputLow);
    EEPROM.put(EEPROM_ADDRESS_SENSOR9 + sizeof(float), calibrationInputHigh);
    EEPROM.put(EEPROM_ADDRESS_SENSOR9 + sizeof(float) * 2, calibrationOutputLow);
    EEPROM.put(EEPROM_ADDRESS_SENSOR9 + sizeof(float) * 3, calibrationOutputHigh);
    EEPROM.commit();
  }
  if (strcmp(topic, ("Device/" + macAddress + "/tempSensor/Calibration").c_str()) == 0){
    Serial.println("Calibration Reading For Temp Sensor");
    float calibrationInputLow = value.substring(value.indexOf("inputLowValue:") + 14, value.indexOf("inputHighValue:")).toFloat();
    float calibrationInputHigh = value.substring(value.indexOf("inputHighValue:") + 15, value.indexOf("outputLowValue:")).toFloat();
    float calibrationOutputLow = value.substring(value.indexOf("outputLowValue:") + 14, value.indexOf("outputHighValue:")).toFloat();
    float calibrationOutputHigh = value.substring(value.indexOf("outputHighValue") + 15).toFloat();  // Adjusted this line

    Serial.println("Calibration Values");
    Serial.print("inputLowValue: ");Serial.print(calibrationInputLow);Serial.print("\tinputHighValue: ");Serial.print(calibrationInputHigh);
    Serial.print("\toutputLowValue: ");Serial.print(calibrationOutputLow);Serial.print("\toutputHighValue: ");Serial.println(calibrationOutputHigh);

    // Store calibration values in EEPROM with specific addresses
    EEPROM.put(EEPROM_ADDRESS_SENSOR10, calibrationInputLow);
    EEPROM.put(EEPROM_ADDRESS_SENSOR10 + sizeof(float), calibrationInputHigh);
    EEPROM.put(EEPROM_ADDRESS_SENSOR10 + sizeof(float) * 2, calibrationOutputLow);
    EEPROM.put(EEPROM_ADDRESS_SENSOR10 + sizeof(float) * 3, calibrationOutputHigh);
    EEPROM.commit();
  }
  if (strcmp(topic, ("Device/" + macAddress + "/Valve_1").c_str()) == 0){
    if(value=="1"){IO_0.digitalWrite(P0,HIGH);}
    if(value=="0"){IO_0.digitalWrite(P0,LOW);}
  }
  if (strcmp(topic, ("Device/" + macAddress + "/Valve_2").c_str()) == 0){
    if(value=="1"){IO_0.digitalWrite(P1,HIGH);}
    if(value=="0"){IO_0.digitalWrite(P1,LOW);}
  }
  if (strcmp(topic, ("Device/" + macAddress + "/Motor_1").c_str()) == 0){
    if(value=="1"){IO_0.digitalWrite(P2,HIGH);}
    if(value=="0"){IO_0.digitalWrite(P2,LOW);}
  }
  if (strcmp(topic, ("Device/" + macAddress + "/Timer").c_str()) == 0){
    ScheduleData data;
  
    // Create a non-const copy of the string
    char payloadCopy[length + 1];
    strncpy(payloadCopy, (char*)payload, length);
    payloadCopy[length] = '\0';
    
    // Split the payload into tokens based on the delimiter "/"
    char* token = strtok(payloadCopy, "/");
    
    // Extract Time Interval
    if (token != NULL) {
      strcpy(data.StartTime, token);
      token = strtok(NULL, "/");
    }
    if (token != NULL) {
      strcpy(data.EndTime, token);
      token = strtok(NULL, "/");
    }
    
    // Extract Valve Name
    if (token != NULL) {
      strcpy(data.valveName, token);
      token = strtok(NULL, "/");
    }
    
    // Extract Valve Status
    if (token != NULL) {
      data.valveStatus = atoi(token);
      token = strtok(NULL, "/");
    }
    
    // Extract Day
    if (token != NULL) {
      strcpy(data.day, token);
    }
    
    // Print the data to Serial Monitor
    Serial.print("Start Time : ");
    Serial.println(data.StartTime);
    Serial.print("End Time : ");
    Serial.println(data.EndTime);
    Serial.print("Valve Name: ");
    Serial.println(data.valveName);
    Serial.print("Valve Status: ");
    Serial.println(data.valveStatus);
    Serial.print("Day: ");
    Serial.println(data.day);

    EEPROM.put(EEPROM_ADDRESS_SENSOR16, data.StartTime);
    EEPROM.put(EEPROM_ADDRESS_SENSOR16 + sizeof(data.StartTime), data.EndTime);
    EEPROM.put(EEPROM_ADDRESS_SENSOR16 + 2 * sizeof(data.StartTime), data.valveName);
    EEPROM.put(EEPROM_ADDRESS_SENSOR16 + 2 * sizeof(data.StartTime) + sizeof(data.valveName), data.valveStatus);
    EEPROM.put(EEPROM_ADDRESS_SENSOR16 + 2 * sizeof(data.StartTime) + sizeof(data.valveName) + sizeof(data.valveStatus), data.day);
    EEPROM.commit();
  }
  if (strcmp(topic, ("Device/" + macAddress + "/RestartBtn").c_str()) == 0){
    esp_restart();
  }
  if (strcmp(topic, ("Device/" + macAddress + "/Mode").c_str()) == 0){
    if(value=="Auto")
    {
      Serial.println(value);
      bool mode = true;
      EEPROM.put(EEPROM_ADDRESS_SENSOR18, mode);
      EEPROM.commit();
    } else{
      Serial.println(value);
      bool mode = false;
      EEPROM.put(EEPROM_ADDRESS_SENSOR18, mode);
      EEPROM.commit();
    }
  }
}

void loop() {
  String macAddress = getMacAddress();

  unsigned long currentMillis_1 = millis();
  if (currentMillis_1 - previousMillis_1 >= 1000) {
    previousMillis_1 = currentMillis_1;

    DateTime now = rtc.now();
    String time = String(now.hour())+":"+String(now.minute())+":"+String(now.second());
    String date = String(now.year())+"-"+String(now.month())+"-"+String(now.day());
    // Serial.print("Date: ");Serial.print(date);Serial.print("\tTime: ");Serial.println(time);Serial.println();
    sensors.requestTemperatures();
    ds18b20TempC = sensors.getTempCByIndex(0); // Index 0 for a single sensor

    EEPROM.get(EEPROM_ADDRESS_SENSOR10, retrievedInputLow);
    EEPROM.get(EEPROM_ADDRESS_SENSOR10 + sizeof(float), retrievedInputHigh);
    EEPROM.get(EEPROM_ADDRESS_SENSOR10 + sizeof(float) * 2, retrievedOutputLow);
    EEPROM.get(EEPROM_ADDRESS_SENSOR10 + sizeof(float) * 3, retrievedOutputHigh);
    // Serial.println("EEPROM Saved Temperature Calibration Values");
    // Serial.print("Input Low: ");Serial.print(retrievedInputLow);Serial.print("\tInput High: ");Serial.print(retrievedInputHigh);
    // Serial.print("\tOutput Low: ");Serial.print(retrievedOutputLow);Serial.print("\tOutput High: ");Serial.println(retrievedOutputHigh);

    float tempReadings = map(ds18b20TempC,retrievedInputLow,retrievedInputHigh,retrievedOutputLow,retrievedOutputHigh);

    char SoilTemperatureStr[10];
    dtostrf(ds18b20TempC, 4, 2, SoilTemperatureStr);

    if (ds18b20TempC == -127.00) {
      // Serial.println("Error reading temperature.");
      float PreviousSoilTemp;
      EEPROM.get(220, PreviousSoilTemp);
      dtostrf(PreviousSoilTemp, 4, 2, SoilTemperatureStr);
      // Serial.print("Soil Temperature: ");Serial.print(SoilTemperatureStr);Serial.print(" °C");
      client.publish(("Device/"+ macAddress +"/Soiltemperature").c_str(), SoilTemperatureStr);
    } else {
      // Serial.print("Soil Temperature: ");Serial.print(SoilTemperatureStr);Serial.print(" °C");
      client.publish(("Device/"+ macAddress +"/Soiltemperature").c_str(), SoilTemperatureStr);
      EEPROM.put(220, ds18b20TempC);
      EEPROM.commit();
    }

    // Read humidity and temperature from DHT21
    humidity = dht.readHumidity();
    dhtTempC = dht.readTemperature();

    EEPROM.get(EEPROM_ADDRESS_SENSOR9, retrievedInputLow);
    EEPROM.get(EEPROM_ADDRESS_SENSOR9 + sizeof(float), retrievedInputHigh);
    EEPROM.get(EEPROM_ADDRESS_SENSOR9 + sizeof(float) * 2, retrievedOutputLow);
    EEPROM.get(EEPROM_ADDRESS_SENSOR9 + sizeof(float) * 3, retrievedOutputHigh);
    // Serial.println("EEPROM Saved Humidity Calibration Values");
    // Serial.print("Input Low: ");Serial.print(retrievedInputLow);Serial.print("\tInput High: ");Serial.print(retrievedInputHigh);
    // Serial.print("\tOutput Low: ");Serial.print(retrievedOutputLow);Serial.print("\tOutput High: ");Serial.println(retrievedOutputHigh);
    float humidityReadings = map(humidity,retrievedInputLow,retrievedInputHigh,retrievedOutputLow,retrievedOutputHigh);

    // Serial.print("\tTemperature: ");Serial.print(dhtTempC);Serial.print(" °C");
    // Serial.print("\tHumidity: ");Serial.print(humidity);Serial.println(" %");
    char TemperatureStr[10];dtostrf(dhtTempC, 4, 2, TemperatureStr); 
    char HumidityStr[10];dtostrf(humidity, 4, 2, HumidityStr);
    client.publish(("Device/"+ macAddress +"/temperature").c_str(), TemperatureStr);
    client.publish(("Device/"+ macAddress +"/humidity").c_str(), HumidityStr);

    int rawValue1 = analogRead(AOUT_PIN_1);
    int rawValue2 = analogRead(AOUT_PIN_2);

    // Update the readings and totals for sensor 1
    total1 = total1 - readings1[currentIndex];
    readings1[currentIndex] = rawValue1;
    total1 = total1 + readings1[currentIndex];

    // Update the readings and totals for sensor 2
    total2 = total2 - readings2[currentIndex];
    readings2[currentIndex] = rawValue2;
    total2 = total2 + readings2[currentIndex];

    currentIndex = (currentIndex + 1) % numReadings;
    // Calculate the smoothed values for both sensors
    int smoothedValue1 = total1 / numReadings;
    int smoothedValue2 = total2 / numReadings;
    
    EEPROM.get(EEPROM_ADDRESS_SENSOR1, retrievedInputLow);
    EEPROM.get(EEPROM_ADDRESS_SENSOR1 + sizeof(float), retrievedInputHigh);
    EEPROM.get(EEPROM_ADDRESS_SENSOR1 + sizeof(float) * 2, retrievedOutputLow);
    EEPROM.get(EEPROM_ADDRESS_SENSOR1 + sizeof(float) * 3, retrievedOutputHigh);
    // Serial.println("EEPROM Saved Soil Sensor Calibration Values");
    // Serial.print("Input Low: ");Serial.print(retrievedInputLow);Serial.print("\tInput High: ");Serial.print(retrievedInputHigh);
    // Serial.print("\tOutput Low: ");Serial.print(retrievedOutputLow);Serial.print("\tOutput Low: ");Serial.println(retrievedOutputHigh);
    soilmoisturepercent_01 = map(smoothedValue1, retrievedInputHigh, retrievedInputLow, retrievedOutputLow, retrievedOutputHigh);
    // Serial.print("Soil Moisture 01 Actual Value : ");Serial.println(smoothedValue1);
    // Serial.print("Soil Moisture 01 Mapped Value : ");Serial.println(soilmoisturepercent_01);

    EEPROM.get(EEPROM_ADDRESS_SENSOR2, retrievedInputLow);
    EEPROM.get(EEPROM_ADDRESS_SENSOR2 + sizeof(float), retrievedInputHigh);
    EEPROM.get(EEPROM_ADDRESS_SENSOR2 + sizeof(float) * 2, retrievedOutputLow);
    EEPROM.get(EEPROM_ADDRESS_SENSOR2 + sizeof(float) * 3, retrievedOutputHigh);
    // Serial.println("EEPROM Saved Soil Sensor Calibration Values");
    // Serial.print("Input Low: ");Serial.print(retrievedInputLow);Serial.print("\tInput High: ");Serial.print(retrievedInputHigh);
    // Serial.print("\tOutput Low: ");Serial.print(retrievedOutputLow);Serial.print("\tOutput High: ");Serial.println(retrievedOutputHigh);
    soilmoisturepercent_02 = map(smoothedValue2, retrievedInputHigh, retrievedInputLow, retrievedOutputLow, retrievedOutputHigh);
    // Serial.print("Soil Moisture 02 Actual Value : ");Serial.println(smoothedValue2);
    // Serial.print("Soil Moisture 02 Mapped Value : ");Serial.println(soilmoisturepercent_02);

    char ActualSoilMoisture1[10];dtostrf(smoothedValue1, 4, 2, ActualSoilMoisture1);
    char ActualSoilMoisture2[10];dtostrf(smoothedValue2, 4, 2, ActualSoilMoisture2);

    client.publish(("Device/"+ macAddress +"/ActualSoilMoisture1").c_str(), ActualSoilMoisture1);
    client.publish(("Device/"+ macAddress +"/ActualSoilMoisture2").c_str(), ActualSoilMoisture2);

    char SoilMoisture_1[10];dtostrf(soilmoisturepercent_01, 4, 2, SoilMoisture_1);
    char SoilMoisture_2[10];dtostrf(soilmoisturepercent_02, 4, 2, SoilMoisture_2);

    client.publish(("Device/"+ macAddress +"/SoilMoisture1").c_str(), SoilMoisture_1);
    client.publish(("Device/"+ macAddress +"/SoilMoisture2").c_str(), SoilMoisture_2);

    client.publish(("Device/"+ macAddress +"/time").c_str(), time.c_str());
    client.publish(("Device/"+ macAddress +"/date").c_str(), date.c_str());
    client.publish(("Device/"+ macAddress +"/type").c_str(),"Irrigation");
  }

  unsigned long currentMillis_2 = millis();
  if (currentMillis_2 - previousMillis_2 >= 60000) {
    previousMillis_2 = currentMillis_2;

    char SoilMoisture_1[10];dtostrf(soilmoisturepercent_01, 4, 2, SoilMoisture_1); 
    char SoilMoisture_2[10];dtostrf(soilmoisturepercent_02, 4, 2, SoilMoisture_2);
    char TemperatureStr[10];dtostrf(dhtTempC, 4, 2, TemperatureStr);
    char HumidityStr[10];dtostrf(humidity, 4, 2, HumidityStr);
    char SoilTemperatureStr[10];dtostrf(ds18b20TempC, 4, 2, SoilTemperatureStr);

    client.publish(("Device/"+ macAddress +"/SoilMoisture1_Graph").c_str(), SoilMoisture_1);
    client.publish(("Device/"+ macAddress +"/SoilMoisture2_Graph").c_str(), SoilMoisture_2);
    client.publish(("Device/"+ macAddress +"/temperature_Graph").c_str(), TemperatureStr);
    client.publish(("Device/"+ macAddress +"/humidity_Graph").c_str(), HumidityStr);
    client.publish(("Device/"+ macAddress +"/Soiltemperature_Graph").c_str(), SoilTemperatureStr);

    DateTime now = rtc.now();
    String time = String(now.hour())+":"+String(now.minute())+":"+String(now.second());
    String fileName = "/Irrigation("+String(now.year()) + "-" + String(now.month()) + "-" + String(now.day()) + ").txt";

    // Open the file in "append" mode again in each loop iteration
    File file = SD.open(fileName, FILE_APPEND);

    if (file) {
      // Append the data to the file
      file.print(time);file.print(" -> ");file.print("\tTemperature : ");file.print(dhtTempC);
      file.print("\tHumidity : ");file.print(humidity);file.print("\tSoil Temperature : ");file.println(ds18b20TempC);
      file.close();
      
      Serial.println("Data appended to the file.");
    } else {
      Serial.println("Error opening file for appending.");
    }
  }
  bool retriveMode;
  EEPROM.get(EEPROM_ADDRESS_SENSOR18, retriveMode);
  if (retriveMode) {
    unsigned long currentMillis_3 = millis();
    if (currentMillis_3 - previousMillis_3 >= 1000) {
      previousMillis_3 = currentMillis_3;

      //Read the Soil Moisture 01 High,low Limits and check it for sensor reading and ON and OFF Relays
      EEPROM.get(EEPROM_ADDRESS_SENSOR11, retrievedLowLimit);
      EEPROM.get(EEPROM_ADDRESS_SENSOR11 + sizeof(float), retrievedHighLimit);
      EEPROM.get(EEPROM_ADDRESS_SENSOR11 + sizeof(float) * 2, readValveString);
      EEPROM.get(EEPROM_ADDRESS_SENSOR11 + sizeof(float) * 3, retrievedValveStatus);
      
      if(soilmoisturepercent_01 >= retrievedHighLimit)
      {
        if(strcasecmp(readValveString, "v1") == 0 && retrievedValveStatus == 1.00){Serial.println("Relay-1 ON");IO_0.digitalWrite(P0,HIGH);}//Relay-1 ON
        if(strcasecmp(readValveString, "v1") == 0 && retrievedValveStatus == 0.00){Serial.println("Relay-1 OFF");IO_0.digitalWrite(P0,LOW);}//Relay-1 OFF
        if(strcasecmp(readValveString, "v2") == 0 && retrievedValveStatus == 1.00){Serial.println("Relay-2 ON");IO_0.digitalWrite(P1,HIGH);}//Relay-1 ON
        if(strcasecmp(readValveString, "v2") == 0 && retrievedValveStatus == 0.00){Serial.println("Relay-2 OFF");IO_0.digitalWrite(P1,LOW);}//Relay-1 OFF
      } 
      else if (soilmoisturepercent_01 <= retrievedLowLimit)
      {
        if(strcasecmp(readValveString, "v1") == 0 && retrievedValveStatus == 1.00){Serial.println("Relay-1 OFF");IO_0.digitalWrite(P0,LOW);}//Relay-1 OFF
        if(strcasecmp(readValveString, "v1") == 0 && retrievedValveStatus == 0.00){Serial.println("Relay-1 ON");IO_0.digitalWrite(P0,HIGH);}//Relay-1 ON
        if(strcasecmp(readValveString, "v2") == 0 && retrievedValveStatus == 1.00){Serial.println("Relay-2 OFF");IO_0.digitalWrite(P1,LOW);}//Relay-1 OFF
        if(strcasecmp(readValveString, "v2") == 0 && retrievedValveStatus == 0.00){Serial.println("Relay-2 ON");IO_0.digitalWrite(P1,HIGH);}//Relay-1 ON
      }
      //Read the Soil Moisture 02 High,low Limits and check it for sensor reading and ON and OFF Relays
      EEPROM.get(EEPROM_ADDRESS_SENSOR12, retrievedLowLimit);
      EEPROM.get(EEPROM_ADDRESS_SENSOR12 + sizeof(float), retrievedHighLimit);
      EEPROM.get(EEPROM_ADDRESS_SENSOR12 + sizeof(float) * 2, readValveString);
      EEPROM.get(EEPROM_ADDRESS_SENSOR12 + sizeof(float) * 3, retrievedValveStatus);
      
      if(soilmoisturepercent_02 <= retrievedLowLimit)
      {
        if(strcasecmp(readValveString, "v1") == 0 && retrievedValveStatus == 1.00){Serial.println("Relay-1 ON");IO_0.digitalWrite(P0,HIGH);}//Relay-1 ON
        if(strcasecmp(readValveString, "v1") == 0 && retrievedValveStatus == 0.00){Serial.println("Relay-1 OFF");IO_0.digitalWrite(P0,LOW);}//Relay-1 OFF
        if(strcasecmp(readValveString, "v2") == 0 && retrievedValveStatus == 1.00){Serial.println("Relay-2 ON");IO_0.digitalWrite(P1,HIGH);}//Relay-1 ON
        if(strcasecmp(readValveString, "v2") == 0 && retrievedValveStatus == 0.00){Serial.println("Relay-2 OFF");IO_0.digitalWrite(P1,LOW);}//Relay-1 OFF
      } 
      else if (soilmoisturepercent_02 >= retrievedHighLimit)
      {
        if(strcasecmp(readValveString, "v1") == 0 && retrievedValveStatus == 1.00){Serial.println("Relay-1 OFF");IO_0.digitalWrite(P0,LOW);}//Relay-1 OFF
        if(strcasecmp(readValveString, "v1") == 0 && retrievedValveStatus == 0.00){Serial.println("Relay-1 ON");IO_0.digitalWrite(P0,HIGH);}//Relay-1 ON
        if(strcasecmp(readValveString, "v2") == 0 && retrievedValveStatus == 1.00){Serial.println("Relay-2 OFF");IO_0.digitalWrite(P1,LOW);}//Relay-1 OFF
        if(strcasecmp(readValveString, "v2") == 0 && retrievedValveStatus == 0.00){Serial.println("Relay-2 ON");IO_0.digitalWrite(P1,HIGH);}//Relay-1 ON
      }
      //Read the Temperature High,low Limits and check it for sensor reading and ON and OFF Relays
      EEPROM.get(EEPROM_ADDRESS_SENSOR13, retrievedLowLimit);
      EEPROM.get(EEPROM_ADDRESS_SENSOR13 + sizeof(float), retrievedHighLimit);
      EEPROM.get(EEPROM_ADDRESS_SENSOR13 + sizeof(float) * 2, readValveString);
      EEPROM.get(EEPROM_ADDRESS_SENSOR13 + sizeof(float) * 3, retrievedValveStatus);
      if(dhtTempC < retrievedLowLimit)
      {
        if(strcasecmp(readValveString, "v1") == 0 && retrievedValveStatus == 1.00){Serial.println("Relay-1 ON");IO_0.digitalWrite(P0,HIGH);}//Relay-1 ON
        if(strcasecmp(readValveString, "v1") == 0 && retrievedValveStatus == 0.00){Serial.println("Relay-1 OFF");IO_0.digitalWrite(P0,LOW);}//Relay-1 OFF
        if(strcasecmp(readValveString, "v2") == 0 && retrievedValveStatus == 1.00){Serial.println("Relay-2 ON");IO_0.digitalWrite(P1,HIGH);}//Relay-1 ON
        if(strcasecmp(readValveString, "v2") == 0 && retrievedValveStatus == 0.00){Serial.println("Relay-2 OFF");IO_0.digitalWrite(P1,LOW);}//Relay-1 OFF
      } 
      else if (dhtTempC > retrievedHighLimit)
      {
        if(strcasecmp(readValveString, "v1") == 0 && retrievedValveStatus == 1.00){Serial.println("Relay-1 OFF");IO_0.digitalWrite(P0,LOW);}//Relay-1 OFF
        if(strcasecmp(readValveString, "v1") == 0 && retrievedValveStatus == 0.00){Serial.println("Relay-1 ON");IO_0.digitalWrite(P0,HIGH);}//Relay-1 ON
        if(strcasecmp(readValveString, "v2") == 0 && retrievedValveStatus == 1.00){Serial.println("Relay-2 OFF");IO_0.digitalWrite(P1,LOW);}//Relay-1 OFF
        if(strcasecmp(readValveString, "v2") == 0 && retrievedValveStatus == 0.00){Serial.println("Relay-2 ON");IO_0.digitalWrite(P1,HIGH);}//Relay-1 ON
      }

      //Read the Humidity High,low Limits and check it for sensor reading and ON and OFF Relays
      EEPROM.get(EEPROM_ADDRESS_SENSOR17, retrievedLowLimit);
      EEPROM.get(EEPROM_ADDRESS_SENSOR17 + sizeof(float), retrievedHighLimit);
      EEPROM.get(EEPROM_ADDRESS_SENSOR17 + sizeof(float) * 2, readValveString);
      EEPROM.get(EEPROM_ADDRESS_SENSOR17 + sizeof(float) * 3, retrievedValveStatus);

      Serial.println(".................Humidity Settings...................");
      Serial.print("Humidity Sensor reading : ");Serial.println(humidity);
      Serial.print("Low limit : ");Serial.print(retrievedLowLimit);Serial.print("\tHigh limit : ");Serial.print(retrievedHighLimit);
      Serial.print("\tValve : ");Serial.print(readValveString);Serial.print("\tValve Status : ");Serial.println(retrievedValveStatus);
      if(humidity <= retrievedLowLimit)
      {
        if(strcasecmp(readValveString, "v1") == 0 && retrievedValveStatus == 1.00){Serial.println("Relay-1 ON");IO_0.digitalWrite(P0,HIGH);}//Relay-1 ON
        if(strcasecmp(readValveString, "v1") == 0 && retrievedValveStatus == 0.00){Serial.println("Relay-1 OFF");IO_0.digitalWrite(P0,LOW);}//Relay-1 OFF
        if(strcasecmp(readValveString, "v2") == 0 && retrievedValveStatus == 1.00){Serial.println("Relay-2 ON");IO_0.digitalWrite(P1,HIGH);}//Relay-1 ON
        if(strcasecmp(readValveString, "v2") == 0 && retrievedValveStatus == 0.00){Serial.println("Relay-2 OFF");IO_0.digitalWrite(P1,LOW);}//Relay-1 OFF
      } 
      else if (humidity >= retrievedHighLimit)
      {
        if(strcasecmp(readValveString, "v1") == 0 && retrievedValveStatus == 1.00){Serial.println("Relay-1 OFF");IO_0.digitalWrite(P0,LOW);}//Relay-1 OFF
        if(strcasecmp(readValveString, "v1") == 0 && retrievedValveStatus == 0.00){Serial.println("Relay-1 ON");IO_0.digitalWrite(P0,HIGH);}//Relay-1 ON
        if(strcasecmp(readValveString, "v2") == 0 && retrievedValveStatus == 1.00){Serial.println("Relay-2 OFF");IO_0.digitalWrite(P1,LOW);}//Relay-1 OFF
        if(strcasecmp(readValveString, "v2") == 0 && retrievedValveStatus == 0.00){Serial.println("Relay-2 ON");IO_0.digitalWrite(P1,HIGH);}//Relay-1 ON
      }

      //Read the Soil Temp High,low Limits and check it for sensor reading and ON and OFF Relays
      EEPROM.get(EEPROM_ADDRESS_SENSOR15, retrievedLowLimit);
      EEPROM.get(EEPROM_ADDRESS_SENSOR15 + sizeof(float), retrievedHighLimit);
      EEPROM.get(EEPROM_ADDRESS_SENSOR15 + sizeof(float) * 2, readValveString);
      EEPROM.get(EEPROM_ADDRESS_SENSOR15 + sizeof(float) * 3, retrievedValveStatus);

      Serial.println(".................Soil Temperature Settings...................");
      Serial.print("Soil Temperature sensor Reading : ");Serial.println(ds18b20TempC);
      Serial.print("Low limit : ");Serial.print(retrievedLowLimit);Serial.print("\tHigh limit : ");Serial.print(retrievedHighLimit);
      Serial.print("\tValve : ");Serial.print(readValveString);Serial.print("\tValve Status : ");Serial.println(retrievedValveStatus);
      if(ds18b20TempC >= retrievedHighLimit)
      {
        if(strcasecmp(readValveString, "v1") == 0 && retrievedValveStatus == 1.00){Serial.println("Relay-1 ON");IO_0.digitalWrite(P0,HIGH);}//Relay-1 ON
        if(strcasecmp(readValveString, "v1") == 0 && retrievedValveStatus == 0.00){Serial.println("Relay-1 OFF");IO_0.digitalWrite(P0,LOW);}//Relay-1 OFF
        if(strcasecmp(readValveString, "v2") == 0 && retrievedValveStatus == 1.00){Serial.println("Relay-2 ON");IO_0.digitalWrite(P1,HIGH);}//Relay-1 ON
        if(strcasecmp(readValveString, "v2") == 0 && retrievedValveStatus == 0.00){Serial.println("Relay-2 OFF");IO_0.digitalWrite(P1,LOW);}//Relay-1 OFF
      } 
      else if (ds18b20TempC <= retrievedLowLimit)
      {
        if(strcasecmp(readValveString, "v1") == 0 && retrievedValveStatus == 1.00){Serial.println("Relay-1 OFF");IO_0.digitalWrite(P0,LOW);}//Relay-1 OFF
        if(strcasecmp(readValveString, "v1") == 0 && retrievedValveStatus == 0.00){Serial.println("Relay-1 ON");IO_0.digitalWrite(P0,HIGH);}//Relay-1 ON
        if(strcasecmp(readValveString, "v2") == 0 && retrievedValveStatus == 1.00){Serial.println("Relay-2 OFF");IO_0.digitalWrite(P1,LOW);}//Relay-1 OFF
        if(strcasecmp(readValveString, "v2") == 0 && retrievedValveStatus == 0.00){Serial.println("Relay-2 ON");IO_0.digitalWrite(P1,HIGH);}//Relay-1 ON
      }
    }
  }
  

  //Timer 
  unsigned long currentMillis_4 = millis();
  if (currentMillis_4 - previousMillis_4 >= 1000) {
    previousMillis_4 = currentMillis_4;

    ScheduleData data;
    EEPROM.get(EEPROM_ADDRESS_SENSOR16, data.StartTime);
    EEPROM.get(EEPROM_ADDRESS_SENSOR16 + sizeof(data.StartTime), data.EndTime);
    EEPROM.get(EEPROM_ADDRESS_SENSOR16 + 2 * sizeof(data.StartTime), data.valveName);
    EEPROM.get(EEPROM_ADDRESS_SENSOR16 + 2 * sizeof(data.StartTime) + sizeof(data.valveName), data.valveStatus);
    EEPROM.get(EEPROM_ADDRESS_SENSOR16 + 2 * sizeof(data.StartTime) + sizeof(data.valveName) + sizeof(data.valveStatus), data.day);

    // Serial.println("Read data from EEPROM:");
    // Serial.print("Start Time : ");
    // Serial.println(data.StartTime);
    // Serial.print("End Time : ");
    // Serial.println(data.EndTime);
    // Serial.print("Valve Name: ");
    // Serial.println(data.valveName);
    // Serial.print("Valve Status: ");
    // Serial.println(data.valveStatus);
    // Serial.print("Day: ");
    // Serial.println(data.day);

    // DateTime now = rtc.now();
    // String time = String(now.hour()) + ":" + String(now.minute());
    DateTime now = rtc.now();
    String hourString = (now.hour() < 10) ? "0" + String(now.hour()) : String(now.hour());
    String minuteString = (now.minute() < 10) ? "0" + String(now.minute()) : String(now.minute());
    String time = hourString + ":" + minuteString;

    String date = String(now.year()) + "-" + String(now.month()) + "-" + String(now.day());
    String dayOfWeek = now.dayOfTheWeek() == 1 ? "Mon" : 
                      now.dayOfTheWeek() == 2 ? "Tue" :
                      now.dayOfTheWeek() == 3 ? "Wed" :
                      now.dayOfTheWeek() == 4 ? "Thu" :
                      now.dayOfTheWeek() == 5 ? "Fri" :
                      now.dayOfTheWeek() == 6 ? "Sat" :
                      "Sun";
    // Serial.print("Date: ");
    // Serial.print(date);
    // Serial.print("\tDay: ");
    // Serial.print(dayOfWeek);  // Print the short name of the day
    // Serial.print("\tTime: ");
    // Serial.println(time);
    // Serial.println();

    String dataDayString = String(data.day);

    // Check if dayOfWeek matches any of the days in data.day
    if (dataDayString.indexOf(dayOfWeek) != -1) {
      if(time==data.StartTime)
      {
        // Serial.println("Timer Start Soon");
        if (String(data.valveName)==" Valve-1" && data.valveStatus == 1) {IO_0.digitalWrite(P0,HIGH);}
        if (String(data.valveName)==" Valve-1" && data.valveStatus == 0) {IO_0.digitalWrite(P0,LOW);}
        if (String(data.valveName)==" Valve-2" && data.valveStatus == 1) {IO_0.digitalWrite(P1,HIGH);}
        if (String(data.valveName)==" Valve-2" && data.valveStatus == 0) {IO_0.digitalWrite(P1,LOW);}
        if (String(data.valveName)==" Valve-3" && data.valveStatus == 1) {IO_0.digitalWrite(P2,HIGH);}
        if (String(data.valveName)==" Valve-3" && data.valveStatus == 0) {IO_0.digitalWrite(P2,LOW);}  
        // else {IO_0.digitalWrite(P0,LOW);IO_0.digitalWrite(P1,LOW);IO_0.digitalWrite(P2,LOW);}
      }
      // Serial.print("Timer End Time:");
      // Serial.println(data.EndTime);
      if(time==data.EndTime)
      {
        // Serial.println("Timer Stop Soon");
        if (String(data.valveName)==" Valve-1" && data.valveStatus == 1) {IO_0.digitalWrite(P0,LOW);}
        if (String(data.valveName)==" Valve-1" && data.valveStatus == 0) {IO_0.digitalWrite(P0,HIGH);}
        if (String(data.valveName)==" Valve-2" && data.valveStatus == 1) {IO_0.digitalWrite(P0,LOW);}
        if (String(data.valveName)==" Valve-2" && data.valveStatus == 0) {IO_0.digitalWrite(P0,HIGH);}
        if (String(data.valveName)==" Valve-3" && data.valveStatus == 1) {IO_0.digitalWrite(P0,LOW);}
        if (String(data.valveName)==" Valve-3" && data.valveStatus == 0) {IO_0.digitalWrite(P0,HIGH);}
      }
      // else
      // {
      //   Serial.println("No Timer found.");
      // }
    } else {
      // Serial.println("Today Not Found At Timer");
    }
    
  }

  unsigned long currentMillis_5 = millis();
  if (currentMillis_5 - previousMillis_5 >= 1000) {
    previousMillis_5 = currentMillis_5;
    
    bool relay_1Status = IO_0.digitalRead(P0);
    bool relay_2Status = IO_0.digitalRead(P1);
    // Serial.print("Relay-01 Staus : ");Serial.println(relay_1Status);
    if(relay_1Status==1){
      Serial.println("Relay-01 is ON");
      client.publish(("Device/"+ macAddress +"/Relay_1").c_str(),"HIGH");
    }
    else if(relay_1Status==0){
      Serial.println("Relay-01 is OFF");
      client.publish(("Device/"+ macAddress +"/Relay_1").c_str(),"LOW");
    }
    if(relay_2Status==1){
      Serial.println("Relay-02 is ON");
      client.publish(("Device/"+ macAddress +"/Relay_2").c_str(),"HIGH");
    }
    else if(relay_2Status==0){
      Serial.println("Relay-02 is OFF");
      client.publish(("Device/"+ macAddress +"/Relay_2").c_str(),"LOW");
    }
  }
}

void npkTaskFuction(void* parameter) {
  (void)parameter;
  String macAddress = getMacAddress();
  while (1) {
    byte val1,val2,val3,val4,val5,val6,val7;
    val1 = nitrogen();
    delay(2000);
    val2 = phosphorus();
    delay(2000);
    val3 = potassium();
    delay(2000);
    val4 = phValue();
    delay(2000);
    val5 = eCValue();
    delay(2000);
    val6 = tempValue();
    delay(2000);
    val7 = humidityValue();
    delay(2000);

    // Serial.print("Temperature: ");Serial.print(val6);Serial.print(" °C");
    // Serial.print("\tHumidity: ");Serial.print(val7);Serial.println(" %");
    // Serial.print("Nitrogen: ");Serial.print(val1);Serial.print(" mg/kg");
    // Serial.print("\tPhosphorus: ");Serial.print(val2);Serial.print(" mg/kg");
    // Serial.print("\tPotassium: ");Serial.print(val3);Serial.println(" mg/kg");
    // Serial.print("PH Value: ");Serial.print(val4);
    // Serial.print("\tEC Value: ");Serial.print(val5);Serial.println(" us/cm");
    // Serial.println();
    
    delay(1000);
  }
}

byte nitrogen(){
  mySerial.write(nitrogenAddress,8);
  delay(100);
  for(byte i =0;i<7;i++)
  {
    values[i] = mySerial.read();
    // Serial.print("0x");
    // Serial.print(values[i],HEX);
    // Serial.print(" ");
  }
  // Serial.println("----> Nitrogen Response Address");
  unsigned long combinedValue = ((unsigned long)values[3] << 8) | values[4];
  float NitrogenValue = strtol(String(combinedValue, HEX).c_str(), NULL, 16);
  // Serial.print("Nitrogen: ");Serial.print(NitrogenValue);Serial.println(" mg/kg");
  char NitrogenStr[10];dtostrf(NitrogenValue, 4, 2, NitrogenStr);
  client.publish(("Device/"+ macAddress +"/NPK_Nitrogen").c_str(), NitrogenStr);
  return NitrogenValue;
}
byte phosphorus(){
  mySerial.write(phosphorusAddress,8);
  delay(100);
  for(byte i =0;i<7;i++)
  {
    values[i] = mySerial.read();
    // Serial.print("0x");
    // Serial.print(values[i],HEX);
    // Serial.print(" ");
  }
  // Serial.println("----> Phosphorus Response Address");
  unsigned long combinedValue = ((unsigned long)values[3] << 8) | values[4];
  float PhosphorusValue = strtol(String(combinedValue, HEX).c_str(), NULL, 16);
  // Serial.print("Phosphorus: ");Serial.print(PhosphorusValue);Serial.println(" mg/kg");
  char PhosphorusStr[10];dtostrf(PhosphorusValue, 4, 2, PhosphorusStr);
  client.publish(("Device/"+ macAddress +"/NPK_Phosphorus").c_str(), PhosphorusStr);
  return PhosphorusValue;
}
byte potassium(){
  mySerial.write(potassiumAddress,8);
  delay(100);
  for(byte i =0;i<7;i++)
  {
    values[i] = mySerial.read();
    // Serial.print("0x");
    // Serial.print(values[i],HEX);
    // Serial.print(" ");
  }
  // Serial.println("----> Pottassium Response Address");
  unsigned long combinedValue = ((unsigned long)values[3] << 8) | values[4];
  float PotassiumValue = strtol(String(combinedValue, HEX).c_str(), NULL, 16);
  // Serial.print("Potassium: ");Serial.print(PotassiumValue);Serial.println(" mg/kg");
  char PotassiumStr[10];dtostrf(PotassiumValue, 4, 2, PotassiumStr);
  client.publish(("Device/"+ macAddress +"/NPK_Potassium").c_str(), PotassiumStr);
  return PotassiumValue;
}
byte phValue(){
  mySerial.write(pHAddress,8);
  delay(100);
  for(byte i =0;i<7;i++)
  {
    values[i] = mySerial.read();
    // Serial.print("0x");
    // Serial.print(values[i],HEX);
    // Serial.print(" ");
  }
  // Serial.println("----> PH Value Response Address");
  unsigned long combinedValue = ((unsigned long)values[3] << 8) | values[4];
  float PhValue = strtol(String(combinedValue, HEX).c_str(), NULL, 16) / 100.0;
  // Serial.print("PH Value: ");Serial.println(PhValue);
  char PHValueStr[10];dtostrf(PhValue, 4, 2, PHValueStr);
  client.publish(("Device/"+ macAddress +"/NPK_PH").c_str(), PHValueStr);
  return PhValue;
}
byte eCValue(){
  mySerial.write(eCAddress,8);
  delay(100);
  for(byte i =0;i<7;i++)
  {
    values[i] = mySerial.read();
    // Serial.print("0x");
    // Serial.print(values[i],HEX);
    // Serial.print(" ");
  }
  // Serial.println("----> EC Value Respone Address");
  unsigned long combinedValue = ((unsigned long)values[3] << 8) | values[4];
  float EcValue = strtol(String(combinedValue, HEX).c_str(), NULL, 16);
  // Serial.print("EC Value: ");Serial.print(EcValue);Serial.println(" us/cm");
  char ECValueStr[10];dtostrf(EcValue, 4, 2, ECValueStr);
  client.publish(("Device/"+ macAddress +"/NPK_EC").c_str(), ECValueStr);
  return EcValue;
}
byte tempValue(){
  mySerial.write(tempAddress,8);
  delay(100);
  for(byte i =0;i<7;i++)
  {
    values[i] = mySerial.read();
    // Serial.print("0x");
    // Serial.print(values[i],HEX);
    // Serial.print(" ");
  }
  // Serial.println("----> Temperature Response Address");
  unsigned long combinedValue = ((unsigned long)values[3] << 8) | values[4];
  float tempValue = strtol(String(combinedValue, HEX).c_str(), NULL, 16) / 10.0;
  // Serial.print("Temperature: ");Serial.print(tempValue);Serial.println(" °C");
  char NPKTemperatureStr[10];dtostrf(tempValue, 4, 2, NPKTemperatureStr); 
  client.publish(("Device/"+ macAddress +"/NPK_Temperature").c_str(), NPKTemperatureStr);
  return tempValue;
}
byte humidityValue(){
  mySerial.write(humidityAddress,8);
  delay(100);
  for(byte i =0;i<7;i++)
  {
    values[i] = mySerial.read();
    // Serial.print("0x");
    // Serial.print(values[i],HEX);
    // Serial.print(" ");
  }
  // Serial.println("----> Humidity Response Address");
  unsigned long combinedValue = ((unsigned long)values[3] << 8) | values[4];
  float HumidityValue = strtol(String(combinedValue, HEX).c_str(), NULL, 16) / 10.0;
  // Serial.print("Humidity: ");Serial.print(HumidityValue);Serial.println(" %");
  char NPKHumidityStr[10];dtostrf(HumidityValue, 4, 2, NPKHumidityStr);
  client.publish(("Device/"+ macAddress +"/NPK_Humidity").c_str(), NPKHumidityStr);
  return HumidityValue;
}

void mqttTaskFunction(void* parameter) {
  (void)parameter;
  String macAddress = getMacAddress();
  while (1) {
    if (!client.connected()) {
      // Reconnect to MQTT broker
      Serial.println("Connecting to MQTT broker...");
      if (client.connect(clientId)) {
        Serial.println("Connected to MQTT broker");
        digitalWrite(LED_BUILTIN, HIGH);

        client.subscribe(("Device/" + macAddress + "/tempSensor/Calibration").c_str());
        client.subscribe(("Device/" + macAddress + "/humiditySensor/Calibration").c_str());
        client.subscribe(("Device/" + macAddress + "/soilSensor01/Calibration").c_str());
        client.subscribe(("Device/" + macAddress + "/soilSensor02/Calibration").c_str());
        
        client.subscribe(("Device/" + macAddress + "/soilSensor01/Settings").c_str());
        client.subscribe(("Device/" + macAddress + "/soilSensor02/Settings").c_str());
        client.subscribe(("Device/" + macAddress + "/SoilTemp/Settings").c_str());
        client.subscribe(("Device/" + macAddress + "/humiditySensor/Settings").c_str());
        client.subscribe(("Device/" + macAddress + "/tempSensor/Settings").c_str());

        client.subscribe(("Device/" + macAddress + "/Valve_1").c_str());
        client.subscribe(("Device/" + macAddress + "/Valve_2").c_str());
        client.subscribe(("Device/" + macAddress + "/Motor_1").c_str());

        client.subscribe(("Device/" + macAddress + "/Timer").c_str());
        client.subscribe(("Device/" + macAddress + "/RestartBtn").c_str());
        client.subscribe(("Device/" + macAddress + "/Mode").c_str());

      } else {
        digitalWrite(LED_BUILTIN, LOW);
        Serial.println("Connection failed. Retrying in 5 seconds...");
        delay(5000);
        continue;
      }
    }
    // Maintain MQTT connection
    client.loop();
    delay(1000);
  }
}
