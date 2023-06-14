// Copyright (c) Microsoft Corporation. All rights reserved.
// SPDX-License-Identifier: MIT

/*
 * This is an Arduino-based Azure IoT Hub sample for ESPRESSIF ESP32 boards.
 * It uses our Azure Embedded SDK for C to help interact with Azure IoT.
 * For reference, please visit https://github.com/azure/azure-sdk-for-c.
 *
 * To connect and work with Azure IoT Hub you need an MQTT client, connecting, subscribing
 * and publishing to specific topics to use the messaging features of the hub.
 * Our azure-sdk-for-c is an MQTT client support library, helping composing and parsing the
 * MQTT topic names and messages exchanged with the Azure IoT Hub.
 *
 * This sample performs the following tasks:
 * - Synchronize the device clock with a NTP server;
 * - Initialize our "az_iot_hub_client" (struct for data, part of our azure-sdk-for-c);
 * - Initialize the MQTT client (here we use ESPRESSIF's esp_mqtt_client, which also handle the tcp
 * connection and TLS);
 * - Connect the MQTT client (using server-certificate validation, SAS-tokens for client
 * authentication);
 * - Periodically send telemetry data to the Azure IoT Hub.
 *
 * To properly connect to your Azure IoT Hub, please fill the information in the `iot_configs.h`
 * file.
 */

// C99 libraries
#include <cstdlib>
#include <string.h>
#include <time.h>

// Libraries for MQTT client and WiFi connection
#include <WiFi.h>
#include <mqtt_client.h>

// Azure IoT SDK for C includes
#include <az_core.h>
#include <az_iot.h>
#include <azure_ca.h>

// Additional sample headers
#include "AzIoTSasToken.h"
#include "SerialLogger.h"
#include "iot_configs.h"


// Own
#include  "MQ135.h"
#include <movingAvg.h>

// When developing for your own Arduino-based platform,
// please follow the format '(ard;<platform>)'.
#define AZURE_SDK_CLIENT_USER_AGENT "c%2F" AZ_SDK_VERSION_STRING "(ard;esp32)"

// Utility macros and defines
#define sizeofarray(a) (sizeof(a) / sizeof(a[0]))
#define NTP_SERVERS "pool.ntp.org", "time.nist.gov"
#define MQTT_QOS1 1
#define DO_NOT_RETAIN_MSG 0
#define SAS_TOKEN_DURATION_IN_MINUTES 60
#define UNIX_TIME_NOV_13_2017 1510592825

#define PST_TIME_ZONE -8
#define PST_TIME_ZONE_DAYLIGHT_SAVINGS_DIFF 1

#define GMT_OFFSET_SECS (PST_TIME_ZONE * 3600)
#define GMT_OFFSET_SECS_DST ((PST_TIME_ZONE + PST_TIME_ZONE_DAYLIGHT_SAVINGS_DIFF) * 3600)

// Translate iot_configs.h defines into variables used by the sample
static const char* ssid = IOT_CONFIG_WIFI_SSID;
static const char* password = IOT_CONFIG_WIFI_PASSWORD;
static const char* host = IOT_CONFIG_IOTHUB_FQDN;
static const char* mqtt_broker_uri = "mqtts://" IOT_CONFIG_IOTHUB_FQDN;
static const char* device_id = IOT_CONFIG_DEVICE_ID;
static const int mqtt_port = AZ_IOT_DEFAULT_MQTT_CONNECT_PORT;

// Memory allocated for the sample's variables and structures.
static esp_mqtt_client_handle_t mqtt_client;
static az_iot_hub_client client;

static char mqtt_client_id[128];
static char mqtt_username[128];
static char mqtt_password[200];
static uint8_t sas_signature_buffer[256];
static unsigned long next_telemetry_send_time_ms = 0;
static char telemetry_topic[128];
static uint8_t telemetry_payload[100];
static uint32_t telemetry_send_count = 0;

#define INCOMING_DATA_BUFFER_SIZE 128
static char incoming_data[INCOMING_DATA_BUFFER_SIZE];

// Auxiliary functions
#ifndef IOT_CONFIG_USE_X509_CERT
static AzIoTSasToken sasToken(
    &client,
    AZ_SPAN_FROM_STR(IOT_CONFIG_DEVICE_KEY),
    AZ_SPAN_FROM_BUFFER(sas_signature_buffer),
    AZ_SPAN_FROM_BUFFER(mqtt_password));
#endif // IOT_CONFIG_USE_X509_CERT



unsigned int samplingTime = 280;
unsigned int deltaTime = 40;
unsigned int sleepTime = 9680;



// ========================================================================

const int mqANALOGPIN=32;
MQ135 gasSensor = MQ135(mqANALOGPIN);

// sharp
int sharpMeasurePin = 26;
int sharpLedPower = 27;

// Define CO2 breakpoints
float co2_Clow[] = {0, 400, 1000, 1500, 2000, 5000};
float co2_Chigh[] = {399.9, 999.9, 1499.9, 1999.9, 4999.9, 10000};

// Define PM2.5 breakpoints
float pm25_Clow[] = {0, 10, 20, 25, 50, 75};
float pm25_Chigh[] = {9.9, 19.9, 24.9, 49.9, 74.9, 800};

// Define AQI breakpoints
int aqi_Ilow[] = {0, 26, 51, 76, 101, 301};
int aqi_Ihigh[] = {25, 50, 75, 100, 300, 500};

int co2_C = 0;
int co2_AQI = 0;

int pm25_C = 0;
int pm25_AQI = 0;


//int AQI;



int calculeCo2AQI() {
  int I = 0;
  for (int i = 0; i < 6; i++) {
          if (co2_C >= co2_Clow[i] && co2_C <= co2_Chigh[i]) {
            // Calculate AQI based on PM2.5 concentration range
            I = (aqi_Ihigh[i] - aqi_Ilow[i]) / (co2_Chigh[i] - co2_Clow[i]) * (co2_C - co2_Clow[i]) + aqi_Ilow[i];
            
            //Serial.print("CO2 AQI: ");
            //Serial.println(I);

            return I;
            
            break;
          }
  }
}

int calculePmAQI() {
  int I = 0;
  for (int i = 0; i < 6; i++) {
          if (pm25_C >= pm25_Clow[i] && pm25_C <= pm25_Chigh[i]) {
            // Calculate AQI based on PM2.5 concentration range
            I = (aqi_Ihigh[i] - aqi_Ilow[i]) / (pm25_Chigh[i] - pm25_Clow[i]) * (pm25_C - pm25_Clow[i]) + aqi_Ilow[i];
            
            Serial.print("PM2.5 AQI: ");
            Serial.println(I);
            
            break;
          }
  }

  return I;
}

// ==================================================================================
#define ZERO_VALUE 0.123

float voMeasured = 0;
float voltage = 0;
int dustDensity = 0;
 
int mean = 0;
int meanBuff = 0;
int meanCount = 0;

const int buffSize = 10;
int buffer[buffSize];
int bufferVoltage[buffSize];
int bufferCounter = 0;
int median = 0;

int AQI = 0;


int calculate_median(int array[]) {
  // sort buffer
  for (int i = 0; i < buffSize - 1; i++) {
    for (int j = i + 1; j < buffSize; j++) {
      if (array[j] < array[i]) {
        int temp = array[i];
        array[i] = array[j];
        array[j] = temp;
      }
    }
  }

  //median value
  if (buffSize % 2 == 1) {
    return array[buffSize / 2];
  } else {
    return (array[buffSize / 2 - 1] + array[buffSize / 2]) / 2.0;
  }
}

//float filter(int buff[], int size, int median, float error) {
int filter(int median, float error) {
  float max_error = median + error;
  float min_error = median - error;

  int sum_filtered = 0;
  int num_filtered = 0;
  //for (int i = 0; i < size; i++) {
  for (int i = 0; i < buffSize; i++) {
    //if(buff[i] >= min_error && buff[i] <= max_error) {
    if(buffer[i] >= min_error && buffer[i] <= max_error) {
      sum_filtered += buffer[i];
      num_filtered++;
    }
  }

  if (num_filtered != 0){
    int mean_filtered = sum_filtered / num_filtered;
    return mean_filtered;
  }
  
  return -1;
}


// ========================================================================

int buffMovingAverage[buffSize];
int sum_moving_average = 0;
int idx = 0;
int num_values = 0;

int moving_average = 0;

int MovingAverage(int value) {
  /*sum_moving_average -= buffMovingAverage[idx];
  buffMovingAverage[idx] = value;
  sum_moving_average += value;
  idx = (idx + 1) % buffSize;*/

  sum_moving_average += value;

  if(num_values == buffSize) sum_moving_average -= buffMovingAverage[idx];
  buffMovingAverage[idx] = value;
  if(num_values < buffSize) num_values++;
  idx++;
  if (idx >= buffSize) idx = 0;

  return sum_moving_average / buffSize;
}


movingAvg mqMovingAvg(buffSize);


// ========================================================================

static void connectToWiFi()
{
  Logger.Info("Connecting to WIFI SSID " + String(ssid));

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");

  Logger.Info("WiFi connected, IP address: " + WiFi.localIP().toString());
}

static void initializeTime()
{
  Logger.Info("Setting time using SNTP");

  configTime(GMT_OFFSET_SECS, GMT_OFFSET_SECS_DST, NTP_SERVERS);
  time_t now = time(NULL);
  while (now < UNIX_TIME_NOV_13_2017)
  {
    delay(500);
    Serial.print(".");
    now = time(nullptr);
  }
  Serial.println("");
  Logger.Info("Time initialized!");
}

void receivedCallback(char* topic, byte* payload, unsigned int length)
{
  Logger.Info("Received [");
  Logger.Info(topic);
  Logger.Info("]: ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println("");
}

static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event)
{
  switch (event->event_id)
  {
    int i, r;

    case MQTT_EVENT_ERROR:
      Logger.Info("MQTT event MQTT_EVENT_ERROR");
      break;
    case MQTT_EVENT_CONNECTED:
      Logger.Info("MQTT event MQTT_EVENT_CONNECTED");

      r = esp_mqtt_client_subscribe(mqtt_client, AZ_IOT_HUB_CLIENT_C2D_SUBSCRIBE_TOPIC, 1);
      if (r == -1)
      {
        Logger.Error("Could not subscribe for cloud-to-device messages.");
      }
      else
      {
        Logger.Info("Subscribed for cloud-to-device messages; message id:" + String(r));
      }

      break;
    case MQTT_EVENT_DISCONNECTED:
      Logger.Info("MQTT event MQTT_EVENT_DISCONNECTED");
      break;
    case MQTT_EVENT_SUBSCRIBED:
      Logger.Info("MQTT event MQTT_EVENT_SUBSCRIBED");
      break;
    case MQTT_EVENT_UNSUBSCRIBED:
      Logger.Info("MQTT event MQTT_EVENT_UNSUBSCRIBED");
      break;
    case MQTT_EVENT_PUBLISHED:
      Logger.Info("MQTT event MQTT_EVENT_PUBLISHED");
      break;
    case MQTT_EVENT_DATA:
      Logger.Info("MQTT event MQTT_EVENT_DATA");

      for (i = 0; i < (INCOMING_DATA_BUFFER_SIZE - 1) && i < event->topic_len; i++)
      {
        incoming_data[i] = event->topic[i];
      }
      incoming_data[i] = '\0';
      Logger.Info("Topic: " + String(incoming_data));

      for (i = 0; i < (INCOMING_DATA_BUFFER_SIZE - 1) && i < event->data_len; i++)
      {
        incoming_data[i] = event->data[i];
      }
      incoming_data[i] = '\0';
      Logger.Info("Data: " + String(incoming_data));

      break;
    case MQTT_EVENT_BEFORE_CONNECT:
      Logger.Info("MQTT event MQTT_EVENT_BEFORE_CONNECT");
      break;
    default:
      Logger.Error("MQTT event UNKNOWN");
      break;
  }

  return ESP_OK;
}

static void initializeIoTHubClient()
{
  az_iot_hub_client_options options = az_iot_hub_client_options_default();
  options.user_agent = AZ_SPAN_FROM_STR(AZURE_SDK_CLIENT_USER_AGENT);

  if (az_result_failed(az_iot_hub_client_init(
          &client,
          az_span_create((uint8_t*)host, strlen(host)),
          az_span_create((uint8_t*)device_id, strlen(device_id)),
          &options)))
  {
    Logger.Error("Failed initializing Azure IoT Hub client");
    return;
  }

  size_t client_id_length;
  if (az_result_failed(az_iot_hub_client_get_client_id(
          &client, mqtt_client_id, sizeof(mqtt_client_id) - 1, &client_id_length)))
  {
    Logger.Error("Failed getting client id");
    return;
  }

  if (az_result_failed(az_iot_hub_client_get_user_name(
          &client, mqtt_username, sizeofarray(mqtt_username), NULL)))
  {
    Logger.Error("Failed to get MQTT clientId, return code");
    return;
  }

  Logger.Info("Client ID: " + String(mqtt_client_id));
  Logger.Info("Username: " + String(mqtt_username));
}

static int initializeMqttClient()
{
#ifndef IOT_CONFIG_USE_X509_CERT
  if (sasToken.Generate(SAS_TOKEN_DURATION_IN_MINUTES) != 0)
  {
    Logger.Error("Failed generating SAS token");
    return 1;
  }
#endif

  esp_mqtt_client_config_t mqtt_config;
  memset(&mqtt_config, 0, sizeof(mqtt_config));
  mqtt_config.uri = mqtt_broker_uri;
  mqtt_config.port = mqtt_port;
  mqtt_config.client_id = mqtt_client_id;
  mqtt_config.username = mqtt_username;

#ifdef IOT_CONFIG_USE_X509_CERT
  Logger.Info("MQTT client using X509 Certificate authentication");
  mqtt_config.client_cert_pem = IOT_CONFIG_DEVICE_CERT;
  mqtt_config.client_key_pem = IOT_CONFIG_DEVICE_CERT_PRIVATE_KEY;
#else // Using SAS key
  mqtt_config.password = (const char*)az_span_ptr(sasToken.Get());
#endif

  mqtt_config.keepalive = 30;
  mqtt_config.disable_clean_session = 0;
  mqtt_config.disable_auto_reconnect = false;
  mqtt_config.event_handle = mqtt_event_handler;
  mqtt_config.user_context = NULL;
  mqtt_config.cert_pem = (const char*)ca_pem;

  mqtt_client = esp_mqtt_client_init(&mqtt_config);

  if (mqtt_client == NULL)
  {
    Logger.Error("Failed creating mqtt client");
    return 1;
  }

  esp_err_t start_result = esp_mqtt_client_start(mqtt_client);

  if (start_result != ESP_OK)
  {
    Logger.Error("Could not start mqtt client; error code:" + start_result);
    return 1;
  }
  else
  {
    Logger.Info("MQTT client started");
    return 0;
  }
}

/*
 * @brief           Gets the number of seconds since UNIX epoch until now.
 * @return uint32_t Number of seconds.
 */
static uint32_t getEpochTimeInSecs() { return (uint32_t)time(NULL); }

static void establishConnection()
{
  connectToWiFi();
  initializeTime();
  initializeIoTHubClient();
  (void)initializeMqttClient();
}

int messageID = 0;
static void getTelemetryPayload(az_span payload, az_span* out_payload) {
  az_span original_payload = payload;

  payload = az_span_copy(payload, AZ_SPAN_FROM_STR("{\"deviceID\":\""));
  payload = az_span_copy(payload, az_span_create_from_str(IOT_CONFIG_DEVICE_ID));
  payload = az_span_copy(payload, AZ_SPAN_FROM_STR("\", \"messageID\":"));
  (void)az_span_i32toa(payload, messageID, &payload);
  messageID++;
  payload = az_span_copy(payload, AZ_SPAN_FROM_STR(", \"PPM\":"));
  (void)az_span_i32toa(payload, co2_C, &payload);
  //(void)az_span_i32toa(payload, moving_average, &payload);
  payload = az_span_copy(payload, AZ_SPAN_FROM_STR(", \"AQI\":"));
  (void)az_span_i32toa(payload, AQI, &payload);
  payload = az_span_copy(payload, AZ_SPAN_FROM_STR("}"));
  payload = az_span_copy_u8(payload, '\0');

  *out_payload = az_span_slice(
      original_payload, 0, az_span_size(original_payload) - az_span_size(payload) - 1);
}


static void sendTelemetry()
{
  az_span telemetry = AZ_SPAN_FROM_BUFFER(telemetry_payload);

  Logger.Info("Sending telemetry ...");

  // The topic could be obtained just once during setup,
  // however if properties are used the topic need to be generated again to reflect the
  // current values of the properties.
  if (az_result_failed(az_iot_hub_client_telemetry_get_publish_topic(
          &client, NULL, telemetry_topic, sizeof(telemetry_topic), NULL)))
  {
    Logger.Error("Failed az_iot_hub_client_telemetry_get_publish_topic");
    return;
  }

  getTelemetryPayload(telemetry, &telemetry);

  if (esp_mqtt_client_publish(
          mqtt_client,
          telemetry_topic,
          (const char*)az_span_ptr(telemetry),
          az_span_size(telemetry),
          MQTT_QOS1,
          DO_NOT_RETAIN_MSG)
      == 0)
  {
    Logger.Error("Failed publishing");
  }
  else
  {
    Logger.Info("Message published successfully");
  }
}

// Arduino setup and loop main functions.

int counter = 0;
int numMeasurements = 10;

void setup() {

  Serial.println("Waiting for MQ135 to warm up...");
  delay(20000);
  
  pinMode(34, INPUT);
  pinMode(35, INPUT);

  establishConnection(); 
}

void loop()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    connectToWiFi();
  }
#ifndef IOT_CONFIG_USE_X509_CERT
  else if (sasToken.IsExpired())
  {
    Logger.Info("SAS token expired; reconnecting with a new one.");
    (void)esp_mqtt_client_destroy(mqtt_client);
    initializeMqttClient();
  }
#endif
  else if (millis() > next_telemetry_send_time_ms)
  {

    float rzero = gasSensor.getRZero();
    Serial.print("RZero: ");
    Serial.println(rzero);

    int reading = gasSensor.getPPM();
    //Serial.print("CO2 PPM: ");
    //Serial.println(reading);
    //delay(1000);

    int sensorMovingAverage = mqMovingAvg.reading(reading);

    co2_C = mqMovingAvg.getAvg();
    Serial.print("CO2 PPM: ");
    Serial.println(reading);

    Serial.print("CO2 AQI: ");
    co2_AQI = calculeCo2AQI();
    Serial.println(co2_AQI);


    digitalWrite(sharpLedPower,LOW);
    delayMicroseconds(samplingTime);

    voMeasured = analogRead(sharpMeasurePin);
    //Serial.println("values measured");

    delayMicroseconds(deltaTime);
    digitalWrite(sharpLedPower,HIGH);
    delayMicroseconds(sleepTime);

    voltage = (voMeasured)*(5.000/1024);
    dustDensity = ((voltage - ZERO_VALUE) * 255) / (5 - ZERO_VALUE);
    if (dustDensity < 0)
      dustDensity = 0;
    
    meanBuff += dustDensity;
    meanCount++;

    if (meanCount == buffSize) {
      mean = meanBuff / meanCount;
      //Serial.print("Mean ppms: ");
      //Serial.println(mean);
      Serial.print(mean);
      Serial.print(";");

      meanCount = 0;
      meanBuff = 0;
    }

    moving_average = MovingAverage(dustDensity);
    Serial.print(moving_average);
    Serial.print(";");


    buffer[bufferCounter] = dustDensity;
    bufferCounter++;

    pm25_AQI = calculePmAQI();

    if (pm25_AQI < co2_AQI) AQI = co2_AQI;
    else AQI = pm25_AQI;

    bufferCounter = 0;
    memset(buffer, 0, sizeof(buffer));

    counter++;

    if (counter == numMeasurements) {
      sendTelemetry();
      next_telemetry_send_time_ms = millis() + TELEMETRY_FREQUENCY_MILLISECS;

      counter = 0;
    }

    Serial.println("");

    delay(500);
  }
}
