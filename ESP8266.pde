/**

 **/

#if defined(ESP32)
  #include <WiFiMulti.h>
  WiFiMulti wifiMulti;
  #define DEVICE "ESP32"
#elif defined(ESP8266)
  #include <ESP8266WiFiMulti.h>
  ESP8266WiFiMulti wifiMulti;
  #define DEVICE "MIC1"
#endif

#define WIFI_SSID "PSSLAN"
#define WIFI_PASSWORD "PostHorn15"

#include "EspMQTTClient.h"

EspMQTTClient MQTT_client(
  WIFI_SSID,
  WIFI_PASSWORD,
  "192.168.0.114",  // MQTT Broker server domain or ip
  "MQTTpi",         // Can be omitted if not needed
  "MQTTpi",         // Can be omitted if not needed
  "Mic1",           // Client name that uniquely identify your device for mqtt
  1883              // The MQTT port, default to 1883. this line can be omitted
);
// Triggers the measurement
bool message_global = false;
String measurement_name;
int n = 1;

#include <InfluxDbClient.h>

// InfluxDB  server url
#define INFLUXDB_URL "http://192.168.0.114:8086"

// InfluxDB v1 database name
#define INFLUXDB_DB_NAME "fft"
#define INFLUXDB_USER "admin"
#define INFLUXDB_PASSWORD "gopi"

// Set timezone
#define TZ_INFO "CET-1CEST,M3.5.0,M10.5.0/3"
#define NTP_SERVER1  "pool.ntp.org"
#define NTP_SERVER2  "time.nis.gov"
#define WRITE_PRECISION WritePrecision::MS
#define MAX_BATCH_SIZE 70
#define WRITE_BUFFER_SIZE 140

// InfluxDB client
InfluxDBClient influx_client(INFLUXDB_URL, INFLUXDB_DB_NAME);
// Data write point

// Number for loops to sync time using NTP
int iterations = 0;

//-------------------------FFT INIT------------------------------
#include "arduinoFFT.h"  // FFT Lib

arduinoFFT FFT = arduinoFFT(); /* Create FFT object */

// FFT init
#define CHANNEL A0 // ADC0 for microphone input
const uint16_t samples = 128; //This value MUST ALWAYS be a power of 2
const double samplingFrequency = 1000; //Hz, must be less than 10000 due to ADC

unsigned int sampling_period_us;
unsigned long microseconds;

//const int measure_offset_low = 0;     // cut the measurement at the low end for better result
//const int measure_offset_high = 0;  // cut the measurement at the high end for better result

double vReal[samples];
double vImag[samples];

double max_pegel = 0;

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

void setup() {
  sampling_period_us = round(1000000*(1.0/samplingFrequency));

  Serial.begin(115200);

  // Setup wifi
  WiFi.mode(WIFI_STA);
  wifiMulti.addAP(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to wifi");
  while (wifiMulti.run() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  Serial.println();
  MQTT_client.enableDebuggingMessages(); // Enable debugging messages sent to serial output
  // Set InfluxDB 1 authentication params
  influx_client.setConnectionParamsV1(INFLUXDB_URL, INFLUXDB_DB_NAME, INFLUXDB_USER, INFLUXDB_PASSWORD);


  // Accurate time for batch write
  timeSync(TZ_INFO, NTP_SERVER1, NTP_SERVER2);

  // Check server connection
  if (influx_client.validateConnection()) {
    Serial.print("Connected to InfluxDB: ");
    Serial.println(influx_client.getServerUrl());
  } else {
    Serial.print("InfluxDB connection failed: ");
    Serial.println(influx_client.getLastErrorMessage());
  }

  // Enable messages batching and retry buffer
  influx_client.setWriteOptions(WriteOptions().writePrecision(WRITE_PRECISION).batchSize(MAX_BATCH_SIZE).bufferSize(WRITE_BUFFER_SIZE));
}

// This function is called once everything is connected (Wifi and MQTT)
// WARNING : YOU MUST IMPLEMENT IT IF YOU USE EspMQTTClient
void onConnectionEstablished()
{
  //Subscribe to "measure"
  MQTT_client.subscribe("measure", [](const String& message) {
      message_global = true;
      if( measurement_name == message){
        n++;
      }
      else{
        measurement_name = message;
      }
  });
}

// ------------- compute FFT --------------
void measure_FFT()
{
  microseconds = micros();
  for(int i=0; i<samples; i++)
  {
      vReal[i] = analogRead(CHANNEL);
      vImag[i] = 0;
      microseconds += sampling_period_us;
  }

  FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
  FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */
  // Serial.println("Computed magnitudes:");

}

//---------write to db ---------------
void inflush_write()
{
  // Sync time for batching, all the same time
  timeSync(TZ_INFO, NTP_SERVER1, NTP_SERVER2);
  time_t tnow = time(nullptr);

  // Report fft
  if (influx_client.isBufferEmpty()) {
    for (int i = 0; i < samples; i++) {
      double abscissa;
      abscissa = ((i * 1.0 * samplingFrequency) / samples);

      String SerialData_a="";
      String SerialData_v="";
      SerialData_a = String(abscissa,1);
      SerialData_v = String(vReal[i],1);
      Point all("fft_all");
      all.addTag("bin", SerialData_a);      // bin
      all.addTag("pegel", SerialData_v);    // pegel
      all.addField("id", measurement_name); // measurement name
      all.addField("id_number", n);         // measurement name
      all.setTime(tnow);                    //set the time

        // Write point buffer
        influx_client.writePoint(all);
    }
    Point measure_handover("measure_filter");
    measure_handover.addField("measurement", measurement_name);
    influx_client.writePoint(measure_handover);
  }

    // If no Wifi signal, try to reconnect it
  if (wifiMulti.run() != WL_CONNECTED) {
    Serial.println("Wifi connection lost");
  }

  // End of the iteration - force write of all the values into InfluxDB as single transaction
  if (!influx_client.flushBuffer()) {
    Serial.print("InfluxDB flush failed: ");
    Serial.println(influx_client.getLastErrorMessage());
    Serial.print("Full buffer: ");
    Serial.println(influx_client.isBufferFull() ? "Yes" : "No");
  }
}

void loop() {
  if (message_global == true)
  {
     measure_FFT();
     inflush_write();
     message_global = false;
  }
  MQTT_client.loop();

}
