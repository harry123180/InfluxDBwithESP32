
#if defined(ESP32)
#include <WiFiMulti.h>
WiFiMulti wifiMulti;
#define DEVICE "ESP32"

#elif defined(ESP8266)
#include <ESP8266WiFiMulti.h>
ESP8266WiFiMulti wifiMulti;
#define DEVICE "ESP8266"
#endif

#include <WiFiManager.h>
#include <InfluxDbClient.h>
#include <InfluxDbCloud.h>

// WiFi AP SSID
#define WIFI_SSID "SHC"
// WiFi password
#define WIFI_PASSWORD "datadata"
// InfluxDB v2 server url, e.g. https://eu-central-1-1.aws.cloud2.influxdata.com (Use: InfluxDB UI -> Load Data -> Client Libraries)
#define INFLUXDB_URL "http://125.229.142.15:9453"
// InfluxDB v2 server or cloud API token (Use: InfluxDB UI -> Data -> API Tokens -> Generate API Token)
#define INFLUXDB_TOKEN "yoN726RbcJejGIjB4wAKblcVwidlM2B65QV9tWbYh96GQ1px8p4l8AY_OWA2Pe7bTRhrLFtFz9ll5zGYx6qCBg=="
// InfluxDB v2 organization id (Use: InfluxDB UI -> User -> About -> Common Ids )
#define INFLUXDB_ORG "K1082"
// InfluxDB v2 bucket name (Use: InfluxDB UI ->  Data -> Buckets)
#define INFLUXDB_BUCKET "test"

// Set timezone string according to https://www.gnu.org/software/libc/manual/html_node/TZ-Variable.html
// Examples:
//  Pacific Time: "PST8PDT"
//  Eastern: "EST5EDT"
//  Japanesse: "JST-9"
//  Central Europe: "CET-1CEST,M3.5.0,M10.5.0/3"
#define TZ_INFO "GMT-8"

// InfluxDB client instance with preconfigured InfluxCloud certificate
InfluxDBClient client(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN, InfluxDbCloud2CACert);

// Data point
Point sensor("mem2");
WiFiManager wm;
void setup() {
  Serial.begin(115200);
 
  wm.resetSettings();
  bool res;
    // res = wm.autoConnect(); // auto generated AP name from chipid
    // res = wm.autoConnect("AutoConnectAP"); // anonymous ap
  res = wm.autoConnect("AutoConnectAP","password"); // password protected ap
  
  // Setup wifi
  //WiFi.mode(WIFI_STA);
  //wifiMulti.addAP(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting to wifi");
  if(!res) {
          Serial.println("Failed to connect");
          // ESP.restart();
      } 
      else {
          //if you get here you have connected to the WiFi    
          Serial.println("connected...yeey :)");
      }
  // Add tags
  sensor.addTag("device", DEVICE);
  //sensor.addTag("SSID", WiFi.SSID());

  timeSync(TZ_INFO, "pool.ntp.org", "time.nis.gov");

  // Check server connection
  if (client.validateConnection()) {
    Serial.print("Connected to InfluxDB: ");
    Serial.println(client.getServerUrl());
  } else {
    Serial.print("InfluxDB connection failed: ");
    Serial.println(client.getLastErrorMessage());
  }
}
void loop() {
  sensor.clearFields();
  for(int i=0;i<128;i++){
  sensor.addField("bin"+String(i),random(0, 100));
  }
  Serial.print("Writing: ");
  Serial.println(sensor.toLineProtocol());

  Serial.println(wm.getWLStatusString());
  if (!client.writePoint(sensor)) {
    Serial.print("InfluxDB write failed: ");
    Serial.println(client.getLastErrorMessage());
  }
  Serial.println("Wait 10s");
  delay(10000);
}
