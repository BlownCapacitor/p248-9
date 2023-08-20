
#include <WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include "DHT.h"
#include <Adafruit_BMP085.h>
Adafruit_BMP085 bmp;
#define WLAN_SSID       "*"
#define WLAN_PASS       "*"
const long INTERVAL = 60000;
unsigned long previousMillis = 0;
byte rpin = 13;
byte gpin = 12;
byte bpin = 14;
byte rchannel = 0;
byte gchannel = 1;
byte bchannel = 2;
byte resolution = 8;
int frequency = 8000;
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883                   
#define AIO_USERNAME  "*"
#define AIO_KEY       "*"
#define DHTPIN 2
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

WiFiClient client;

Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

Adafruit_MQTT_Publish temperature = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temperature");
Adafruit_MQTT_Publish humidity = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/hum280");
Adafruit_MQTT_Publish pressure = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/level");
Adafruit_MQTT_Publish altitude = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Altitude");
Adafruit_MQTT_Publish heatindex = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/heatindex");
Adafruit_MQTT_Publish dewpoint = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/dew280");
Adafruit_MQTT_Publish farenheit = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temperaturef");
Adafruit_MQTT_Publish kelvin = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temperaturek");
Adafruit_MQTT_Subscribe hexadec = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/rgb");

void MQTT_connect();

void setup() {
  Serial.begin(9600);
    ledcSetup(rchannel , frequency , resolution);
     ledcSetup(gchannel , frequency , resolution);
    ledcSetup(bchannel , frequency , resolution);
  ledcAttachPin(rpin , rchannel);
  ledcAttachPin(gpin , gchannel);
  ledcAttachPin(bpin , bchannel);
  if (!bmp.begin()) {
Serial.println("Could not find a valid BMP085 sensor, check wiring!");
  while (1) {}}
  dht.begin();
  delay(10);

  Serial.println(F("Adafruit MQTT demo"));

  Serial.println(); Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.println("WiFi connected");
  Serial.println("IP address: "); Serial.println(WiFi.localIP());
  mqtt.subscribe(&hexadec);
}

uint32_t x=0;

void loop() {

    MQTT_connect();

  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(5000))) {
    if (subscription == &hexadec) {
      Serial.print(F("Got: "));
      Serial.println((char *)hexadec.lastread);
     String hexstring = (char *)hexadec.lastread;
    long number = (long) strtol( &hexstring[1], NULL, 16);
     int r = number >> 16;
     int g = number >> 8 & 0xFF;
     int b = number & 0xFF;
 
    Serial.print("red is ");
    Serial.println(r);
    Serial.print("green is ");
    Serial.println(g);
    Serial.print("blue is ");
    Serial.println(b);     
    ledcWrite(rchannel, 255 - r);
    ledcWrite(gchannel, 255 - g);
    ledcWrite(bchannel, 255 - b);
    }
  }
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  float p = bmp.readPressure();
  float a = bmp.readAltitude(101500);
  float hi = dht.computeHeatIndex(t, h, false);
  float f = dht.readTemperature(true);
  float k = t + 273.15;
  float dp = (t - (100 - h) / 5); 
   temperature.publish(t);
  humidity.publish(h);
  pressure.publish(p);
  altitude.publish(a);
  farenheit.publish(f);
  kelvin.publish(k);
  heatindex.publish(hi);
  dewpoint.publish(dp);

      delay(60000);

  

}
void MQTT_connect() {
  int8_t ret;

  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { 
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  
       retries--;
       if (retries == 0) {
         while (1);
       }
  }
  Serial.println("MQTT Connected!");
}
