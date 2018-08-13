// CHANGELOG
// + WEMOS D1 mini
// + LUX
// + ECHO
// + PINS RELÉ


// LIBRARIES
#include <Wire.h>
#include <BH1750.h>
#include <DHT.h>;
#include <NewPing.h>
#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

// PINLAYOUT & SETTINGS

#define soilPin A0
#define ledPin D0
// SCL D1
// SDA D2
#define ECHO_PIN     D3
#define TRIGGER_PIN  D4
#define DHTPIN D5
// D6
// AMARILLO D7 -> RELE IN2
// NARANJA D8 -> RELE IN1

#define WLAN_SSID       "leviwifi "
#define WLAN_PASS       "borges2260"

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883                   // use 8883 for SSL
#define AIO_USERNAME    "nanmaran"
#define AIO_KEY         "db1c233f1e7643fe97a31d163381ddf2"


#define MAX_DISTANCE 400
#define DHTTYPE DHT22   // DHT 22  (AM2302)

float hum;  //Stores humidity value
float temp; //Stores temperature value
int soil;
int intHum;
int intTemp;
int sensorHumedad = 0;  // variable to store the value coming from the sensor
int sensorSoil = 0;
int duration;
int distance;
int altura;
int error;

String luz;
String sensorsData = "hum,temp,soil,lux,echo*"; //Data que se envía al ESP8266
char sentData;

// INITIALIZE

DHT dht(DHTPIN, DHTTYPE, 15); //// Initialize DHT sensor for normal 16mhz Arduino
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
BH1750 lightMeter;

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Publish humedadCloud = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/humedad");
Adafruit_MQTT_Publish temperaturaCloud = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temperatura");
Adafruit_MQTT_Publish soilCloud = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/soil");
Adafruit_MQTT_Publish luxCloud = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/lux");
Adafruit_MQTT_Publish echoCloud = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/echo");
// Adafruit_MQTT_Subscribe onoffbutton = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/onoff");
void MQTT_connect();

//
// SETUP
//

void setup(){

  Serial.begin(9600);
  dht.begin();
  Wire.begin();
  lightMeter.begin();
  pinMode(ledPin,OUTPUT);
  // Initialize the I2C bus (BH1750 library doesn't do this automatically)
  Wire.begin();
  // On esp8266 you can select SCL and SDA pins using Wire.begin(D4, D3);

  Serial.println(F("mightyJungle V2.0"));
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(ledPin, HIGH);
    delay(500);
    Serial.print(".");
    digitalWrite(ledPin, LOW);
  }
  digitalWrite(ledPin, HIGH);
  Serial.println();
  Serial.println("WiFi connected");
  Serial.println("IP address: "); Serial.println(WiFi.localIP());
  delay(3000);
  digitalWrite(ledPin, LOW);


}


void loop() {

  // VARIABLES

  // READ DATA
  hum = dht.readHumidity();
  temp = dht.readTemperature();
  sensorSoil = analogRead(soilPin);
  soil = map(sensorSoil,300,1024,100,0);
  uint16_t lux = lightMeter.readLightLevel();
  luz = String(lux); //Convertion to String
  digitalWrite(TRIGGER_PIN,HIGH);
  delay(1);
  digitalWrite(TRIGGER_PIN,LOW);
  duration = pulseIn(ECHO_PIN,HIGH);
  distance = (duration/2)/29.1;

  // CALIBRAR

  altura = 70 - distance;

  // PRINT DATA
  Serial.print("Humedad: %");
  Serial.println(hum);
  Serial.print("Temperatura: ");
  Serial.print(temp);
  Serial.println("ºC");
  Serial.print("Soil: %");
  Serial.println(soil);
  Serial.print("Light: ");
  Serial.print(luz);
  Serial.println(" lx");

  if(distance>500 or distance==0) Serial.println("Out of Range");
  else{
    Serial.print("Altura: ");
    Serial.print(altura);
    Serial.println("cm");
  }

  // UPLOAD DATA

  MQTT_connect();
  Serial.println("Sending data");
  if (! humedadCloud.publish(hum)) {
    error ++;
  }
  if (! temperaturaCloud.publish(temp)) {
    error ++;
  }
  if (! soilCloud.publish(soil)) {
    error ++;
  }
  if (! luxCloud.publish(lux)) {
    error ++;
  }
  if (! echoCloud.publish(altura)) {
    error ++;
  }

  if (error < 1){
    Serial.println("DATA SENT");
    digitalWrite(ledPin, HIGH);
    delay(3000);
    digitalWrite(ledPin, LOW);
  }
  else{
    Serial.print("ERROR ");
    Serial.println(error);
    digitalWrite(ledPin, HIGH);
    delay(1000);
    digitalWrite(ledPin, LOW);
    delay(1000);
    digitalWrite(ledPin, HIGH);
    delay(1000);
    digitalWrite(ledPin, LOW);
    delay(1000);
    digitalWrite(ledPin, HIGH);
    delay(1000);
    digitalWrite(ledPin, LOW);
    delay(1000);
  }
  error = 0;

  // DELAY 5 MIN

  delay(30000);

  // delay(1000);
}

void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }
  Serial.println("MQTT Connected!");
}
