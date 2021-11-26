/*
  Codi final que unifica tots els sensors utilitzats. Pel desenvolupament del mateix,
  s'ha extret el codi de diverses fonts indicades a continuació:
  - Brúixola (HMC5883L): https://github.com/jarzebski/Arduino-HMC5883L
  - Acceleròmetre i giròscop (MPU6050): https://randomnerdtutorials.com/esp32-mpu-6050-accelerometer-gyroscope-arduino/
  - Sensor de temperatura i humitat (DHT11): DHT Sensor Library Adafruit
*/

// S'inclouen les llibreries
#include <Adafruit_MPU6050.h>   // MPU6050 Adafruit
#include <Adafruit_Sensor.h>
#include <DHT.h>                // DHT11 Adafruit
#include <DHT_U.h>              // DHT11
#include <HMC5883L.h>           // HMC5883L inclosa en la carpeta
#include <Wire.h>               // I2C
#include <WiFi.h>
#include <PubSubClient.h>               

// Declarem funcions
float get_corriente(int n_muestras);
void setup_wifi();
void callback(char* topic, byte* message, unsigned int length);
void reconnect();

// Definim constants
#define DHTPIN 18               // Pin digital connectat al sensor DHT
#define DHTTYPE DHT11           // S'indica que s'utilitza concretament el DHT11
#define WATER_LEVEL 4           // Tipus de sensor d'inundació 
#define CURRENT_PIN 34

// Declarem objectes
Adafruit_MPU6050 mpu;
DHT_Unified dht(DHTPIN, DHTTYPE);
HMC5883L compass;

// Variables
uint32_t delayMS;
int nivell_aigua = 0;
float Sensibilidad = 0.09625; //Sensibilitat en Volts/Amper per sensor de 20 A


// Replace the next variables with your SSID/Password combination
const char* ssid = "iPhone de: Marta";
const char* password = "test1234";

// Add your MQTT Broker IP address, example:
//const char* mqtt_server = "192.168.1.144";
const char* mqtt_server = "172.20.10.12"; //YOUR_MQTT_BROKER_IP_ADDRESS

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

void setup(void) {

  // Inicialització del serial
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  while (!Serial)
    delay(10);

  // Intenta inicialitzar el MPU6050
  if (!mpu.begin()) {
    Serial.println("No s'ha pogut trobar el xip MPU6050");
    while (1) {
      delay(10);
    }
  }

  // ----------------- MPU6050 -----------------
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);   // Estableix el rang de mesura de l'acceleròmetre
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);        // Estableix el rang de mesura del giròscop
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);      // Estableix el filtre


  // Intenta inicialitzar el HMC5883L
  while (!compass.begin())
  {
    Serial.println("No s'ha trobat un sensor HMC5883L vàlid, comprova el cablejat");
    delay(500);
  }

  // --------------- HMC5883L ---------------
  compass.setRange(HMC5883L_RANGE_1_3GA);             // Estableix el rang de mesura
  compass.setMeasurementMode(HMC5883L_CONTINOUS);     // Estableix el mode de mesura
  compass.setDataRate(HMC5883L_DATARATE_30HZ);        // Estableix la velocitat de dades
  compass.setSamples(HMC5883L_SAMPLES_8);             // Estableix el nombre mitjà de mostres
  compass.setOffset(24, -166);                        // Es calibra establint l'offset. Els valors es calculen amb el codi calibratge_HMC5883L.ino

  // ----------------- DHT11 ----------------
  dht.begin();                                        // Inicialitza el DHT11
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  delayMS = sensor.min_delay / 1000;

  // ----------------- VMA ------------------
  pinMode(WATER_LEVEL, INPUT);

  delay(100);
}

void loop() {
  // --------------- Wi-Fi ------------------
    if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > 5000) {
    lastMsg = now;
  }
  // --------------- HMC5883L ---------------
  /* S'obtenen els nous esdeveniments del sensor amb les lectures */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  Vector norm = compass.readNormalize();

  // Es calcula la direcció
  float heading = atan2(norm.YAxis, norm.XAxis);

  // Es calcula la declinació magnètica = (deg + (min / 60.0)) / (180 / M_PI)
  float declinationAngle = (1.0 + (30.0 / 60.0)) / (180 / M_PI);  // La declinació magnètica a Barcelona és 1º 30'
  heading += declinationAngle;

  // Corregir la direcció: 360 < direcció < 0
  if (heading < 0)
  {
    heading += 2 * PI;
  }

  if (heading > 2 * PI)
  {
    heading -= 2 * PI;
  }

  float headingDegrees = heading * 180 / M_PI;    // Converteix la direcció de radians a graus

  // --------------- VMA ----------------
  nivell_aigua = analogRead(WATER_LEVEL);

  /* Imprimim per pantalla els valors */
  // ------------- MPU6050 --------------
  // Acceleròmetre
  Serial.print("Acceleració X: ");
  Serial.print(a.acceleration.x - 0.58);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y + 0.19);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z + 0.69);
  Serial.println(" m/s^2");

  // Giròscop
  Serial.print("Rotació X: ");
  Serial.print(g.gyro.x + 0.03);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y + 0.08);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z + 0.03);
  Serial.println(" rad/s");

  // ------------- HMC5883L --------------
  // Brúixola
  Serial.print("Direcció = ");
  Serial.print(heading);
  Serial.print(" Graus = ");
  Serial.print(headingDegrees);
  Serial.print("°");
  Serial.println();

  // ----------------- DHT11 ------------------
  // S'obté la temperatura
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("Error llegint la temperatura"));
  }
  // S'imprimeix el valor de la temperatura
  else {
    Serial.print(F("Temperatura: "));
    Serial.print(event.temperature);
    Serial.println(F("°C"));
  }

  // --------------- VMA303 ----------------
  // S'imprimeix el nivell d'aigua
  Serial.print("Nivell d'aigua: ");
  Serial.println(nivell_aigua);

  // --------------- ACS712 ----------------
  float I = get_corriente(20);    // Fem un filtre fent mitjana aritmètica, si no volem filtre escriure un 1
  Serial.print("Corriente: ");
  Serial.println(I, 3); // Visualitzem 3 decimals

  Serial.println("");
  delay(1000);
}


// ------------ - Filtre del sensor de corrent ------------------ 
float get_corriente(int n_muestras) {
  float voltajeSensor;
  float corriente = 0;

  if (n_muestras == 1) {
    voltajeSensor = analogRead(CURRENT_PIN) * (3.3 / 4095.0); // Lectura del sensor
    corriente = (voltajeSensor - 2.1); // Equació per obtenir la corrent
  }
  else {
    for (int i = 0; i < n_muestras; i++)
    {
      voltajeSensor = analogRead(CURRENT_PIN) * (3.3 / 4095.0); // Lectura del sensor
      corriente = corriente + (voltajeSensor - 2.1); // Equació per obtenir la corrent
    }
    corriente = corriente / n_muestras;
  }
  return (corriente);
}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // Feel free to add more if statements to control more GPIOs with MQTT

  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off". 
  // Changes the output state according to the message
  if (String(topic) == "esp32/output") {
    Serial.print("Changing output to ");
    if(messageTemp == "on"){
      Serial.println("on");
    }
    else if(messageTemp == "off"){
      Serial.println("off");
    }
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("esp32/output");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
