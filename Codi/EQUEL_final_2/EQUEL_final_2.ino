/*
  Codi que ajunta la sensòrica amb la comunicació del ESP32 al Node-Red i el control dels motors de forma manual.
*/

// S'inclouen les llibreries
#include <Adafruit_MPU6050.h>   // MPU6050 Adafruit
#include <Adafruit_Sensor.h>
#include <DHT.h>                // DHT11 Adafruit
#include <DHT_U.h>              // DHT11
#include <HMC5883L.h>           // HMC5883L inclosa en la carpeta
#include <Wire.h>               // I2C
#include <SoftwareSerial.h>     // GPS
#include <WiFi.h>               // Comuniació
#include <PubSubClient.h>       // Comunicació
#include "ArduinoJson.h"        // Json per GPS            

// Definim constants
#define DHTPIN 18               // Pin digital connectat al sensor DHT
#define DHTTYPE DHT11           // S'indica que s'utilitza concretament el DHT11
#define WATER_LEVEL 35          // Tipus de sensor d'inundació
#define CURRENT_PIN 34
#define V_BAT_PIN 36
#define n_muestras 200          // Filtre sensor corrent

#define RIGHT_PWM 32
#define LEFT_PWM  33
#define PWM1_Ch    0
#define PWM1_Res   8
#define PWM1_Freq  5000
#define PWM2_Ch    1
#define CONTROL_RIGHT 5
#define CONTROL_LEFT  25
#define POWER 27

#define angle_margin 5          // 5 graus d'error en la orientació
#define distance_margin 2.5     // 2,5 metres d'error en la distancia

// Declarem objectes
Adafruit_MPU6050 mpu;
DHT_Unified dht(DHTPIN, DHTTYPE);
HMC5883L compass;

// Variables
uint32_t delayMS;
int nivell_aigua = 0;
float gpstime, gpsdate, latitude, longitude, altitude, gpsknots, gpstrack;
char latNS, lonEW, gpsstatus;
int fixquality, numsatelites;
volatile int ptr = 0;
volatile bool flag = true;
volatile char redbuffer[120];
volatile char blubuffer[120];
String inputgpsSerial = "";
boolean IsReadygpsSerial = false;
const char* ssid = "iPhone de: Marta";    // Replace SSID
const char* password = "test1234";        // Replace Password combination
const char* mqtt_server = "172.20.10.8"; // Add your MQTT Broker IP address
long lastMsg = 0;
char msg[50];
int value = 0;
float Sensibilitat = 113;   //Sensibilitat en mV/A per sensor de 20 A
float mVref = 2244;         // V quan I = 0 A
int ADC_reading;
float mV, corrent, mV_total;
float estat_ta, inundacio, estat_ind, estat_current, estat_mpu, estat_gps, estat_hmc = 0.0;    // Sesnors conectats o no conectats
unsigned char cnt_gps = 0;

unsigned char inici_hmc = 1;  // Flag pel setup del hmc
unsigned char inici_mpu = 1;  // Flag pel setup del mpu

unsigned char activacio_navegacio_autonoma = 0;
float waypoints[8] = {41.408631146270274, 2.2262997046293087, 41.40864019619339, 2.226228607155201,
                      41.40860500204085, 2.226216533999206, 41.40859695766045, 2.2262768997791405
                     };
unsigned char angle_correct = 0;    // Flag que indica si estem disn els marges d'error de la orientacio
unsigned int dades_cnt = 0; // Compatdor per no enviar dades cada us.
float lat1_deg, long1_deg; // Coordenades inicials
float lat2_deg, long2_deg; // Coordenades finals
float distance_m, angle_deg; // Distància i angle entre punt A i punt B
float angle_dif; // Diferència entre el nord i l'angle entre el punt A i B

String motor1, motor2;
int lectura_right_motor, lectura_left_motor;
unsigned char right_motor, left_motor, switch_on, valor;


// JSON
char buffer[100];
const int capacity = JSON_OBJECT_SIZE(2);
StaticJsonDocument<capacity> GPS_data;

// Funcions
void wifi(void);
void dades_imu(void);
void dades_bruixola(void);
void dades_temperatura(void);
void dades_nivell_aigua(void);
void dades_corrent(void);
void dades_gps(void);
void dades_tensio(void);

void get_lat_long(unsigned char waypoint);
void get_gps_info();
void get_dist_bearing();
void show_compass();
void correct_angle_dif();
void correct_distance_m();

void listen();
void read(char nextChar);
bool CheckSum(char* msg);
float DegreeToDecimal(float num, byte sign);
void parseString(char* msg);
void messageGGA(char* msg);
void messageRMC(char* msg);
void AllSentences();
void SelectSentences();
void SelectGGAonly();
void SelectMode(int mode);
void setup_wifi();
void callback(char* topic, byte* message, unsigned int length);
void reconnect();

SoftwareSerial gpsSerial(16, 17);   // gpsSerial(receive from GPS,transmit to the GPS module)
WiFiClient espClient;
PubSubClient client(espClient);

////////////////////////////////////////////////////////////////////// VOID SETUP /////////////////////////////////////////////////

void setup(void) {

  // Inicialització del serial
  Serial.begin(115200);
  analogReadResolution(12);
  // --------------- Wi-Fi ------------------
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  while (!Serial) {
    delay(10);
  }
  // ----------------- DHT11 ----------------
  dht.begin();                   // Inicialitza el DHT11
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  delayMS = sensor.min_delay / 1000;

  // ----------------- VMA ------------------
  pinMode(WATER_LEVEL, INPUT);

  // ----------------- ACS712 ---------------
  pinMode(CURRENT_PIN, INPUT);

  // ----------------- GPS ------------------
  pinMode(16, INPUT);   //Receive from the GPS device (the NMEA sentences) - RX
  pinMode(17, OUTPUT);  //Transmit to the GPS device - TX

  gpsSerial.begin(9600);      // Connect to the GPS module
  SelectSentences();
  pinMode(POWER, OUTPUT);


  // ----------------- Motors ---------------
  // Initialize the PWM and DIR pins as digital outputs.
  pinMode(RIGHT_PWM, OUTPUT);
  ledcAttachPin(RIGHT_PWM, PWM1_Ch);
  ledcSetup(PWM1_Ch, PWM1_Freq, PWM1_Res);

  pinMode(LEFT_PWM, OUTPUT);
  ledcAttachPin(LEFT_PWM, PWM2_Ch);
  ledcSetup(PWM2_Ch, PWM1_Freq, PWM1_Res);

  pinMode(CONTROL_LEFT, OUTPUT);
  pinMode(CONTROL_RIGHT, OUTPUT);

  delay(100);
}

////////////////////////////////////////////////////////////////////// VOID LOOP /////////////////////////////////////////////////

void loop() {


  wifi();
  client.publish("esp32/alive", "alive");
  delay(500);
  client.publish("esp32/alive", "0");

  if (activacio_navegacio_autonoma == 0) {
    dades_imu();          // Dades del imu i enviar al node red
    dades_bruixola();     // Dades de la bruixola i enviar al node red
    dades_temperatura();  // Dades de la temperatura i enviar al node red
    dades_nivell_aigua(); // Dades del nivell d'aigua i enviar al node red
    dades_corrent();      // Dades del corrent i enviar al node red
    dades_gps();          // Dades del gps i enviar al node red
    dades_tensio();       // Dades de tensio bateria i enviar al node red

    Serial.println("");
    delay(1000);
  }

  else {

    for (unsigned char i = 0; i < 4; i++) {
      get_lat_long(2 * i); // Obtenim latitud i longitud d'el punt on volem anar

      while (distance_m > distance_margin) {  // Mentre no haguem arribat al lloc obtenim telemetria i ens apropem

        if ((estat_gps == 0.0) or (estat_hmc == 0.0)) { // Si perdem connexio del gpso o bruixola aturem motors fins reprendre connexio

          lectura_right_motor  = 0;
          lectura_left_motor = 0;
          digitalWrite(CONTROL_RIGHT, HIGH);
          digitalWrite(CONTROL_LEFT, HIGH);
          right_motor = map(lectura_right_motor, 0, 100, 0, 50);
          left_motor = map(lectura_left_motor, 0, 100, 0, 50);
          ledcWrite(PWM1_Ch, right_motor);
          ledcWrite(PWM2_Ch, left_motor);
        }

        wifi();

        if (dades_cnt == 1000) {      // Les dades dels sensors s'envien cada x temps, nose si posar 1000 o 100 o 1000000...
          dades_cnt = 0;              // Reset del comptador

          dades_imu();          // Dades del imu i enviar al node red
          dades_temperatura();  // Dades de la temperatura i enviar al node red
          dades_nivell_aigua(); // Dades del nivell d'aigua i enviar al node red
          dades_corrent();      // Dades del corrent i enviar al node red
          dades_tensio();       // Dades de tensio bateria i enviar al node red
        }

        // Telemetria
        get_gps_info();         // Obtenim latitud i longitud d'on estem
        get_dist_bearing();     // Obtenim distancia i orientació entre dues coordenades
        show_compass();         // Dades brúxiola i calcul de desviació d'orientació

        // Moviment autonom
        correct_angle_dif();
        if (angle_correct) {    // Només avancem si estem ben orientats, dins els marges d'error
          angle_correct = 0;
          correct_distance_m();
        }
        dades_cnt++;
      }
    }
  }
}

////////////////////////////////////////////////////////////////////// Funcions per obtenir dades de cada sensor i enviarles al pc /////////////////////////////////////////////////


// --------------- Wi-Fi ------------------
void wifi(void) {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > 5000) {
    lastMsg = now;
  }
}

// ------------- MPU6050 - IMU --------------
void dades_imu(void) {
  Serial.print("IMU: ");

  if (!mpu.begin()) {
    Serial.println("No conectada");
    estat_mpu = 0.0;
  }
  else {
    estat_mpu = 1.0;

    if (inici_mpu) {
      inici_mpu = 0;
      // Intenta inicialitzar el MPU6050
      mpu.setAccelerometerRange(MPU6050_RANGE_8_G);   // Estableix el rang de mesura de l'acceleròmetre
      mpu.setGyroRange(MPU6050_RANGE_500_DEG);        // Estableix el rang de mesura del giròscop
      mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);      // Estableix el filtre
    }

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Acceleròmetre
    float accelerationX = a.acceleration.x - 0.47;      // Es sumen els offsets per cada eix
    float accelerationY = a.acceleration.y + 0.01;
    float accelerationZ = a.acceleration.z + 0.69;
    Serial.print("Acceleració X: ");
    Serial.print(accelerationX);
    Serial.print(", Y: ");
    Serial.print(accelerationY);
    Serial.print(", Z: ");
    Serial.print(accelerationZ);
    Serial.println(" m/s^2");
    char accelerationXString[8];
    char accelerationYString[8];
    char accelerationZString[8];
    dtostrf(accelerationX, 1, 2, accelerationXString);  // Converteix el valor a un char array
    dtostrf(accelerationY, 1, 2, accelerationYString);
    dtostrf(accelerationZ, 1, 2, accelerationZString);
    client.publish("esp32/accelerationX", accelerationXString);
    client.publish("esp32/accelerationY", accelerationYString);
    client.publish("esp32/accelerationZ", accelerationZString);

    // Giròscop
    float gyroX = g.gyro.x + 0.03;                      // Es sumen els offsets per cada eix
    float gyroY = g.gyro.y + 0.08;
    float gyroZ = g.gyro.z + 0.02;
    Serial.print("Rotació X: ");
    Serial.print(gyroX);
    Serial.print(", Y: ");
    Serial.print(gyroY);
    Serial.print(", Z: ");
    Serial.print(gyroZ);
    Serial.println(" rad/s");
    char gyroXString[8];
    char gyroYString[8];
    char gyroZString[8];
    dtostrf(gyroX, 1, 2, gyroXString);                   // Converteix el valor a un char array
    dtostrf(gyroY, 1, 2, gyroYString);
    dtostrf(gyroZ, 1, 2, gyroZString);
    client.publish("esp32/gyroX", gyroXString);
    client.publish("esp32/gyroY", gyroYString);
    client.publish("esp32/gyroZ", gyroZString);
  }

  char estatMPUString[8];
  dtostrf(estat_mpu, 1, 2, estatMPUString);
  client.publish("esp32/estat_mpu", estatMPUString);
}

// ------------- HMC5883L - Bruixola --------------
void dades_bruixola(void) {
  Serial.print("Brúixola: ");

  if (!compass.begin())
  {
    Serial.println("No connectada");
    estat_hmc = 0.0;

  }
  else {
    estat_hmc = 1.0;

    if (inici_hmc) {
      inici_hmc = 0;
      compass.setRange(HMC5883L_RANGE_1_3GA);             // Estableix el rang de mesura
      compass.setMeasurementMode(HMC5883L_CONTINOUS);     // Estableix el mode de mesura
      compass.setDataRate(HMC5883L_DATARATE_30HZ);        // Estableix la velocitat de dades
      compass.setSamples(HMC5883L_SAMPLES_8);             // Estableix el nombre mitjà de mostres
      compass.setOffset(24, -166);                        // Es calibra establint l'offset. Els valors es calculen amb el codi calibratge_HMC5883L.ino
    }

    /* S'obtenen els nous esdeveniments del sensor amb les lectures */
    Vector norm = compass.readNormalize();

    // Es calcula la direcció
    float heading = atan2(norm.YAxis, norm.XAxis);

    // Es calcula la declinació magnètica = (deg + (min / 60.0)) / (180 / M_PI)
    float declinationAngle = (1.0 + (30.0 / 60.0)) / (180 / M_PI);  // La declinació magnètica a Barcelona és 1º 30'
    heading += declinationAngle;

    // Corregir la direcció: 360 < direcció < 0
    if (heading < 0) {
      heading += 2 * PI;
    }
    if (heading > 2 * PI) {
      heading -= 2 * PI;
    }

    float headingDegrees = heading * 180 / M_PI;    // Converteix la direcció de radians a graus

    Serial.print("Direcció = ");
    Serial.print(heading);
    Serial.print(" i Graus = ");
    Serial.print(headingDegrees);
    Serial.println("°");

    char headingString[8];
    dtostrf(headingDegrees, 1, 2, headingString);         // Converteix el valor a un char array
    client.publish("esp32/headingDegrees", headingString);
  }

  char estatHmcString[8];
  dtostrf(estat_hmc, 1, 2, estatHmcString);
  client.publish("esp32/estat_hmc", estatHmcString);
}

// ----------------- DHT11 - Temperatura ------------------
void dades_temperatura(void) {
  // S'obté la temperatura
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    estat_ta = 0.0;
    Serial.println(F("Error llegint la temperatura"));
  }
  // S'imprimeix el valor de la temperatura
  else {
    estat_ta = 1.0;
    float temperature = event.temperature;
    Serial.print(F("Temperatura: "));
    Serial.print(temperature);
    Serial.println(F("°C"));
    char tempString[8];
    dtostrf(temperature, 1, 2, tempString);             // Converteix el valor a un char array
    client.publish("esp32/temperature", tempString);
  }

  char estatTaString[8];
  dtostrf(estat_ta, 1, 2, estatTaString);
  client.publish("esp32/estat_ta", estatTaString);
}

// --------------- VMA303 - Nivell aigua ----------------
void dades_nivell_aigua(void) {
  nivell_aigua = analogRead(WATER_LEVEL);

  // S'imprimeix el nivell d'aigua
  Serial.print("Nivell d'aigua: ");

  if (nivell_aigua > 2000) {
    estat_ind = 0.0;
    Serial.println("Sensor desconectat");
  }
  else {
    // Dins els límits bons, enviem que el sensor està funcionant bé
    estat_ind = 1.0;
    if (nivell_aigua < 700) {
      inundacio = 0.0;
    }
    else {
      inundacio = 1.0;
    }
    Serial.println(nivell_aigua);
    Serial.print("Alarma inundació: ");
    Serial.println(inundacio);
    char inundacioString[8];
    dtostrf(inundacio, 1, 2, inundacioString);            // Converteix el valor a un char array
    client.publish("esp32/inundacio", inundacioString);
  }
  char estatIndString[8];
  dtostrf(estat_ind, 1, 2, estatIndString);
  client.publish("esp32/estat_ind", estatIndString);
}

// --------------- ACS712 - Corrent ----------------
void dades_corrent(void) {
  corrent, mV_total = 0;  // Resetejem variable

  for (int i = 0; i < n_muestras; i++) {
    ADC_reading = analogRead(CURRENT_PIN);
    mV_total += (((ADC_reading * 3300) / 4095) * (5 / 3.3)); // Lectura del sensor
    mV = (((ADC_reading * 3300) / 4095) * (5 / 3.3)); // Lectura actual
    corrent += ((mV - mVref) / Sensibilitat); // Equació per obtenir la corrent
  }

  mV = mV_total / n_muestras;
  corrent = corrent / n_muestras;
  Serial.print("Sensor de corrent: ");

  if (corrent < -10.0) {
    estat_current = 0.0;
    Serial.println("Sensor desconectat");
  }
  else {        // Dins els límits bons, enviem que el sensor està funcionant bé
    estat_current = 1.0;
    Serial.print("Lectura ADC: ");
    Serial.print(ADC_reading);
    Serial.print(" bits | ");

    Serial.print("Tensió: ");
    Serial.print(mV);
    Serial.print(" mV | ");

    Serial.print("Corrent: ");
    Serial.print(corrent, 5);   // Visualitzem 3 decimals
    Serial.println(" A");

    char currentString[8];
    dtostrf(corrent, 1, 2, currentString);                // Converteix el valor a un char array
    client.publish("esp32/current", currentString);
  }
  char estatCurrentString[8];
  dtostrf(estat_current, 1, 2, estatCurrentString);
  client.publish("esp32/estat_current", estatCurrentString);
}

// ----------------- Dades GPS ------------------
void dades_gps(void) {

  Serial.print("GPS: ");
  if (gpsstatus != 'A') {
    estat_gps = 0.0;
    Serial.println("Sensor desconectat");
  }
  else {        // Dins els límits bons, enviem que el sensor està funcionant bé
    estat_gps = 1.0;

    cnt_gps += 1;
    if (cnt_gps == 4) {
      cnt_gps = 0;
      Serial.print("Latitud: ");
      Serial.print(latitude, 8);
      Serial.print(",");
      Serial.print("Longitud: ");
      Serial.print(longitude, 8);
      Serial.print(",");
      Serial.println(gpsstatus);

      GPS_data["longitude"] = longitude;
      GPS_data["latitude"] = latitude;

      char output[128];
      serializeJson(GPS_data, output);
      //Serial.println(output);
      client.publish("esp32/GPS_data", output);
      // http://localhost:1880/worldmap/
    }
  }

  char estatgpsString[8];
  dtostrf(estat_gps, 1, 2, estatgpsString);
  client.publish("esp32/estat_gps", estatgpsString);

  listen();
}


// ----------------- Dades Tensió bateria ------------------
void dades_tensio(void) {
  float battery_voltage = analogRead(V_BAT_PIN);

  Serial.print("Voltatge bateria: ");
  battery_voltage = battery_voltage * 3.1588493 / 4095; //Valor máximo de tensión por el diseño actual
  Serial.print(battery_voltage);
  Serial.print(" V | ");

  Serial.print("SoC bateria: ");
  battery_voltage = 100 * (2.510906797 - battery_voltage) / (2.510906797 - 3.1588493);
  Serial.print(battery_voltage);
  Serial.println(" %");

  char battery_voltageString[8];
  dtostrf(battery_voltage, 1, 2, battery_voltageString);            // Converteix el valor a un char array
  client.publish("esp32/soc", battery_voltageString);
}
/////////////////////////////////////////////////////////////////// G E T   L A T   L O N G ////////////////////////////////////////////////////////////////////////////////////////
void get_lat_long(unsigned char waypoint) {

  lat2_deg = waypoints[waypoint];
  long2_deg = waypoints[waypoint + 1];

  //---------------------FI DEL JSON---------------------------

  Serial.print("Us esteu dirigint cap al punt amb latitud i longitud de: ");
  Serial.print(lat2_deg, 6);
  Serial.print(", ");
  Serial.println(long2_deg, 6);

}

/////////////////////////////////////////////////////////////////////////// G E T   G P S   I N F O ///////////////////////////////////////////////////////////////////////////////////////////////////////////

void get_gps_info() {

  Serial.print("GPS: ");
  if (gpsstatus != 'A') {
    estat_gps = 0.0;
    Serial.println("Sensor desconectat");
  }
  else {        // Dins els límits bons, enviem que el sensor està funcionant bé
    estat_gps = 1.0;

    lat1_deg = latitude;
    long1_deg = longitude;

    Serial.print("Latitud: ");
    Serial.print(latitude, 8);
    Serial.print(",");
    Serial.print("Longitud: ");
    Serial.print(longitude, 8);
    Serial.print(",");
    Serial.println(gpsstatus);

    if (dades_cnt == 1000) {  // Per no enviar les dades cada dos per tres
      GPS_data["longitude"] = longitude;
      GPS_data["latitude"] = latitude;

      char output[128];
      serializeJson(GPS_data, output);
      //Serial.println(output);
      client.publish("esp32/GPS_data", output);
      // http://localhost:1880/worldmap/
    }
  }
  char estatgpsString[8];
  dtostrf(estat_gps, 1, 2, estatgpsString);
  client.publish("esp32/estat_gps", estatgpsString);

  listen();
}

///////////////////////////////////////////////////////////////// G E T   D I S T   B E A R I N G//////////////////////////////////////////////////////////////////////////////////////////
void get_dist_bearing() {
  const float radi = 6371000;

  float lat1_rad = (lat1_deg) * (M_PI / 180.0);
  float long1_rad = (long1_deg) * (M_PI / 180.0);
  float lat2_rad = (lat2_deg) * (M_PI / 180.0);
  float long2_rad = (long2_deg) * (M_PI / 180.0);
  float delta_lat_rad = (lat2_deg - lat1_deg) * (M_PI / 180.0);
  float delta_long_rad = (long2_deg - long1_deg) * (M_PI / 180.0);

  float a = pow(sin(delta_lat_rad / 2.0), 2) + cos(lat1_rad) * cos(lat2_rad) * pow(sin(delta_long_rad / 2.0), 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  distance_m = radi * c;

  float y = sin(delta_long_rad) * cos(lat2_rad);
  float x = cos(lat1_rad) * sin(lat2_rad) - sin(lat1_rad) * cos(lat2_rad) * cos(delta_long_rad);
  float angle_rad = atan2(y, x);
  int angle_pre = (angle_rad * 180 / M_PI) + 360;
  angle_deg = (float)(angle_pre % 360);


  Serial.print("La distància és de ");
  Serial.print(distance_m, 2);
  Serial.print("m ");
  Serial.print("amb un angle de ");
  Serial.print(angle_deg, 4);
  Serial.println("º respecte el Nord.");
}

//////////////////////////////////////////////////////////////////////////// S H O W   C O M P A S S //////////////////////////////////////////////////////////////////////////////////////////////////////////

// ------------- HMC5883L - Bruixola ---------------------------------------------------------------------------------------------------------------------------------------------------------
void show_compass() {
  Serial.print("Brúixola: ");

  if (!compass.begin())
  {
    Serial.println("No conectada");
    estat_hmc = 0.0;

  }
  else {
    estat_hmc = 1.0;

    if (inici_hmc) {
      inici_hmc = 0;
      compass.setRange(HMC5883L_RANGE_1_3GA);             // Estableix el rang de mesura
      compass.setMeasurementMode(HMC5883L_CONTINOUS);     // Estableix el mode de mesura
      compass.setDataRate(HMC5883L_DATARATE_30HZ);        // Estableix la velocitat de dades
      compass.setSamples(HMC5883L_SAMPLES_8);             // Estableix el nombre mitjà de mostres
      compass.setOffset(24, -166);                        // Es calibra establint l'offset. Els valors es calculen amb el codi calibratge_HMC5883L.ino
    }

    /* S'obtenen els nous esdeveniments del sensor amb les lectures */
    Vector norm = compass.readNormalize();

    // Es calcula la direcció
    float heading = atan2(norm.YAxis, norm.XAxis);

    // Es calcula la declinació magnètica = (deg + (min / 60.0)) / (180 / M_PI)
    float declinationAngle = (1.0 + (30.0 / 60.0)) / (180 / M_PI);  // La declinació magnètica a Barcelona és 1º 30'
    heading += declinationAngle;

    // Corregir la direcció: 360 < direcció < 0
    if (heading < 0) {
      heading += 2 * PI;
    }
    if (heading > 2 * PI) {
      heading -= 2 * PI;
    }

    float headingDegrees = heading * 180 / M_PI;    // Converteix la direcció de radians a graus

    angle_dif = headingDegrees - angle_deg; // Diferència respecte nord
    if (angle_dif < 0) {
      angle_dif += 360;
    }


    Serial.print("Direcció = ");
    Serial.print(heading);
    Serial.print(" i Graus = ");
    Serial.print(headingDegrees);
    Serial.print("°, amb una dif respecte punts de ");
    Serial.print(angle_dif, 2);
    Serial.println("°");

    if (dades_cnt == 1000) {
      char headingString[8];
      dtostrf(headingDegrees, 1, 2, headingString);         // Converteix el valor a un char array
      client.publish("esp32/headingDegrees", headingString);
    }
  }

  char estatHmcString[8];
  dtostrf(estat_hmc, 1, 2, estatHmcString);
  client.publish("esp32/estat_hmc", estatHmcString);
}


//////////////////////////////////////////////////////////////////////////// CORRECT ANGLE DIFF //////////////////////////////////////////////////////////////////////////////////////////////////////////
void correct_angle_dif() {
  if (angle_dif > angle_margin and angle_dif < 180) { // Girar esquerra
    lectura_right_motor  = 20;
    lectura_left_motor = 0;
  }
  else if (angle_dif > 180 and angle_dif < (360 - angle_margin)) { // Girar dreta
    lectura_right_motor  = 0;
    lectura_left_motor = 20;
  }
  else {    // Aturar
    angle_correct = 1;
    lectura_right_motor  = 0;
    lectura_left_motor = 0;
  }

  digitalWrite(CONTROL_RIGHT, HIGH);  // Direccio endavant
  digitalWrite(CONTROL_LEFT, HIGH);   // Direccio endavant

  right_motor = map(lectura_right_motor, 0, 100, 0, 50);
  left_motor = map(lectura_left_motor, 0, 100, 0, 50);

  ledcWrite(PWM1_Ch, right_motor);
  ledcWrite(PWM2_Ch, left_motor);
}

//////////////////////////////////////////////////////////////////////////// CORRECT DISTANCE //////////////////////////////////////////////////////////////////////////////////////////////////////////
void correct_distance_m() {
  if (distance_m > distance_margin) {     // Avançar recte
    lectura_right_motor  = 20;
    lectura_left_motor = 20;
  }
  else {    // Aturar
    lectura_right_motor  = 0;
    lectura_left_motor = 0;
  }

  digitalWrite(CONTROL_RIGHT, HIGH);
  digitalWrite(CONTROL_LEFT, HIGH);

  right_motor = map(lectura_right_motor, 0, 100, 0, 50);
  left_motor = map(lectura_left_motor, 0, 100, 0, 50);

  ledcWrite(PWM1_Ch, right_motor);
  ledcWrite(PWM2_Ch, left_motor);
}
///////////////////////////////////////////////////////////////////////// Funcions d'inicialitzacio i ús d'alguns sensors i servei, són com les llibreries ////////////////////////////////////////////////////////////
// -------------- Funcions GPS ------------------
void listen() {

  while (gpsSerial.available())
  {
    read(gpsSerial.read());
  }
}

void read(char nextChar) {

  // Start of a GPS message
  if (nextChar == '$') {

    flag ? redbuffer[ptr] = '\0' : blubuffer[ptr] = '\0';

    ptr = 0;
  }

  // End of a GPS message
  if (nextChar == '\n') {

    if (flag) {
      flag = false;

      // Set termination character of the current buffer
      redbuffer[ptr] = '\0';

      // Process the message if the checksum is correct
      if (CheckSum((char*) redbuffer )) {
        parseString((char*) redbuffer );
      }
    }
    else
    {
      flag = true;

      // Set termination character of the current buffer
      blubuffer[ptr] = '\0';

      // Process the message if the checksum is correct
      if (CheckSum((char*) blubuffer )) {
        parseString((char*) blubuffer );
      }
    }

    // Reset the pointer
    ptr = 0;
  }

  // Add a new character
  flag ? redbuffer[ptr] = nextChar : blubuffer[ptr] = nextChar;

  // Check we stay within allocated memory
  if (ptr < 119) ptr++;
}

bool CheckSum(char* msg) {

  // Check the checksum
  //$GNGGA,.........................0000*6A

  // Length of the GPS message
  int len = strlen(msg);

  // Does it contain the checksum, to check
  if (msg[len - 4] == '*')
  {

    // Read the checksum from the message
    int cksum = 16 * Hex2Dec(msg[len - 3]) + Hex2Dec(msg[len - 2]);

    // Loop over message characters
    for (int i = 1; i < len - 4; i++)
    {
      cksum ^= msg[i];
    }

    // The final result should be zero
    if (cksum == 0) {
      return true;
    }
  }

  return false;
}

float DegreeToDecimal(float num, byte sign)
{
  // Want to convert DDMM.MMMM to a decimal number DD.DDDDD

  int intpart = (int) num;
  float decpart = num - intpart;

  int degree = (int)(intpart / 100);
  int mins = (int)(intpart % 100);

  if (sign == 'N' || sign == 'E')
  {
    // Return positive degree
    return (degree + (mins + decpart) / 60);
  }

  // Return negative degree
  return -(degree + (mins + decpart) / 60);
}

void parseString(char* msg) {

  messageGGA(msg);
  messageRMC(msg);
}

void messageGGA(char* msg)
{
  // Ensure the checksum is correct before doing this
  // Replace all the commas by end-of-string character '\0'
  // Read the first string
  // Knowing the length of the first string, can jump over to the next string
  // Repeat the process for all the known fields.

  // Do we have a GGA message?
  if (!strstr(msg, "GGA")) return;

  // Length of the GPS message
  int len = strlen(msg);

  // Replace all the commas with end character '\0'
  for (int j = 0; j < len; j++) {
    if (msg[j] == ',' || msg[j] == '*') {
      msg[j] = '\0';
    }
  }

  // Allocate working variables
  int i = 0;

  //$GPGGA

  // GMT time  094728.000
  i += strlen(&msg[i]) + 1;
  gpstime = atof(&msg[i]);

  // Latitude
  i += strlen(&msg[i]) + 1;
  latitude = atof(&msg[i]);

  // North or South (single char)
  i += strlen(&msg[i]) + 1;
  latNS = msg[i];
  if (latNS == '\0') latNS = '.';

  // Longitude
  i += strlen(&msg[i]) + 1;
  longitude = atof(&msg[i]);

  // East or West (single char)
  i += strlen(&msg[i]) + 1;
  lonEW = msg[i];
  if (lonEW == '\0') lonEW = '.';

  // Fix quality (1=GPS)(2=DGPS)
  i += strlen(&msg[i]) + 1;
  fixquality = atof(&msg[i]);

  // Number of satellites being tracked
  i += strlen(&msg[i]) + 1;
  numsatelites = atoi(&msg[i]);

  // Horizontal dilution of position
  i += strlen(&msg[i]) + 1;

  // Altitude
  i += strlen(&msg[i]) + 1;
  altitude = atof(&msg[i]);

  // Height of geoid (mean sea level)
  i += strlen(&msg[i]) + 1;

  // Time in seconds since last DGPS update
  i += strlen(&msg[i]) + 1;

  // DGPS station ID number
  i += strlen(&msg[i]) + 1;

  // Convert from degrees and minutes to degrees in decimals
  latitude = DegreeToDecimal(latitude, latNS);
  longitude = DegreeToDecimal(longitude, lonEW);
}


void messageRMC(char* msg)
{
  // Ensure the checksum is correct before doing this
  // Replace all the commas by end-of-string character '\0'
  // Read the first string
  // Knowing the length of the first string, can jump over to the next string
  // Repeat the process for all the known fields.

  // Do we have a RMC message?
  if (!strstr(msg, "RMC")) return;

  // Length of the GPS message
  int len = strlen(msg);

  // Replace all the commas with end character '\0'
  for (int j = 0; j < len; j++) {
    if (msg[j] == ',' || msg[j] == '*') {
      msg[j] = '\0';
    }
  }

  // Allocate working variables
  int i = 0;

  //$GPRMC

  // GMT time  094728.000
  i += strlen(&msg[i]) + 1;
  gpstime = atof(&msg[i]);

  // Status A=active or V=Void.
  i += strlen(&msg[i]) + 1;
  gpsstatus = msg[i];

  // Latitude
  i += strlen(&msg[i]) + 1;
  latitude = atof(&msg[i]);

  // North or South (single char)
  i += strlen(&msg[i]) + 1;
  latNS = msg[i];
  if (latNS == '\0') latNS = '.';

  // Longitude
  i += strlen(&msg[i]) + 1;
  longitude = atof(&msg[i]);

  // East or West (single char)
  i += strlen(&msg[i]) + 1;
  lonEW = msg[i];
  if (lonEW == '\0') lonEW = '.';

  // // Speed over the ground in knots
  i += strlen(&msg[i]) + 1;
  gpsknots = atof(&msg[i]);

  // Track angle in degrees True North
  i += strlen(&msg[i]) + 1;
  gpstrack = atof(&msg[i]);

  // Date - 31st of March 2018
  i += strlen(&msg[i]) + 1;
  gpsdate = atof(&msg[i]);

  // Magnetic Variation

  // Convert from degrees and minutes to degrees in decimals
  latitude = DegreeToDecimal(latitude, latNS);
  longitude = DegreeToDecimal(longitude, lonEW);
}

// Convert HEX to DEC
int Hex2Dec(char c) {

  if (c >= '0' && c <= '9') {
    return c - '0';
  }
  else if (c >= 'A' && c <= 'F') {
    return (c - 'A') + 10;
  }
  else {
    return 0;
  }
}

void AllSentences()
{
  // Turn-off Static mode
  // PMTK_API_SET_STATIC_NAV_THD
  // Command in the MT3337 Platform NMEA Message Specification_V1.00
  gpsSerial.println("$PMTK386,0*23");
  delay(100);

  // Select output sentences
  // PMTK_API_SET_MNEA_OUTPUT
  // Supported NMEA Sentences:
  //    0 NMEA_SEN_GLL,  // GPGLL interval - Geographic Latitude longitude
  //    1 NMEA_SEN_RMC,  // GPRMC interval - Recomended Minimum Specific
  //    2 NMEA_SEN_VTG,  // GPVTG interval - Course Over Ground Speed
  //    3 NMEA_SEN_GGA,  // GPGGA interval - GPS Fix Data
  //    4 NMEA_SEN_GSA,  // GPGSA interval - GNSS Satellites Active
  //    5 NMEA_SEN_GSV,  // GPGSV interval - GNSS Satellites in View
  //    6 NMEA_SEN_GRS,  // GPGRS interval – GNSS Range Residuals
  //    7 NMEA_SEN_GST,  // GPGST interval – GNSS Pseudorange Errors Statistics
  //   17 NMEA_SEN_ZDA,  // GPZDA interval – Time & Date
  //   18 NMEA_SEN_MCHN, // PMTKCHN interval – GNSS channel status
  //   19 NMEA_SEN_DTM,  // GPDTM interval – Datum reference

  // To work out the checksum I used the spreadsheet below
  // https://www.roboticboat.uk/Excel/NMEAchecksum.xlsx

  gpsSerial.println("$PMTK314,1,1,1,1,1,5,0,0,0,0,0,0,0,0,0,0,0,0,0,0*30");
  delay(100);
}


void SelectSentences()
{
  // Turn-off Static mode
  // PMTK_API_SET_STATIC_NAV_THD
  // Command in the MT3337 Platform NMEA Message Specification_V1.00
  gpsSerial.println("$PMTK386,0*23");
  delay(100);

  // Select output sentences
  // PMTK_API_SET_MNEA_OUTPUT
  // Supported NMEA Sentences:
  //    0 NMEA_SEN_GLL,  // GPGLL interval - Geographic Latitude longitude
  // *  1 NMEA_SEN_RMC,  // GPRMC interval - Recomended Minimum Specific
  //    2 NMEA_SEN_VTG,  // GPVTG interval - Course Over Ground Speed
  // *  3 NMEA_SEN_GGA,  // GPGGA interval - GPS Fix Data
  //    4 NMEA_SEN_GSA,  // GPGSA interval - GNSS Satellites Active
  //    5 NMEA_SEN_GSV,  // GPGSV interval - GNSS Satellites in View
  //    6 NMEA_SEN_GRS,  // GPGRS interval – GNSS Range Residuals
  //    7 NMEA_SEN_GST,  // GPGST interval – GNSS Pseudorange Errors Statistics
  //   17 NMEA_SEN_ZDA,  // GPZDA interval – Time & Date
  //   18 NMEA_SEN_MCHN, // PMTKCHN interval – GNSS channel status
  //   19 NMEA_SEN_DTM,  // GPDTM interval – Datum reference

  // To work out the checksum I used the spreadsheet below
  // https://www.roboticboat.uk/Excel/NMEAchecksum.xlsx

  gpsSerial.println("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*34");
  delay(100);

}

void SelectGGAonly()
{
  // Turn-off Static mode
  // PMTK_API_SET_STATIC_NAV_THD
  // Command in the MT3337 Platform NMEA Message Specification_V1.00
  gpsSerial.println("$PMTK386,0*23");
  delay(100);

  // Receive GGA messages only
  gpsSerial.println("$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*35");
  delay(100);
}

void SelectMode(int mode)
{
  // Set the sensitivity of the GPS module
  // Mode can be 0,1,2 or 3

  switch (mode)
  {
    case 0:
      // Vehicle mode (max altitude 10km)
      gpsSerial.println("$PMTK886,0*28");
      break;
    case 1:
      // Pedestrian mode (max altitude 10km)
      gpsSerial.println("$PMTK886,1*29");
      break;
    case 2:
      // Avionic mode (max altitude 10km)
      gpsSerial.println("$PMTK886,2*2A");
      break;
    case 3:
      // Balloon mode (max altitude 80km, speed<515m/s)
      gpsSerial.println("$PMTK886,3*2B");
      break;
    default:
      // Do nothing
      break;
  }

  // Give GPS module some time
  delay(100);
}


// -------------- Funcions Wi-Fi -----------------
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

void callback(char* topic, byte * message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String stream;

  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    stream += (char)message[i];
  }
  Serial.println();

  // Feel free to add more if statements to control more GPIOs with MQTT

  // If a message is received on the topic esp32/POWER, you check if the message is either "on" or "off".
  // Changes the output state according to the message
  if (String(topic) == "esp32/POWER") {
    Serial.print("Changing output to ");
    if (stream == "on") {
      Serial.println("on");
      digitalWrite(POWER, HIGH);
      digitalWrite(CONTROL_RIGHT, HIGH);
      digitalWrite(CONTROL_LEFT, HIGH);
      //ledcWrite(PWM1_Ch, 100);
      //ledcWrite(PWM2_Ch, 100);

    }
    else if (stream == "off") {
      Serial.println("off");
      digitalWrite(POWER, LOW);
      digitalWrite(CONTROL_RIGHT, HIGH);
      digitalWrite(CONTROL_LEFT, HIGH);
      ledcWrite(PWM1_Ch, 0);
      ledcWrite(PWM2_Ch, 0);
    }
  }

  if (String(topic) == "esp32/motor1")
  {
    motor1 = stream;
    lectura_right_motor  = motor1.toInt();
    digitalWrite(CONTROL_RIGHT, HIGH);
    right_motor = map(lectura_right_motor, 0, 100, 0, 50);
    ledcWrite(PWM1_Ch, right_motor);
  }

  if (String(topic) == "esp32/motor2")
  {
    motor2 = stream;
    lectura_left_motor  = motor2.toInt();
    digitalWrite(CONTROL_LEFT, HIGH);
    left_motor = map(lectura_left_motor, 0, 100, 0, 50);
    ledcWrite(PWM2_Ch, left_motor);
  }

  if (String(topic) == "esp32/demo") {
    if (stream == "on") {
      activacio_navegacio_autonoma = 1;
    }
    else if (stream == "off") {
      activacio_navegacio_autonoma = 0;
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
      client.subscribe("esp32/POWER");
      client.subscribe("esp32/motor1");
      client.subscribe("esp32/motor2");
      client.subscribe("esp32/demo");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
