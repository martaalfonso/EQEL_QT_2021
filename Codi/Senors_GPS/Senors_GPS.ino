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
#include <SoftwareSerial.h>     // GPS              

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
float gpstime;
float gpsdate;
float latitude;
float longitude;
float altitude;
float gpsknots;
float gpstrack;
char latNS, lonEW;
char gpsstatus;
int fixquality;
int numsatelites;
volatile int ptr = 0;
volatile bool flag = true;
volatile char redbuffer[120];
volatile char blubuffer[120];

// Global variables
String inputgpsSerial = "";         // a string to hold incoming data
boolean IsReadygpsSerial = false;  // whether the string is complete

// Funcions
float get_corriente(int n_muestras);
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

// gpsSerial(receive from GPS,transmit to the GPS module)
SoftwareSerial gpsSerial(16, 17);

void setup(void) {

  // Inicialització del serial
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  // Intenta inicialitzar el MPU6050
  if (!mpu.begin()) {
    Serial.println("El MPU6050 no està connectat");
  }
  
  // ----------------- MPU6050 -----------------
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);   // Estableix el rang de mesura de l'acceleròmetre
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);        // Estableix el rang de mesura del giròscop
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);      // Estableix el filtre


  // Intenta inicialitzar el HMC5883L
  if (!compass.begin())
  {
    Serial.println("El HMC5883L no està connectat");
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

  // ----------------- GPS ------------------
  pinMode(16, INPUT);   //Receive from the GPS device (the NMEA sentences) - RX
  pinMode(17, OUTPUT);  //Transmit to the GPS device - TX

  gpsSerial.begin(9600);              // Connect to the GPS module

  // Because we have to process the GPS messages, we can make our life
  // easier if we only request the sentences we require.
  SelectSentences();

  delay(100);
}

void loop() {
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
  Serial.print("Corrent: ");
  Serial.print(I, 3); // Visualitzem 3 decimals
  Serial.println(" A");

  // ----------------- GPS ------------------
  Serial.print("GPS,");
  Serial.print(gpsdate, 0);
  Serial.print(",");
  Serial.print(gpstime + 20000, 0);
  Serial.print(",");
  Serial.print(latitude, 8);
  Serial.print(",");
  Serial.print(latNS);
  Serial.print(",");
  Serial.print(longitude, 8);
  Serial.print(",");
  Serial.print(lonEW);
  Serial.print(",");
  Serial.print(altitude);
  Serial.print(",");
  Serial.print(fixquality);
  Serial.print(",");
  Serial.print(numsatelites);
  Serial.print(",");
  Serial.print(gpsknots);
  Serial.print(",");
  Serial.print(gpstrack);
  Serial.print(",");
  Serial.println(gpsstatus);
  delay(100);
  listen();

  Serial.println("");
  delay(1000);
}


// -------------- Filtre del sensor de corrent ------------------
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

// -------------- Funcions GPS ------------------
void listen(){

  while (gpsSerial.available())
  {
     read(gpsSerial.read());
  }
}

void read(char nextChar){

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
      if (CheckSum((char*) redbuffer )) {parseString((char*) redbuffer );}
    }
    else
    {
      flag = true;
      
      // Set termination character of the current buffer
      blubuffer[ptr] = '\0';

      // Process the message if the checksum is correct
      if (CheckSum((char*) blubuffer )) {parseString((char*) blubuffer );}
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
  if (msg[len-4] == '*') 
  {

    // Read the checksum from the message
    int cksum = 16 * Hex2Dec(msg[len-3]) + Hex2Dec(msg[len-2]);

    // Loop over message characters
    for (int i=1; i < len-4; i++) 
    {
      cksum ^= msg[i];
    }

    // The final result should be zero
    if (cksum == 0){
      return true;
    }
  }

  return false;
}

float DegreeToDecimal(float num, byte sign)
{
   // Want to convert DDMM.MMMM to a decimal number DD.DDDDD

   int intpart= (int) num;
   float decpart = num - intpart;

   int degree = (int)(intpart / 100);
   int mins = (int)(intpart % 100);

   if (sign == 'N' || sign == 'E')
   {
     // Return positive degree
     return (degree + (mins + decpart)/60);
   }   

   // Return negative degree
   return -(degree + (mins + decpart)/60);
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
  for (int j=0; j<len; j++){
    if (msg[j] == ',' || msg[j] == '*'){
      msg[j] = '\0';
    }
  }

  // Allocate working variables
  int i = 0;

  //$GPGGA

  // GMT time  094728.000
  i += strlen(&msg[i])+1;
  gpstime = atof(&msg[i]);
  
  // Latitude
  i += strlen(&msg[i])+1;
  latitude = atof(&msg[i]);
  
  // North or South (single char)
  i += strlen(&msg[i])+1;
  latNS = msg[i];
  if (latNS == '\0') latNS = '.';
  
  // Longitude
  i += strlen(&msg[i])+1;
  longitude = atof(&msg[i]);
  
  // East or West (single char)
  i += strlen(&msg[i])+1;
  lonEW = msg[i];
  if (lonEW == '\0') lonEW = '.';
  
  // Fix quality (1=GPS)(2=DGPS)
  i += strlen(&msg[i])+1;
  fixquality = atof(&msg[i]);   
      
  // Number of satellites being tracked
  i += strlen(&msg[i])+1;
  numsatelites = atoi(&msg[i]); 
  
  // Horizontal dilution of position
  i += strlen(&msg[i])+1;
  
  // Altitude
  i += strlen(&msg[i])+1;
  altitude = atof(&msg[i]);     
  
  // Height of geoid (mean sea level)
  i += strlen(&msg[i])+1;
  
  // Time in seconds since last DGPS update
  i += strlen(&msg[i])+1;
  
  // DGPS station ID number
  i += strlen(&msg[i])+1;
  
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
  for (int j=0; j<len; j++){
    if (msg[j] == ',' || msg[j] == '*'){
      msg[j] = '\0';
    }
  }

  // Allocate working variables
  int i = 0;

  //$GPRMC

  // GMT time  094728.000
  i += strlen(&msg[i])+1;
  gpstime = atof(&msg[i]);

  // Status A=active or V=Void.
  i += strlen(&msg[i])+1;
  gpsstatus = msg[i];

  // Latitude
  i += strlen(&msg[i])+1;
  latitude = atof(&msg[i]);

  // North or South (single char)
  i += strlen(&msg[i])+1;
  latNS = msg[i];
  if (latNS == '\0') latNS = '.';

  // Longitude
  i += strlen(&msg[i])+1;
  longitude = atof(&msg[i]);

  // East or West (single char)
  i += strlen(&msg[i])+1;
  lonEW = msg[i];     
  if (lonEW == '\0') lonEW = '.';        

  // // Speed over the ground in knots
  i += strlen(&msg[i])+1;
  gpsknots = atof(&msg[i]);

  // Track angle in degrees True North
  i += strlen(&msg[i])+1;
  gpstrack = atof(&msg[i]); 
  
  // Date - 31st of March 2018
  i += strlen(&msg[i])+1;
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
  delay(1000);
}
