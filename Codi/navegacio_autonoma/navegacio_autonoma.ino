/* Codi navegació autònoma */

// S'inclouen les llibreries
#include <SoftwareSerial.h>     // GPS
#include <math.h>
#include <HMC5883L.h>           // HMC5883L
#include <Wire.h>               // I2C

// Variables
float gpstime, gpsdate, latitude, longitude, altitude, gpsknots, gpstrack;
char latNS, lonEW, gpsstatus;
int fixquality, numsatelites;
volatile int ptr = 0;
volatile bool flag = true;
volatile char redbuffer[120], blubuffer[120];
String inputgpsSerial = "";         // a string to hold incoming data
boolean IsReadygpsSerial = false;   // whether the string is complete

float lat1_deg, long1_deg; // Coordenades inicials
float lat2_deg, long2_deg; // Coordenades finals
float distance_m, angle_deg; // Distància i angle entre punt A i punt B
float angle_dif; // Diferència entre el nord i l'angle entre el punt A i B

// Declarem objectes
HMC5883L compass;

SoftwareSerial gpsSerial(16, 17);

////////////////////////////////////////////////////////////////////////////////// S E T    U P ////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {

  Serial.begin(115200);

  // --------------- HMC5883L ---------------
  compass.setRange(HMC5883L_RANGE_1_3GA);             // Estableix el rang de mesura
  compass.setMeasurementMode(HMC5883L_CONTINOUS);     // Estableix el mode de mesura
  compass.setDataRate(HMC5883L_DATARATE_30HZ);        // Estableix la velocitat de dades
  compass.setSamples(HMC5883L_SAMPLES_8);             // Estableix el nombre mitjà de mostres
  compass.setOffset(24, -166);                        // Es calibra establint l'offset. Els valors es calculen amb el codi calibratge_HMC5883L.ino

  // ----------------- GPS ------------------
  pinMode(16, INPUT);       //Receive from the GPS device (the NMEA sentences) - RX
  pinMode(17, OUTPUT);      //Transmit to the GPS device - TX

  gpsSerial.begin(9600);    // Connect to the GPS module
  SelectSentences();
}

////////////////////////////////////////////////////////////////////////////////// L O O P ////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() {

  get_lat_long(); // Obtenim latitud i longitud d'on volem anar
  get_gps_info(); // Obtenim latitud i longitud d'on estem
  get_dist_bearing(); // Obtenim distancia i orientació entre dues coordenades
  show_compass();
  correct_angle_dif();
  correct_distance_m();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void get_lat_long() {

  // 4 decimals seria suficient ja
  lat2_deg = 41.403926;
  long2_deg = 2.174304;

  Serial.print("Us esteu dirigint cap al punt amb latitud i longitud de: ");
  Serial.print(lat2_deg, 6);
  Serial.print(", ");
  Serial.println(long2_deg, 6);

}

/////////////////////////////////////////////////////////////////////////// G E T   G P S   I N F O ///////////////////////////////////////////////////////////////////////////////////////////////////////////

void get_gps_info() {
  if (gpsstatus == 'A') {
    lat1_deg = latitude;
    long1_deg = longitude;
    Serial.print("latitud, longitud:  ");
    Serial.println(lat1_deg, 6);
    Serial.println(long1_deg, 6);
    Serial.println();
  }
  listen();
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

void show_compass() {
  /* S'obtenen els nous esdeveniments del sensor amb les lectures */
  Vector norm = compass.readNormalize();

  float heading = atan2(norm.YAxis, norm.XAxis);   // Es calcula la direcció
  float declinationAngle = (1.0 + (30.0 / 60.0)) / (180 / M_PI);  // Declinació magnètica a Barcelona = 1º 30'
  heading += declinationAngle;

  // Corregir la direcció: 360 < direcció < 0
  if (heading < 0) {
    heading += 2 * PI;
  }
  if (heading > 2 * PI) {
    heading -= 2 * PI;
  }

  float heading_deg = heading * 180 / M_PI;    // Converteix la direcció de radians a graus

  Serial.println(heading_deg);
  Serial.println();
  angle_dif = heading_deg - angle_deg; // Diferència respecte nord
  if (angle_dif < 0) {
    angle_dif += 360;
  }
  Serial.println(angle_dif, 2);
}

//////////////////////////////////////////////////////////////////////////// CORRECT ANGLE DIFF //////////////////////////////////////////////////////////////////////////////////////////////////////////
void correct_angle_dif() {
  while (angle_dif > 5 and angle_dif < 180) {
    get_gps_info(); // Obtenim latitud i longitud d'on estem
    get_dist_bearing(); // Obtenim distancia i orientació entre dues coordenades
    show_compass();
    // Girar esquerra
  }
  while (angle_dif > 180 and angle_dif < 355) {
    get_gps_info(); // Obtenim latitud i longitud d'on estem
    get_dist_bearing(); // Obtenim distancia i orientació entre dues coordenades
    show_compass();
    // Girar dreta
  }
  // Atura de girar

}

//////////////////////////////////////////////////////////////////////////// CORRECT DISTANCE //////////////////////////////////////////////////////////////////////////////////////////////////////////
void correct_distance_m() {
  while (distance_m > 2.5) {
    get_gps_info(); // Obtenim latitud i longitud d'on estem
    get_dist_bearing(); // Obtenim distancia i orientació entre dues coordenades
    show_compass();
    // Avança el vaixell
  }
  // Atura el vaixell
}

//////////////////////////////////////////////////////////////////////////// FUNCIONS GPS //////////////////////////////////////////////////////////////////////////////////////////////////////////
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
  delay(100);
}
