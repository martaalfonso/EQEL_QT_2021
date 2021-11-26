/*******************************************************************************
 * LIBRARIES                                                                *
 *******************************************************************************/
#include <WiFi.h>
#include <PubSubClient.h>

/*******************************************************************************
 * IO DEFINITION                                                                *
 *******************************************************************************/
#define RIGHT_PWM 2
#define LEFT_PWM  4
#define PWM1_Ch    0
#define PWM1_Res   8
#define PWM1_Freq  5000
#define PWM2_Ch    1
#define CONTROL_RIGHT 5
#define CONTROL_LEFT  18



/*******************************************************************************
 * PRIVATE GLOBAL VARIABLES                                                     *
 ******************************************************************************/
unsigned char valor;
unsigned char lectura_right_motor;
unsigned char lectura_left_motor;
unsigned char right_motor;
unsigned char left_motor;
unsigned char switch_on;
const char* ssid = "iPhone de: Marta";
const char* password = "test1234";
const char* mqtt_server = "172.20.10.14";
long lastMsg = 0;
char msg[50];
int value = 0;

/*******************************************************************************
 * OBJECTS                                                                     *
 ******************************************************************************/
WiFiClient espClient;
PubSubClient client(espClient);

/*******************************************************************************
 * FUNCTIONS                                                                    *
 *******************************************************************************/

// The setup routine runs once when you press reset.
void setup() {
  Serial.begin(115200);
  //init wifi
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);                
  // Initialize the PWM and DIR pins as digital outputs.
  pinMode(RIGHT_PWM, OUTPUT);
  ledcAttachPin(RIGHT_PWM, PWM1_Ch);
  ledcSetup(PWM1_Ch, PWM1_Freq, PWM1_Res);
  
  pinMode(LEFT_PWM, OUTPUT);
  ledcAttachPin(LEFT_PWM, PWM2_Ch);
  ledcSetup(PWM2_Ch, PWM1_Freq, PWM1_Res);
  
  pinMode(CONTROL_LEFT, OUTPUT);
  pinMode(CONTROL_RIGHT, OUTPUT);
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

  // If a message is received on the topic esp32/POWER, you check if the message is either "on" or "off". 
  // Changes the output state according to the message
  if (String(topic) == "esp32/POWER") {
    Serial.print("Changing output to ");
    if(messageTemp == "on"){
      Serial.println("on");
      switch_on = 1;
      //digitalWrite(ledPin, HIGH);
      digitalWrite(CONTROL_RIGHT, HIGH);
      ledcWrite(PWM1_Ch, 100);
      
    }
    else if(messageTemp == "off"){
      Serial.println("off");
      switch_on = 0;
      digitalWrite(CONTROL_RIGHT, HIGH);
      ledcWrite(PWM1_Ch, 0);
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
      client.subscribe("esp32/POWER");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


// The loop routine runs over and over again forever.
void loop() {
  /*
  if(switch_on == 1)
  {
    //DIRECCIÓ EN FUNCIÓ DE LA COMANDA DE LA INTERFÍCIE
    
    lectura_right_motor  = 20;  //AQUÍ VA LA LECTURA PER WIFI DE LA PALANCA DRETA
    lectura_left_motor = 20;    //AQUÍ VA LA LECTURA PER WIFI DE LA PALANCA ESQUERRA
    digitalWrite(CONTROL_RIGHT, HIGH);
    digitalWrite(CONTROL_LEFT, HIGH);
    
    
    right_motor = map(lectura_right_motor, 0, 100, 0, 50);
    left_motor = map(lectura_left_motor, 0, 100, 0, 50);

    ledcWrite(PWM1_Ch, right_motor);
    ledcWrite(PWM2_Ch, left_motor);

  }
  else
  {
    digitalWrite(CONTROL_RIGHT, LOW);
    digitalWrite(CONTROL_LEFT, LOW);      
  }
  */
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}
