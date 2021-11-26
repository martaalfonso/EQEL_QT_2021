/*******************************************************************************
 * IO DEFINITION                                                                *
 *******************************************************************************/
#define RIGHT_PWM 3
#define LEFT_PWM  4
#define PWM1_Ch    0
#define PWM1_Res   8
#define PWM1_Freq  5000
#define PWM2_Ch    1
#define CONTROL_RIGHT 4
#define CONTROL_LEFT  5



/*******************************************************************************
 * PRIVATE GLOBAL VARIABLES                                                     *
 ******************************************************************************/
unsigned char valor;
unsigned char lectura_right_motor;
unsigned char lectura_left_motor;
unsigned char right_motor;
unsigned char left_motor;
unsigned char switch_on;


/*******************************************************************************
 * FUNCTIONS                                                                    *
 *******************************************************************************/

// The setup routine runs once when you press reset.
void setup() {
  Serial.begin(115200);                
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



// The loop routine runs over and over again forever.
void loop() {

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
  
}
