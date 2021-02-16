#include <Arduino.h>

byte motorA1 = 2;
byte motorA2 = 4;
byte motorA_pwm = 5;

struct Encoder 
{
  const uint8_t PIN;
  int counted;
  int RPM;
  int newRPM;
};
Encoder encoder1 = {18, 0, 0, 0};

byte dir = 0;
int newRPM = 0;

// Interrupts: https://techtutorialsx.com/2017/10/07/esp32-arduino-timer-interrupts/
// In case it goes wrong, use milis
//ref: https://lastminuteengineers.com/handling-esp32-gpio-interrupts-tutorial/
void IRAM_ATTR encoder_count() 
{
    encoder1.counted ++;
}

volatile int interruptCounter;
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile bool interrupt2flag;
hw_timer_t * timer2 = NULL;
portMUX_TYPE timerMux2 = portMUX_INITIALIZER_UNLOCKED;

//Timer and Encoder interrupts
void IRAM_ATTR onTimer() 
{
  portENTER_CRITICAL_ISR(&timerMux);
  //interruptCounter++;
  encoder1.RPM = encoder1.counted*5; //divide by disk rips(30) * 60 * 2.5 (0.4*2.5 = 1sec) to convert to RPM
  //if(encoder1.RPM>100)
  //encoder1.RPM = 100;
  encoder1.counted = 0;
  interrupt2flag = true;
  portEXIT_CRITICAL_ISR(&timerMux);
}

//Control stuff
unsigned long lastTime;   
float errSum, lastErr;  
//int Controlled_rpm;
float kp = 0.01, ki= 0.001;
int old_control;
float Controlled_rpm;
float error;

 float PI_Control(float newRPM, float oldRPM)  
{  
   /*How long since we last calculated*/  
   unsigned long now = millis();  
   unsigned long timeChange = (float)(now - lastTime);  

   /*Compute all the working error variables*/  
   error = newRPM - oldRPM;  
   //Serial.println("error:");
   //Serial.print(error);
  // Serial.println();
   errSum += (error * timeChange);  
   //double dErr = (error - lastErr) / timeChange;  

   /*Compute PID Output*/  
  Controlled_rpm = kp * error + ki * errSum;  
  if(Controlled_rpm>=110 || Controlled_rpm<0)
   errSum -= (error * timeChange);  
   /*Remember some variables for next time*/  
   lastErr = error;  
   lastTime = now;  
   return Controlled_rpm;
}

void IRAM_ATTR onTimer2() 
{
  portENTER_CRITICAL_ISR(&timerMux2);
  portEXIT_CRITICAL_ISR(&timerMux2);
}


/* L298 definitions: 
Channel A:
Forward: IN1---5V   IN2---GND  
Reverse: IN1---GND  IN2--5V 
ENA: PWM
*/

void fwd(int spd)
{
digitalWrite(motorA1, HIGH);
digitalWrite(motorA2, LOW);

ledcWrite(0, spd);
}

void rvs(int spd)
{
digitalWrite(motorA1, LOW);
digitalWrite(motorA2, HIGH);

ledcWrite(0, spd);
} 

void stop()
{
digitalWrite(motorA1, LOW);
digitalWrite(motorA2, LOW);
}

int newPWM;
void RPM2PWM(byte dir, float newRPM)
{
  // Polinomial regression without pow()
  if(dir==1)
  {
  //First polinomial reg. try
  //newPWM = round((0.996 * newRPM) + (0.0116 * newRPM * newRPM) + 54.7);
  newPWM = round((2.19 * newRPM)+33.5) ;
    if(newPWM<0) 
      newPWM = 0;
    else 
      if (newPWM>255)
          newPWM = 255;
  fwd(newPWM);
    }
  else
  { 
   newPWM = (0.345 * newRPM * newRPM) -(12.4 * newRPM) + 1059;
   rvs(newPWM);  
  }
} 

void setup() 
{
  Serial.begin(115200);
  pinMode(CONNECT_PIN, OUTPUT);
  
  //interrupt stuff
  pinMode(encoder1.PIN, INPUT_PULLDOWN);
  attachInterrupt(encoder1.PIN, &encoder_count, RISING);
  
  //PS chosen in order to make 80MHz/80
  // Receives: (Timer used, PS, flag to count up or down)
  timer = timerBegin(0, 80, true);
  timer2 = timerBegin(1, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAttachInterrupt(timer2, &onTimer2, true);
  
  //interrupt every two seconds
  timerAlarmWrite(timer, 400000, true);
  timerAlarmEnable(timer);

  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  
  pinMode(motorA_pwm, OUTPUT);
  ledcAttachPin(motorA_pwm, 0); // It is possible to choose from 0-15
  /* frequency: 1-40Mhz and resolution of 1-16 bits
  resolution used: 2^12 = 4096 
  base: https://portal.vidadesilicio.com.br/controle-de-potencia-via-pwm-esp32/
  e: https://portal.vidadesilicio.com.br/ponte-h-l298n-controle-velocidade-motor/
  */
 //frequency discovered empirically
  ledcSetup(0, 500, 8); //receives: (channel, frequency, resolution)
}

void loop() 
{
  if (interrupt2flag) 
  {
    static float rpm, rpmlast = 0;
     rpm = encoder1.RPM; 
     rpm = 0.7*rpm+0.3*rpmlast;
     if(rpm>110 || rpm<0)
     rpm = rpmlast;
     rpmlast = rpm;

    encoder1.newRPM = 30;
    //Serial.println("Novo RPM:");
    //Serial.print(encoder1.newRPM);
    
    float  test = PI_Control(encoder1.newRPM,rpm);
    RPM2PWM(1,test);
    interrupt2flag = false;
    
    Serial.print(error);
    Serial.print("\t");
    //Serial.print("valor calculado:");
    Serial.print(Controlled_rpm);
    Serial.print("\t");
    //Serial.print("RPM atual:");
    Serial.print(newPWM);
    Serial.print("\t");
    Serial.print(rpm);
    Serial.print("\t");
    Serial.print(encoder1.RPM);
    Serial.println();
  }
}
