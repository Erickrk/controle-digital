#include <Arduino.h>
//Pinos ponte H:
byte motorA1 = 2;
byte motorA2 = 4;
byte motorA_pwm = 5;

struct Encoder {
  const uint8_t PIN;
  int counted;
  int RPM;
};
Encoder encoder1 = {18, 0, 0};


// Interrupts: https://techtutorialsx.com/2017/10/07/esp32-arduino-timer-interrupts/
// In case it goes wrong, use milis
volatile int interruptCounter;
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

//Timer and Encoder interrupts
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter++;
  encoder1.RPM = encoder1.counted*5; //divide by disk rips(30) * 60 to convert to RPM* 2.5 (0.4*2.5=1 second)
  //Prototype for recursive function
  Serial.print(vel);
  Serial.print("\t");
  Serial.print(encoder1.RPM);
  Serial.println("");
  encoder1.counted = 0;
  portEXIT_CRITICAL_ISR(&timerMux);
}

//ref: https://lastminuteengineers.com/handling-esp32-gpio-interrupts-tutorial/
void IRAM_ATTR encoder_count() {
    encoder1.counted ++;
}

void setup() {
  Serial.begin(115200);
  //interrupt stuff
  //encoder interrupt
  pinMode(encoder1.PIN, INPUT_PULLDOWN);
  attachInterrupt(encoder1.PIN, &encoder_count, RISING);
  //PS chosen in order to make 80MHz/80
  // Receives: (Timer used, PS, flag to count up or down)
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  //interrupt every 400 mili
  timerAlarmWrite(timer, 400000, true);
  timerAlarmEnable(timer);

  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  
  pinMode(motorA_pwm, OUTPUT);
  ledcAttachPin(motorA_pwm, 0); //Atribui o pino do PWM ao canal 0, é possível ir de 0-15
  /* frequência: 1-40Mhz e Resoução de 1-16 bits
  //resoluçãoa atual: 2^12 = 4096 
  //base: https://portal.vidadesilicio.com.br/controle-de-potencia-via-pwm-esp32/
  e: https://portal.vidadesilicio.com.br/ponte-h-l298n-controle-velocidade-motor/
  */
 //frequency discovered empirically
  ledcSetup(0, 5, 8); // Recebe: (canal, frequência, resolução)
}

/*Definições L298: 
Channel A:
Forward: IN1---5V   IN2---GND  
Reverse: IN1---GND  IN2--5V 
ENA: PWM
*/
void fwd(int spd){
digitalWrite(motorA1, HIGH);
digitalWrite(motorA2, LOW);
//Muda a velocidade de acordo com o input
ledcWrite(0, spd);
}

void rvs(int spd){
digitalWrite(motorA1, LOW);
digitalWrite(motorA2, HIGH);
//Muda a velocidade de acordo com o input
ledcWrite(0, spd);
} 


void loop() {
for(int count = 0; count<261; count++){
    fwd(vel);
    delay(2000);
    vel=vel+20;    
  }

//fwd(128);
/*if (interruptCounter > 0) {
  portENTER_CRITICAL(&timerMux);
  //encoder1.counted = encoder1.counted/60;
  Serial.print(encoder1.counted);
  Serial.println();
  //encoder1.counted = 0;
  interruptCounter--;
  portEXIT_CRITICAL(&timerMux);
 }
*/
}
