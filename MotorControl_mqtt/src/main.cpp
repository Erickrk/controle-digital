#include <Arduino.h>
//WiFi, MQTT and Json libs
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

//needed for library
#include <DNSServer.h> //Local DNS Server used for redirecting all requests to the configuration portal ( https://github.com/zhouhan0126/DNSServer---esp32 )
#include <WebServer.h> //Local WebServer used to serve the configuration portal ( https://github.com/zhouhan0126/WebServer-esp32 )
#include <WiFiManager.h>   // WiFi Configuration Magic ( https://github.com/zhouhan0126/WIFIMANAGER-ESP32 ) >> https://github.com/tzapu/WiFiManager (ORIGINAL)


//#define SSID "yourSSID"
//#define PASSWORD "yourPASS"

const String MQTT_SERVER = "your_server";

//Pin for mqtt connection feedback
#define CONNECT_PIN 23
bool cb_flag = false;

byte motorA1 = 2;
byte motorA2 = 4;
byte motorA_pwm = 5;

struct Encoder {
  const uint8_t PIN;
  int counted;
  int RPM;
  int newRPM;
};
Encoder encoder1 = {18, 0, 0, 50};
byte dir = 0;
int newRPM = 0;

//connectivity
WiFiClient wifiClient;

PubSubClient client(MQTT_SERVER.c_str(), 15669, wifiClient);

long lastMsg = 0;

// Interrupts: https://techtutorialsx.com/2017/10/07/esp32-arduino-timer-interrupts/
// In case it goes wrong, use milis
//ref: https://lastminuteengineers.com/handling-esp32-gpio-interrupts-tutorial/
void IRAM_ATTR encoder_count() {
    encoder1.counted ++;
}

volatile int interruptCounter;
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool interrupt2flag;

//hw_timer_t * timer2 = NULL;
//portMUX_TYPE timerMux2 = portMUX_INITIALIZER_UNLOCKED;

//Timer and Encoder interrupts
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  //interruptCounter++;
  encoder1.RPM = encoder1.counted*5; //divide by disk rips * 60 * 20 to convert to RPM
  encoder1.counted = 0;
  interrupt2flag = true;
  portEXIT_CRITICAL_ISR(&timerMux);
}

//Control stuff
unsigned long lastTime = 0;   
float errSum, lastErr, Controlled_rpm, error;
float kp = 0.5, ki= 0.01;

 float PI_Control(float newRPM, float oldRPM)  
{  
   /*How long since we last calculated*/  
   //unsigned long now = millis();  
   unsigned long timeChange = 0.4;  
  
   /*Compute all the working error variables*/  
   error = newRPM - oldRPM;  
   //Serial.println("erro:");
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
   return Controlled_rpm;
}

/*
void IRAM_ATTR onTimer2() {
  portENTER_CRITICAL_ISR(&timerMux2);
  interrupt2flag = true;
  portEXIT_CRITICAL_ISR(&timerMux2);
}
 /*

/*WIFIMANAGER*/
/*void configModeCallback (WiFiManager *myWiFiManager) {  
//  Serial.println("Entered config mode");
  Serial.println("Entrou no modo de configuração");
  Serial.println(WiFi.softAPIP()); //imprime o IP do AP
  Serial.println(myWiFiManager->getConfigPortalSSID()); //imprime o SSID criado da rede

}

//callback notifying us of the need to save config
//callback que indica que salvamos uma nova rede para se conectar (modo estação)
void saveConfigCallback () {
//  Serial.println("Should save config");
  Serial.println("Configuração salva");
  Serial.println(WiFi.softAPIP()); //imprime o IP do AP
}
*/

void fwd(int spd){
digitalWrite(motorA1, HIGH);
digitalWrite(motorA2, LOW);
//Muda a velocidade de acordo com o input
ledcWrite(0, spd);
//while(!cb_flag){
  //Serial.println("entrei");
  //int cont_spd = PI_Control(spd);
  //ledcWrite(0, cont_spd);
//}
}

void rvs(int spd){
digitalWrite(motorA1, LOW);
digitalWrite(motorA2, HIGH);
//Muda a velocidade de acordo com o input
ledcWrite(0, spd);
//while(!cb_flag){
//}

} 
int newPWM=0;
void RPM2PWM(byte dir, float newRPM){
  // Polinomial regression without pow()
  //Serial.println("entrei");
  if(dir==1){
   //newPWM = round((0.996 * newRPM) + (0.0116 * newRPM * newRPM) + 54.7);
  newPWM = round((2.19 * newRPM)+33.5) ;
  //newPWM = round(newRPM);
  if(newPWM<0)
  newPWM = 0;
  else if (newPWM>255)
  newPWM = 255;
   fwd(newPWM);
    }
  else{ 
   newPWM = (0.345 * newRPM * newRPM) -(12.4 * newRPM) + 1059;
   rvs(newPWM);  
  }
  cb_flag = false;
} 

void callback(char* topic, unsigned char* payload, unsigned int length) {
    Serial.print("topic ");
    Serial.println(topic);
    //Faz o parse do payload para um objeto json
    //Payload example: "{"Direction": 0,"New_RPM": 80}"
    StaticJsonDocument<50> doc;
    DeserializationError error =  deserializeJson(doc, payload);

    //Se não conseguiu fazer o parser
    if(error)
    {
        //Exibe mensagem de erro
        Serial.print(F("deserializeJson() failed with code "));
        Serial.println(error.c_str());
        return;
    }

    //Recupera o atributo "value" do json
    dir = doc["Direction"];
    encoder1.newRPM = doc["New_RPM"];
    cb_flag = true;
    //RPM2PWM(dir, encoder1.newRPM);
    /*Serial.print("Novo RPM:");
    Serial.print(value);
    Serial.println();*/
    }
  

//mqtt ref: https://randomnerdtutorials.com/esp32-mqtt-publish-subscribe-arduino-ide/
void connectMQTTServer() {
  Serial.println("Connecting to MQTT Server...");
  //Se conecta com as credenciais obtidas no site do Watson IoT
  //quando cadastramos um novo device
  if (client.connect("Esp32","MqttId","MqttPassword")) {
    //Se a conexão foi bem sucedida
    Serial.println("Connected to MQTT Broker");
    //Quando algo for postado em algum tópico que estamos inscritos
    //a função "callback" será executada
    client.setCallback(callback);
    //Se inscreve nos tópicos de interesse
    client.subscribe("Set_RPM");
    digitalWrite(CONNECT_PIN, HIGH);
  } else {
    //Se ocorreu algum erro
    Serial.print("error = ");
    Serial.println(client.state());
    connectMQTTServer(); //tenta conectar novamente
  } 
}

void setup() {
  Serial.begin(115200);
  pinMode(CONNECT_PIN, OUTPUT);

  //declaração do objeto wifiManager
  WiFiManager wifiManager;

  //callback para quando entra em modo de configuração AP
  //wifiManager.setAPCallback(configModeCallback); 
  //callback para quando se conecta em uma rede, ou seja, quando passa a trabalhar em modo estação
  //wifiManager.setSaveConfigCallback(saveConfigCallback); 

  //caso queira iniciar o Portal para se conectar a uma rede toda vez, sem tentar conectar 
  //a uma rede salva anteriormente, use o startConfigPortal em vez do autoConnect
  //wifiManager.startConfigPortal("ESP_AP");


  //IP DO PORTAL --> 192.168.4.1
  wifiManager.autoConnect("ESP_AP"); //cria uma rede sem senha

  //Conectamos à rede WiFi
  //setupWiFi();
  //Conectamos ao server MQTT
  connectMQTTServer();

  //interrupt stuff
  pinMode(encoder1.PIN, INPUT_PULLDOWN);
  attachInterrupt(encoder1.PIN, &encoder_count, RISING);
  //PS chosen in order to make 80MHz/80
  // Receives: (Timer used, PS, flag to count up or down)
  timer = timerBegin(0, 80, true);
  //timer2 = timerBegin(1, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  //timerAttachInterrupt(timer2, &onTimer2, true);
  //interrupt every two seconds
  timerAlarmWrite(timer, 400000, true);
  //timerAlarmWrite(timer2, 100000, true);
  timerAlarmEnable(timer);
  //timerAlarmEnable(timer2);

  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  
  pinMode(motorA_pwm, OUTPUT);
  ledcAttachPin(motorA_pwm, 0); //Atribui o pino do PWM ao canal 0, é possível ir de 0-15
  /* frequência: 1-40Mhz e Resoução de 1-16 bits
  //resoluçãoa atual: 2^8 = 256 
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


/*void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("Esp32")) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("teste");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}*/

void loop() {
 /* if (!client.connected()) {
    reconnect();
    
  }*/
  static float rpm, rpmlast = 0;

  if (interrupt2flag) {
     rpm = encoder1.RPM; 
     rpm = 0.7*rpm+0.3*rpmlast;
     if(rpm>120 || rpm<0)
      rpm = rpmlast;
     rpmlast = rpm;
    //Serial.println("oi");
    //Serial.println("Novo RPM:");
    //Serial.print(encoder1.newRPM);
    float  teste = PI_Control(encoder1.newRPM,rpm);
    RPM2PWM(dir,teste);
    interrupt2flag = false;

    Serial.print(error);
    Serial.println();
    //Serial.print("\t");
    //Serial.print("valor calculado:");
    //Serial.print(Controlled_rpm);
    //Serial.print("\t");
    //Serial.print(newPWM);
    //Serial.print("\t");
    //Serial.print(rpm);
    //Serial.print("\t");
    //Serial.print(encoder1.RPM);
    //Serial.println();
  }
  client.loop();
  long now = millis();
    if (now - lastMsg > 1000) {
    lastMsg = now;  
    //Current RPM
    // Convert the value to a char array
    if(rpm-rpmlast<10)
    rpm=rpmlast;
    char rpmString[8];
    dtostrf(rpm, 1, 2, rpmString);
    client.publish("RPM", rpmString);
  }
    
}
