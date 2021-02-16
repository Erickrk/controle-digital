#include <Arduino.h>

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

//needed for WiFiManager library
#include <DNSServer.h> //Local DNS Server used for redirecting all requests to the configuration portal ( https://github.com/zhouhan0126/DNSServer---esp32 )
#include <WebServer.h> //Local WebServer used to serve the configuration portal ( https://github.com/zhouhan0126/WebServer-esp32 )

#include <WiFiManager.h>   // WiFi Configuration Magic ( https://github.com/zhouhan0126/WIFIMANAGER-ESP32 ) >> https://github.com/tzapu/WiFiManager (ORIGINAL)


#define SSID "yourSSID"
#define PASSWORD "yourPASS"

const String MQTT_SERVER = "your_server";

//Pin for mqtt connection feedback
#define CONNECT_PIN 23
bool cb_flag = false;

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
void IRAM_ATTR encoder_count() 
{
    encoder1.counted ++;
}

volatile int interruptCounter;
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool interrupt2flag;

//Timer and Encoder interrupts
void IRAM_ATTR onTimer() 
{
  portENTER_CRITICAL_ISR(&timerMux);
  encoder1.RPM = encoder1.counted*5; //divide by disk rips * 60 * 20 to convert to RPM
  encoder1.counted = 0;
  interrupt2flag = true;
  portEXIT_CRITICAL_ISR(&timerMux);
}

//Control stuff
//Ref: https://www.element14.com/community/community/arduino/blog/2020/01/06/simple-arduino-dc-motor-control-with-encoder-part-2
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

/*WIFIMANAGER*/
/*void configModeCallback (WiFiManager *myWiFiManager) 
{  
//  Serial.println("Entered config mode");
  Serial.println("Entrou no modo de configuração");
  Serial.println(WiFi.softAPIP()); //imprime o IP do AP
  Serial.println(myWiFiManager->getConfigPortalSSID()); //imprime o SSID criado da rede
}

//callback notifying us of the need to save config
//callback que indica que salvamos uma nova rede para se conectar (modo estação)
void saveConfigCallback ()
{
//  Serial.println("Should save config");
  Serial.println("Configuração salva");
  Serial.println(WiFi.softAPIP()); //imprime o IP do AP
}
*/

/*L298 definitions: 
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

int newPWM=0;
void RPM2PWM(byte dir, float newRPM)
{
  // Polinomial regression without pow()
  if(dir==1){
  //newPWM = round((0.996 * newRPM) + (0.0116 * newRPM * newRPM) + 54.7);
  newPWM = round((2.19 * newRPM)+33.5) ;
  //newPWM = round(newRPM);
    if(newPWM<0)
      newPWM = 0;
    else 
      if (newPWM>255)
           newPWM = 255;
   fwd(newPWM);
    }
  else{ 
   newPWM = (0.345 * newRPM * newRPM) -(12.4 * newRPM) + 1059;
   rvs(newPWM);  
  }
  cb_flag = false;
} 

void callback(char* topic, unsigned char* payload, unsigned int length)
{
    Serial.print("topic ");
    Serial.println(topic);
    //Payload example: "{"Direction": 0,"New_RPM": 80}"
    StaticJsonDocument<50> doc;
    DeserializationError error =  deserializeJson(doc, payload);
    if(error)
    {
        Serial.print(F("deserializeJson() failed with code "));
        Serial.println(error.c_str());
        return;
    }
    //Recover json atributes
    dir = doc["Direction"];
    encoder1.newRPM = doc["New_RPM"];
    cb_flag = true;
    //RPM2PWM(dir, encoder1.newRPM);
    /*Serial.print("Novo RPM:");
    Serial.print(value);
    Serial.println();*/
    }
  
//mqtt ref: https://randomnerdtutorials.com/esp32-mqtt-publish-subscribe-arduino-ide/
void connectMQTTServer() 
{
  Serial.println("Connecting to MQTT Server...");
  if (client.connect("Esp32","MqttId","MqttPassword")) 
  {
    Serial.println("Connected to MQTT Broker");
    client.setCallback(callback);
    client.subscribe("Set_RPM");
    digitalWrite(CONNECT_PIN, HIGH);
  } 
  else 
  {
    Serial.print("error = ");
    Serial.println(client.state());
    connectMQTTServer(); 
  } 
}

void setup() {
  Serial.begin(115200);
  pinMode(CONNECT_PIN, OUTPUT);

  WiFiManager wifiManager;
  
  //callback when entering AP configuration mode
  //wifiManager.setAPCallback(configModeCallback); 
  //callback when connecting to a network, that is, when working in station mode
  //wifiManager.setSaveConfigCallback(saveConfigCallback); 

  // if you want to start the Portal to connect to a network every time, without trying to connect
  // to a previously saved network, use startConfigPortal instead of autoConnect
  //wifiManager.startConfigPortal("ESP_AP");


  //Portal IP  --> 192.168.4.1
  wifiManager.autoConnect("ESP_AP"); //creates an open network  
  //setupWiFi();

  connectMQTTServer();

  //interrupt stuff
  pinMode(encoder1.PIN, INPUT_PULLDOWN);
  attachInterrupt(encoder1.PIN, &encoder_count, RISING);
  //PS chosen in order to make 80MHz/80
  //Receives: (Timer used, PS, flag to count up or down)
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
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

/*void reconnect() 
{
  // Loop until we're reconnected
  while (!client.connected()) 
  {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("Esp32")) 
    {
      Serial.println("connected");
      // Subscribe
      client.subscribe("teste");
    } 
    else 
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}*/

void loop() 
{
 /* if (!client.connected()) 
  {
    reconnect();
    
  }*/
  static float rpm, rpmlast = 0;

  if (interrupt2flag) 
  {
     rpm = encoder1.RPM; 
     rpm = 0.7*rpm+0.3*rpmlast;
     if(rpm > 120 || rpm < 0)
     rpm = rpmlast;
     rpmlast = rpm;
    //Serial.println("Novo RPM:");
    //Serial.print(encoder1.newRPM);
    float  teste = PI_Control(encoder1.newRPM,rpm);
    RPM2PWM(dir,teste);
    interrupt2flag = false;

    Serial.print(error);
    Serial.println();
    //Serial.print("\t");
    //Serial.print("calculated value:");
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
    if (now - lastMsg > 1000) 
    {
      lastMsg = now;  
      if(rpm-rpmlast<10)
      rpm=rpmlast;
      char rpmString[8];
      dtostrf(rpm, 1, 2, rpmString);
      client.publish("RPM", rpmString);
    }
    
}
