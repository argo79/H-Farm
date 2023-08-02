//############################
//#                          #
//#   CENTRALINA LAGHETTO    #
//#        v. 3.2            #
//#                          #
//#        by Argo           #
//#                          #
//############################


#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  5        /* Time ESP32 will go to sleep (in seconds) */

RTC_DATA_ATTR int bootCount = 0;

#define LATITUDE        41.12345
#define LONGITUDE       -87.98765
#define DST_OFFSET      -5

//////////////////////////////////
//          LIBRERIE            //
//////////////////////////////////
#include <WiFi.h>
#include "time.h"
#include <TimeLib.h>
#include <PubSubClient.h>
#include <sunset.h>
#include <math.h>

// OTA
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
// OTA
#include "BluetoothSerial.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Tone32.h>
#include "melodies.h"
#include "buzzer.h"

//////////////////////////////////
//           BUZZER             //
//////////////////////////////////
#define BUZZER_PIN 14   // 5
#define BUZZER_CHANNEL 0
#define pinPir 25
#define pinSen 32
#define volBC 12    // pin pwm per volume   // DA CONTROLLARE


//////////////////////////////////
//         NTP CLIENT           //
//////////////////////////////////
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 3600;
int   daylightOffset_sec = 0;


//////////////////////////////////
//           WIFI               //
//////////////////////////////////
const char* ssid = "*********";
const char* password = "******";
const char* mqtt_server = "x.x.x.x";


//////////////////////////////////
//           SUNSET             //
//////////////////////////////////
const uint8_t _usDSTStart[22] = { 8,14,13,12,10, 9, 8,14,12,11,10, 9,14,13,12,11, 9};
const uint8_t _usDSTEnd[22]   = { 1, 7, 6, 5, 3, 2, 1, 7, 5, 4, 3, 2, 7, 6, 5, 4, 2};
static const char ntpServerName[] = "us.pool.ntp.org";
const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

unsigned int localPort = 8888;  // local port to listen for UDP packets


//////////////////////////////////
//         PARAMETRI            //
//////////////////////////////////
// Configurazione Mosquitto

#define nomeClient        "Centralina Giardino"

#define Messaggio_topic   "sonda/esp32/giardino/messaggio"
#define pompaUno_topic    "sonda/esp32/giardino/pompaUno"
#define pompaDue_topic    "sonda/esp32/giardino/pompaDue"
#define Valvola_topic     "sonda/esp32/giardino/Valvola"
#define Luce_topic        "sonda/esp32/giardino/Luce"
#define LuceA_topic       "sonda/esp32/giardino/LuceA"
#define SogliaSen_topic   "sonda/esp32/giardino/SogliaSen"
#define RitardoLamp_topic "sonda/esp32/giardino/RitardoLamp"
#define RitardoOnda_topic "sonda/esp32/giardino/RitardoOnda"
#define VolumeBuz_topic   "sonda/esp32/giardino/VolumeBuz"
#define LuceProg_topic    "sonda/esp32/giardino/LuceProg"
#define Midi_topic        "sonda/esp32/giardino/Midi"
#define TestAlarm_topic   "sonda/esp32/giardino/TestAlarm"
#define PirActive_topic   "sonda/esp32/giardino/PirActive"
#define LedRed_topic      "sonda/esp32/giardino/LedRed"
#define LedGreen_topic    "sonda/esp32/giardino/LedGreen"
#define LedBlu_topic      "sonda/esp32/giardino/LedBlu"
#define Pressione_topic   "sonda/esp32/giardino/Pressione"
#define Temperatura_topic "sonda/esp32/giardino/Temperatura"
#define Umidita_topic     "sonda/esp32/giardino/Umidita"
#define PresenzaIN_topic  "sonda/esp32/cucina/Presenza"

#define SEALEVELPRESSURE_HPA (1013.25)

const int pinPump1 = 13;  // PIN CONTROLLO POMPA UNO
const int pinPump2 = 15;  // PIN CONTROLLO POMPA DUE
const int ledRed = 17;  // 17 30W - 940mA - 34V - 1.8Ω (3W)      LUCE NATURALE // -> 19
const int ledGreen = 19;  // 19 30W - 940mA - 34V - 1.8Ω (3W)       SPETTRO TOTALE
const int ledBlu = 16;  // 16 50W - 1700mA - 34V - 1Ω (10W)     LUCE BIANCA 4000K  // ->16
//const int pinVentola=8;
const int sensoreLuce=32;   // fotoresistenza
const int ledBianco=18; 
//int timer=500; //millisecondi
int lastReconnectAttempt = 0;

// setting PWM properties 
const int ledChannelRed = 3;
const int ledChannelGreen = 4;
const int ledChannelBlu = 5;

const int resolution = 8;
const int freq = 1000;
const int freqV = 16000;

unsigned long timerMillis=60000;                        // TEMPO TIMER SERIALE
unsigned long tempoMillis;
unsigned long tempoPub;
unsigned long tempoLoop;
unsigned long tempoSpento;
unsigned long tempoAcceso;
unsigned long tempoOnda;

int ritardoLamp=250;
int ritardoOnda=50;
//int thisNote;
int midi=1;

//////////////////////////////////
//         VARIABILI            //
//////////////////////////////////

boolean statoPompa1=0;
boolean statoPompa2=0;
boolean statoPUOLD=0;
boolean presenzaIN=1;

char giorno[8], ora[3],minuto[3], result[8], valore[10];
int dstStart,dstStop,output_state;
int c=0; //contatore while BME nel SETUP
unsigned long luceA;
int sogliaSen=3500;
int pirSen;
int volumeAllarme=200;
int timerAl=375;
int luceProg=0;
boolean luceOnda=0;
byte ondaRed=0;
byte ondaGreen=0;
byte ondaBlu=0;
byte turnoLed=1;
byte luceNatale=1; // range 1-3 per i tre canali RGB

float Temp,Press,Umid,Alt;


byte luce=0;
byte luceRed=0;
byte luceGreen=0;
byte luceBlu=0;

// Variabili comunicazione
String dataPC;
String dataBT;
char serPC;
char serBT;
String dataMQTT;
int oraRise,minutoRise,oraSet,minutoSet,oraI,minutoI;


// Creazione oggetti

WiFiClient esp32Client;
PubSubClient client(esp32Client);
WiFiUDP Udp;
BluetoothSerial SerialBT;
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
Adafruit_BME280 bme; // I2C
bool status;
unsigned long delayTime;

SunSet sun;

//////////////////////////////////
//          MELODIE             //
//////////////////////////////////

// file melodies.h


void setup() {
// put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(pinPump1,OUTPUT);
  pinMode(pinPump2,OUTPUT);
  pinMode(ledBianco,OUTPUT);
  pinMode(pinPir,INPUT);
  digitalWrite(pinPump1,LOW);
  digitalWrite(pinPump2,LOW);
  digitalWrite(ledBianco,HIGH);
  delay(1500);
  digitalWrite(ledBianco,LOW);

  
// Associazione LED e PIN
  ledcSetup(ledChannelRed, freq, resolution);
  ledcSetup(ledChannelGreen, freq, resolution);
  ledcSetup(ledChannelBlu, freq, resolution);
  ledcSetup(ledChannelVol, freqV, resolution);
  
  ledcAttachPin(ledRed, ledChannelRed);
  ledcAttachPin(ledGreen, ledChannelGreen);
  ledcAttachPin(ledBlu, ledChannelBlu);
  ledcAttachPin(volBC, ledChannelVol);

// connessione bluetooth
  SerialBT.begin(nomeClient);                   //Bluetooth device name
  SerialBT.println("Bluetooth avviato!");
  delay(100);

  
// connessione alla rete wifi  
  SerialBT.print("Connecting to ");
  SerialBT.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    SerialBT.print(".");
  }


// Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
   ArduinoOTA.setHostname("centralina laghetto-OTA");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();

  
// Print local IP address and start web server
  SerialBT.println("");
  SerialBT.println("WiFi connected.");
  SerialBT.println("IP address: ");
  SerialBT.println(WiFi.localIP());

// inizializza il client NTP  //init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
// ottieni l'orario NTP  
  printLocalTime();

  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    SerialBT.println("Failed to obtain time");
    return;
  }

// Calcola ANNO

  char numYearC[5];
  strftime(numYearC,5, "%Y", &timeinfo);
  String numYearS=String(numYearC);
  int numYearI=numYearS.toInt();

// Imposta le date di DST (Ora legale/ora solare) *************
  switch (numYearI) {
    case 2021:
      dstStart=72;
      dstStop=302;
      break;
    case 2022:
      dstStart=72;
      dstStop=310;
      break;
  }

  
// Calcola GIORNO DELL'ANNO **************

  char numDayC[4];
  strftime(numDayC,4, "%j", &timeinfo);
  String numDayS=String(numDayC);
  int numDayI=numDayS.toInt();
  SerialBT.print("Siamo nell'anno "+numYearS+" e oggi c'è l'ora ");


// Calcola e imposta il DST ***************

  if (numDayI>=dstStart && numDayI<dstStop) {
    daylightOffset_sec = 3600;
    SerialBT.println("solare!");
  }
  else {
    daylightOffset_sec = 0;
    SerialBT.println("legale!");
  }


// reinizializza il client NTP  //init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
// ottieni l'orario NTP  
  printLocalTime();
  SerialBT.println(&timeinfo, "%A, %d %B %Y %H:%M:%S");
  
  // Avvio la connessione con il server mqtt (mosquitto)
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  
  status = bme.begin(0x76);  

  if (!status) {    
    while (c<10) {
      SerialBT.println("Could not find a valid BME280 sensor, check wiring!");
      c++;
    }
  }

  /* Get our time sync started */
  /* Set our position and a default timezone value */
  sun.setPosition(LATITUDE, LONGITUDE, DST_OFFSET);
  sun.setTZOffset(DST_OFFSET);
  //setSyncProvider(getNtpTime);
  //setSyncInterval(60*60);  
  
  
  // Inizializzo i timer
  
  tempoMillis=millis();
  tempoPub=millis();
  tempoLoop=millis();
  tempoSpento=millis();
  statoPompa1=0;
  statoPompa2=0;
  delayTime = 1000;

// Fine SETUP

  ledcWrite(ledChannelVol,9); 
  intro();
}


void loop() {
  
  ArduinoOTA.handle();
  
  if (!client.connected()) {
    reconnect();
  }  

  //strftime(oraH,3, "%H", &timeinfo);
  //strftime(minH,3, "%M", &timeinfo);


   
  if (millis()>tempoPub+timerMillis) {  

    sunSet();
    if(((oraI==oraRise && minutoI>=minutoRise) || oraI>oraRise) && (oraI<oraSet || (oraI==oraSet && minutoI<=minutoSet))) {
      SerialBT.println("Giorno!");
    }
    //Serial.println(oraI);
    else SerialBT.println("Notte!");

    
    //printValues();
    //delay(1);
    mqttpub();    
    tempoPub=millis(); 
  }
  
  BTSerialRicevi();                     // RICEVO MESSAGGI BLUETOOTH
  
/*
  if (millis()>tempoLoop+timerMillis/5) {  
    tempoLoop=millis();   
    struct tm timeinfo;
    getLocalTime(&timeinfo);
//    SerialBT.println(&timeinfo, "%A, %d %B %Y %H:%M:%S");
  }  
*/

  
/*
  if (statoPompa1==1) {
    accendoPompa();
  }
  else spengoPompa();
*/
  
  
  digitalWrite(pinPump1,statoPompa1);  
  digitalWrite(pinPump2,statoPompa2);
  if (statoPompa1==1 && statoPompa1==!statoPUOLD) {
    statoPUOLD=1;
    accendoPompa();
  }
  
  else if (statoPompa1==0 && statoPompa1==!statoPUOLD) {
    statoPUOLD=0;
    spengoPompa();
  }
  
  else {      
  }


  if (luceProg==0) {
    spengo();  
  }
  
  else if (luceProg==1) {
    accendo();
  }
  
  else if (luceProg==2) {
    lampeggio();
  }
  
  else if (luceProg==3) {
    ledcWrite(ledChannelRed,luceRed); 
    ledcWrite(ledChannelGreen,luceGreen);  
    ledcWrite(ledChannelBlu,luceBlu);      
  }

  else if (luceProg==4) {    
    onda();
  }
  
  else if (luceProg==5) {    
    luceBlu=240;
    ritardoLamp=350;  
    polizia();
  }
  
  else if (luceProg==6) {  
    ritardoOnda=20;
    luceRed=127;
    luceGreen=127;
    luceBlu=127;    
    onda();
  }
  
  else if (luceProg==7) {    // luce di Natale       
    natale();
  }


  allarmePir();                   // CONTROLLO ATTIVAZIONE ALLARME
  
  
  luceA=analogRead(pinSen);  

  if (luceA>sogliaSen) {    
    digitalWrite(ledBianco,1);
    luce=1;
  }   
  else {
    digitalWrite(ledBianco,0); 
    luce=0;   
  }
  
  client.loop();
}


// LOCAL TIME

void printLocalTime()
{
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    SerialBT.println("Failed to obtain time");
    return;
  }
  
  SerialBT.println(&timeinfo, "%A, %d %B %Y %H:%M:%S");
  strftime(ora,3, "%H", &timeinfo);
  strftime(minuto,3, "%M", &timeinfo);
/*
 * 
  Serial.println(&timeinfo, "%A, %d %B %Y %H:%M:%S");
  Serial.println(&timeinfo, "%A");

  giorno[8];
  strftime(giorno,8, "%A", &timeinfo);
  ora[3];
  strftime(ora,3, "%M", &timeinfo);

  float decimale=147.7;
  result[8]; // Buffer big enough for 7-character float
  dtostrf(decimale, 6, 2, result); // Leave room for too large numbers!

// Per costruire una stringa con dentro variabili
  buffer[80];
  sprintf(buffer, "Oggi è %s e sono i minuti %s, il valore decimale è: %s.", giorno, ora, result);
  Serial.println(buffer);

*/
}


void callback(char* topic, byte* payload, unsigned int length) {        // Ricezione messaggi MQTT dal broker
  
  String response;  
  for (int i = 0; i < length; i++) {
    response+=(char)payload[i];
  }
  
  String topicS=topic;
  //SerialBT.println(topicS);  
  
  if (topicS=="sonda/esp32/giardino/pompaUno") {
    //SerialBT.println(response);
    statoPUOLD=statoPompa1;    
    statoPompa1=response.toInt();    
  }
  else if (topicS=="sonda/esp32/giardino/pompaDue") {
    statoPompa2=response.toInt();
  }  
  else if (topicS=="sonda/esp32/giardino/LedRed") {
    luceRed=response.toInt();
  }
  else if (topicS=="sonda/esp32/giardino/LedGreen") {
    luceGreen=response.toInt();
  }
  else if (topicS=="sonda/esp32/giardino/LedBlu") {
    luceBlu=response.toInt();
  }
  else if (topicS=="sonda/esp32/giardino/SogliaSen") {
    sogliaSen=response.toInt();
  }
  else if (topicS=="sonda/esp32/giardino/VolumeBuz") {
    volume=response.toInt();
    SerialBT.println(response);
    SerialBT.println(volume);
    ledcWrite(ledChannelVol,volume); 
    tone(BUZZER_PIN,NOTE_A7,50, BUZZER_CHANNEL);
    delay(50);
    noTone(BUZZER_PIN, BUZZER_CHANNEL);
  }  

  else if (topicS=="sonda/esp32/giardino/RitardoLamp") {
    ritardoLamp=response.toInt();       
  }   
  else if (topicS=="sonda/esp32/giardino/RitardoOnda") {
    ritardoOnda=response.toInt();       
  }   

  else if (topicS=="sonda/esp32/giardino/LuceProg") {
    luceProg=response.toInt();
  }  
  
  else if (topicS=="sonda/esp32/giardino/TestAlarm") {
    int timerAl=response.toInt();
    volume=127;
    for (int i=0;i<10;i++) {      
      ledcWrite(ledChannelVol,volume+=2); 
      tone(BUZZER_PIN,NOTE_A7,timerAl-=15, BUZZER_CHANNEL);
      delay(timerAl/2);
      noTone(BUZZER_PIN, BUZZER_CHANNEL);
      delay(timerAl/2);
      tone(BUZZER_PIN,NOTE_A7,timerAl-=15, BUZZER_CHANNEL);
      delay(timerAl/2);
      noTone(BUZZER_PIN, BUZZER_CHANNEL);
      delay(timerAl/2);   
    }
    tone(BUZZER_PIN,NOTE_D8,1000, BUZZER_CHANNEL);
    delay(1500);
    noTone(BUZZER_PIN, BUZZER_CHANNEL);
  }
    
    
  else if (topicS=="sonda/esp32/giardino/Midi") {
    dataMQTT=String(response);
    dataMQTT.trim();
    
    if (dataMQTT.length()>0) {
          
      if (dataMQTT=="jingle") {  
        SerialBT.print("Midi: ");    
        SerialBT.println(dataMQTT);    
        jingle();          
      }
      else if (dataMQTT=="merry1") {                           
          
        SerialBT.print("Midi: ");    
        SerialBT.println("Merry Christmas short"); 
        merryC();       
      }
      else if (dataMQTT=="merry2") {                           
          
        SerialBT.print("Midi: ");    
        SerialBT.println("Merry Christmas long"); 
        merryC2();       
      }
      else if (dataMQTT=="santaC") {                           
          
        SerialBT.print("Midi: ");    
        SerialBT.println("Santa Claus"); 
        santaC();       
      }
      else if (dataMQTT=="star") {                           
          
        SerialBT.print("Midi: ");    
        SerialBT.println("Star Wars"); 
        starWars();       
      }
      else if (dataMQTT=="march") {                           
          
        SerialBT.print("Midi: ");    
        SerialBT.println("Imperial March"); 
        imperialMarch();       
      }
      else if (dataMQTT=="hazard") {                           
          
        SerialBT.print("Midi: ");    
        SerialBT.println("Hazard"); 
        hazard();       
      }
      else if (dataMQTT=="pirates") {                           
          
        SerialBT.print("Midi: ");    
        SerialBT.println("Pirati dei Caraibi"); 
        pirates();       
      }

      else if (dataMQTT=="pirates2") {                           
          
        SerialBT.print("Midi: ");    
        SerialBT.println("Pirati dei Caraibi2"); 
        tempoSong=millis();
        int thisNote=0;
        pirates2();       
      }
      
      else if (dataMQTT=="harry") {                           
          
        SerialBT.print("Midi: ");    
        SerialBT.println("Harry Potter"); 
        HarryP();       
      }
      else if (dataMQTT=="star") {                           
          
        SerialBT.print("Midi: ");    
        SerialBT.println("Star Trek intro"); 
        starTrekIntro();       
      }
      else if (dataMQTT=="brau") {                           
          
        SerialBT.print("Midi: ");    
        SerialBT.println("Brau rit"); 
        brauRit();       
      } 
    }  
      
    dataMQTT="";
  }   
        
  else if (topicS=="sonda/esp32/cucina/Presenza") {
    presenzaIN=response.toInt();
  }

    
  //delay(1); 
}


void reconnect() {                            // CONNESSIONE BROKER MQTT
  // Loop until we're reconnected
  while (!client.connected()) {
    SerialBT.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(nomeClient)) {
      SerialBT.println("connected");      
      client.subscribe(pompaUno_topic,1);
      client.subscribe(pompaDue_topic,1);
      client.subscribe(SogliaSen_topic,1);
      client.subscribe(VolumeBuz_topic,1);
      client.subscribe(LuceProg_topic,1);
      client.subscribe(Midi_topic,1);
      client.subscribe(TestAlarm_topic,1);
      client.subscribe(Luce_topic,1);
      client.subscribe(LedRed_topic,1);
      client.subscribe(LedGreen_topic,1);
      client.subscribe(LedBlu_topic,1);
      client.subscribe(PresenzaIN_topic,1);
      client.subscribe(RitardoLamp_topic,1);
      client.subscribe(RitardoOnda_topic,1);
      
    } else {
      SerialBT.print("failed, rc=");
      SerialBT.print(client.state());
      SerialBT.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      yield();
      delay(1000);
    }
  }
}


void BTSerialRicevi() {                    // GESTIONE CONTROLLO BLUETOOTH
  if (SerialBT.available()) {
    
    while (SerialBT.available()) {
      serBT=SerialBT.read();
      dataBT+=serBT;
      yield();        
    }          
    //Serial.println(dataBT); 
    dataBT.trim();
    
    if (dataBT.length()>0) {
          
      if (dataBT=="reset") {      
        restartESP32();  
        }             
      
      else if (dataBT.startsWith("ledRed")) {                           
        String luceRedS=dataBT.substring(6);
        luceRed=luceRedS.toInt();     
        SerialBT.print("PWM Rosso: ");    
        SerialBT.println(luceRed);        
      } 
      
      else if (dataBT.startsWith("ledGre")) {                    
        String luceGreS=dataBT.substring(6);
        luceGreen=luceGreS.toInt();     
        SerialBT.print("PWM Verde: ");    
        SerialBT.println(luceGreen);
      }

      else if (dataBT.startsWith("ledBlu")) {                      
        String luceBluS=dataBT.substring(6);
        luceBlu=luceBluS.toInt();     
        SerialBT.print("PWM Blu: ");    
        SerialBT.println(luceBlu);
      }

      else if (dataBT.startsWith("spegni")) {                      
        luceRed=0;
        luceGreen=0;
        luceBlu=0;
        statoPompa1=0;
        snprintf (valore, 10, "%d", statoPompa1);
        client.publish(pompaUno_topic, valore);
        statoPompa2=0;
        snprintf (valore, 10, "%d", statoPompa2);
        client.publish(pompaDue_topic, valore); 
        SerialBT.println("Spengo tutto");    
      }

      else if (dataBT.startsWith("accendi")) {                     
        luceRed=255;
        luceGreen=255;
        luceBlu=255; 
        statoPompa1=1;
        snprintf (valore, 10, "%d", statoPompa1);
        client.publish(pompaUno_topic, valore);
        statoPompa2=1;
        snprintf (valore, 10, "%d", statoPompa2);
        client.publish(pompaDue_topic, valore); 
        SerialBT.println("Accendo tutto");    
      }

      else if (dataBT.startsWith("pompaUno")) {                      
        String pompaUnoS=dataBT.substring(8);
        statoPUOLD=statoPompa1;
        statoPompa1=pompaUnoS.toInt();   
        SerialBT.print("PompaUno: ");    
        SerialBT.println(statoPompa1);  
        snprintf (valore, 10, "%d", statoPompa1);
        client.publish(pompaUno_topic, valore);
      }

      else if (dataBT.startsWith("pompaDue")) {                
        String pompaDueS=dataBT.substring(8);
        statoPompa2=pompaDueS.toInt(); 
        SerialBT.print("PompaDue: ");    
        SerialBT.println(statoPompa2);
        snprintf (valore, 10, "%d", statoPompa2);
        client.publish(pompaDue_topic, valore);
      }

      else if (dataBT.startsWith("laps")) {                    
         String lapsS=dataBT.substring(4);
         timerMillis=lapsS.toInt()*60000; 
         SerialBT.print("Intervallo: ");    
         SerialBT.println(timerMillis/60000);                
      }

      else if (dataBT.startsWith("soglia")) {                    
         String sogliaS=dataBT.substring(6);
         sogliaSen=sogliaS.toInt(); 
         SerialBT.print("Soglia: ");    
         SerialBT.println(sogliaSen);                
      }

      else if (dataBT.startsWith("volTot")) {                    
        String volTotS=dataBT.substring(6);
        volume=volTotS.toInt(); 
        SerialBT.print("Volume: ");    
        SerialBT.println(volume);
        ledcWrite(ledChannelVol,volume); 
        spengoPompa();                
      }
      
      else if (dataBT.startsWith("help")) {                       // HELP -> Comando        
        SerialBT.println("Comandi:"); 
        SerialBT.println("");
        SerialBT.println("reset          -> Resetta Esp32");    
        SerialBT.println("accendi        -> Accendi tutto!");        
        SerialBT.println("spegni         -> Spegni tutto!");  
        SerialBT.println("ledRed(0-255)  -> PWM led rosso.");    
        SerialBT.println("ledGre(0-255)  -> PWM led verde.");    
        SerialBT.println("ledBlu(0-255)  -> PWM led blu.");  
        SerialBT.println("pompaUno(1/0)  -> Pompa uno ON/OFF");            
        SerialBT.println("pompaDue(1/0)  -> Pompa due ON/OFF");  
        SerialBT.println("laps(1-60)     -> Intervallo valori");
        SerialBT.println("soglia(1-4096) -> Soglia sensore luce");    
        SerialBT.println("volTot(1-255)  -> Volume buzzer [15]");                               
        SerialBT.println("info           -> Stampa valori.");
        SerialBT.println("takeom         -> Melodia A-ha Take on me");
        SerialBT.println("star           -> Melodia Star Wars");
        SerialBT.println("pirates        -> Melodia Pirati Caraibi");
        SerialBT.println("harry          -> Melodia Harry Potter");
        SerialBT.println("intro          -> Melodia Intro Star Trek");
        SerialBT.println("march          -> Melodia Imperial March");
        SerialBT.println("hazard         -> Melodia Hazard");
        SerialBT.println("bevo           -> Melodia Bevo bevo");
        SerialBT.println("brau           -> Melodia Brau Rit");
        SerialBT.println("santaC         -> Santa Claus");
        SerialBT.println("merry1         -> Merry Christmas");
        SerialBT.println("merry2         -> Merry Christmas long");
        SerialBT.println("jingle         -> Jingle Bells");
        SerialBT.println("buzTime(0.5-2) -> Fattore velocita'");
        SerialBT.println("testA(100-1000)-> Test Allarme'");
        SerialBT.println("help           -> Questo comando.");
        SerialBT.println("");
        //delay(50);       
      }

      else if (dataBT.startsWith("info")) {                       // HELP -> Comando        
        printLocalTime();
        
        printValues();    
      }

      else if (dataBT.startsWith("star")) {                       // HELP -> Comando        
        starWars();
      }

      else if (dataBT.startsWith("takeom")) {                       // HELP -> Comando        
        takeOM();
      }

      else if (dataBT.startsWith("pirates")) {                       // HELP -> Comando        
        pirates();
      }

      else if (dataBT.startsWith("pirates2")) {                       // HELP -> Comando        
        pirates2();
      }
      
      else if (dataBT.startsWith("harry")) {                       // HELP -> Comando        
        HarryP();
      }

      else if (dataBT.startsWith("intro")) {                       // HELP -> Comando        
        starTrekIntro();
      }
      
      else if (dataBT.startsWith("march")) {                       // HELP -> Comando        
        imperialMarch();
      }

      else if (dataBT.startsWith("hazard")) {                       // HELP -> Comando        
        hazard();
      }
      
      else if (dataBT.startsWith("bevo")) {                       // HELP -> Comando        
        bevobevo();
      }

      else if (dataBT.startsWith("brau")) {                       // HELP -> Comando        
        brauRit();
      }

      else if (dataBT.startsWith("jingle")) {                       // HELP -> Comando        
        jingle();
      }

      else if (dataBT.startsWith("santaC")) {                       // HELP -> Comando        
        santaC();
      }

      else if (dataBT.startsWith("merry1")) {                       // HELP -> Comando        
        merryC();
      }

      else if (dataBT.startsWith("merry2")) {                       // HELP -> Comando        
        merryC2();
      }
      
      else if (dataBT.startsWith("buzTime")) {                       // HELP -> Comando        
        String buzT=dataBT.substring(7);
        fattoreBuzzer=buzT.toFloat();
        SerialBT.print("Tempo melodia: ");    
        SerialBT.println(fattoreBuzzer);                
      }


      else if (dataBT.startsWith("testA")) {                       // HELP -> Comando        
        String timerAlT=dataBT.substring(5);
        int timerAl=timerAlT.toInt();
        if (volume>31) {
          volume=31;
        }
        //volume=4;
        for (int i=0;i<10;i++) {      
          ledcWrite(ledChannelVol,volume+=2); 
          tone(BUZZER_PIN,NOTE_A7,timerAl-=15, BUZZER_CHANNEL);
          delay(timerAl/2);
          noTone(BUZZER_PIN, BUZZER_CHANNEL);
          delay(timerAl/2);
          tone(BUZZER_PIN,NOTE_A7,timerAl-=15, BUZZER_CHANNEL);
          delay(timerAl/2);
          noTone(BUZZER_PIN, BUZZER_CHANNEL);
          delay(timerAl/2);   
        }
        tone(BUZZER_PIN,NOTE_D8,1000, BUZZER_CHANNEL);
        delay(1500);
        noTone(BUZZER_PIN, BUZZER_CHANNEL);
      }
      
      else {
        SerialBT.println("Valore sbagliato. Nessun comando riconosciuto.");          
      }
    }
  }
    dataBT="";
    //delay(1);
}


void mqttpub() {                            // MANDO LA STRINGA MQTT

  if (!client.connected()) {
    reconnect();
  }
  
  if (!status) {
    SerialBT.println("Could not find a valid BME280 sensor, check wiring!");
    c=0;
    while (!status && c<10) {
      status=bme.begin(0x76);
      c++;
      yield();
    }
    restartESP32();
    error();
  }
    
  client.publish(Messaggio_topic, "Valori:");
  
  Temp=bme.readTemperature();
  delay(1);
  snprintf (valore, 10, "%.2f",Temp);  
  client.publish(Temperatura_topic, valore);
  Press=bme.readPressure()/100.0F;
  delay(1);
  snprintf (valore, 10, "%.2f",Press);
  client.publish(Pressione_topic, valore);
  Umid=bme.readHumidity();
  delay(1);
  snprintf (valore, 10, "%.2f",Umid);
  client.publish(Umidita_topic, valore);

  snprintf (valore, 10, "%d", luceA);
  client.publish(LuceA_topic, valore);
  snprintf (valore, 10, "%d", sogliaSen);
  client.publish(SogliaSen_topic, valore);
  
  snprintf (valore, 10, "%d",pirSen);
  client.publish(PirActive_topic, valore);
  
  snprintf (valore, 10, "%d", statoPompa1);
  client.publish(pompaUno_topic, valore);
  snprintf (valore, 10, "%d", statoPompa2);
  client.publish(pompaDue_topic, valore);
  
  snprintf (valore, 10, "%d", luce);
  client.publish(Luce_topic, valore);
  snprintf (valore, 10, "%d", luceRed);
  client.publish(LedRed_topic, valore);
  snprintf (valore, 10, "%d", luceGreen);
  client.publish(LedGreen_topic, valore);
  snprintf (valore, 10, "%d", luceBlu);
  client.publish(LedBlu_topic, valore);  
  snprintf (valore, 10, "%d", luceProg);
  client.publish(LuceProg_topic, valore);  
    
}


void printValues() {                        // LEGGE TUTTI I VALORI DEI SENSORI
  
  if (!status) {
    SerialBT.println("Could not find a valid BME280 sensor, check wiring!");
    c=0;
    while (!status && c<10) {
      status=bme.begin(0x76);
      c++;
      yield();
    }
    restartESP32();
    error();
  }

  else {
    SerialBT.println("Valori attuali:");
    SerialBT.println(""); 
    
    Temp=bme.readTemperature();
    delay(1);
    SerialBT.print("Temperatura:     ");
    SerialBT.print(Temp);
    SerialBT.println(" *C");
    
    // Convert temperature to Fahrenheit
    /*Serial.print("Temperature = ");
    Serial.print(1.8 * bme.readTemperature() + 32);
    Serial.println(" *F");*/
  
    Press=bme.readPressure()/100.0F;
    delay(1);
    SerialBT.print("Pressione:       ");
    SerialBT.print(Press);
    SerialBT.println(" hPa");
  
    Alt=bme.readAltitude(SEALEVELPRESSURE_HPA);
    delay(1);
    SerialBT.print("Altitudine:      ");
    SerialBT.print(Alt);
    SerialBT.println(" m");
  
    Umid=bme.readHumidity();
    delay(1);
    SerialBT.print("Umidità':        ");
    SerialBT.print(Umid);
    SerialBT.println(" %");
    SerialBT.println("");

    SerialBT.print("Quantità luce: ");
    SerialBT.println(luceA);
    SerialBT.print("Soglia sensore luce: ");
    SerialBT.println(sogliaSen);
    
    SerialBT.print("Stato pompa uno: ");
    SerialBT.println(statoPompa1);
    SerialBT.print("Stato pompa due: ");
    SerialBT.println(statoPompa2);
    SerialBT.print("Stato led rosso: ");
    SerialBT.println(luceRed);
    SerialBT.print("Stato led verde: ");
    SerialBT.println(luceGreen);
    SerialBT.print("Stato led blu:   ");
    SerialBT.println(luceBlu);
    SerialBT.println("");    
  }  
}


void allarmePir() {
  
  if (presenzaIN==1) {                        // C'È QUALCUNO IN CUCINA
    //SerialBT.println("ALLARME DISATTIVATO");
    
    pirSen=digitalRead(pinPir);
    //SerialBT.println(pirSen);
    //snprintf (valore, 10, "%d",pirSen);
    //client.publish(PirActive_topic, valore);
    if (pirSen==1) {
      //Serial.println("Movimento! in presenza");
      SerialBT.println("Movimento in presenza!");
      snprintf (valore, 10, "%d",pirSen);
      client.publish(PirActive_topic, valore);
      if (volume>31) {
        volume=31;
      }
      //volume=31;
      ledcWrite(ledChannelVol,volume);
      ledcWrite(ledChannelGreen,250); 
      ledcWrite(ledChannelBlu,250);  
      ledcWrite(ledChannelRed,0); 

      switch (midi) {
        case 1:
          SerialBT.println("Jingle!");
          intro();
          midi++;
          break;
        case 2:
          SerialBT.println("SantaC!");
          starWars();
          midi++;
          break;
        case 3:
          SerialBT.println("MerryC!");
          hazard();
          midi=1;
          break;         
      }
      
      ledcWrite(ledChannelRed,0); 
      ledcWrite(ledChannelGreen,0); 
      ledcWrite(ledChannelBlu,0);  
    }
    
    else {
      //spengoPompa(); 
    }
  }
  
  else if (presenzaIN==0) {                   // MODALITÀ ALLARME ATTIVATA
    //SerialBT.println("ALLARME ASSENZA ATTIVATO!");    
    //SerialBT.println(digitalRead(pirPin));
    pirSen=digitalRead(pinPir);
    //SerialBT.println(pirSen);    
    if (pirSen==1) {
      //Serial.println("Movimento!");
      SerialBT.println("Movimento in assenza!"); 
      snprintf (valore, 10, "%d",pirSen);
      client.publish(PirActive_topic, valore);      
      timerAl=375;     
      for (int i=0;i<5;i++) {      
        ledcWrite(ledChannelVol,volumeAllarme+=5); 
        tone(BUZZER_PIN,NOTE_A7,timerAl-=25, BUZZER_CHANNEL);
        delay(timerAl/2);
        noTone(BUZZER_PIN, BUZZER_CHANNEL);
        delay(timerAl/2);
        tone(BUZZER_PIN,NOTE_A7,timerAl-=25, BUZZER_CHANNEL);
        delay(timerAl/2);
        noTone(BUZZER_PIN, BUZZER_CHANNEL);
        delay(timerAl/2);   
      }      
    }
    else {
      //spengoPompa(); 
    }
  }
}


void accendo() {
  ledcWrite(ledChannelRed,255); 
  ledcWrite(ledChannelGreen,255);  
  ledcWrite(ledChannelBlu,255);      
}


void spengo() {
  ledcWrite(ledChannelRed,0); 
  ledcWrite(ledChannelGreen,0);  
  ledcWrite(ledChannelBlu,0);      
}


void lampeggio() {  
  if (millis()>tempoSpento+ritardoLamp) {
    ledcWrite(ledChannelRed,luceRed);
    ledcWrite(ledChannelGreen,luceGreen);  
    ledcWrite(ledChannelBlu,luceBlu);     
    tempoAcceso=millis();
    tempoSpento=tempoAcceso+ritardoLamp;
  }
  else if (millis()>tempoAcceso+ritardoLamp) {
    ledcWrite(ledChannelRed,0);
    ledcWrite(ledChannelGreen,0);  
    ledcWrite(ledChannelBlu,0);
    tempoSpento=millis();     
    tempoAcceso=tempoSpento+ritardoLamp;        
  }    
}


void onda() {
  
  if (millis()>=tempoOnda+ritardoOnda && turnoLed==1 && luceOnda==0) {                  ///// ONDA ROSSA
    ledcWrite(ledChannelRed,ondaRed);
    ledcWrite(ledChannelGreen,0);  
    ledcWrite(ledChannelBlu,0);
    ondaRed=ondaRed+1;
    if (ondaRed>luceRed) {
      luceOnda=1;     
    }
    tempoOnda=millis();
  }
  else if (millis()>=tempoOnda+ritardoOnda && turnoLed==1 && luceOnda==1) {
    ledcWrite(ledChannelRed,ondaRed);
    ledcWrite(ledChannelGreen,0);  
    ledcWrite(ledChannelBlu,0);
    ondaRed=ondaRed-1;   
    if (ondaRed<=0) {
      luceOnda=0; 
      turnoLed=2;
      ondaGreen=0;
      ondaBlu=0;             
    }
    tempoOnda=millis();
  }
    
  if (millis()>=tempoOnda+ritardoOnda && turnoLed==2 && luceOnda==0) {                  ///// ONDA GREEN
    ledcWrite(ledChannelRed,0);
    ledcWrite(ledChannelGreen,ondaGreen);  
    ledcWrite(ledChannelBlu,0);
    ondaGreen=ondaGreen+1;
    if (ondaGreen>luceGreen) {
      luceOnda=1;     
    }
    tempoOnda=millis();
  }
  else if (millis()>=tempoOnda+ritardoOnda && turnoLed==2 && luceOnda==1) {
    ledcWrite(ledChannelRed,0);
    ledcWrite(ledChannelGreen,ondaGreen);  
    ledcWrite(ledChannelBlu,0);
    ondaGreen=ondaGreen-1;   
    if (ondaGreen<=0) {
      luceOnda=0; 
      turnoLed=3;
      ondaRed=0; 
      ondaBlu=0;   
    }
    tempoOnda=millis();
  }  
  
  if (millis()>=tempoOnda+ritardoOnda && turnoLed==3 && luceOnda==0) {                  ///// ONDA BLU
    ledcWrite(ledChannelRed,0);
    ledcWrite(ledChannelGreen,0);  
    ledcWrite(ledChannelBlu,ondaBlu);
    ondaBlu=ondaBlu+1;
    if (ondaBlu>luceBlu) {
      luceOnda=1;     
    }
    tempoOnda=millis();
  }
  else if (millis()>=tempoOnda+ritardoOnda && turnoLed==3 && luceOnda==1) {
    ledcWrite(ledChannelRed,0);
    ledcWrite(ledChannelGreen,0);  
    ledcWrite(ledChannelBlu,ondaBlu);
    ondaBlu=ondaBlu-1;   
    if (ondaBlu<=0) {
      luceOnda=0; 
      turnoLed=1;
      ondaRed=0; 
      ondaGreen=0;       
    }
    tempoOnda=millis();
  }  
}


void polizia() {
  if (millis()>tempoSpento+ritardoLamp) {    
    ledcWrite(ledChannelBlu,luceBlu);     
    tempoAcceso=millis();
    tempoSpento=tempoAcceso+ritardoLamp;
  }
  else if (millis()>tempoAcceso+ritardoLamp) {
    ledcWrite(ledChannelRed,0);
    ledcWrite(ledChannelGreen,0);  
    ledcWrite(ledChannelBlu,0);
    tempoSpento=millis();     
    tempoAcceso=tempoSpento+ritardoLamp;        
  }   
}


void natale() {
  if (millis()>tempoSpento+ritardoLamp) {    
    switch (luceNatale) {
      case 1:
        ledcWrite(ledChannelRed,luceRed);     
        tempoAcceso=millis();
        tempoSpento=tempoAcceso+ritardoLamp;
        //luceNatale++;
        break;
      case 2:
        ledcWrite(ledChannelGreen,luceGreen);     
        tempoAcceso=millis();
        tempoSpento=tempoAcceso+ritardoLamp;
        //luceNatale++;
        break;
      case 3:
        ledcWrite(ledChannelBlu,luceBlu);     
        tempoAcceso=millis();
        tempoSpento=tempoAcceso+ritardoLamp;
        //luceNatale=1;
        break;
    }
    
  }
  else if (millis()>tempoAcceso+ritardoLamp) {
    switch (luceNatale) {
      case 1:
        ledcWrite(ledChannelRed,0);
        ledcWrite(ledChannelGreen,0);  
        ledcWrite(ledChannelBlu,0);
        tempoSpento=millis();     
        tempoAcceso=tempoSpento+ritardoLamp;        
        luceNatale++;
        break;
      case 2:
        ledcWrite(ledChannelRed,0);
        ledcWrite(ledChannelGreen,0);  
        ledcWrite(ledChannelBlu,0);
        tempoSpento=millis();     
        tempoAcceso=tempoSpento+ritardoLamp;        
        luceNatale++;
        break;
      case 3:
        ledcWrite(ledChannelRed,0);
        ledcWrite(ledChannelGreen,0);  
        ledcWrite(ledChannelBlu,0);
        tempoSpento=millis();     
        tempoAcceso=tempoSpento+ritardoLamp;        
        luceNatale=1;
        break;
    }
  }   
}

void restartESP32()
{
  esp_sleep_enable_timer_wakeup(uS_TO_S_FACTOR * TIME_TO_SLEEP);
  esp_deep_sleep_start();
}


String twoDigits(int digits)
{
    if(digits < 10) {
        String i = '0'+String(digits);
        return i;
    }
    else {
        return String(digits);
    }
}

void sunSet() {
  static int currentDay = 32;
  int sunrise;
  int sunset;
  double civilsunrise;
  double civilsunset;
  double astrosunrise;
  double astrosunset;
  double nauticalsunrise;
  double nauticalsunset;
  double customsunrise;
  double customsunset;

  if (currentDay != day()) {
      sun.setCurrentDate(year(), month(), day());
      currentDay = day();
  }
  sunrise = static_cast<int>(sun.calcSunrise());
  sunset = static_cast<int>(sun.calcSunset());
  civilsunrise = sun.calcCivilSunrise();
  civilsunset = sun.calcCivilSunset();
  nauticalsunrise = sun.calcNauticalSunrise();
  nauticalsunset = sun.calcNauticalSunset();
  astrosunrise = sun.calcAstronomicalSunrise();
  astrosunset = sun.calcAstronomicalSunset();
  customsunrise = sun.calcCustomSunrise(90.0);
  customsunset = sun.calcCustomSunset(90.0);

/*
  Serial.print("Sunrise at ");
  Serial.print(sunrise/60);
  Serial.print(":");
  Serial.print(twoDigits(sunrise%60));
  Serial.print("am, Sunset at ");
  Serial.print(sunset/60);
  Serial.print(":");
  Serial.print(twoDigits(sunset%60));
  Serial.println("pm");
  */
  
  oraRise=sunrise/60;
  minutoRise=(twoDigits(sunrise%60)).toInt();
  oraSet=sunset/60;
  minutoSet=(twoDigits(sunset%60)).toInt();
  //Serial.print(oraRise);
  //Serial.println(minutoRise);
}
