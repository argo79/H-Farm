/***************************************************************************
  This is a library for the BME280 humidity, temperature & pressure sensor

  Designed specifically to work with the Adafruit BME280 Breakout
  ----> http://www.adafruit.com/products/2650

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface. The device's I2C address is either 0x76 or 0x77.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
  See the LICENSE file for details.
 ***************************************************************************/
#include <WiFi.h>
#include <TFT_eSPI.h> // Graphics and font library for ST7735 driver chip
#include <SPI.h>
#include "time.h"
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "BluetoothSerial.h"
#include <PubSubClient.h>
#include <math.h>

// OTA
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
// OTA

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10


#define temperaturaEXT_topic "sonda/esp32/giardino/Temperatura"                                  // TEMPERATURA AMBIENTE
#define pressioneEXT_topic "sonda/esp32/giardino/Pressione"                                // STATO POMPA
#define umiditaEXT_topic "sonda/esp32/giardino/Umidita"        
#define temperaturaIN_topic "sonda/esp32/cucina/Temperatura"                                  // TEMPERATURA AMBIENTE
#define pressioneIN_topic "sonda/esp32/cucina/Pressione"                                // STATO POMPA
#define umiditaIN_topic "sonda/esp32/cucina/Umidita"                
#define presenzaIN_topic "sonda/esp32/cucina/Presenza"
#define diffPressione_topic "sonda/esp32/cucina/diffP"
#define diffPressione1_topic "sonda/esp32/cucina/diffP1"
#define diffPressione2_topic "sonda/esp32/cucina/diffP2"
#define forecast_topic "sonda/esp32/cucina/forecast"
#define maxLevel_topic "mygateway1-out/10/11/1/0/24"               
#define pump_topic "mygateway1-out/10/9/1/0/2"            // ATTIVA IL SENSORE DI LUCE
#define tempLed_topic "mygateway1-out/10/12/1/0/0"   

#define EEPROM_SIZE 10

#define SEALEVELPRESSURE_HPA (1013.25)

#define TFT_BLACK       0x0000      /*   0,   0,   0 */
#define TFT_NAVY        0x000F      /*   0,   0, 128 */
#define TFT_DARKGREEN   0x03E0      /*   0, 128,   0 */
#define TFT_DARKCYAN    0x03EF      /*   0, 128, 128 */
#define TFT_MAROON      0x7800      /* 128,   0,   0 */
#define TFT_PURPLE      0x780F      /* 128,   0, 128 */
#define TFT_OLIVE       0x7BE0      /* 128, 128,   0 */
#define TFT_LIGHTGREY   0xD69A      /* 211, 211, 211 */
#define TFT_DARKGREY    0x7BEF      /* 128, 128, 128 */
#define TFT_BLUE        0x001F      /*   0,   0, 255 */
#define TFT_GREEN       0x07E0      /*   0, 255,   0 */
#define TFT_CYAN        0x07FF      /*   0, 255, 255 */
#define TFT_RED         0xF800      /* 255,   0,   0 */
#define TFT_MAGENTA     0xF81F      /* 255,   0, 255 */
#define TFT_YELLOW      0xFFE0      /* 255, 255,   0 */
#define TFT_WHITE       0xFFFF      /* 255, 255, 255 */
#define TFT_ORANGE      0xFDA0      /* 255, 180,   0 */
#define TFT_GREENYELLOW 0xB7E0      /* 180, 255,   0 */
#define TFT_PINK        0xFE19      /* 255, 192, 203 */ //Lighter pink, was 0xFC9F      
#define TFT_BROWN       0x9A60      /* 150,  75,   0 */
#define TFT_GOLD        0xFEA0      /* 255, 215,   0 */
#define TFT_SILVER      0xC618      /* 192, 192, 192 */
#define TFT_SKYBLUE     0x867D      /* 135, 206, 235 */
#define TFT_VIOLET      0x915C      /* 180,  46, 226 */

#define ULTRASONIC_TRIG_PIN     32   // pin TRIG 
#define ULTRASONIC_ECHO_PIN     35 // pin ECHO 

//////////////////////////////////
//           WIFI               //
//////////////////////////////////
const char* ssid = "*********";
const char* password = "***********";
const char* mqtt_server = "x.x.x.x";

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 3600;
const int   daylightOffset_sec = 0;
const int potPin = 34;
const int ledReset=2;                         // Led azzurro integrato
const int buttonPin = 2;     // the number of the pushbutton pin

float logMisure[3][120];
float pressioneA[24];
float pressMin=1050.0;
float pressMax=950.0;
float tempMin=45.0;
float tempMax=0.0;
float umidMinIN=99.0;
float umidMaxIN=1.0;
float umidMin=99.0;
float umidMax=1.0;
float tempMaxIN=0.0;
float tempMinIN=45.0;
float diffTemp;
float oldPressione, diffPressione,diffPressione1,diffPressione2;

int osservazione;
int tempDiv,pressDiv,umidDiv;
int lastReconnectAttempt = 0;
int r;
int pixelT;
int pixelP;
int pixelU;
int potValue = 0;
float tempMAX, tempMIN;           ////////////// SOGLIE ALLARMI
int pagina=1;

byte red = 31;
byte green = 0;
byte blue = 0;
byte state = 0;
unsigned int colourG = TFT_SILVER;
float tempIN, pressIN, altIN, umidIN;
float tempOUT,pressOUT,umidOUT;
unsigned int colTempIN,colUmidIN,colTempINb,colUmidINb;
unsigned int colTempOUT,colPressOUT,colTempOUTb,colPressOUTb;
unsigned long tempoPrevisione;
unsigned long ritardoPrevisione=300000;     // check ogni 5 minuti [.......old: un'ora di differenza pressione, ora c'è l'array..] 

int tempO, pressO, umidO;
struct tm timeinfo;
char oldMin[3];
char oldDay[3];
char newDay[3];
char oraTempMin[3];
char minTempMin[3];
char oraTempMax[3];
char minTempMax[3];
char oraPressMin[3];
char minPressMin[3];
char oraPressMax[3];
char minPressMax[3];
char oraUmidMin[3];
char minUmidMin[3];
char oraUmidMax[3];
char minUmidMax[3];
char oraTempMinIN[3];
char minTempMinIN[3];
char oraTempMaxIN[3];
char minTempMaxIN[3];


char valore[10];
char serBT;

boolean tempAlMax, tempAlMin, presenza;

String dataBT;
unsigned long inizioLog;
unsigned long inizioLoop;
//int minutiLog=0.05;                               ///////////////   TIMER LOG    /////////////////
float minutiLog=0.1;
unsigned long timerLog=minutiLog*60000;                      
unsigned long delayTime=5000;                     ////////////////  TIMER  ////////////////////
unsigned long timerUno;
long duration, distance;
boolean buttonState;

// AVVIO DISPOSITIVI
Adafruit_BME280 bme; // I2C
//Adafruit_BME280 bme(BME_CS); // hardware SPI
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

// Wifi
WiFiClient espClient;
PubSubClient client(espClient);

BluetoothSerial SerialBT;
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif


TFT_eSPI tft = TFT_eSPI();  // Invoke library, pins defined in User_Setup.h

  //init and get the time
  
  
void setup() {
  Serial.begin(115200);
  while(!Serial);    // time to get serial running
  Serial.println(F("BME280 test"));  

  EEPROM.begin(EEPROM_SIZE);
  SerialBT.begin("ESP32-SondaInOut");                   //Bluetooth device name
  SerialBT.println("Bluetooth avviato!");
  delay(100);
  setup_wifi();
  delay(100);

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  printLocalTime();    
  //strftime(oldDay,3, "%d", &timeinfo);
  
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  strftime(oldDay,3, "%d", &timeinfo);
  Serial.println(oldDay);
  
  delay(100);
  // Avvio la connessione con il server mqtt (mosquitto)  
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  unsigned status;    
  // default settings
  status = bme.begin(0x76); 
   
  // You can also pass in a Wire library object like &Wire2
  // status = bme.begin(0x76, &Wire2)
  if (!status) {
      Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
      Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
      Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
      Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
      Serial.print("        ID of 0x60 represents a BME 280.\n");
      Serial.print("        ID of 0x61 represents a BME 680.\n");
      while (1) delay(10);
  }

  
  Serial.println("-- Default Test --");
  
  Serial.println();

  
// Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
   ArduinoOTA.setHostname("Adamina 4.1-OTA");

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

  // Avvio il display
  tft.init();
  tft.setRotation(2);
  tft.fillScreen(TFT_BLACK);  
//    targetTime = millis() + 1000;   // 

// Creo la griglia di fondo
  home(); 

  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  pinMode(potPin,INPUT);
  pinMode(buttonPin,INPUT);
  
  osservazione=0;        
  printValues();  
  logValues(tempOUT,tempIN,pressOUT,umidOUT);  
  inizioLog=millis();
  inizioLoop=millis();

  minutiLog=EEPROM.read(0);
  if (!minutiLog) {
    minutiLog=5;
  }
  tempMAX=EEPROM.read(1);
  if (!tempMAX) {
    tempMAX=28;
  }
  tempMIN=EEPROM.read(2);
  if (!tempMIN) {
    tempMIN=10;
  }

  
  timerLog=minutiLog*60000;
  timerUno=30000;
  blinkLed(10);
  tempoPrevisione=millis();
    




}



//////////////////////////////////
///            LOOP            ///
//////////////////////////////////

void loop() { 
// Aggiorno data e ora
  
  ArduinoOTA.handle();
  
//////////////////////   LEGGO PULSANTE   /////////////////////
  buttonState = digitalRead(buttonPin);
  delay(100);
  if (buttonState == HIGH) {     
  // turn LED on:     
    pagina++;
    if (pagina>2) {
      pagina=1;
    }
    tft.fillScreen(TFT_BLACK);  
    tft.setTextColor(TFT_RED);
    tft.setCursor (25, 5);
    tft.print("Cambio pagina");
    tft.fillRoundRect(92,124,32,33,1,TFT_GOLD);
    tft.setTextColor(TFT_BLUE);
    tft.drawString("PAGE:",93,129,1);
    tft.drawNumber(pagina,105,142,1);
    delay(1000);    
    home();     
    trackValues();                              ///// CREO IL GRAFICO
    //minime2tft(pressMax,pressMin,tempMax,tempMin,tempMaxIN,tempMinIN,umidMax,umidMin,64);
  } 
  
  if (!client.connected()) {
    reconnect();
  }
  
  if (tempAlMax==1) {
    tft.fillRoundRect(92,124,32,33,1,TFT_RED);
    tft.setTextColor(TFT_CYAN);
    tft.drawString("ALARM",93,136,1);
  }  
  else if (tempAlMin==1) {
    tft.fillRoundRect(92,124,32,33,1,TFT_CYAN);
    tft.setTextColor(TFT_BLUE);
    tft.drawString("ALARM",93,136,1);
  }
  else tft.fillRoundRect(92,124,32,33,1,TFT_BLACK);
    
  riceviBT();

  potValue = analogRead(potPin);
  delay(5);
  //Serial.println(potValue);

  misuraDistanza();
  
  if ((potValue<5 && distance>150) && presenza==1) {            // NON C'È PIÙ NESSUNO
    presenza=0;  
    snprintf (valore, 2, "%d", presenza);
    client.publish(presenzaIN_topic, valore);   
  }
  else if ((potValue<5 && distance>150) && presenza==0) {        // NON C'È GIÀ NESSUNO
    presenza=0;      
  }  
  else if ((potValue>5 && distance<150) && presenza==0) {       // C'È QUALCUNO PRIMA VOLTA
    presenza=1;
    snprintf (valore, 2, "%d", presenza);
    client.publish(presenzaIN_topic, valore);           
  }
  else if ((potValue>5 || distance<150) && presenza==1) {       // C'È GIÀ QUALCUNO
    presenza=1;    
  }

  
  
  if (millis()>inizioLoop+delayTime) {      ////////////   5 SECONDI DI REFRESH
    //configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    printLocalTime();                       
    
    // leggo i valori interni                    // LEGGO I SENSORI LOCALI    
    printValues();  
    delay(10);
    
    allarmi(tempOUT,pressOUT,umidOUT);
    
    // Stampo a monitor i valori --> SWITCH TRA LE VARIE PAGINE
    //home();
    switch(pagina) {
      case 1:
        interno2tft(tempIN, umidOUT, 64);
        esterno2tft(tempOUT, pressOUT, 100);
        break;
      case 2:
        minime2tft(pressMax,pressMin,tempMax,tempMin,tempMaxIN,tempMinIN,umidMax,umidMin,64);
        break;
      default:
        interno2tft(tempIN, umidOUT, 64);
        esterno2tft(tempOUT, pressOUT, 100);
      break;
    }
    previsione();
    inizioLoop=millis();
  }
  
  //allarmi(tempOUT,pressOUT,umidIN);

  // Timer per loggare i valori
  if (millis()>inizioLog+timerUno) {    ////// TIMER LOG VALORI. Da 1 minuto a 120 minuti
    Serial.println("Registro i valori");
    SerialBT.print("Registro i valori ad intervalli di minuti: ");
    SerialBT.println(minutiLog);
    logValues(tempOUT,tempIN,pressOUT,umidOUT);         ///// AGGIORNO IL VETTORE
    delay(10);
    
    trackValues();                              ///// CREO IL GRAFICO
    inizioLog=millis();
    timerUno=timerLog;
  }  
  //logValues(tempOUT,pressOUT,umidIN);
  
  
  delay(1);
  
  // leggo i valori esterni
   
  
  client.loop();

}



//////////////////////////////////
///        PRINT VALUES        ///
//////////////////////////////////

void printValues() {
  tempIN=bme.readTemperature();
  //Serial.print("Temperature = ");
  //Serial.print(tempIN);
  //Serial.println(" *C");

  pressIN=(bme.readPressure()/100.0F);
  //Serial.print("Pressure = ");
  //Serial.print(pressIN);
  //Serial.println(" hPa");

  altIN=bme.readAltitude(SEALEVELPRESSURE_HPA);
  //Serial.print("Approx. Altitude = ");
  //Serial.print(altIN);
  //Serial.println(" m");

  umidIN=bme.readHumidity();
  //Serial.print("Humidity = ");
  //Serial.print(umidIN);
  //Serial.println(" %");
  
  //Serial.println();
  
  
  
}



void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  //WiFi.setMode([WIFI_STA]);
  //randomSeed(micros());  
}


/////////////////////////////////  LEGGO VALORI MQTT DAL SERVER  //////////////////////////
void callback(char* topic, byte* payload, unsigned int length) {  
  
  String response;  
  for (int i = 0; i < length; i++) {
    response+=(char)payload[i];
  }
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  Serial.println(response);
  Serial.println();
  
  Serial.println(response);
  String topicS=topic;  
  Serial.println(topicS);
  
  if (topicS=="sonda/esp32/giardino/Umidita") {
    //Serial.println(response);
    //SerialBT.println(response);    
      umidOUT=response.toFloat();
    //maxL=int(payload[0]);
    //EEPROM.write(9,maxL);
    //EEPROM.commit();
    SerialBT.println(umidOUT); 
    //SerialBT.println("");   
    //SerialBT.print("Livello massimo dei led: ");
    //SerialBT.println(pressOUT);        
  }

  else if (topicS=="sonda/esp32/giardino/Temperatura") {
    //Serial.println(response);
    //SerialBT.println(response);
      tempOUT=response.toFloat();
    //maxL=int(payload[0]);
    //EEPROM.write(9,maxL);
    //EEPROM.commit();
    SerialBT.println(tempOUT); 
    //SerialBT.println("");   
    //SerialBT.print("Livello massimo dei led: ");
    //SerialBT.println(tempOUT);           
  }

  if (topicS=="sonda/esp32/giardino/Pressione") {
    //Serial.println(response);
    //SerialBT.println(response);    
      pressOUT=response.toFloat();
    //maxL=int(payload[0]);
    //EEPROM.write(9,maxL);
    //EEPROM.commit();
    SerialBT.println(pressOUT); 
    //SerialBT.println("");   
    //SerialBT.print("Livello massimo dei led: ");
    //SerialBT.println(pressOUT);   
  } 
    
  // Switch on the LED if an 1 was received as first character 
  
  delay(1); 
}



boolean reconnect() {
  while (!client.connected()) {
    SerialBT.println("Attempting MQTT connection...");
    // Create a random client ID
    //String clientId = "ESP8266Client-";
    //clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect("sondaInOut")) {
      SerialBT.println("connected");      
      // Once connected, publish an announcement...
      //client.subscribe(ledNW_topic);
      //client.subscribe(ledFS_topic);
      //client.subscribe(ledVegW_topic);
      //client.subscribe(ledVegB_topic);
      //client.subscribe(ledFioWW_topic);
      //client.subscribe(ledFioR_topic);
      client.subscribe(temperaturaEXT_topic,1);
      client.subscribe(pressioneEXT_topic,1);
      client.subscribe(umiditaEXT_topic,1);
      Serial.println("Ok..connesso...");
      }
    else {
      SerialBT.print("failed, rc=");
      SerialBT.print(client.state());
      SerialBT.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
  return client.connected();
}



void printLocalTime()
{
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %d %B %Y %H:%M:%S");
  strftime(newDay,3, "%d", &timeinfo);
}





//////////////////////////////////
///         LOG VALUES         ///
//////////////////////////////////

////////////////////////////  CREO IL VETTORE DI VALORI  ///////////////////////////////
void logValues(float tempOUT,float tempIN, float pressOUT,float umidOUT) {
  
  //Serial.println(newDay);
  //Serial.println(oldDay);
  if (strcmp(newDay,oldDay)!=0) {           ///////////// CONTROLLO CAMBIO GIORNO ////////////////
    
    tempMin=50.0;
    tempMax=0.0;
    tempMinIN=50.0;
    tempMaxIN=0.0;
    pressMin=1050.0;
    pressMax=950.0;
    umidMin=99.9;
    umidMax=1.0;    
    tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
      Serial.println("Failed to obtain time");      
      return;
    }
    strftime(oldDay,3, "%d", &timeinfo);    
  }
  
  
  if (tempMin==0.0) {                     ////////////////  TEMPERATURA ESTERNA  ///////////////////                          
    tempMin=50.0;
  }
  if (tempOUT<tempMin) {
    tempMin=tempOUT;    
    getLocalTime(&timeinfo);
    //Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");    
    strftime(oraTempMin,3, "%H", &timeinfo);    
    strftime(minTempMin,3, "%M", &timeinfo);    
  }
  if (tempOUT>tempMax) {
    tempMax=tempOUT;
    getLocalTime(&timeinfo);
    //Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");    
    strftime(oraTempMax,3, "%H", &timeinfo);    
    strftime(minTempMax,3, "%M", &timeinfo);    
  }

  if (tempMinIN==0.0) {                     ////////////////  TEMPERATURA INTERNA  ///////////////////                          
      tempMinIN=50.0;
  }
  if (tempIN<tempMinIN) {
    tempMinIN=tempIN;    
    getLocalTime(&timeinfo);
    //Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");    
    strftime(oraTempMinIN,3, "%H", &timeinfo);    
    strftime(minTempMinIN,3, "%M", &timeinfo);    
  }
  if (tempIN>tempMaxIN) {
    tempMaxIN=tempIN;
    getLocalTime(&timeinfo);
    //Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");    
    strftime(oraTempMaxIN,3, "%H", &timeinfo);    
    strftime(minTempMaxIN,3, "%M", &timeinfo);    
  }

  

  if (pressMin==0.0) {                   /////////////////   PRESSIONE   ////////////////////               
    pressMin=1050.0;
  }
  if (pressOUT<pressMin) {
    pressMin=pressOUT;    
    getLocalTime(&timeinfo);
    //Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");    
    strftime(oraPressMin,3, "%H", &timeinfo);    
    strftime(minPressMin,3, "%M", &timeinfo);    
  }
  if (pressOUT>pressMax) {
    pressMax=pressOUT;
    getLocalTime(&timeinfo);
    //Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");    
    strftime(oraPressMax,3, "%H", &timeinfo);    
    strftime(minPressMax,3, "%M", &timeinfo);    
  }

if (umidMin==0.0) {                   /////////////////   UMIDITÀ   ////////////////////               
    umidMin=99.9;
  }
  if (umidOUT<umidMin) {
    umidMin=umidOUT;    
    getLocalTime(&timeinfo);
    //Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");    
    strftime(oraUmidMin,3, "%H", &timeinfo);    
    strftime(minUmidMin,3, "%M", &timeinfo);    
  }
  if (umidOUT>umidMax) {
    umidMax=umidOUT;
    getLocalTime(&timeinfo);
    //Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");    
    strftime(oraUmidMax,3, "%H", &timeinfo);    
    strftime(minUmidMax,3, "%M", &timeinfo);    
  }  

  
  if (tempOUT!=0.0 && pressOUT>990.0 && pressOUT<1050.0 && umidOUT!=0.0) {
    logMisure[0][osservazione]=tempOUT;
    logMisure[1][osservazione]=pressOUT;
    logMisure[2][osservazione]=umidOUT;
    Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");    
    Serial.println(logMisure[0][osservazione]);
    Serial.println(logMisure[1][osservazione]);
    Serial.println(logMisure[2][osservazione]);    
    Serial.println(osservazione+1);    
    
    osservazione++;
    inviaBT();
  }  
  
  if (osservazione==120) {
    for (int z=0;z<120;z++) {
      logMisure[0][z]=logMisure[0][z+1];
      logMisure[1][z]=logMisure[1][z+1];
      logMisure[2][z]=logMisure[2][z+1];
    }
  osservazione=119;
  }
}


///////////////////////  DISEGNO I GRAFICI  //////////////////////////
void trackValues() {
  
  tft.fillRoundRect(2,16,122,35,1,TFT_BLACK);


  SerialBT.println("********************");
  // temperatura  //////////////////////////////////////////
  float tempMinGx=50.0;
  for (int i=0;i<osservazione;i++) {
    if (logMisure[0][i]<tempMinGx) {
      tempMinGx=logMisure[0][i];
    }
  } 
  tempMinGx=round(tempMinGx*10); 
  int tempMinG=(int)tempMinGx;
  Serial.print("Minima temperatura grafico: ");  
  Serial.println(tempMinG);  
  SerialBT.print("Minima temperatura grafico: ");  
  SerialBT.println(tempMinG);  
    
  float tempMaxGx=-10.0;
  for (int i=0;i<osservazione;i++) {
    if (logMisure[0][i]>tempMaxGx) {
      tempMaxGx=logMisure[0][i];
    }
  }  
  tempMaxGx=round(tempMaxGx*10);
  int tempMaxG=(int)tempMaxGx; 
  Serial.print("Massima temperatura grafico: ");  
  Serial.println(tempMaxG);     
  SerialBT.print("Massima temperatura grafico: ");  
  SerialBT.println(tempMaxG);     
  
  for (int z=0;z<120;z++) {
    int logTemp=round((logMisure[0][z]*10)+100.0);
    Serial.println(logTemp);
    pixelT=map(logTemp,tempMinG+100,tempMaxG+100,12,0);
      
    if (logTemp>=140) {
      //pixelT=map(logTemp,tempMinG+20,tempMaxG+20,12,0);  
      tft.drawPixel(2+z,17+pixelT,TFT_ORANGE);
    }        
    else if (logTemp>100 && logTemp<140) {
      tft.drawPixel(2+z,17+pixelT,TFT_SILVER);
    }
    else if (logTemp>0 && logTemp<100) {
      tft.drawPixel(2+z,17+pixelT,TFT_CYAN);
    }            
  }

  
  // pressione  ////////////////////////////////////////////////
  float pressMinGx=1050.0;
  for (int i=0;i<osservazione;i++) {
    if (logMisure[1][i]<pressMinGx) {
      pressMinGx=logMisure[1][i];
    }
  } 
  pressMinGx=round(pressMinGx*10); 
  int pressMinG=(int)pressMinGx;
  Serial.print("Minima pressione grafico: ");  
  Serial.println(pressMinG);
  SerialBT.print("Minima pressione grafico: ");  
  SerialBT.println(pressMinG);
    
  float pressMaxGx=950.0;
  for (int i=0;i<osservazione;i++) {
    if (logMisure[1][i]>pressMaxGx) {
      pressMaxGx=logMisure[1][i];
    }
  }
  pressMaxGx=round(pressMaxGx*10);
  int pressMaxG=(int)pressMaxGx;
  Serial.print("Massima pressione grafico: ");  
  Serial.println(pressMaxG);   
  SerialBT.print("Massima pressione grafico: ");  
  SerialBT.println(pressMaxG);  
    
  for (int z=0;z<120;z++) {
    int logPress=round(logMisure[1][z]*10);
    if (logPress>0) {
      pixelP=map(logPress,pressMinG,pressMaxG,10,0); 
      tft.drawPixel(2+z,31+pixelP,TFT_GREEN); 
    }    
    /*
    else {
      tft.drawPixel(2+z,29+pixelP,TFT_SILVER);
    }
    */    
  }
  
  // umidità  ///////////////////////////////////////////////////
  float umidMinGx=99.9;
  for (int i=0;i<osservazione;i++) {
    if (logMisure[2][i]<umidMinGx) {
      umidMinGx=logMisure[2][i];
    }
  } 
  umidMinGx=round(umidMinGx*10); 
  int umidMinG=(int)umidMinGx;
  Serial.print("Minima Umidità grafico: ");  
  Serial.println(umidMinG);
  SerialBT.print("Minima Umidità grafico: ");  
  SerialBT.println(umidMinG);
    
  float umidMaxGx=0.0;
  for (int i=0;i<osservazione;i++) {
    if (logMisure[2][i]>umidMaxGx) {
      umidMaxGx=logMisure[2][i];
    }
  }
  umidMaxGx=round(umidMaxGx*10);
  int umidMaxG=(int)umidMaxGx;
  Serial.print("Massima Umidità grafico: ");  
  Serial.println(umidMaxG);   
  SerialBT.print("Massima Umidità grafico: ");  
  SerialBT.println(umidMaxG);     
   
  for (int z=0;z<120;z++) {
    int logUmid=round(logMisure[2][z]*10);
    if (logUmid>0) {
      pixelU=map(logUmid,umidMinG,umidMaxG,8,0); 
      tft.drawPixel(2+z,42+pixelU,TFT_CYAN); 
    }
    
    /*
    else {
      tft.drawPixel(2+z,39+pixelU,TFT_SILVER);
    }  
    */  
  }
  SerialBT.println("********************");
}

void misuraDistanza() {
  int distanza=0;
  for (int i=0;i<5;i++) {
    digitalWrite(ULTRASONIC_TRIG_PIN, LOW);  
    delayMicroseconds(2);   
    digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
    delayMicroseconds(10);     
    digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
    duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH);
    distanza=distanza+((duration/2)/29.1);    
    delay(5);
    }
  distance=distanza/5;  
  //Serial.print("********** Ultrasonic Distance: ");
  //Serial.print(distance);
  //Serial.println(" cm"); 
}



void home() {                       //////////////////////////   HOME   /////////////////////////////
  
  tft.fillScreen(TFT_BLACK);  
  tft.setTextColor(TFT_YELLOW);
  tft.setCursor (6, 5);
  tft.print("Adamina 4.1");
  tft.setTextColor(TFT_ORANGE);
  tft.setCursor (80, 5);
  tft.print("by Arg0");
  
  tft.drawFastVLine(0,0,tft.height(), colourG);
  tft.drawFastVLine(1,0,tft.height(), colourG);
  tft.drawFastVLine(36,51,tft.height(), colourG);
  tft.drawFastVLine(37,51,tft.height(), colourG);
  tft.drawFastVLine(90,51,tft.height(), colourG);
  tft.drawFastVLine(91,51,tft.height(), colourG);
  tft.drawFastVLine(124,0,tft.height(), colourG);
  tft.drawFastVLine(125,0,tft.height(), colourG);
  /*
  for (r=1;r<4;r++) {
    tft.drawFastVLine((41)*r,51,tft.height(), colourG);
  }  
  //tft.drawFastVLine(0,16,tft.height(), colourG);
  */
  
  
  tft.drawFastHLine(0,0,tft.width(), colourG);
  tft.drawFastHLine(0,1,tft.width(), colourG);
  
  for (r=0;r<4;r++) {
    tft.drawFastHLine(0,15+(36)*r,tft.width(), colourG);
  }
  tft.drawFastHLine(0,158,tft.width(), colourG);
  tft.drawFastHLine(0,157, tft.width(), colourG);  
}



void riceviBT() {
  if (SerialBT.available()) {
    yield(); 
    while (SerialBT.available()) {
      serBT=SerialBT.read();
      dataBT+=serBT;       
    }          
    //Serial.println(dataBT); 
    dataBT.trim();
    
    if (dataBT.length()>0) {       
      
      if (dataBT=="reset") {
        SerialBT.println("Sei sicuro che vuoi resettare al primo giorno? (yes/si)");
        delay(100);
        for (int i=0;i<120;i++) {
          logMisure[0][i]=0.0;
          logMisure[1][i]=0.0;
          logMisure[2][i]=0.0;
          //memset(logMisure, 0, sizeof(logMisure));
        }
        osservazione=0;               
      }

      if (dataBT=="pag") {        
        pagina++;        
        if (pagina>2) {
          pagina=1;          
        }
        SerialBT.print("Cambio pagina: ");    
        SerialBT.println(pagina);    
      }         
      
      else if (dataBT.startsWith("setLog")) {        
        String minLog=dataBT.substring(6);
        minutiLog=minLog.toInt();     
        SerialBT.print("Minuti intervallo grafico: ");    
        SerialBT.println(minutiLog);
        delay(10);
        Serial.print("Minuti intervallo grafico: ");    
        Serial.println(minutiLog);    
        EEPROM.write(0,minutiLog); 
        EEPROM.commit();
        delay(10);
        timerLog=minutiLog*60000;
        timerUno=timerLog;
      }

      else if (dataBT.startsWith("setTmax")) {        
        String maxT=dataBT.substring(7);
        tempMAX=maxT.toFloat(); 
        if (tempMAX<tempMIN) {
          tempMIN=tempMAX-1;              
        }
        SerialBT.print("Soglia temperatura massima: ");    
        SerialBT.println(tempMAX);
        delay(10);
        Serial.print("Soglia temperatura massima:  ");    
        Serial.println(tempMAX);    
        EEPROM.write(1,tempMAX); 
        EEPROM.commit();
        delay(10);
      }
      else if (dataBT.startsWith("setTmin")) {        
        String minT=dataBT.substring(7);
        tempMIN=minT.toFloat(); 
        if (tempMIN>tempMAX) {
          tempMAX=tempMIN+1;              
        }    
        SerialBT.println("Soglia temperatura minima: ");    
        SerialBT.println(tempMIN);
        delay(10);
        Serial.print("Soglia temperatura minima:  ");    
        Serial.println(tempMIN);    
        EEPROM.write(2,tempMIN); 
        EEPROM.commit();
        delay(10);
      }
      
      
      else if (dataBT.startsWith("help")) {                       // HELP -> Comando
        
        SerialBT.println("Comandi:");    
        SerialBT.println("reset         -> Resetto il sistema.");    
        SerialBT.println("setLog(1-60)  -> Minuti log.");    
        SerialBT.println("setTmax(0-50) -> Soglia T max.");    
        SerialBT.println("setTmin(0-50) -> Soglia T min.");
        SerialBT.println("pag           -> Cambio pagina.");  
        SerialBT.println("info          -> Mostra ultimi valori.");  
        SerialBT.println("help          -> Questo comando.");
        delay(100);
        Serial.println("Comandi:");    
        Serial.println("reset         -> Resetto il sistema.");  
        Serial.println("minVeg(0-24)  -> Ora alba vegetativa.");  
        Serial.println("maxVeg(0-24)  -> Ora tramonto vegetativa.");  
        Serial.println("minFio(0-24)  -> Ora alba fioritura."); 
        Serial.println("maxFio(0-24)  -> Ora tramonto fioritura.");   
        Serial.println("gioVeg(0-255) -> Giorni vegetativa.");  
        Serial.println("gioFio(0-255) -> Giorni fioritura.");  
        Serial.println("help -> Questo comando.");          
        blinkLed(1);      
      }
      
      else if (dataBT.startsWith("info")) {                       // HELP -> Comando
        
        inviaBT();
        delay(5);   
      }
      
      else {
        SerialBT.println("Valore sbagliato. Nessun comando riconosciuto.");
          for (int i=0;i<5;i++) {
          digitalWrite(ledReset,1);
          delay(150);
          digitalWrite(ledReset,0);
          delay(150);
        }
      }
    }
    dataBT="";
    delay(20);
  }
}


void blinkLed(int numB) {
  for (int i=0;i<numB;i++) {
          digitalWrite(ledReset,HIGH);
          delay(150);
          digitalWrite(ledReset,LOW);
          delay(150);
  }   
}



void allarmi(float tempOUT,float pressOUT,float umidIN) {

  // ALLARME TEMPERATURA
  if (tempOUT>tempMAX) {
    SerialBT.println("Temperatura MASSIMA superata!!!"); 
    tempAlMax=1;
    blinkLed(2);
  }
  else tempAlMax=0;
  
  if (tempOUT<tempMIN) {
    SerialBT.println("Temperatura MINIMA superata!!!"); 
    tempAlMin=1;
    blinkLed(4);
  }
  else tempAlMin=0;
  
}



void inviaBT() {
  
  getLocalTime(&timeinfo);
  //printLocalTime();
  SerialBT.println("###############################"); 
  SerialBT.println("#     Adamina 4.1 by Argo     #"); 
  SerialBT.println("###############################");   
  SerialBT.println(&timeinfo, "%A, %d %B %Y %H:%M:%S"); 
  SerialBT.print("Registro i valori ad intervalli di minuti: ");
  SerialBT.println(minutiLog);  
      
  SerialBT.print("Temperatura: ");
  SerialBT.println(logMisure[0][osservazione-1]);
  SerialBT.print("Temperatura: ");
  SerialBT.print(tempMax);
  SerialBT.print(", massima di oggi, alle ore ");
  SerialBT.print(oraTempMax);
  SerialBT.print(":");
  SerialBT.print(minTempMax);
  SerialBT.println(".");  
  SerialBT.print("Temperatura: ");
  SerialBT.print(tempMin);    
  SerialBT.print(" minima di oggi, alle ore ");
  SerialBT.print(oraTempMin);
  SerialBT.print(":");
  SerialBT.print(minTempMin);
  SerialBT.println(".");
  SerialBT.print("Soglia massima: ");
  SerialBT.println(tempMAX);
  SerialBT.print("Soglia minima: ");
  SerialBT.println(tempMIN);
  
  
  SerialBT.print("Pressione: ");
  SerialBT.println(logMisure[1][osservazione-1]);
  SerialBT.print("Pressione: ");
  SerialBT.print(pressMax);
  SerialBT.print(", massima di oggi, alle ore ");
  SerialBT.print(oraPressMax);
  SerialBT.print(":");
  SerialBT.print(minPressMax);
  SerialBT.println(".");
  
  SerialBT.print("Pressione: ");
  SerialBT.print(pressMin);    
  SerialBT.print(" minima di oggi, alle ore ");
  SerialBT.print(oraPressMin);
  SerialBT.print(":");
  SerialBT.print(minPressMin);
  SerialBT.println(".");
  

  SerialBT.print("Umidità: ");
  SerialBT.println(logMisure[2][osservazione-1]);
  SerialBT.print("Umidità: ");
  SerialBT.print(umidMax);
  SerialBT.print(", massima di oggi, alle ore ");
  SerialBT.print(oraUmidMax);
  SerialBT.print(":");
  SerialBT.print(minUmidMax);
  SerialBT.println(".");
  
  SerialBT.print("Umidità: ");
  SerialBT.print(umidMin);   
  SerialBT.print(", minima di oggi, alle ore ");
  SerialBT.print(oraUmidMin);
  SerialBT.print(":");
  SerialBT.print(minUmidMin);
  SerialBT.println(": ");
  
  
  SerialBT.print("Osservazioni valide: ");
  SerialBT.println(osservazione);  
}




//////////////////////  MENU INTERNO 1  ////////////////////////

void interno2tft(float tempIN, float umidIN, unsigned char textPos)
{
  if (tempIN<12)
  {        
    colTempIN=TFT_WHITE;
    colTempINb=TFT_BLUE;
  }
  else if (tempIN>=12 && tempIN<18)
  {
    colTempIN=TFT_WHITE;
    colTempINb=TFT_DARKCYAN;
  }
  else if (tempIN>=18 && tempIN<22)
  {
    colTempIN=TFT_CYAN;
    colTempINb=TFT_GOLD;
  }
  else if (tempIN>=22 && tempIN<26)
  {
    colTempIN=TFT_DARKCYAN;
    colTempINb=TFT_GOLD;
  }
  else if (tempIN>=26 && tempIN<30)
  {
    colTempIN=TFT_CYAN;
    colTempINb=TFT_ORANGE;
  }  
  else if (tempIN>=30)
  {
    colTempIN=TFT_WHITE;
    colTempINb=TFT_RED;
  }

  if (umidOUT<30)
  {        
    colUmidIN = TFT_WHITE;
    colUmidINb = TFT_BLUE;
  }
  else if (umidOUT>=30 && umidOUT<55)
  {
    colUmidIN = TFT_WHITE;
    colUmidINb = TFT_GOLD;
  }  
  else if (umidOUT>=55 && umidOUT<99)
  {
    colUmidIN = TFT_CYAN;
    colUmidINb = TFT_ORANGE;
  }
  
  //home(); 
  //tft.setTextSize(1);
  tft.fillRoundRect(2,textPos-12,34,35,1,colTempINb);
  tft.setTextColor(colTempIN);
  tft.drawFloat(tempIN, 2, 5, textPos-5, 1);
  tft.drawChar(12,textPos+7, 247,colTempIN,colTempINb,1); //degree symbol
  tft.drawString("C",18,textPos+7,1);  
  //tft.print(tempIN);
  //tft.setCursor(6, textPos);

  tft.fillRoundRect(38,textPos-12,52,35,1,colUmidINb);
  tft.setTextColor(colUmidIN);
  tft.drawFloat(umidOUT, 2, 48, textPos-5, 1);  
  tft.drawString("%",60,textPos+7,1);

  tft.fillRoundRect(92,textPos-12,32,35,1,TFT_BROWN);
  tft.setTextColor(colUmidIN);
  tft.drawString("IN",102,textPos,2);
  //tft.setCursor(48, textPos);
  //tft.print(umidIN);


  tft.fillRoundRect(2,textPos+60,34,33,1,TFT_MAROON);
  tft.setTextColor(TFT_WHITE);
  tft.drawNumber(potValue,8, textPos+65, 1);  
  tft.drawString("lux",10,textPos+78,1);

  tft.fillRoundRect(38,textPos+60,52,33,1,TFT_BROWN);
  tft.setTextColor(TFT_WHITE);
  tft.drawFloat(distance, 2, 48, textPos+65, 1);  
  tft.drawString("cm",58,textPos+78,1);


  

  
  /*
  tft.setCursor(1, text_position + 20);
  
  fix_number_position(temperature);
  tft.setTextColor(text_color, COLOR2);
  tft.print(temperature, 1);
  tft.setCursor(108, text_position + 20);
  tft.print("C");
  tft.drawChar(90, text_position + 20, 247, text_color, COLOR2, 2); //degree symbol
*/
}



//////////////////////  MENU ESTERNO 1  ////////////////////////

void esterno2tft(float tempOUT, float pressOUT, unsigned char textPos)
{
  if (tempOUT<7)
  {        
    colTempOUT=TFT_WHITE;
    colTempOUTb=TFT_BLUE;
  }
  else if (tempOUT>=7 && tempOUT<12)
  {
    colTempOUT=TFT_WHITE;
    colTempOUTb=TFT_DARKCYAN;
  }
  else if (tempOUT>=12 && tempOUT<16)
  {
    colTempOUT=TFT_WHITE;
    colTempOUTb=TFT_MAGENTA;
  } 
  else if (tempOUT>=16 && tempOUT<21)
  {
    colTempOUT=TFT_DARKCYAN;
    colTempOUTb=TFT_GOLD;
  } 
  else if (tempOUT>=21 && tempOUT<26)
  {
    colTempOUT=TFT_WHITE;
    colTempOUTb=TFT_ORANGE;
  }
  else if (tempOUT>=26 && tempOUT<30)
  {
    colTempOUT=TFT_WHITE;
    colTempOUTb=TFT_ORANGE;
  }
  else if (tempOUT>=30)
  {
    colTempOUT=TFT_WHITE;
    colTempOUTb=TFT_RED;
  }  

  if (pressOUT<1013)
  {        
    colPressOUT=TFT_WHITE;
    colPressOUTb=TFT_BLUE;
  }
  else
  {
    colPressOUT=TFT_WHITE;
    colPressOUTb=TFT_RED;
  }  
  
  //home(); 
  //tft.setTextSize(1);
  tft.fillRoundRect(2,textPos-12,34,35,1,colTempOUTb);
  tft.setTextColor(colTempOUT);
  tft.drawFloat(tempOUT, 2, 5, textPos-5, 1);
  tft.drawChar(12,textPos+7, 247,colTempOUT,colTempOUTb,1); //degree symbol
  tft.drawString("C",18,textPos+7,1);
  //tft.print(tempIN);
  //tft.setCursor(6, textPos);

  tft.fillRoundRect(38,textPos-12,52,35,1,colPressOUTb);
  tft.setTextColor(colPressOUT);
  tft.drawFloat(pressOUT, 2, 43, textPos-5, 1);  
  tft.drawString("mBar",52,textPos+7,1);
  //tft.setCursor(48, textPos);
  //tft.print(umidIN);

  tft.fillRoundRect(92,textPos-12,32,35,1,TFT_GREENYELLOW);
  tft.setTextColor(TFT_DARKGREEN);
  tft.drawString("OUT",97,textPos,2);
  
  /*
  tft.setCursor(1, text_position + 20);
  
  fix_number_position(temperature);
  tft.setTextColor(text_color, COLOR2);
  tft.print(temperature, 1);
  tft.setCursor(108, text_position + 20);
  tft.print("C");
  tft.drawChar(90, text_position + 20, 247, text_color, COLOR2, 2); //degree symbol
*/
}



//////////////////////  MENU INTERNO/ESTERNO 2  ////////////////////////

void minime2tft(float pressMax, float pressMin, float tempMax, float tempMin,float tempMaxIN, float tempMinIN, float umidMax, float UmidMin, unsigned char textPos)
{  
  //home(); 
  //tft.setTextSize(1);
  tft.fillRoundRect(2,textPos-12,34,35,1,TFT_GOLD);
  tft.setTextColor(TFT_RED);
  tft.drawFloat(tempMaxIN, 2, 5, 59, 1);
  tft.setTextColor(TFT_BLUE);
  tft.drawFloat(tempMinIN, 2, 5, 71, 1);
  //tft.print(tempIN);
  //tft.setCursor(6, textPos);

  tft.fillRoundRect(38,textPos-12,52,35,1,TFT_CYAN);
  tft.setTextColor(TFT_BLUE);
  tft.drawFloat(umidMax, 2, 48, 59, 1);
  tft.setTextColor(TFT_ORANGE);  
  tft.drawFloat(umidMin, 2, 48, 71, 1);  

  tft.fillRoundRect(92,textPos-12,32,35,1,TFT_BROWN);
  tft.setTextColor(TFT_WHITE);
  tft.drawString("max",100,59,1);  
  tft.setTextColor(TFT_WHITE);
  tft.drawString("min",100,71,1);
  //tft.setCursor(48, textPos);
  //tft.print(umidIN);



////////// OUT /////////////

  tft.fillRoundRect(2,textPos+24,34,35,1,TFT_GOLD);
  tft.setTextColor(TFT_RED);
  tft.drawFloat(tempMax, 2, 5, 95, 1);
  tft.setTextColor(TFT_BLUE);
  tft.drawFloat(tempMin, 2, 5, 107, 1);

  tft.fillRoundRect(38,textPos+24,52,35,1,TFT_GREEN);
  tft.setTextColor(TFT_BLUE);
  tft.drawFloat(pressMax, 2, 43, 95, 1);
  tft.setTextColor(TFT_ORANGE);  
  tft.drawFloat(pressMin, 2, 43, 107, 1);  

  tft.fillRoundRect(92,textPos+24,32,35,1,TFT_BROWN);
  tft.setTextColor(TFT_WHITE);
  tft.drawString("max",100,95,1);  
  tft.setTextColor(TFT_WHITE);
  tft.drawString("min",100,107,1);
  //tft.setCursor(48, textPos);
  //tft.print(umidIN);

  tft.fillRoundRect(2,textPos+60,34,33,1,TFT_MAROON);
  tft.setTextColor(TFT_WHITE);
  tft.drawNumber(potValue,8, textPos+65, 1);  
  tft.drawString("lux",10,textPos+78,1);

  tft.fillRoundRect(38,textPos+60,52,33,1,TFT_BROWN);
  tft.setTextColor(TFT_WHITE);
  tft.drawFloat(distance, 2, 48, textPos+65, 1);  
  tft.drawString("cm",58,textPos+78,1);

  
  
  /*
  tft.setCursor(1, text_position + 20);
  
  fix_number_position(temperature);
  tft.setTextColor(text_color, COLOR2);
  tft.print(temperature, 1);
  tft.setCursor(108, text_position + 20);
  tft.print("C");
  tft.drawChar(90, text_position + 20, 247, text_color, COLOR2, 2); //degree symbol
*/
}


void previsione() {
  
  if (millis()>tempoPrevisione+ritardoPrevisione) {       // CALCOLO PREVISIONE
    
    if (pressOUT<1050 && pressOUT>800) {      
           
      for (int i=23;i>0;i--) {
        pressioneA[i]=pressioneA[i-1];
      }
      
      pressioneA[0]=pressOUT;

      diffPressione=pressioneA[0]-pressioneA[5];
      diffPressione1=pressioneA[0]-pressioneA[11];
      diffPressione2=pressioneA[0]-pressioneA[23];       
             
      tempoPrevisione=millis();     
    }            
  }

  if (diffPressione<-5.0 || diffPressione>5.0) {            // PREVISIONE MEZZ'ORA
    diffPressione=0;
  }
  else if (diffPressione<-1.0 || diffPressione>1.0) {
    // tempo?
    client.publish(forecast_topic, "Vento in arrivo!");
    delay(1);
  }
  else if (diffPressione>0.5 && umidOUT<95.0) {
    client.publish(forecast_topic, "Forse migliora!");
    delay(1);        
  }
  else if (diffPressione<0.5 && diffPressione>-0.5) {
    client.publish(forecast_topic, "Tempo stabile!");
    delay(1);        
  }

  
  if (diffPressione1<-5 || diffPressione1>5) {               // PREVISIONE UN'ORA
    diffPressione1=0;
  }
  else if (diffPressione1<-1.5 || diffPressione1>1.5) {
    // tempo?
    client.publish(forecast_topic, "Vento forte in arrivo!");
    delay(1);
  }
  else if (diffPressione1>0.6 && umidOUT<95.0) {
    client.publish(forecast_topic, "Forse migliora!");
    delay(1);        
  }
  else if (diffPressione1<0.6 && diffPressione1>-0.6) {
    client.publish(forecast_topic, "Tempo stabile!");
    delay(1);        
  }

  
  if (diffPressione2<-5.0 || diffPressione2>5.0) {            // PREVISIONE DUE ORE
    diffPressione2=0;
  }
  else if (diffPressione2<-2.5 || diffPressione2>2.5) {
    // tempo?
    client.publish(forecast_topic, "Vento fortissimo in arrivo!");
    delay(1);
  }
  else if (diffPressione2>0.7 && umidOUT<95.0) {
    client.publish(forecast_topic, "Forse migliora!");
    delay(1);        
  }
  else if (diffPressione2<0.6 && diffPressione2>-0.6) {
    client.publish(forecast_topic, "Tempo stabile!");
    delay(1);        
  }

      
  snprintf (valore, 10, "%.2f", diffPressione);      
  client.publish(diffPressione_topic, valore);
  delay(1);
  snprintf (valore, 10, "%.2f", diffPressione1);
  client.publish(diffPressione1_topic, valore);
  delay(1);
  snprintf (valore, 10, "%.2f", diffPressione2);
  client.publish(diffPressione2_topic, valore);
  delay(1);  
  snprintf (valore, 10, "%.2f", tempIN);      
  client.publish(temperaturaIN_topic, valore);
  delay(1);
  snprintf (valore, 10, "%.2f", pressIN);
  client.publish(pressioneIN_topic, valore);
  delay(1);
  snprintf (valore, 10, "%.2f", umidIN);
  client.publish(umiditaIN_topic, valore);
  delay(1);  
}
