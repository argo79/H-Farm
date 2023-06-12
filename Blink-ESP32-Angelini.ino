/*
  ESP32-IoT Angelini Test
  Trasmissione bluetooth del valore del sensore PIR
  Ricezione comando accensione/spegnimento led
  Ricezione comando impostazione soglia PIR
  Ricezione comando modalità: acceso, spento, rileva movimento
*/

#define builtInLed 2

#include <WiFi.h>
#include <PubSubClient.h>
#include "BluetoothSerial.h"
// OTA
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
// OTA

//////////////////////////////////
//           WIFI               //
//////////////////////////////////
const char* ssid = "blekgek";
const char* password = "radicofani27";
const char* mqtt_server = "192.168.0.69";

//////////////////////////////////
//         PARAMETRI            //
//////////////////////////////////
// Configurazione Mosquitto

#define nomeClient        "IoTAngelini"

#define Messaggio_topic   "IoT/Angelini/messaggio"
#define statoLed_topic    "IoT/Angelini/statoLed"
#define sogliaPir_topic   "IoT/Angelini/sogliaPir"


// Creazione oggetti

WiFiClient esp32Client;
PubSubClient client(esp32Client);
WiFiUDP Udp;
BluetoothSerial SerialBT;
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

bool statoLed=0;
bool statoLedOld;
int sogliaPir=512;

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  Serial.begin(115200);

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

ArduinoOTA.setHostname("IoTAngelini-OTA");
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
  sendBTWifi();

// Avvio la connessione con il server mqtt (mosquitto)
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  pinMode(builtInLed, OUTPUT);
}


// the loop function runs over and over again forever
void loop() {

  ArduinoOTA.handle();
  
  if (!client.connected()) {
    reconnect();
  }  


  digitalWrite(builtInLed, statoLed);  // turn the LED on (HIGH is the voltage level)
//  delay(1000);                      // wait for a second
//  digitalWrite(builtInLed, !statoLed);   // turn the LED off by making the voltage LOW
//  delay(1000);                      // wait for a second




  client.loop();
}


void callback(char* topic, byte* payload, unsigned int length) {        // Ricezione messaggi MQTT dal broker
  
  String response;  
  for (int i = 0; i < length; i++) {
    response+=(char)payload[i];
  }
  
  String topicS=topic;
  //SerialBT.println(topicS);  
  
  if (topicS=="IoT/Angelini/statoLed") {
    //SerialBT.println(response);
    statoLedOld=statoLed;    
    statoLed=response.toInt();    
  }
  else if (topicS=="IoT/Angelini/sogliaPir") {
    sogliaPir=response.toInt();
    SerialBT.println(response);
    SerialBT.println(sogliaPir);
  }  

}

void reconnect() {                            // CONNESSIONE BROKER MQTT
  // Loop until we're reconnected
  while (!client.connected()) {
    SerialBT.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(nomeClient)) {
      SerialBT.println("connected");      
      client.subscribe(Messaggio_topic,1);
      client.subscribe(statoLed_topic,1);
      client.subscribe(sogliaPir_topic,1);
      
      
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





void sendBTWifi() {
  SerialBT.println("");
  SerialBT.println("WiFi connected.");
  SerialBT.println("IP address: ");
  SerialBT.println(WiFi.localIP());
}
