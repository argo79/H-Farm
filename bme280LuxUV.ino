/*
Lettura fotoresistore
Lettura modulo bme280
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define SEALEVELPRESSURE_HPA (1013.25)


int fotoPin=15;
int temtPin=4;
int lumen;
int luceRaw, uvRaw;
float light, volts, amps, microamps, lux;

Adafruit_BME280 bme; // I2C



void setup() {

  Serial.begin(9600);
  pinMode(fotoPin,INPUT);
  pinMode(temtPin,INPUT);

  // Codice per testare il sensore BME280
  Serial.println(F("BME280 test"));
  bool status;
  status = bme.begin(0x76);  
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
}


void loop() {
  
  // Codice per leggere una fotoresistenza / fotodiodo / fototransistor 5800B
  luceRaw=analogRead(fotoPin);
  Serial.print("Valore lettura luce = ");
  Serial.println(luceRaw);

  // Codice per leggere il sensore TEMT6000
  uvRaw = analogRead(temtPin);
  volts = uvRaw * 3.3 / 4096.0;
  
  light = uvRaw * 0.0244141;// percentage calculation
  Serial.print("Valore percentuale UV = ");
  Serial.println(light);
  
  amps = volts / 10000.0; // across 10,000 Ohms
  microamps = amps * 1000000;
  lux = microamps * 2.0;
  Serial.print("Lux = ");
  Serial.println(lux);


  // Codice di lettura del sensore BME280 3.3 - 5 
  Serial.print("Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println(" *C");
  
  // Convert temperature to Fahrenheit
  /*Serial.print("Temperature = ");
  Serial.print(1.8 * bme.readTemperature() + 32);
  Serial.println(" *F");*/
  
  Serial.print("Pressure = ");
  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.print("Humidity = ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");

  delay(500);

}
