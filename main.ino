#include <OneWire.h>
#include <DallasTemperature.h>
#include <DHT22.h>
#include <stdio.h>
#include <SPI.h> 
#include <SD.h>

#define ONE_WIRE_BUS 2 // czujnij DS18B20
// #define DHT22_PIN 3   // czujnik DHT22

int pressurePin=4;
int val=0;

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
// DHT22 myDHT22(DHT22_PIN);
File myFile;

void setup() {
    Serial.begin(9600);
    sensors.begin();
    pinMode(10, OUTPUT);
}

void loop(void)
{
  // czujnik DS18B20
  sensors.requestTemperatures();
  delay(500);
  Serial.println("Czujnik DS18B20");
  Serial.print("Temperatura: ");
  Serial.print(sensors.getTempCByIndex(0)); // wartosc DS18B20
  Serial.println(" C");
  // karta SD
  myFile = SD.open("test.txt", FILE_WRITE);
  // czujnik DHT22
  /* Serial.println("Czujnik DHT22");
  Serial.print("Temperatura: ");
  Serial.print(myDHT22.getTemperatureC());
  Serial.print(" C, ");
  Serial.print("Wilgotnosc");
  Serial.print(myDHT22.getHumidity());
  Serial.println(" % RH");
  */
  // karta SD
  // MPX4115A
  val = analogRead(pressurePin);
  float pressure = ((val/1024.0)+0.095)/0.0009;
  Serial.println("Czujnik MPX4115A");
  Serial.print("Cisnienie = ");
  Serial.print(pressure);
  Serial.println(" hPa");
  delay(500);
}

