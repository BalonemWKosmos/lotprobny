#include <OneWire.h>
#include <DallasTemperature.h>
#include <DHT22.h>
#include <stdio.h>
#include <SPI.h> 
#include <SD.h>

#define ONE_WIRE_BUS 2 // czujnik temperatury DS18B20
#define DHT22_PIN 3   // czujnik temperatury i wilgotnosci DHT22
#define MPX4115AS_PIN 4 // czujnik cisnienia MPX4115AS

// podlaczenie pinow modulu kart microSD:
// MOSI - pin 11
// MISO - pin 12
// CLK - pin 13
// CS - pin 10

int val=0;
unsigned long czas;

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DHT22 myDHT22(DHT22_PIN);
File myFile;

void setup() {
    Serial.begin(9600);
    sensors.begin();
    pinMode(10, OUTPUT);
    myFile = SD.open("pomiary.txt", FILE_WRITE);
    myFile.println("");
    myFile.println("----------");
    myFile.close();
}

void loop(void)
{
  czas = millis();
  // czujnik DS18B20
  sensors.requestTemperatures();
  delay(500);
  Serial.println("Czujnik DS18B20");
  Serial.print("Temperatura: ");
  Serial.print(sensors.getTempCByIndex(0)); // wartosc DS18B20
  Serial.println(" C");
  // czujnik DHT22
  Serial.println("Czujnik DHT22");
  Serial.print("Temperatura: ");
  Serial.print(myDHT22.getTemperatureC());
  Serial.print(" C, ");
  Serial.print("Wilgotnosc: ");
  Serial.print(myDHT22.getHumidity());
  Serial.println(" % RH");
  // MPX4115A
  val = analogRead(pressurePin);
  float cisnienie = ((val/1024.0)+0.095)/0.0009;
  Serial.println("Czujnik MPX4115A");
  Serial.print("Cisnienie: ");
  Serial.print(cisnienie);
  Serial.println(" hPa");
  delay(500);
  myFile = SD.open("pomiary.txt", FILE_WRITE);
  myFile.print(sensors.getTempCByIndex(0));
  myFile.print(",");
  myFile.print(myDHT22.getTemperatureC());
  myFile.print(",");
  myFile.print(myDHT22.getHumidity());
  myFile.print(",");
  myFile.print(cisnienie);
  myFile.print(",");
  myFile.println(czas);
  myFile.close();
}
