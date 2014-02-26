// wszystkie polecenia Serial.print w celu sprawdzenia ukladu przed wypuszczeniem

#include <OneWire.h>
#include <DallasTemperature.h>
#include <DHT22.h>
#include <stdio.h>
#include <SPI.h> 
#include <SD.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
// wszystkie piny w zaleznosci od podlaczenia ukladu
#define DS18B20_PIN 1 // czujnik temperatury DS18B20
#define DHT22_PIN 2   // czujnik temperatury i wilgotnosci DHT22
#define MPX4115AS_PIN 3 // czujnik cisnienia MPX4115AS
#define BUZZER_PIN 4 // buzzer
#define RX_GPS 5 // RX modulu GPS
#define TX_GPS 6 // TX modulu GPS
#define MOSFET_PIN 7 // tranzystor do przepalania linki

TinyGPS gps;
SoftwareSerial GPS(RX_GPS, TX_GPS);

// podlaczenie pinow modulu kart microSD:
// MOSI - pin 11
// MISO - pin 12
// CLK - pin 13
// CS - pin 10

int val=0;
int czas; // czas od pierwszego uruchomienia ukladu
int czas_koniec = 1000; // czas po ktorym zadziala tranzystor
int czas_buzzer = 1000; // czas po jakim moze wlaczyc sie buzzer, zeby nie bzykal od poczatku
byte gps_set_sucess = 0 ;
unsigned long fix_age, time, date; // zmienne czasu i daty
float falt; // zmienna wysokosci
long lat, lon; // zmienna dlugosci i szerokosci
byte opcja=1;

OneWire oneWire(DS18B20_PIN);
DallasTemperature sensors(&oneWire);
DHT22 myDHT22(DHT22_PIN);
File myFile;

void setup(void) {
    GPS.begin(9600); 
    Serial.begin(9600);
    GPS.print("$PUBX,41,1,0007,0003,4800,0*13\r\n");
    GPS.begin(4800);
    GPS.flush();
    sensors.begin();
    pinMode(10, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT); 
    myFile = SD.open("pomiary.txt", FILE_WRITE);
    myFile.println("");
    myFile.println("--------------"); // gdyby uklad sie zresetowal, w pliku tekstowym pojawi sie ciag znakow
    myFile.close();
    // GPS set
    uint8_t setNav[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC };
  while(!gps_set_sucess)
    {
    sendUBX(setNav, sizeof(setNav)/sizeof(uint8_t)); // w koncowym kodzie zostaje tylko to plus setNav[]
    gps_set_sucess=getUBX_ACK(setNav);
    }
    gps_set_sucess=0;
}

void loop()
{
  czas = millis()/1000;
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
  // MPX4115AS
  val = analogRead(MPX4115AS_PIN);
  float cisnienie = ((val/1024.0)+0.095)/0.0009;
  Serial.println("Czujnik MPX4115A");
  Serial.print("Cisnienie: ");
  Serial.print(cisnienie);
  Serial.println(" hPa");
  delay(500);
  // GPS
  if(GPS.available())
  {
  int c = GPS.read();
  if (gps.encode(c))
      {
       // poczatkowe sprawdzenie czy GPS zlapal sygnal, w ta petle wchodzi tylko wtedy, gdy
       // strumien NMEA jest calkowicie wypelniony, czyli wtedy, gdy jest sygnal
       if(czas<180) // sprawdzanie dziala przez 3 minuty, w tym czasie GPS zlapie sygnal na 100%
       {
         digitalWrite(BUZZER_PIN, 100);
         delay(1000); // na 1 sekunde buzzer da znac, ze jestesmy w tej petli
         digitalWrite(BUZZER_PIN, LOW);
       }
       // koniec sprawdzania
      gps.get_datetime(&date, &time, &fix_age); // odczyt czasu i daty
      gps.get_position(&lat, &lon, &fix_age); // odczyt szerokosci i dlugosci
      falt = gps.f_altitude(); // wysokosc

      Serial.println("");
      Serial.print("czas: ");
      Serial.println(time);
      Serial.print("szerokosc: ");
      Serial.println(lat);
      Serial.print("dlugosc: ");
      Serial.println(lon);
      Serial.print("wysokosc: ");
      Serial.print(falt);
      }  
 }
  // zapis danych na karte microSD
  myFile = SD.open("pomiary.txt", FILE_WRITE);
  myFile.print(sensors.getTempCByIndex(0)); // temperatura DS18B20
  myFile.print(",");
  myFile.print(myDHT22.getTemperatureC()); // temperatura DHT22
  myFile.print(",");
  myFile.print(myDHT22.getHumidity()); // wilgotnosc DHT22
  myFile.print(",");
  myFile.print(cisnienie); // cisnienie MPX4115AS
  myFile.print(",");
  myFile.println(time); // godzina, ponizej czas zliczany przez arduino
  myFile.print(lat); // szerokosc
  myFile.print(",");
  myFile.print(lon); // dlugosc
  myFile.print(",");
  myFile.print(falt); // wysokosc
  myFile.print(",");
  myFile.println(czas); // jeszcze raz czas
  myFile.close();
  // buzzer
  if(czas>czas_buzzer && falt<300) // buzzer moze wlaczyc sie po jakims czasie i ponizej 300m
                                   // startujemy z okolo 240
  {
    digitalWrite(BUZZER_PIN, HIGH);
  }
  // przepalanie linki
  if(opcja==1) // aby tranzystor zalaczyl sie tylko raz, a nie w kazdej petli
  {
    if(czas>czas_koniec || falt>1000) // jesli czas wiekszy od ustawionego LUB wysokosc wieksza od 1km
                                    // na wypadek gdyby GPS sie spieprzyl
    {
    digitalWrite(MOSFET_PIN, HIGH); // 5V na bramke mosfeta
    delay(1000); // opoznienie do przetestowania, nie wiem ile wystarczy
    digitalWrite(MOSFET_PIN, LOW); // 0V na bramke mosfeta
    opcja=0;
    }
  }
  Serial.println(czas); // na koniec calego maina sprawdzenie zliczania CALKOWITEGO czasu
}

// Send a byte array of UBX protocol to the GPS
void sendUBX(uint8_t *MSG, uint8_t len) {
  for(int i=0; i<len; i++) {
    GPS.write(MSG[i]);
    Serial.print(MSG[i], HEX);
  }
  GPS.println();
}

// Calculate expected UBX ACK packet and parse UBX response from GPS
// funkcja do sprawdzenia czy gps wysyla prawidlowy strumien danych
// w koncowym kodzie tego nie ma, sprawdzanie za pomoca buzzera przze kilka sekund
boolean getUBX_ACK(uint8_t *MSG) {
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();
  Serial.print(" * Reading ACK response: ");
 
  // Construct the expected ACK packet    
  ackPacket[0] = 0xB5;	// header
  ackPacket[1] = 0x62;	// header
  ackPacket[2] = 0x05;	// class
  ackPacket[3] = 0x01;	// id
  ackPacket[4] = 0x02;	// length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2];	// ACK class
  ackPacket[7] = MSG[3];	// ACK id
  ackPacket[8] = 0;		// CK_A
  ackPacket[9] = 0;		// CK_B
 
  // Calculate the checksums
  for (uint8_t i=2; i<8; i++) {
    ackPacket[8] = ackPacket[8] + ackPacket[i];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }
 
  while (1) {
 
    // Test for success
    if (ackByteID > 9) {
      // All packets in order!
      Serial.println(" (SUCCESS!)");
      return true;
    }
 
    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) { 
      Serial.println(" (FAILED!)");
      return false;
    }
 
    // Make sure data is available to read
    if (GPS.available()) {
      b = GPS.read();
 
      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID]) { 
        ackByteID++;
        Serial.print(b, HEX);
      } 
      else {
        ackByteID = 0;	// Reset and look again, invalid order
      }
 
    }
  }
}
