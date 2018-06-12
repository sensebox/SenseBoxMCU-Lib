#include <SenseBoxMCU.h>

GPS gps;
float lat; //Geografische Breite
float lng; //Geografische Länge
float alt; //Höhe über Meeresspiegel in Metern

void setup() {
  gps.begin();
}

void loop() {
  lat = gps.getLatitude();
  lng = gps.getLongitude();
  alt = gps.getAltitude();
  
  Serial.print(lat,6);
  Serial.print(F(","));
  Serial.print(lng,6);
  Serial.print(F(","));
  Serial.println(alt,1);
  delay(100);
}

