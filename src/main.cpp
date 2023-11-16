#include <Arduino.h>
#include <Wire.h>     // Import de la bibliothèque de gestion de l'I2C
#include "rgb_lcd.h"  // Import du fichier de gestion de l'afficheur LCD
#include "DHT.h"
#define DHTTYPE DHT20  // DHT 20
rgb_lcd lcd;
int Analog_Bruit = A0;  // Entrée analogique
int Digital_Bruit = 3;  // Entrée digitale
DHT dht(DHTTYPE);
#if defined(ARDUINO_ARCH_AVR)
#define debug Serial

#elif defined(ARDUINO_ARCH_SAMD) || defined(ARDUINO_ARCH_SAM)
#define debug SerialUSB
#else
#define debug Serial
#endif
#define LED1 12
int alarme = 0;
bool btn = false;

void gereralarme();

void setup() {
  // put your setup code here, to run once:
  pinMode(Analog_Bruit, INPUT);
  pinMode(Digital_Bruit, INPUT);
  Serial.begin(9600);
  lcd.begin(16, 2);
  debug.begin(9600);
  debug.println("DHTxx test!");
  Wire.begin();
  dht.begin();
  pinMode(LED1, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(10, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  float Analog;
  int Digital;
  float temp_hum_val[2] = { 0 };
  float sensor_volt;
  float sensorValue;
  alarme=0;
  //Les valeurs sont lues, sont converties en tension...
  Analog = analogRead(Analog_Bruit) * (5.0 / 1023.0);
  Digital = digitalRead(Digital_Bruit);

  //... et envoyées à la sortie série.
  Serial.print("Tension analogique:");
  Serial.print(Analog, 4);
  Serial.print("V, ");
  Serial.print("Limite:");

  if (Digital == 1) {
    Serial.println(" atteinte");
  } else {
    Serial.println(" pas encore atteinte");
  }
  Serial.println("----------------------------------------------------------------");

  if (!dht.readTempAndHumidity(temp_hum_val)) {
    debug.print("Humidity: ");
    debug.print(temp_hum_val[0]);
    debug.print(" %\t");
    debug.print("Temperature: ");
    debug.print(temp_hum_val[1]);
    debug.println(" *C");
  } else {
    debug.println("Failed to get temprature and humidity value.");
  }

  sensorValue = analogRead(A2);
  sensor_volt = sensorValue / 1024 * 5.0;
  Serial.print("sensor_volt = ");
  Serial.print(sensor_volt);
  Serial.println("V");

  if (Digital == 1) {
    lcd.clear();            // Efface tout avant d'écrire
    lcd.setRGB(255, 0, 0);  // Met le rétro-éclairage en rouge (les 3 valeurs = Rouge,Vert,Bleu)
    lcd.setCursor(0, 0);    // Place le curseur à la colonne 0 et ligne 0 (la 1ère ligne est la ligne 0)
    lcd.print("Bruit");     // Écrit à partir de l'emplacement du curseur
    lcd.setCursor(0, 1);    // Place le curseur à la colonne 0 et ligne 1 (la 1ère ligne est la ligne 0)
    lcd.print("Probleme");  // Écrit à partir de l'emplacement du curseur
  } else {
    lcd.clear();            // Efface tout avant d'écrire
    lcd.setRGB(0, 255, 0);  // Met le rétro-éclairage en rouge (les 3 valeurs = Rouge,Vert,Bleu)
    lcd.setCursor(0, 0);    // Place le curseur à la colonne 0 et ligne 0 (la 1ère ligne est la ligne 0)
    lcd.print("Bruit");     // Écrit à partir de l'emplacement du curseur
    lcd.setCursor(0, 1);    // Place le curseur à la colonne 0 et ligne 1 (la 1ère ligne est la ligne 0)
    lcd.print("Normal");    // Écrit à partir de l'emplacement du curseur
  }

  if ((Digital == 1 || temp_hum_val[0] > 75 || temp_hum_val[1] > 35 || sensor_volt > 0.5) && alarme!=2) {
    alarme = 1;
  } else if (alarme!=2) {
    alarme = 0;
  }
  if (alarme == 1) {
    gereralarme();
    alarme=2;
  } else {
    digitalWrite(LED1, LOW);
    digitalWrite(8, LOW);
  }

  delay(1000);

  if (temp_hum_val[0] > 75) {
    lcd.clear();                                // Efface tout avant d'écrire
    lcd.setRGB(255, 0, 0);                      // Met le rétro-éclairage en rouge (les 3 valeurs = Rouge,Vert,Bleu)
    lcd.setCursor(0, 0);                        // Place le curseur à la colonne 0 et ligne 0 (la 1ère ligne est la ligne 0)
    lcd.print("Humidite");                      // Écrit à partir de l'emplacement du curseur
    lcd.setCursor(0, 1);                        // Place le curseur à la colonne 0 et ligne 1 (la 1ère ligne est la ligne 0)
    lcd.print(String(temp_hum_val[0]) + " %");  // Écrit à partir de l'emplacement du curseur
  } else {
    lcd.clear();                                // Efface tout avant d'écrire
    lcd.setRGB(0, 255, 0);                      // Met le rétro-éclairage en rouge (les 3 valeurs = Rouge,Vert,Bleu)
    lcd.setCursor(0, 0);                        // Place le curseur à la colonne 0 et ligne 0 (la 1ère ligne est la ligne 0)
    lcd.print("Humidite");                      // Écrit à partir de l'emplacement du curseur
    lcd.setCursor(0, 1);                        // Place le curseur à la colonne 0 et ligne 1 (la 1ère ligne est la ligne 0)
    lcd.print(String(temp_hum_val[0]) + " %");  // Écrit à partir de l'emplacement du curseur
  }

  if ((Digital == 1 || temp_hum_val[0] > 75 || temp_hum_val[1] > 35 || sensor_volt > 0.5) && alarme!=2) {
    alarme = 1;
  } else if (alarme!=2) {
    alarme = 0;
  }
  if (alarme == 1) {
    gereralarme();
    alarme=2;
  } else {
    digitalWrite(LED1, LOW);
    digitalWrite(8, LOW);
  }

  delay(1000);

  if (temp_hum_val[1] > 35) {
    lcd.clear();                                 // Efface tout avant d'écrire
    lcd.setRGB(255, 0, 0);                       // Met le rétro-éclairage en rouge (les 3 valeurs = Rouge,Vert,Bleu)
    lcd.setCursor(0, 0);                         // Place le curseur à la colonne 0 et ligne 0 (la 1ère ligne est la ligne 0)
    lcd.print("Temperature");                    // Écrit à partir de l'emplacement du curseur
    lcd.setCursor(0, 1);                         // Place le curseur à la colonne 0 et ligne 1 (la 1ère ligne est la ligne 0)
    lcd.print(String(temp_hum_val[1]) + " *C");  // Écrit à partir de l'emplacement du curseur
  } else {
    lcd.clear();                                 // Efface tout avant d'écrire
    lcd.setRGB(0, 255, 0);                       // Met le rétro-éclairage en rouge (les 3 valeurs = Rouge,Vert,Bleu)
    lcd.setCursor(0, 0);                         // Place le curseur à la colonne 0 et ligne 0 (la 1ère ligne est la ligne 0)
    lcd.print("Temperature");                    // Écrit à partir de l'emplacement du curseur
    lcd.setCursor(0, 1);                         // Place le curseur à la colonne 0 et ligne 1 (la 1ère ligne est la ligne 0)
    lcd.print(String(temp_hum_val[1]) + " *C");  // Écrit à partir de l'emplacement du curseur
  }

  if ((Digital == 1 || temp_hum_val[0] > 75 || temp_hum_val[1] > 35 || sensor_volt > 0.5) && alarme!=2) {
    alarme = 1;
  } else if (alarme!=2) {
    alarme = 0;
  }
  if (alarme == 1) {
    gereralarme();
    alarme=2;
  } else {
    digitalWrite(LED1, LOW);
    digitalWrite(8, LOW);
  }

  delay(1000);

  if (sensor_volt > 0.5) {
    lcd.clear();             // Efface tout avant d'écrire
    lcd.setRGB(255, 0, 0);   // Met le rétro-éclairage en rouge (les 3 valeurs = Rouge,Vert,Bleu)
    lcd.setCursor(0, 0);     // Place le curseur à la colonne 0 et ligne 0 (la 1ère ligne est la ligne 0)
    lcd.print("Pollution");  // Écrit à partir de l'emplacement du curseur
    lcd.setCursor(0, 1);     // Place le curseur à la colonne 0 et ligne 1 (la 1ère ligne est la ligne 0)
    lcd.print(sensor_volt);  // Écrit à partir de l'emplacement du curseur
  } else {
    lcd.clear();             // Efface tout avant d'écrire
    lcd.setRGB(0, 255, 0);   // Met le rétro-éclairage en rouge (les 3 valeurs = Rouge,Vert,Bleu)
    lcd.setCursor(0, 0);     // Place le curseur à la colonne 0 et ligne 0 (la 1ère ligne est la ligne 0)
    lcd.print("Pollution");  // Écrit à partir de l'emplacement du curseur
    lcd.setCursor(0, 1);     // Place le curseur à la colonne 0 et ligne 1 (la 1ère ligne est la ligne 0)
    lcd.print(sensor_volt);  // Écrit à partir de l'emplacement du curseur
  }

  if ((Digital == 1 || temp_hum_val[0] > 75 || temp_hum_val[1] > 35 || sensor_volt > 0.5) && alarme!=2) {
    alarme = 1;
  } else if (alarme!=2) {
    alarme = 0;
  }
  if (alarme == 1) {
    gereralarme();
    alarme=2;
  } else {
    digitalWrite(LED1, LOW);
    digitalWrite(8, LOW);
  }

  delay(1000);
}

void gereralarme() {
  for (int i = 0; i < 20; i++) {
    digitalWrite(LED1, HIGH);
    digitalWrite(8, HIGH);
    delay(100);
    digitalWrite(LED1, LOW);
    digitalWrite(8, LOW);
    delay(100);
  }
}
