#include <Wire.h>     // Import de la bibliothèque de gestion de l'I2C
#include "rgb_lcd.h"  // Import du fichier de gestion de l'afficheur LCD
#include "DHT.h"
#include <Arduino_JSON.h>

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


//========== REGLAGES
  uint32_t timerDelay     = 1000;       // Fréquence d'envoi vers le NODE
  uint32_t speedSerial    = 115200;     // Vitesse moniteur série
  uint32_t speedSerial1   = 115200;     // Vitesse envoi des données vers le NODE
  


//========== OBJETS ET VARABLES GLOBAUX
  JSONVar readings;

//========== BUFFER et autres
  //----- Variables devant contenir les valeurs de capteurs (variables à mettre à jour dans le programme)
  uint32_t data_mega_5;           // Exemple : Entier 32bits non signé
  String   data_mega_6;           // Exemple 
  float temp_mega;
  float hum_mega;
  float poll_mega;
  float bruit_mega;
  uint32_t bruit_digital;

//======== FACILITATEURS
  // MACRO : Limiteur de fréquence
    #define LIMIT_FREQ_BEGIN(varName, duration) static uint32_t varName = 0; if(millis()>varName+duration){
    #define LIMIT_FREQ_END(varName) varName = millis();}

  // MACRO : Limiteur de répétition
    #define LIMIT_REP_BEGIN(varName, nbrOfRep) static uint8_t varName = 0; if(varName<nbrOfRep){
    #define LIMIT_REP_END(varName) varName++;}

  // MACRO : Setup virtuel
    #define SETUP_VIR_BEGIN(varName) static uint8_t varName = 0; if(varName<1){
    #define SETUP_VIR_END(varName) varName++;}


//========== PROTOTYPES DE FONCTIONS
String getSensorReadings();

void gereralarme();
void gererBruit();
void gererTempHum();
void gererPollution();
void gererLCD();
void setLCD(int R, int G, int B, String texte, String valeur);

void setup() {
  //======== Initialisation Serie
  Serial.begin(speedSerial);  while(!Serial);
  Serial1.begin(speedSerial1);while(!Serial1);  
  Serial.println("Serials:ok");
  //======== Initialisation Capteurs
  pinMode(Analog_Bruit, INPUT);
  pinMode(Digital_Bruit, INPUT);
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
  alarme = 0;
  //----- Récupération des données des capteurs la MEGA (capteurs locaux)      
      data_mega_5  = random(0,10000);                       // Exemple nbr 4 octets non signés (au hasard)
      data_mega_6  = "<span style='color:green;'>GREEN<span>";  // Exemple String avec injection html
    gererBruit();
    gererTempHum();
    gererPollution();
    gererLCD();
  //----- Envoie les données complètes à la NODE
    LIMIT_FREQ_BEGIN(sendingNodeData, timerDelay)
      Serial.println(getSensorReadings());
      Serial1.println(getSensorReadings());
    LIMIT_FREQ_END(sendingNodeData)


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

String getSensorReadings() {
  //Exemple JSON {"DATA1":"1234","DATA2":"3.14","DATA3":"Hello world","DATA4":"-12345678","DATA5":"123456789","DATA6":"a"}
  // Les noms des étiquettes DATAx doivent être les mêmes que dans l'id html ! (exemple avec DATAx)
  readings["TEMPERATURE-MEGA"]    = String( temp_mega );
  readings["HUMIDITE-MEGA"]    = String( hum_mega );
  readings["POLLUTION-MEGA"]    = String( poll_mega );
  readings["BRUIT-MEGA"]    = String( bruit_mega );
  readings["DATA-MEGA-5"]    = String( data_mega_5 );
  readings["DATA-MEGA-6"]    = String( data_mega_6 );

  String jsonString = JSON.stringify(readings);
  return jsonString;
}

void gererBruit(){
  // Capteur de Bruit
  bruit_mega = analogRead(Analog_Bruit) * (5.0 / 1023.0);
  bruit_digital = digitalRead(Digital_Bruit);

  //... et envoyées à la sortie série.
  // Serial.print("Tension analogique:");
  // Serial.print(Analog, 4);
  // Serial.print("V, ");
  // Serial.print("Limite:");

  // if (bruit_digital == 1) {
  //   Serial.println(" atteinte");
  // } else {
  //   Serial.println(" pas encore atteinte");
  // }
  // Serial.println("----------------------------------------------------------------");
}

void gererTempHum(){
  // Capteur de temperature/humidite  
  float temp_hum_val[2] = { 0 };
  if (!dht.readTempAndHumidity(temp_hum_val)) {
    temp_mega = temp_hum_val[1];
    hum_mega = temp_hum_val[0];
    // déboguage
    // debug.print("Humidity: ");
    // debug.print(temp_hum_val[0]);
    // debug.print(" %\t");
    // debug.print("Temperature: ");
    // debug.print(temp_hum_val[1]);
    // debug.println(" *C");
  } else {
    debug.println("Failed to get temprature and humidity value.");
  }
}

void gererPollution(){
  // Capteur de pollution
  float sensor_volt;
  float sensorValue;
  sensorValue = analogRead(A2);
  sensor_volt = sensorValue / 1024 * 5.0;
  poll_mega = sensor_volt;
  // Serial.print("sensor_volt = ");
  // Serial.print(sensor_volt);
  // Serial.println("V");
}

void gererLCD(){
  // Ecran Bruit
  if (bruit_digital == 1) {
    setLCD(255, 0, 0, "Bruit", "Probleme");
  } else {
    setLCD(0, 255, 0, "Bruit", "Normal");
  }

  if ((bruit_digital == 1 || hum_mega > 75 || temp_mega > 35 || poll_mega > 0.5) && alarme!=2) {
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
  
  // Ecran humidite
  if (hum_mega > 75) {
    setLCD(255, 0, 0, "Humidite", String(hum_mega) + " %");
  } else {
    setLCD(0, 255, 0, "Humidite", String(hum_mega) + " %");
  }

  if ((bruit_digital == 1 || hum_mega > 75 || temp_mega > 35 || poll_mega > 0.5) && alarme!=2) {
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
  
  // Ecran Temperature
  if (temp_mega > 35) {
    setLCD(255, 0, 0, "Temperature", String(temp_mega) + " *C");
  } else {
    setLCD(0, 255, 0, "Temperature", String(temp_mega) + " *C");
  }

  if ((bruit_digital == 1 || hum_mega > 75 || temp_mega > 35 || poll_mega > 0.5) && alarme!=2) {
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
  // Ecran pollution
  if (poll_mega > 0.5) {
    setLCD(255, 0, 0, "Pollution", String(poll_mega));
  } else {
    setLCD(0, 255, 0, "Pollution", String(poll_mega));
  }

  if ((bruit_digital == 1 || hum_mega > 75 || temp_mega > 35 || poll_mega > 0.5) && alarme!=2) {
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

void setLCD(int R, int G, int B, String texte, String valeur){
  lcd.setRGB(R, G, B);     // Met le rétro-éclairage en rouge (les 3 valeurs = Rouge,Vert,Bleu)
  lcd.setCursor(0, 0);     // Place le curseur à la colonne 0 et ligne 0 (la 1ère ligne est la ligne 0)
  lcd.clear();             // Efface tout avant d'écrire
  lcd.print(texte);        // Écrit à partir de l'emplacement du curseur
  lcd.setCursor(0, 1);     // Place le curseur à la colonne 0 et ligne 1 (la 1ère ligne est la ligne 0)
  lcd.print(valeur);       // Écrit à partir de l'emplacement du curseur
}
