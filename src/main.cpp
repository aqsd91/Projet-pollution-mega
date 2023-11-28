#include <Wire.h>     // Import de la bibliothèque de gestion de l'I2C
#include "rgb_lcd.h"  // Import du fichier de gestion de l'afficheur LCD
#include "DHT.h"      // Import du fichier du capteur DHT20
#include <Arduino_JSON.h>
//======= Définition variables capteurs
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
  uint32_t speedSerial1   = 9600;     // Vitesse envoi des données vers le NODE
  


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


//========== Déclarations fonctions
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
  //======== Initialisation Radio
  int setPin = 7; // Car la broche SET du HC12 est sur 7
  pinMode(setPin, OUTPUT); // La broche 7 est une sortie à piloter
  digitalWrite(setPin,LOW); // On met SET en mode changement canal
  Serial1.print(F("AT+DEFAULT"));delay(100); // Paramètres usine
  Serial1.print(F("AT+C032"));
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
    // Valeur déboguage sèrie
    temp_mega = random(0,1000);
    hum_mega = random(0,1000);
    poll_mega = random(0,1000);
    bruit_mega = random(0, 1000);
    
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

//======== Gestion de l'alarme
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

//======== Création de la chaine JSON
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

//======== Gestion capteur de bruit
void gererBruit(){
  bruit_mega = analogRead(Analog_Bruit) * (5.0 / 1023.0);
  bruit_digital = digitalRead(Digital_Bruit);

  //... et envoyées à la sortie série.
  // debug.print("Tension analogique:");
  // debug.print(Analog, 4);
  // debug.print("V, ");
  // debug.print("Limite:");

  // if (bruit_digital == 1) {
  //   debug.println(" atteinte");
  // } else {
  //   debug.println(" pas encore atteinte");
  // }
  // debug.println("----------------------------------------------------------------");
}

//======== Gestion capteur de température/humidité
void gererTempHum(){
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

//======== Gestion capteur pollution
void gererPollution(){
  // Capteur de pollution
  float sensor_volt;
  float sensorValue;
  sensorValue = analogRead(A2);
  sensor_volt = sensorValue / 1024 * 5.0;
  poll_mega = sensor_volt;
  // debug.print("sensor_volt = ");
  // debug.print(sensor_volt);
  // debug.println("V");
}

//======== Gestion de l'écran LCD
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
  
  // Ecran humidité
  if (hum_mega > 75) {
    setLCD(255, 0, 0, "Humidite", String(hum_mega) + " %");
  } else {
    setLCD(0, 255, 0, "Humidite", String(hum_mega) + " %");
  }
  // Alarme humidité
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
  
  // Ecran temperature
  if (temp_mega > 35) {
    setLCD(255, 0, 0, "Temperature", String(temp_mega) + " *C");
  } else {
    setLCD(0, 255, 0, "Temperature", String(temp_mega) + " *C");
  }
  // Alarme temperature
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
  // Alarme pollution
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
//======== Édition de l'écran LCD
void setLCD(int R, int G, int B, String texte, String valeur){
  lcd.setRGB(R, G, B);     // Met le rétro-éclairage en rouge (les 3 valeurs = Rouge,Vert,Bleu)
  lcd.setCursor(0, 0);     // Place le curseur à la colonne 0 et ligne 0 (la 1ère ligne est la ligne 0)
  lcd.clear();             // Efface tout avant d'écrire
  lcd.print(texte);        // Écrit à partir de l'emplacement du curseur
  lcd.setCursor(0, 1);     // Place le curseur à la colonne 0 et ligne 1 (la 1ère ligne est la ligne 0)
  lcd.print(valeur);       // Écrit à partir de l'emplacement du curseur
}
