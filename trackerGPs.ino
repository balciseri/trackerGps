#include <TinyGPS++.h> // parsing NMEA per il gps
#include <LiquidCrystal.h> // per usare LCD
#include <SoftwareSerial.h> // per parlare col gps
#include <StateMachine.h> // implementazione semplice di una macchina a stati

TinyGPSPlus gps;

//Pin utilizzati da SoftwareSerial per parlare col Gps
#define RXPin 6 //TX sul Gps
#define TXPin 7 //RX sul Gps

#define pinTemp A0 //GPIO dal quale riceveremo la tensione del termistore

int GPSBaud = 9600; //Default baud of NEO-6M is 9600

SoftwareSerial gpsSerial(RXPin, TXPin); // Creo la software serial port chiamata "gpsSerial"

//Comoda funziona per mostrare cosa rileva il gps
void displayInfo()
{
  if (gps.location.isValid()) {
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6);
    Serial.print("Altitude: ");
    Serial.println(gps.altitude.meters());
  }
  else {
    Serial.println("Location: Not Available");
  }
}

//Configuro Display LCD
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

//Calcolo la temperatura utilizzando l'equazione Steinhart–Hart 
double Thermistor(int RawADC) {
  double Temp;
  Temp = log(10000.0 * ((1024.0 / RawADC - 1))); //10000 e' la Resistenza , 1024 e' l'ADC a 2^10
  Temp = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * Temp * Temp)) * Temp);
  Temp = Temp - 273.15;
  return Temp;
}

double leggiTemp() {
  return Thermistor(analogRead(pinTemp));
}

const int SOGLIA = 10; //Quanti gradi deve salire di temperatura il mio motore per considerarlo acceso
double TemperaturaIniziale;


// Procedura utilizzata per parlare col GPS ( per spegnerlo e accenderlo)
void sendUBX(uint8_t *MSG, uint8_t len) {
  for (int i = 0; i < len; i++) {
    gpsSerial.write(MSG[i]);
  }
}

//*** MACHINA A STATI INIZIO ***//
const unsigned long STATE_DELAY = 1000*60*5; // 60 secondi * 5 = Controllo ogni 5 minuti
StateMachine machine = StateMachine();
State* S0 = machine.addState(&state0);
State* S1 = machine.addState(&state1);

/*unsigned long previousMillis = 0;
unsigned long previousMillis2 = 0;
unsigned long interval = 2000;*/

void state0() {
  if (machine.executeOnce) {
    Serial.println("Entro stato 0");
    lcd.noDisplay(); // Spengo il display

    //Set GPS to backup mode (sets it to never wake up on its own)
    //uint8_t GPSoff[] = {0xB5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x4D, 0x3B};
    //sendUBX(GPSoff, sizeof(GPSoff) / sizeof(uint8_t));

  }
  Serial.println("Rimango stato 0");
}

void state1() {
  if (machine.executeOnce) {
    Serial.println("Entro State 1");
    //Restart GPS
    //uint8_t GPSon[] = {0xB5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x4C, 0x37};
    //sendUBX(GPSon, sizeof(GPSon) / sizeof(uint8_t));
  }
  Serial.println("Rimango State 1");
  // Leggo i dati GPS
   while (gpsSerial.available() > 0) {
     if (gps.encode(gpsSerial.read())) {
       displayInfo(); // mostro sul serial monitor
     }
   }
  //Aggiorno il display con le coordinate rilevate
  lcd.display();
  lcd.setCursor(0, 0); // Colonna 0, Riga 0
  lcd.print("Lati:");
  lcd.setCursor(6, 0); // Colonna 6, Riga 0
  lcd.print(gps.location.lat());
  lcd.setCursor(0, 1); // Colonna 0, Riga 1
  lcd.print("Long: ");
  lcd.setCursor(6, 1); // Colonna 6, Riga 1
  lcd.print(gps.location.lng());
}

// Transizione su me stesso (S0)
bool transitionS0() {
  double newTemp = leggiTemp();
  // Se il motore e' ancora spento rimango in S0
  if (newTemp < TemperaturaIniziale + SOGLIA) {
    return true;
  }
  return false;
}
// Transizione da S0 a S1
bool transitionS0S1() {
  double newTemp = leggiTemp();
  // Se il motore era spento ed ora è acceso vado in S1
  if (newTemp < TemperaturaIniziale + SOGLIA) {
    return false;
  }
  return true;
}
// Transizione su me stesso (S1)
bool transitionS1() {
  double newTemp = leggiTemp();
  // Se il motore e' ancora acceso rimango in S1
  if (newTemp > TemperaturaIniziale + SOGLIA) {
    return true;
  }
  return false;
}
// Transizione da S1 a S0
bool transitionS1S0() {
  double newTemp = leggiTemp();
  // Se il motore era acceso ed ora è spento vado in S0
  if (newTemp < TemperaturaIniziale + SOGLIA) {
    return true;
  }
  return false;
}
//*** MACHINA A STATI FINE ***//

void setup() {
  Serial.begin(9600);
  
  gpsSerial.begin(GPSBaud); // Inizializzo la porta software serial alla velocità di default gel Gps

  lcd.begin(16, 2); //Configurazione LCD Colonne, Righe

  TemperaturaIniziale =  leggiTemp(); // leggo temperatura a motore spento.

  S0->addTransition(&transitionS0, S0);   // Transizione su me stesso (S0)
  S0->addTransition(&transitionS0S1, S1); // Transizione da S0 a S1
  S1->addTransition(&transitionS1, S1);   // Transizione su me stesso (S1)
  S1->addTransition(&transitionS1S0, S0); // Transizione da S1 a S0

}

void loop() {
  machine.run();
  //delay(100);
  delay(STATE_DELAY);
}
