#include <Servo.h>

// Servo-Objekte
Servo servoBox;
Servo servoFuss;

// Pin-Definitionen
const int PIN_SERVO_BOX = 9;
const int PIN_SERVO_FUSS = 10;

// Servo-Grenzen
const int MIN_POS_BOX = 60;
const int MAX_POS_BOX = 120;
const int MIN_POS_FUSS = 60;
const int MAX_POS_FUSS = 120;

// Sinuskurven-Parameter
float zeit = 0.0;                    // Zeitvariable
const float ZEITSCHRITT = 0.05;     // Zeitinkrement pro Loop
const float FREQUENZ = 1.0;         // Frequenz der Sinuskurve (Hz)
const float PHASEN_OFFSET = 0.0;    // Phasenverschiebung zwischen Box und Fuss (in Radiant)
                                    // 0.0 = synchron
                                    // PI/2 = 90° versetzt
                                    // PI = 180° versetzt

// Berechnete Mittelpunkte und Amplituden
const int MITTE_BOX = (MIN_POS_BOX + MAX_POS_BOX) / 2;
const int AMPLITUDE_BOX = (MAX_POS_BOX - MIN_POS_BOX) / 2;
const int MITTE_FUSS = (MIN_POS_FUSS + MAX_POS_FUSS) / 2;
const int AMPLITUDE_FUSS = (MAX_POS_FUSS - MIN_POS_FUSS) / 2;

void setup() {
  Serial.begin(9600);
  
  // Servos initialisieren
  servoBox.attach(PIN_SERVO_BOX);
  servoFuss.attach(PIN_SERVO_FUSS);
  
  // Startposition: Mitte
  servoBox.write(MITTE_BOX);
  servoFuss.write(MITTE_FUSS);
  
  Serial.println("=== Sinus-Steuerung gestartet ===");
  Serial.print("Box-Bereich: ");
  Serial.print(MIN_POS_BOX);
  Serial.print(" - ");
  Serial.println(MAX_POS_BOX);
  Serial.print("Fuss-Bereich: ");
  Serial.print(MIN_POS_FUSS);
  Serial.print(" - ");
  Serial.println(MAX_POS_FUSS);
  Serial.print("Phasen-Offset: ");
  Serial.print(PHASEN_OFFSET);
  Serial.println(" Radiant");
  Serial.println("Zeit,Box_Position,Fuss_Position");
  
  delay(2000); // Kurz warten
}

void loop() {
  // Sinuskurven berechnen
  float sinus_box = sin(2 * PI * FREQUENZ * zeit);
  float sinus_fuss = sin(2 * PI * FREQUENZ * zeit + PHASEN_OFFSET);
  
  // Positionen berechnen (von -1 bis +1 auf Min-Max skalieren)
  int position_box = MITTE_BOX + (int)(sinus_box * AMPLITUDE_BOX);
  int position_fuss = MITTE_FUSS + (int)(sinus_fuss * AMPLITUDE_FUSS);
  
  // Sicherheitsbegrenzung
  position_box = constrain(position_box, MIN_POS_BOX, MAX_POS_BOX);
  position_fuss = constrain(position_fuss, MIN_POS_FUSS, MAX_POS_FUSS);
  
  // Servos bewegen
  servoBox.write(position_box);
  servoFuss.write(position_fuss);
  
  // Debug-Ausgabe (alle 0.2 Sekunden)
  static unsigned long letzteAusgabe = 0;
  if (millis() - letzteAusgabe > 200) {
    Serial.print(zeit, 2);
    Serial.print(",");
    Serial.print(position_box);
    Serial.print(",");
    Serial.println(position_fuss);
    letzteAusgabe = millis();
  }
  
  // Zeit weiterzählen
  zeit += ZEITSCHRITT;
  
  // Nach einer vollen Periode zurücksetzen (optional, verhindert Overflow)
  if (zeit >= (1.0 / FREQUENZ) * 10) { // 10 komplette Zyklen
    zeit = 0.0;
  }
  
  // Warten bis zum nächsten Schritt
  delay((int)(ZEITSCHRITT * 1000)); // ZEITSCHRITT in Millisekunden
}
