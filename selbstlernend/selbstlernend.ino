// ===========================
// === BIBLIOTHEKEN ===
// ===========================
#include "Servo.h" // Servo-Bibliothek fuer Motorsteuerung
#include "Wire.h"  // I2C-Bibliothek fuer Sensorkommunikation
#include <math.h>  // Mathematische Funktionen (sqrt, etc.)

// ===========================
// === SERVOS ===
// ===========================
Servo servoBox;  // Servo-Objekt fuer Box-Bewegung
Servo servoFuss; // Servo-Objekt fuer Fuss-Bewegung

// Bewegungsbereiche und Schrittweiten
const int bewegungsBereich1[] = {20, 90}; // Bereich fuer Servo Box (Min, Max)
const int bewegungsBereich2[] = {20, 150}; // Bereich fuer Servo Fuss (Min, Max)
const int schrittweite1 = 10; // Schrittweite fuer Servo Box
const int schrittweite2 = 10; // Schrittweite fuer Servo Fuss

// Berechnete Werte
int anzahlSchritte1 = (bewegungsBereich1[1] - bewegungsBereich1[0]) / schrittweite1; // Anzahl Schritte fuer Servo Box
int anzahlSchritte2 = (bewegungsBereich2[1] - bewegungsBereich2[0]) / schrittweite2; // Anzahl Schritte fuer Servo Fuss

// ===========================
// === LED-KONFIGURATION ===
// ===========================
const int LED_PIN = 13; // Pin fuer Status-LED (meist eingebaute LED)

// ===========================
// === SENSOR-KONFIGURATION ===
// ===========================
const int MPU_ADDR = 0x68; // I2C-Adresse des MPU-6050 Sensors

// Sensor-Rohdaten
int16_t accelerometer_x, accelerometer_y, accelerometer_z; // Beschleunigungssensor-Werte
int16_t gyro_x, gyro_y, gyro_z;                           // Gyroskop-Werte
int16_t temperature;                                       // Temperatur-Wert
char acc_z;                                                // Variable fuer Z-Achsen-Bewegung

// ===========================
// === LERN-ALGORITHMUS ===
// ===========================
int neuePositionen1[] = {0, 0};          // Neue Positionen fuer Servo Box (Start, Ziel)
int neuePositionen2[] = {0, 0};          // Neue Positionen fuer Servo Fuss (Start, Ziel)
volatile float bestePositionen[] = {50, 50, 50, 50, 0}; // Beste Positionen: [Box1, Fuss1, Box2, Fuss2, Bewegung] - mit sinnvollen Startwerten

// Basis-Werte fuer Bewegungserkennung
int16_t basis_x = 0, basis_y = 0, basis_z = 0; // Ruhe-Referenzwerte
bool basisKalibriert = false; // Wurde die Basis bereits kalibriert?

// ===========================
// === SYSTEM-VARIABLEN ===
// ===========================
static int schleifenZaehler = 0; // Zaehler fuer Hauptschleifen-Durchlaeufe
static int startPhase = 0;       // Aktueller Status der Lernphasen
int versuche = 0;                // Zaehler fuer Lernversuche
int lernFortschritt = 0;         // Zaehler fuer erfolgreiche Lernschritte

// Lern-Parameter
const int MAX_VERSUCHE = 5;     // Maximale Anzahl Versuche pro Lernzyklus
const int MIN_ERFOLGE = 3;       // Minimale Anzahl Erfolge fuer Demo

// Globale Bewegungsmessung
float aktuelleBewegung = 0.0;   // Gesamtbewegungswert für aktuelle Sequenz

// Demonstrations-Variablen
int demonstrationMax = 20;        // Maximale Anzahl an Demonstrationen
int demonstrationZyklus = 0;     // Aktueller Demonstrations-Zyklus (0-4)
int demonstrationSchritt = 0;    // Aktueller Schritt im Zyklus (0=Start, 1=Mitte, 2=Ziel)
unsigned long letzteAktion = 0;  // Zeitpunkt der letzten Demonstrations-Aktion

// ===========================
// === HILFSFUNKTIONEN ===
// ===========================
char tmp_str[7]; // Temporaerer String-Puffer fuer Zahlkonvertierung

// Konvertiert int16 zu String mit fester Laenge fuer Debug-Ausgaben
char* convert_int16_to_str(int16_t i) {
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
}

void setup() {
  // === Serielle Kommunikation initialisieren ===
  Serial.begin(9600); // Starte die serielle Kommunikation fuer Debug-Ausgaben
  Serial.println("=== Selbstlernender Roboter startet ===");
  
  // === LED initialisieren ===
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // LED initial ausschalten
    
  // === Servo-Motoren initialisieren ===
  servoBox.attach(6);   // Verbinde Servo Box mit Pin 6
  servoFuss.attach(4);  // Verbinde Servo Fuss mit Pin 4
  Serial.println("Servos initialisiert");
  
  // === MPU-6050 Sensor initialisieren ===
  Serial.println("Starte Wire.begin()");
  Wire.begin();
  Serial.println("Wire.begin() abgeschlossen");

  Serial.println("Starte I2C-Übertragung an MPU-6050");
  Wire.beginTransmission(MPU_ADDR); // Startet eine Uebertragung an den I2C-Slave (GY-521 Board)
  Serial.println("Schreibe PWR_MGMT_1 Register (0x6B)");
  Wire.write(0x6B); // PWR_MGMT_1 Register
  Serial.println("Setze Wert auf 0 (Wakeup)");
  Wire.write(0);     // auf null setzen (weckt den MPU-6050 auf)
  Serial.println("Beende I2C-Übertragung");
  Wire.endTransmission(true);

  Serial.println("Warte 100ms nach Initialisierung");
  delay(100); // Kurze Pause nach Initialisierung, damit der Sensor hochfahren kann
  Serial.println("MPU-6050 Sensor initialisiert");
  
  Serial.println("=== Initialisierung abgeschlossen ===");
  Serial.println();
}

// ===========================
// === HAUPTFUNKTIONEN ===
// ===========================

/**
 * Liest Sensordaten vom MPU-6050 und berechnet die +Z Bewegung
 * @return float - Optimierungswert für +Z Bewegung (nach oben/springen)
 */
float pruefeBewegung() {
  // === Sensor-Daten abfragen ===
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // Startet mit Register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false); // Verbindung aktiv halten fuer weiteres Lesen
  Wire.requestFrom(MPU_ADDR, 7*2, true); // Fordert 14 Register an (7 x 2 Bytes)

  // === Register auslesen ===
  accelerometer_x = Wire.read()<<8 | Wire.read(); // X-Beschleunigung (0x3B, 0x3C)
  accelerometer_y = Wire.read()<<8 | Wire.read(); // Y-Beschleunigung (0x3D, 0x3E)
  accelerometer_z = Wire.read()<<8 | Wire.read(); // Z-Beschleunigung (0x3F, 0x40)
  temperature     = Wire.read()<<8 | Wire.read(); // Temperatur (0x41, 0x42)
  gyro_x          = Wire.read()<<8 | Wire.read(); // X-Rotation (0x43, 0x44)
  gyro_y          = Wire.read()<<8 | Wire.read(); // Y-Rotation (0x45, 0x46)
  gyro_z          = Wire.read()<<8 | Wire.read(); // Z-Rotation (0x47, 0x48)
  
  // === Einmalige Basis-Kalibrierung ===
  if (!basisKalibriert) {
    basis_x = accelerometer_x;
    basis_y = accelerometer_y;
    basis_z = accelerometer_z;
    basisKalibriert = true;
    Serial.println("=== SENSOR KALIBRIERT ===");
    Serial.println("Basis-Werte: X=" + String(basis_x) + ", Y=" + String(basis_y) + ", Z=" + String(basis_z));
  }
  
  // === Bewegung als Abweichung von der Ruhelage berechnen ===
  int16_t delta_x = accelerometer_x - basis_x;
  int16_t delta_y = accelerometer_y - basis_y;
  int16_t delta_z = accelerometer_z - basis_z;
  
  // === +Z OPTIMIERUNG ===
  // Nur positive Z-Bewegungen (nach oben) werden belohnt
  float zBewegung = 0.0;
  if (delta_z > 0) {
    zBewegung = delta_z; // Positive Z-Bewegung = gut!
  } else {
    zBewegung = 0; // Negative Z-Bewegung = ignorieren
  }
  
  // Gesamtbewegung für Referenz
  float gesamtBewegung = abs(delta_x) + abs(delta_y) + abs(delta_z);
  
  // === Debug-Ausgabe ===
  Serial.print("Roh: aX=" + String(accelerometer_x) + " aY=" + String(accelerometer_y) + " aZ=" + String(accelerometer_z));
  Serial.print(" | Delta: dX=" + String(delta_x) + " dY=" + String(delta_y) + " dZ=" + String(delta_z));
  Serial.print(" | +Z-Wert=" + String(zBewegung) + " | Gesamt=" + String(gesamtBewegung));
  
  // Richtungsanalyse für Debug
  String richtung = "0";
  if (abs(delta_z) > abs(delta_x) && abs(delta_z) > abs(delta_y)) {
    richtung = (delta_z > 0) ? "+Z" : "-Z";
  } else if (abs(delta_x) > abs(delta_y)) {
    richtung = (delta_x > 0) ? "+X" : "-X";
  } else {
    richtung = (delta_y > 0) ? "+Y" : "-Y";
  }
  Serial.print(" | Richtung=" + richtung);
  Serial.println();

  // Normalisierte +Z Bewegung zurueckgeben (das ist unser Optimierungsziel!)
  return zBewegung / 1000.0f;
}

/**
 * Hauptschleife - vereinfachte Version
 * Startet sofort mit dem Lernen, LED zeigt den Status
 */
void loop() {
   digitalWrite(LED_PIN, HIGH);
  
  // während dem Durchlaufen Lernschritt machen
  pruefeLernen();
  
  // Kleine Pause zwischen den Zyklen
  digitalWrite(LED_PIN, LOW);
  delay(100);
}

// ===========================
// === HILFSFUNKTIONEN FUER SERVOS ===
// ===========================

/**
 * Berechnet die Mittelposition eines Servos
 * @param bereich Array mit Min/Max-Werten
 * @return int Mittelposition
 */
int berechneMittelposition(const int bereich[]) {
  return bereich[0] + (bereich[1] - bereich[0]) / 2;
}

/**
 * Generiert zufaellige Positionen innerhalb des erlaubten Bereichs
 */
void generiereZufaelligePositionen() {
  neuePositionen1[0] = bewegungsBereich1[0] + (random(anzahlSchritte1) * schrittweite1);
  neuePositionen1[1] = bewegungsBereich1[0] + (random(anzahlSchritte1) * schrittweite1);
  neuePositionen2[0] = bewegungsBereich2[0] + (random(anzahlSchritte2) * schrittweite2);
  neuePositionen2[1] = bewegungsBereich2[0] + (random(anzahlSchritte2) * schrittweite2);
  
  Serial.println("Neue Positionen generiert: Box(" + 
                String(neuePositionen1[0]) + "→" + String(neuePositionen1[1]) + 
                "), Fuss(" + String(neuePositionen2[0]) + "→" + String(neuePositionen2[1]) + ")");
}

/**
 * Setzt beide Servos auf ihre Mittelpositionen (feste Werte)
 */
void setzeServosMittelposition() {
  servoBox.write(90);   // Feste Mittelposition für Servo Box
  servoFuss.write(90);  // Feste Mittelposition für Servo Fuss
}

// ===========================
// === LERN-ALGORITHMUS ===
// ===========================

/**
 * Hauptfunktion des Lernalgorithmus - State Machine
 * Durchlaeuft verschiedene Phasen um optimale Bewegungen zu finden
 */
void pruefeLernen() {
  if (startPhase == 0) { // === PHASE 0: Neue Positionen generieren und Bewegungsmessung starten ===
      generiereZufaelligePositionen();
      
      // Bewegungsmessung zurücksetzen für neue Sequenz
      aktuelleBewegung = 0.0;
      Serial.println("=== NEUE BEWEGUNGSSEQUENZ STARTET ===");
      
      // Zur Mittelposition und erste Messung vorbereiten
      setzeServosMittelposition();
      delay(100); // Warten bis Position erreicht
      startPhase = 1001;
      
    } else if (startPhase == 1001) { // === PHASE 1001: Mittelposition -> Position 1 (MESSUNG 1) ===
      Serial.println("=== MESSUNG 1: Mittelposition -> Position 1 ===");
      
      // Bewegung von Mittelposition zu Position 1
      servoBox.write(neuePositionen1[0]);
      servoFuss.write(neuePositionen2[0]);
      Serial.println("Anfahrt Position 1: Box=" + String(neuePositionen1[0]) + ", Fuss=" + String(neuePositionen2[0]));
      
      // Bewegung messen während Transition
      delay(50); // Kurze Pause, damit Bewegung startet
      float bewegung1 = pruefeBewegung();
      aktuelleBewegung += bewegung1;
      Serial.println("Bewegung 1 (Mitte->Pos1): +" + String(bewegung1) + ", Summe: " + String(aktuelleBewegung));
      
      delay(100); // Warten bis Position vollständig erreicht
      startPhase = 1002;
      
    } else if (startPhase == 1002) { // === PHASE 1002: Position 1 -> Position 2 (MESSUNG 2) ===
      Serial.println("=== MESSUNG 2: Position 1 -> Position 2 ===");
      
      // Bewegung von Position 1 zu Position 2 (kritische Sprung-Phase!)
      servoBox.write(neuePositionen1[1]);
      servoFuss.write(neuePositionen2[1]);
      Serial.println("Anfahrt Position 2: Box=" + String(neuePositionen1[1]) + ", Fuss=" + String(neuePositionen2[1]));
      
      // Bewegung messen während Transition (hier passiert der Sprung!)
      delay(50); // Kurze Pause, damit Bewegung startet
      float bewegung2 = pruefeBewegung();
      aktuelleBewegung += bewegung2;
      Serial.println("Bewegung 2 (Pos1->Pos2): +" + String(bewegung2) + ", Summe: " + String(aktuelleBewegung));
      
      delay(100); // Warten bis Position vollständig erreicht
      startPhase = 1003;
      
    } else if (startPhase == 1003) { // === PHASE 1003: Position 2 -> Mittelposition (MESSUNG 3) ===
      Serial.println("=== MESSUNG 3: Position 2 -> Mittelposition ===");
      
      // Bewegung von Position 2 zurück zur Mittelposition
      setzeServosMittelposition();
      
      // Bewegung messen während Rückkehr
      delay(50); // Kurze Pause, damit Bewegung startet
      float bewegung3 = pruefeBewegung();
      aktuelleBewegung += bewegung3;
      Serial.println("Bewegung 3 (Pos2->Mitte): +" + String(bewegung3) + ", Summe: " + String(aktuelleBewegung));
      
      delay(100); // Warten bis Mittelposition erreicht
      startPhase = 1004;
    } else if (startPhase == 1004) { // === PHASE 1004: Bewertung der Gesamtsequenz ===
      versuche++;
      
      Serial.println("=== SEQUENZ BEENDET ===");
      Serial.println("Messung #" + String(versuche) + " - GESAMTBEWEGUNG: +" + String(aktuelleBewegung) + 
                    ", Beste bisher: " + String(bestePositionen[4]));
      
      // Neue beste Bewegungssequenz gefunden?
      if (aktuelleBewegung > bestePositionen[4]) {
        bestePositionen[0] = neuePositionen1[0]; // Box Start
        bestePositionen[1] = neuePositionen2[0]; // Fuss Start
        bestePositionen[2] = neuePositionen1[1]; // Box Ziel
        bestePositionen[3] = neuePositionen2[1]; // Fuss Ziel
        bestePositionen[4] = aktuelleBewegung;   // Gesamtbewegungswert
        lernFortschritt++;
        
        Serial.println("*** NEUE BESTE BEWEGUNGSSEQUENZ *** Wert: " + String(aktuelleBewegung) + 
                      " (Fortschritt: " + String(lernFortschritt) + "/" + String(MIN_ERFOLGE) + ") - ROBOTER SPRINGT HÖHER!");
        Serial.println("Gespeicherte Positionen: Box(" + String(bestePositionen[0]) + "→" + String(bestePositionen[2]) + 
                      "), Fuss(" + String(bestePositionen[1]) + "→" + String(bestePositionen[3]) + ")");
      }
      
      // Genug gelernt oder zu viele Versuche?
      if (lernFortschritt >= MIN_ERFOLGE || versuche >= MAX_VERSUCHE) {
        Serial.println("=== LERNPHASE BEENDET ===");
        Serial.println("Erfolge: " + String(lernFortschritt) + "/" + String(MIN_ERFOLGE) + 
                      ", Versuche: " + String(versuche) + "/" + String(MAX_VERSUCHE));
        startPhase = 1007; // Zur Demonstrationsphase
      } else {
        startPhase = 0; // Neuen Versuch starten
      }
      Serial.println("============================");
    } else if (startPhase == 1007) { // Demo-Phase initialisieren
      Serial.println("DEMO START");
      digitalWrite(LED_PIN, HIGH);
      
      // Demonstrations-Variablen zurücksetzen
      demonstrationZyklus = 0;
      demonstrationSchritt = 0;
      letzteAktion = millis();
      
      startPhase = 1008;
      Serial.println("-> 1008");
      
    } else if (startPhase == 1007) {
    Serial.println("DEMO START");
    digitalWrite(LED_PIN, HIGH);
    
    // Demonstrations-Variablen zurücksetzen
    demonstrationZyklus = 0;
    demonstrationSchritt = 0;
    letzteAktion = millis();
    
    startPhase = 1008;
  } else if (startPhase == 1008) { // === PHASE 1008: Demonstrations-Ausführung ===
      // Alle Demonstrationszyklen durchlaufen
      while (demonstrationZyklus < demonstrationMax) {

        // Schritt 0: Mittelposition anfahren
        setzeServosMittelposition();
        delay(100);

        // Schritt 1: Starte mit Startposition
        servoBox.write((int)bestePositionen[0]);
        servoFuss.write((int)bestePositionen[1]);
        Serial.println("Demo Zyklus " + String(demonstrationZyklus + 1) + ": Startposition Box=" + String((int)bestePositionen[0]) + ", Fuss=" + String((int)bestePositionen[1]));
        delay(500);

        // Schritt 2: Gehe zur Zielposition
        servoBox.write((int)bestePositionen[2]);
        servoFuss.write((int)bestePositionen[3]);
        Serial.println("Demo Zyklus " + String(demonstrationZyklus + 1) + ": Zielposition Box=" + String((int)bestePositionen[2]) + ", Fuss=" + String((int)bestePositionen[3]));
        delay(500);

        demonstrationZyklus++;
      }
      
      // Demonstration beendet
      digitalWrite(LED_PIN, LOW);
      Serial.println("=== DEMONSTRATION BEENDET ===");
      startPhase = 0;
      versuche = 0;
      lernFortschritt = 0;
      
    } else {
      Serial.println("FEHLER: Unbekannte Phase " + String(startPhase));
      startPhase = 0; // Zurueck zum Anfang
    }
  
  delay(100); // Kurze Pause fuer Systemstabilitaet
}
