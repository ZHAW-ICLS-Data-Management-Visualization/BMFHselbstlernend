// ===========================
// === BIBLIOTHEKEN ===
// ===========================
#include "Servo.h" // Servo-Bibliothek fuer Motorsteuerung
#include "Wire.h"  // I2C-Bibliothek fuer Sensorkommunikation

// ===========================
// === SERVO-KONFIGURATION ===
// ===========================
Servo servoBox;  // Servo-Objekt fuer Box-Bewegung
Servo servoFuss; // Servo-Objekt fuer Fuss-Bewegung

// Bewegungsbereiche und Schrittweiten
const int bewegungsBereich1[] = {50, 130}; // Bereich fuer Servo Box (Min, Max)
const int bewegungsBereich2[] = {50, 130}; // Bereich fuer Servo Fuss (Min, Max)
const int schrittweite1 = 10; // Schrittweite fuer Servo Box
const int schrittweite2 = 10; // Schrittweite fuer Servo Fuss

// Berechnete Werte
int anzahlSchritte1 = (bewegungsBereich1[1] - bewegungsBereich1[0]) / schrittweite1; // Anzahl Schritte fuer Servo Box
int anzahlSchritte2 = (bewegungsBereich2[1] - bewegungsBereich2[0]) / schrittweite2; // Anzahl Schritte fuer Servo Fuss

// ===========================
// === SENSOR-KONFIGURATION ===
// ===========================
const int MPU_ADDR = 0x68; // I2C-Adresse des MPU-6050 Sensors

// Sensor-Rohdaten
int16_t accelerometer_x, accelerometer_y, accelerometer_z; // Beschleunigungssensor-Werte
int16_t gyro_x, gyro_y, gyro_z;                           // Gyroskop-Werte
int16_t temperature;                                       // Temperatur-Wert
char acc_y;                                                // Variable fuer Vorwaertsbewegung

// ===========================
// === LERN-ALGORITHMUS ===
// ===========================
int neuePositionen1[] = {0, 0};          // Neue Positionen fuer Servo Box (Start, Ziel)
int neuePositionen2[] = {0, 0};          // Neue Positionen fuer Servo Fuss (Start, Ziel)
volatile float bestePositionen[] = {0, 0, 0, 0, 0}; // Beste Positionen: [Box1, Fuss1, Box2, Fuss2, Bewegung]

int bewegungsRichtung = 4; // Messrichtung (1-3 = Beschleunigung, 4-6 = Geschwindigkeit)

// ===========================
// === SYSTEM-VARIABLEN ===
// ===========================
static int schleifenZaehler = 0; // Zaehler fuer Hauptschleifen-Durchlaeufe
static int startPhase = 0;       // Aktueller Status der Lernphasen
int versuche = 0;                // Zaehler fuer Lernversuche
int lernFortschritt = 0;         // Zaehler fuer erfolgreiche Lernschritte

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
  
  // === Servo-Motoren initialisieren ===
  servoBox.attach(6);   // Verbinde Servo Box mit Pin 6
  servoFuss.attach(4);  // Verbinde Servo Fuss mit Pin 4
  Serial.println("Servos initialisiert");
  
  // === MPU-6050 Sensor initialisieren ===
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR); // Startet eine Uebertragung an den I2C-Slave (GY-521 Board)
  Wire.write(0x6B); // PWR_MGMT_1 Register
  Wire.write(0);     // auf null setzen (weckt den MPU-6050 auf)
  Wire.endTransmission(true);
  Serial.println("MPU-6050 Sensor initialisiert");
  
  Serial.println("=== Initialisierung abgeschlossen ===");
  Serial.println();
}

// ===========================
// === HAUPTFUNKTIONEN ===
// ===========================

/**
 * Liest Sensordaten vom MPU-6050 und gibt die Y-Beschleunigung zurueck
 * @return float - Normalisierte Y-Beschleunigung als Bewegungswert
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
  
  // === Debug-Ausgabe aller Sensorwerte ===
  Serial.print("aX = "); Serial.print(convert_int16_to_str(accelerometer_x));
  Serial.print(" | aY = "); Serial.print(convert_int16_to_str(accelerometer_y));
  Serial.print(" | aZ = "); Serial.print(convert_int16_to_str(accelerometer_z));
  Serial.print(" | tmp = "); Serial.print(temperature/340.00+36.53); // Temperatur in Celsius
  Serial.print(" | gX = "); Serial.print(convert_int16_to_str(gyro_x));
  Serial.print(" | gY = "); Serial.print(convert_int16_to_str(gyro_y));
  Serial.print(" | gZ = "); Serial.print(convert_int16_to_str(gyro_z));
  Serial.println();

  // Normalisierte Y-Beschleunigung zurueckgeben (Hauptbewegungsrichtung)
  return (float) accelerometer_y / 1000.0f;
}

/**
 * Hauptschleife - wird kontinuierlich ausgefuehrt
 * Startet den Lernprozess alle 20000 Durchlaeufe
 */
void loop() {
  schleifenZaehler++;
  
  // Alle 20000 Durchlaeufe einen Lernzyklus starten
  if (schleifenZaehler >= 20000) {
    pruefeLernen();
    schleifenZaehler = 0; // Zaehler zuruecksetzen
  }
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
 * Setzt beide Servos auf ihre Mittelpositionen
 */
void setzeServosMittelposition() {
  int mittelBox = berechneMittelposition(bewegungsBereich1);
  int mittelFuss = berechneMittelposition(bewegungsBereich2);
  
  servoBox.write(mittelBox);
  servoFuss.write(mittelFuss);
  Serial.println("Servos auf Mittelposition: Box=" + String(mittelBox) + ", Fuss=" + String(mittelFuss));
}

/**
 * Prueft ob beide Servos ihre Zielpositionen erreicht haben
 * @param zielBox Zielposition fuer Servo Box
 * @param zielFuss Zielposition fuer Servo Fuss
 * @return bool True wenn beide Servos an der Zielposition sind
 */
bool servosErreichtPosition(int zielBox, int zielFuss) {
  return (servoBox.read() == zielBox && servoFuss.read() == zielFuss);
}

// ===========================
// === LERN-ALGORITHMUS ===
// ===========================

/**
 * Hauptfunktion des Lernalgorithmus - State Machine
 * Durchlaeuft verschiedene Phasen um optimale Bewegungen zu finden
 */
void pruefeLernen() {
  // Gelegentliche Debug-Ausgabe der aktuellen Phase
  if (random(20) == 0) {
    Serial.println("Phase: " + String(startPhase));
  }
  
  switch (startPhase) {
    case 0: // === PHASE 0: Neue Positionen generieren ===
      generiereZufaelligePositionen();
      startPhase = 1;
      break;
      
    case 1: // === PHASE 1: Servos in Mittelposition bringen ===
      setzeServosMittelposition();
      startPhase = 2;
      break;
      
    case 2: // === PHASE 2: Warten bis Servos Mittelposition erreicht haben ===
      {
        int mittelBox = berechneMittelposition(bewegungsBereich1);
        int mittelFuss = berechneMittelposition(bewegungsBereich2);
        
        if (servosErreichtPosition(mittelBox, mittelFuss)) {
          delay(500); // Stabilisierungszeit
          startPhase = 1001;
        }
      }
      break;
      break;
      
    case 1001: // === PHASE 1001: Erste Position anfahren ===
      servoBox.write(neuePositionen1[0]);
      servoFuss.write(neuePositionen2[0]);
      Serial.println("Anfahrt Position 1: Box=" + String(neuePositionen1[0]) + ", Fuss=" + String(neuePositionen2[0]));
      startPhase = 1002;
      break;
      
    case 1002: // === PHASE 1002: Warten bis erste Position erreicht ===
      if (servosErreichtPosition(neuePositionen1[0], neuePositionen2[0])) {
        delay(100); // Kurze Stabilisierungszeit
        startPhase = 1003;
      }
      break;
      
    case 1003: // === PHASE 1003: Zweite Position anfahren ===
      servoBox.write(neuePositionen1[1]);
      servoFuss.write(neuePositionen2[1]);
      Serial.println("Anfahrt Position 2: Box=" + String(neuePositionen1[1]) + ", Fuss=" + String(neuePositionen2[1]));
      startPhase = 1004;
      break;
      
    case 1004: // === PHASE 1004: Bewegung messen und bewerten ===
      versuche++;
      {
        float aktuelleBewegung = pruefeBewegung();
        Serial.println("Messung #" + String(versuche) + ": Bewegung=" + String(aktuelleBewegung) + 
                      ", Beste=" + String(bestePositionen[4]));
        
        // Neue beste Bewegung gefunden?
        if (aktuelleBewegung > bestePositionen[4]) {
          bestePositionen[0] = neuePositionen1[0]; // Box Start
          bestePositionen[1] = neuePositionen2[0]; // Fuss Start
          bestePositionen[2] = neuePositionen1[1]; // Box Ziel
          bestePositionen[3] = neuePositionen2[1]; // Fuss Ziel
          bestePositionen[4] = aktuelleBewegung;   // Bewegungswert
          lernFortschritt++;
          
          Serial.println("*** NEUE BESTE BEWEGUNG *** Wert: " + String(aktuelleBewegung) + 
                        " (Fortschritt: " + String(lernFortschritt) + "/5)");
        }
        
        // Warten bis Bewegung abgeschlossen, dann entscheiden wie weiter
        if (servosErreichtPosition(neuePositionen1[1], neuePositionen2[1])) {
          delay(100);
          
          // Genug gelernt oder zu viele Versuche?
          if (lernFortschritt >= 5 || versuche >= 20) {
            Serial.println("=== LERNPHASE BEENDET ===");
            Serial.println("Erfolge: " + String(lernFortschritt) + ", Versuche: " + String(versuche));
            startPhase = 1007; // Zur Demonstrationsphase
          } else {
            startPhase = 0; // Neuen Versuch starten
          }
        }
        Serial.println("============================");
      }
      break;
      
    case 1007: // === PHASE 1007: Beste Bewegung demonstrieren ===
      Serial.println("=== DEMONSTRATION DER BESTEN BEWEGUNG ===");
      for (int i = 0; i < 10; i++) {
        // Zur Mittelposition
        setzeServosMittelposition();
        delay(500);
        
        // Erste Position der besten Bewegung
        servoBox.write(bestePositionen[0]);
        servoFuss.write(bestePositionen[1]);
        delay(500);
        
        // Zweite Position der besten Bewegung
        servoBox.write(bestePositionen[2]);
        servoFuss.write(bestePositionen[3]);
        delay(500);
      }
      
      // Zuruecksetzen fuer neuen Lernzyklus
      lernFortschritt = 0;
      versuche = 0;
      startPhase = 0;
      Serial.println("=== NEUER LERNZYKLUS STARTET ===");
      break;
      
    default:
      Serial.println("FEHLER: Unbekannte Phase " + String(startPhase));
      startPhase = 0; // Zurueck zum Anfang
      break;
  }
  
  delay(100); // Kurze Pause fuer Systemstabilitaet
}
