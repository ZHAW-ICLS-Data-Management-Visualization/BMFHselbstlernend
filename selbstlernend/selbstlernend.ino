#include "Servo.h" // Wir benutzen die Servo-Bibliothek, um Motoren zu steuern
#include <DFRobot_BMI160.h> // Bibliothek für den Bewegungssensor

Servo servoBox; // Erstes Servo-Objekt für Servo Box
Servo servoFuss; // Zweites Servo-Objekt für Servo Fuss

DFRobot_BMI160 bewegungsSensor; // Objekt für den Bewegungssensor
const int8_t i2cAdresse = 0x69; // Adresse für den Sensor (I2C)

const int bewegungsBereich1[] = {50, 130}; // Bereich für Servo Box (Minimum und Maximum)
const int bewegungsBereich2[] = {50, 130}; // Bereich für Servo Fuss (Minimum und Maximum)
const int schrittweite1 = 10; // Schrittweite für Servo Box
const int schrittweite2 = 10; // Schrittweite für Servo Fuss
int anzahlSchritte1 = (bewegungsBereich1[1] - bewegungsBereich1[0]) / schrittweite1; // Anzahl Schritte für Servo Box
int anzahlSchritte2 = (bewegungsBereich2[1] - bewegungsBereich2[0]) / schrittweite2; // Anzahl Schritte für Servo Fuss
int neuePositionen1[] = {0, 0}; // Neue Positionen für Servo Box (Start und Ziel)
int neuePositionen2[] = {0, 0}; // Neue Positionen für Servo Fuss (Start und Ziel)
volatile float bestePositionen[] = {0, 0, 0, 0, 0}; // Beste gefundene Positionen und Bewegung

int bewegungsRichtung = 4; // Richtung für die Messung (1-3 = Beschleunigung, 4-6 = Geschwindigkeit)

static int schleifenZaehler = 0; // Zählt die Durchläufe der Hauptschleife
static int startPhase = 0; // Status der Lernschritte

int versuche = 0; // Zählt die Versuche
int lernFortschritt = 0; // Zählt die erfolgreichen Lernschritte

void setup() {
  servoBox.attach(6);  // Verbinde Servo 1 mit Pin 6
  servoFuss.attach(4);  // Verbinde Servo 2 mit Pin 4

  Serial.begin(9600); // Starte die serielle Kommunikation

  // Sensor zurücksetzen
  if (bewegungsSensor.softReset() != BMI160_OK){
    Serial.println("Reset fehlgeschlagen");
    while(1); // Stoppe das Programm
  }

  // Sensor mit Adresse initialisieren
  if (bewegungsSensor.I2cInit(i2cAdresse) != BMI160_OK){
    Serial.println("Init fehlgeschlagen");
    while(1); // Stoppe das Programm
  }
}

float pruefeBewegung() {
  int i = 0; // Zähler für die Schleife
  int resultat; // Speichert das Ergebnis
  int16_t sensorDaten[6]={0}; // Array für Sensorwerte
  // Hole die Daten vom Sensor (Beschleunigung und Gyro)
  resultat = bewegungsSensor.getAccelGyroData(sensorDaten);
  /*
  // Das funktioniert gut
  if(resultat == 0){
    for(i=0; i<6; i++){
      if ( i<3 ){
        // Die ersten drei Werte sind Gyro-Daten
        Serial.println(String(i) + ":" + String(sensorDaten[i]*3.14/180.0));
      } else {
        // Die letzten drei Werte sind Beschleunigungsdaten
        Serial.println(String(i) + ":" + String(sensorDaten[i]/16384.0));
      }
    }
  }*/
  return sensorDaten[abs(bewegungsRichtung)-1]/16384.0; // Gib die Bewegung zurück
}

void loop() {
  schleifenZaehler++; // Zähle die Schleifen
  if (schleifenZaehler >= 20000) { // Nach 20000 Durchläufen
    pruefeLernen(); // Starte die Lernfunktion
    schleifenZaehler = 0; // Setze den Zähler zurück
  }
}

void pruefeLernen() {
  if (random(20) == 0) { // Zufällig, manchmal ausgeben
    Serial.println(startPhase);
  }
  
  if (startPhase == 0) { // Anfang: neue Positionen wählen
    neuePositionen1[0] = bewegungsBereich1[0] + (random(schrittweite1) * anzahlSchritte1);
    neuePositionen1[1] = bewegungsBereich1[0] + (random(schrittweite1) * anzahlSchritte1);
    neuePositionen2[0] = bewegungsBereich2[0] + (random(schrittweite2) * anzahlSchritte2);
    neuePositionen2[1] = bewegungsBereich2[0] + (random(schrittweite2) * anzahlSchritte2);
    Serial.println("Neue Positionen alle: " + String(neuePositionen1[0]) + ", " + String(neuePositionen1[1]) + ", " + String(neuePositionen2[0]) + ", " + String(neuePositionen2[1]));
    startPhase = 1; // Weiter zum nächsten Schritt
  } else if (startPhase == 1) { // Setze Servos in Mittelposition
    servoBox.write(bewegungsBereich1[0]+(bewegungsBereich1[1]-bewegungsBereich1[0])/2);
    servoFuss.write(bewegungsBereich2[0]+(bewegungsBereich2[1]-bewegungsBereich2[0])/2);
    Serial.println("Setze Servos auf Mittelposition: " + String(bewegungsBereich1[0]+(bewegungsBereich1[1]-bewegungsBereich1[0])/2) + ", " + String(bewegungsBereich2[0]+(bewegungsBereich2[1]-bewegungsBereich2[0])/2));
    startPhase = 2; // Weiter zum nächsten Schritt
  } else if (startPhase == 2) { // Warten bis Servos in Mittelposition sind
    if (servoBox.read() == bewegungsBereich1[0]+(bewegungsBereich1[1]-bewegungsBereich1[0])/2 && servoFuss.read() == bewegungsBereich2[0]+(bewegungsBereich2[1]-bewegungsBereich2[0])/2) {
      delay(500); // Warte 0.5 Sekunden
      startPhase = 1001; // Weiter zum nächsten Schritt
    }
  } else if (startPhase == 1001) { // Servos auf neue Positionen setzen
    servoBox.write(neuePositionen1[0]);
    servoFuss.write(neuePositionen2[0]);
    Serial.println("Neue Positionen: " + String(neuePositionen1[0]) + ", " + String(neuePositionen2[0]));
    startPhase = 1002; // Weiter zum nächsten Schritt
  } else if (startPhase == 1002) { // Warten bis Servos auf Position sind
    if (servoBox.read() == neuePositionen1[0] && servoFuss.read() == neuePositionen2[0]) {
      delay(100); // Warte 0.1 Sekunden
      startPhase = 1003; // Weiter zum nächsten Schritt
    }
  } else if (startPhase == 1003) { // Servos auf zweite neue Position setzen
    servoBox.write(neuePositionen1[1]);
    servoFuss.write(neuePositionen2[1]);
    Serial.println("Neue Positionen: " + String(neuePositionen1[1]) + ", " + String(neuePositionen2[1]));
    startPhase = 1004; // Weiter zum nächsten Schritt
    return;
  } else if (startPhase == 1004) { // Bewegung messen und vergleichen
    versuche++; // Zähle die Versuche
    float aktuelleBewegung = pruefeBewegung(); // Messe die Bewegung
    Serial.println("Bewegung: " + String(aktuelleBewegung) + " für Richtung: " + String(bewegungsRichtung) + ", Versuche: " +  String(versuche) + ", alte beste: " + String(bestePositionen[4]));
    if (aktuelleBewegung > bestePositionen[4]) { // Wenn besser als bisher
      bestePositionen[0] = neuePositionen1[0];
      bestePositionen[1] = neuePositionen2[0];
      bestePositionen[2] = neuePositionen1[1];
      bestePositionen[3] = neuePositionen2[1];
      bestePositionen[4] = aktuelleBewegung;
      lernFortschritt++; // Zähle erfolgreiche Lernschritte
      Serial.println("Neue beste Bewegung: " + String(aktuelleBewegung));
    }
    if (servoBox.read() == neuePositionen1[1] && servoFuss.read() == neuePositionen2[1]) {
      delay(100); // Warte 0.1 Sekunden
      if (lernFortschritt >= 5 || versuche >= 20) { // Genug gelernt oder viele Versuche
        Serial.println("Lernen abgeschlossen oder viele Versuche");
        startPhase = 1007; // Weiter zum Abschluss
      } else {
        startPhase = 0; // Starte von vorne
      }
    }
  } else if (startPhase == 1007) { // Zeige die beste Bewegung mehrmals
    Serial.println("============================");
    for (int i = 0; i < 10; i++) { // Wiederhole 10-mal
      servoBox.write(bewegungsBereich1[0]+(bewegungsBereich1[1]-bewegungsBereich1[0])/2);
      servoFuss.write(bewegungsBereich2[0]+(bewegungsBereich2[1]-bewegungsBereich2[0])/2);
      delay(500); // Warte für Bewegung
      servoBox.write(bestePositionen[0]);
      servoFuss.write(bestePositionen[1]);
      delay(500); // Warte für Bewegung
      servoBox.write(bestePositionen[2]);
      servoFuss.write(bestePositionen[3]);
      delay(500); // Warte für Bewegung
    }
    lernFortschritt = 0; // Setze Lernfortschritt zurück
    versuche = 0; // Setze Versuche zurück
    startPhase = 0; // Starte von vorne
  }
  delay(100); // Kurze Pause für Stabilität
}
