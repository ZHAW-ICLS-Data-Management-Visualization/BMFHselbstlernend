#include <Servo.h> // Bibliothek für die Steuerung von Servomotoren einbinden

Servo servoBox; // Servo-Objekt für die Box
Servo servoFuss; // Servo-Objekt für den Fuss

int servoGeschwindigkeit = 5; // Zeit in Sekunden für eine komplette Bewegung (Periode)

const int boxPositionen[] = {130, 0, 130}; // Positionen, die die Box anfahren soll
const int boxAnzahlPositionen = sizeof(boxPositionen) / sizeof(boxPositionen[0]); // Anzahl der Box-Positionen berechnen

const int fussPositionen[] = {130, 130, 130, 50, 0, 0, 130}; // Positionen, die der Fuss anfahren soll
const int fussAnzahlPositionen = sizeof(fussPositionen) / sizeof(fussPositionen[0]); // Anzahl der Fuss-Positionen berechnen
float fussOffsetSekunden = 0; // Zeitverschiebung für den Fuss (Phasenverschiebung)

unsigned long startZeit; // Variable für den Startzeitpunkt

void setup() {
  servoBox.attach(6); // Box-Servo an Pin 6 anschließen
  servoFuss.attach(4); // Fuss-Servo an Pin 4 anschließen
  startZeit = millis(); // Startzeit speichern
}

// Funktion zum Berechnen der Zwischenpositionen (Interpolation)
int interpolieren(const int* positionen, int anzahlPositionen, float t, float periode) {
  float segmentZeit = periode / (anzahlPositionen - 1); // Zeit für einen Abschnitt berechnen
  int segment = int(t / segmentZeit); // Abschnitt bestimmen
  if (segment >= anzahlPositionen - 1) segment = anzahlPositionen - 2; // Sicherstellen, dass wir nicht über das Ende hinausgehen
  float lokalT = (t - segment * segmentZeit) / segmentZeit; // Anteil innerhalb des Abschnitts berechnen
  return positionen[segment] + (positionen[segment + 1] - positionen[segment]) * lokalT; // Zwischenposition berechnen
}

void loop() {
  float t = fmod((millis() - startZeit) / 1000.0, servoGeschwindigkeit); // Zeit seit Start berechnen und auf Periode begrenzen

  int boxPosition = interpolieren(boxPositionen, boxAnzahlPositionen, t, servoGeschwindigkeit); // Box-Position berechnen
  servoBox.write(boxPosition); // Box-Servo auf die berechnete Position stellen

  float fussT = fmod(t + fussOffsetSekunden, servoGeschwindigkeit); // Zeit für den Fuss mit Verschiebung berechnen
  int fussPosition = interpolieren(fussPositionen, fussAnzahlPositionen, fussT, servoGeschwindigkeit); // Fuss-Position berechnen
  servoFuss.write(fussPosition); // Fuss-Servo auf die berechnete Position stellen

  delay(20); // 20 Millisekunden warten, damit die Bewegung flüssig bleibt
}
