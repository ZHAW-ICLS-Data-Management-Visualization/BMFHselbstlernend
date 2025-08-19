#include <Servo.h> // Bibliothek für die Steuerung von Servomotoren

Servo servobox;    // Objekt für den Box-Servo
Servo servofuss;   // Objekt für den Fuss-Servo

int position = 0;  // Variable für die aktuelle Position des Servos

void setup() {
  servobox.attach(6);    // Verbinde Box-Servo mit Analog-Pin 6
  servofuss.attach(4);   // Verbinde Fuss-Servo mit Analog-Pin 4
}

void loop() {
  servobox.write(95);    // Box-Servo in Mittelstellung bringen
  delay(500);            // 0,5 Sekunden warten

  servobox.write(50);    // Box-Servo nach links bewegen
  servofuss.write(130);  // Fuss-Servo nach vorne bewegen
  delay(500);            // 0,5 Sekunden warten

  servofuss.write(50);   // Fuss-Servo nach hinten bewegen
  delay(500);            // 0,5 Sekunden warten

  servobox.write(90);    // Box-Servo zurück zur Mittelstellung
  servofuss.write(90);   // Fuss-Servo zurück zur Mittelstellung
  delay(250);            // 0,25 Sekunden warten
}