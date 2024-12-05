#include <ESP32Servo.h>
#include <BluetoothSerial.h>

// Déclarations générales
BluetoothSerial SerialBT;  // Initialisation Bluetooth
Servo monServo;            // Initialisation du servomoteur

// Broches et constantes
const int servoPin = 5;
const int LED1 = 26;
const int LED2 = 27;
const int soundAnalog = 34;
const int PIR_PIN = 4;

const int seuilBebeMin = 2700;
const int seuilBebeMax = 3000;

bool servoActive = false;
bool ledAnimationActive = false;
bool capteurSonActif = false;
bool capteurMouvementActif = false;
bool bebeCriDetecte = false;
bool bebePresent = false;

// Délais pour éviter des répétitions inutiles
unsigned long dernierEnvoi = 0;
const unsigned long delaiEnvoi = 20000;

// Initialisation des composants
void setup() {
  // Initialisation Bluetooth
  SerialBT.begin("ESP32_MultiControl");
  Serial.begin(115200);
  Serial.println("Système prêt à recevoir des commandes !");

  // Initialisation servomoteur
  monServo.attach(servoPin);
  monServo.write(90);  // Position initiale au centre

  // Initialisation LEDs et capteurs
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(PIR_PIN, INPUT);
}

// Fonction pour activer/désactiver le servomoteur
void gererServo(char commande) {
  if (commande == '1') {  // Activer le servo
    servoActive = true;
    SerialBT.println("Servo activé.");
  } else if (commande == '2') {  // Désactiver le servo
    servoActive = false;
    monServo.write(90);  // Revenir au centre
    SerialBT.println("Servo désactivé.");
  }

  if (servoActive) {
    for (int position = 0; position <= 180; position++) {
      monServo.write(position);
      delay(15);
    }
    for (int position = 180; position >= 0; position--) {
      monServo.write(position);
      delay(15);
    }
  }
}

// Fonction pour gérer l'état des LEDs
void gererLEDs(char commande) {
  if (commande == '3') {
    digitalWrite(LED1, !digitalRead(LED1));
    SerialBT.println("LED 1 basculée !");
  } else if (commande == '4') {
    digitalWrite(LED2, !digitalRead(LED2));
    SerialBT.println("LED 2 basculée !");
  } else if (commande == '5') {  // Animation des LEDs
    ledAnimationActive = !ledAnimationActive;
    SerialBT.println(ledAnimationActive ? "Animation activée." : "Animation désactivée.");
    while (ledAnimationActive) {
      digitalWrite(LED1, HIGH);
      digitalWrite(LED2, LOW);
      delay(500);
      digitalWrite(LED1, LOW);
      digitalWrite(LED2, HIGH);
      delay(500);
      if (SerialBT.available() && SerialBT.read() == '5') break;  // Arrêter l'animation
    }
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);
  }
}

// Fonction pour surveiller le capteur de son
void surveillerCapteurSon() {
  if(bebePresent){
    int valAnalog = analogRead(soundAnalog);
    SerialBT.print("Sound Level: ");
    SerialBT.println(valAnalog);
    if (valAnalog >= seuilBebeMin && valAnalog <= seuilBebeMax) {
      if (!bebeCriDetecte && millis() - dernierEnvoi > delaiEnvoi) {
        SerialBT.println("Bébé pleure !");
        bebeCriDetecte = true;
        dernierEnvoi = millis();
      }
    } else if (bebeCriDetecte && millis() - dernierEnvoi > delaiEnvoi) {
      SerialBT.println("Pas de son.");
      bebeCriDetecte = false;
      dernierEnvoi = millis();
    }
     delay(100);
  }
  //  else {
  //   SerialBT.println("Bébé n'est pas présent pour détecter le son");
  // }
}

// Fonction pour surveiller le capteur PIR
void surveillerCapteurPIR() {
  int etatPIR = digitalRead(PIR_PIN);
  if (etatPIR == LOW && !bebePresent) {
    SerialBT.println("Mouvement détecté : Bébé présent !");
    bebePresent = true;
    dernierEnvoi = millis();
  } else if (etatPIR == HIGH && bebePresent && millis() - dernierEnvoi > delaiEnvoi) {
    SerialBT.println("Aucune présence détectée.");
    bebePresent = false;
    dernierEnvoi = millis();
  }
   delay(100);
}

// Boucle principale
void loop() {
  if (SerialBT.available()) {
    char commande = SerialBT.read();
    if (commande == '1' || commande == '2') gererServo(commande);
    else if (commande == '3' || commande == '4' || commande == '5') gererLEDs(commande);
    else if (commande == '6') {
      capteurSonActif = !capteurSonActif;
      SerialBT.println(capteurSonActif ? "Capteur de son activé." : "Capteur de son désactivé.");
    } else if (commande == '7') {
      capteurMouvementActif = !capteurMouvementActif;
      SerialBT.println(capteurMouvementActif ? "Capteur PIR activé." : "Capteur PIR désactivé.");
    }
  }

  if (capteurSonActif) surveillerCapteurSon();
  if (capteurMouvementActif) surveillerCapteurPIR();
}
