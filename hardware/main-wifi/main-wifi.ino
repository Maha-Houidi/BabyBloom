#include <ESP32Servo.h>
#include <arduinoFFT.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>

//Debug
bool DEBUG = true;

// Déclarations générales

// Wi-Fi credentials
const char* ssid = "T_T";       // Wi-Fi network name
const char* password = "HelloHiBonjour"; // Wi-Fi password

// Create a web server on port 80
WebServer server(80);
          
// Initialisation du servomoteur
Servo monServo;

// Pin configuration
const int servoPin = 5;
const int LED1 = 26;
const int LED2 = 27;
const int PIR_PIN = 4;

//states variables
bool servoActive = false;
bool ledAnimationActive = false;
bool capteurSonActif = false;
bool capteurMouvementActif = false;
bool babyCryDetected = false;
bool babyPresent = false;

// Délais pour éviter des répétitions inutiles
unsigned long dernierEnvoi = 0;
const unsigned long delaiEnvoi = 20000;

unsigned long animationStartMillis = 0; // To track time for animation
const unsigned long animationInterval = 500; // Interval between LED toggles

// Constants for sound sensor
const int SOUND_SENSOR_PIN = 34;
const int SAMPLES = 512; // Must be a power of 2
const double SAMPLING_FREQUENCY = 1000; // Hz, must be less than 10000 due to ADC
const int SOUND_THRESHOLD = 45; // Example threshold in decibel

// Constants for detectSoundPattern
const int MIN_SPIKES = 75; // Minimum number of spikes to consider for a crying pattern
const int MAX_SPIKES = 160; // Maximum number to avoid continuous noise
const double SPIKE_THRESHOLD = 15.0; // Threshold for detecting a spike in intensity

// Constants for isBabyCrying
const double BABY_CRY_FREQ_MIN = 250.0; // Minimum frequency for baby cry (in Hz)
const double BABY_CRY_FREQ_MAX = 550.0; // Maximum frequency for baby cry (in Hz)
const double INTENSITY_THRESHOLD = 2000.0; // Minimum intensity to consider
const int MATCH_THRESHOLD = 100; // Number of matches within the frequency range to consider as baby crying

double vReal[SAMPLES];
double vImag[SAMPLES];
ArduinoFFT FFT(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);

double readDecibelLevel() {
    int analogValue = analogRead(SOUND_SENSOR_PIN);
    return double(analogValue); // Convert analog value to double
}

bool detectSoundPattern() {
    double lastIntensity = 0.0;
    int spikeCount = 0;
    bool isSpike = false;

    for (int i = 0; i < SAMPLES; i++) {
        vReal[i] = analogRead(SOUND_SENSOR_PIN);
        double currentIntensity = vReal[i];

        if (!isSpike && currentIntensity > lastIntensity + SPIKE_THRESHOLD) {
            isSpike = true;
        } else if (isSpike && currentIntensity < lastIntensity) {
            spikeCount++;
            isSpike = false;
        }
        lastIntensity = currentIntensity;
        delay(20);
    }

    if (DEBUG){
      Serial.print("Spike Count: ");
      Serial.println(spikeCount);
    }
    

    return spikeCount >= MIN_SPIKES && spikeCount <= MAX_SPIKES;
}

bool isBabyCrying(double frequencies[], double intensities[], int count) {
    int targetFrequencyMatches = 0;
    for (int i = 0; i < count; i++) {
        if (frequencies[i] >= BABY_CRY_FREQ_MIN && frequencies[i] <= BABY_CRY_FREQ_MAX &&
            intensities[i] > INTENSITY_THRESHOLD) {
            if (DEBUG){
              //Print frequency and intensity
              Serial.print("Frequency: ");
              Serial.print(frequencies[i]);
              Serial.print("Intensity: ");
              Serial.println(intensities[i]);
            }
            targetFrequencyMatches++;
        }
    }

    if(DEBUG){
      Serial.print("Target Frequency Matches: ");
      Serial.println(targetFrequencyMatches);
    }
    
    return targetFrequencyMatches > MATCH_THRESHOLD;
}


void babyCryDetector() {
  if(babyPresent){
    double decibelLevel = readDecibelLevel();

    if (DEBUG){
      Serial.print("Decibel Level: ");
      Serial.println(decibelLevel);
    }
  
    if (decibelLevel > SOUND_THRESHOLD) {
        if (detectSoundPattern()) {
            // Perform FFT and intensity analysis
            FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
            FFT.compute(FFT_FORWARD);
            FFT.complexToMagnitude();

            double frequencies[SAMPLES / 2];
            double intensities[SAMPLES / 2];
            for (int i = 0; i < SAMPLES / 2; i++) {
                frequencies[i] = (i * SAMPLING_FREQUENCY) / SAMPLES;
                intensities[i] = vReal[i];
            }
            if (isBabyCrying(frequencies, intensities, SAMPLES / 2)) {
                //server.send(200, "text/plain", "Baby is crying");
                sendJsonResponse("Baby is crying",true);
                babyCryDetected = true;
                servoActive = true;
                if(DEBUG){
                  Serial.println("Baby is crying");
                }
            } else {
                //server.send(200, "text/plain", "No baby crying detected.");
                sendJsonResponse("No baby crying detected.", true);
                babyCryDetected = false;
                servoActive = false;
                if(DEBUG){
                  Serial.println("No baby crying detected.");
                }
            }
        } else {
            if (DEBUG){
              Serial.println("No significant crying pattern detected.");
            }
            babyCryDetected = false;
            servoActive = false;
        }
    }
    delay(1000); // Wait for a second before next reading
  }
}

// Function to send JSON response
void sendJsonResponse(String message, bool status) {
  StaticJsonDocument<200> doc;
  doc["message"] = message;
  doc["status"] = status;

  String jsonResponse;
  serializeJson(doc, jsonResponse);

  server.send(200, "application/json", jsonResponse);
}

// Initialisation des composants
void setup() {

  // Initialisation
  Serial.begin(115200);
  Serial.println("Système prêt à recevoir des commandes !");

  // Initialisation servomoteur
  monServo.attach(servoPin);
  monServo.write(90);  // Position initiale au centre

  // Initialisation LEDs et capteurs
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(PIR_PIN, INPUT);
  pinMode(SOUND_SENSOR_PIN, INPUT); 

  // Connect to Wi-Fi
  Serial.println("Connecting to Wi-Fi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected.");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Configure HTTP server routes
  server.on("/servo", HTTP_GET, gererServo);         // Control servo
  server.on("/led", HTTP_GET, gererLEDs);            // Control LEDs
  server.on("/sound", HTTP_GET, toggleSoundSensor);  // Toggle sound sensor
  server.on("/movement", HTTP_GET, toggleMovementSensor); // Toggle movement sensor
  server.on("/status", HTTP_GET, sendStatus);        // Send device status

  // Start the server
  server.begin();
  Serial.println("HTTP server started.");
}

void activerServo(){
  servoActive = true;
  //server.send(200, "text/plain", "Servo activated.");
  sendJsonResponse("Servo activated.", true);
  if(DEBUG){
    Serial.println("Servo activated.");
  }
}

void desactiverServo(){
  servoActive = false;
  monServo.write(90);  // Revenir au centre
  //server.send(200, "text/plain", "Servo deactivated.");
  sendJsonResponse("Servo deactivated.", true);
  if(DEBUG){
    Serial.println("Servo deactivated.");
  }
}

void swing(){
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

// Fonction pour gerer le servomoteur
void gererServo() {
  String command = server.arg("action");
  if (command == "activate") { // Activer le servo
    activerServo();
  } else if (command == "deactivate") { // Désactiver le servo
    desactiverServo();
  } else {
    //server.send(400, "text/plain", "Invalid command.");
    sendJsonResponse("Invalid command", false);
  }
}

// Fonction pour gérer l'état des LEDs
// void gererLEDs() {
//   String led = server.arg("led");
//   if (led == "blue") {
//     digitalWrite(LED1, !digitalRead(LED1));
//     server.send(200, "text/plain", "LED 1 toggled.");
//   } else if (led == "white") {
//     digitalWrite(LED2, !digitalRead(LED2));
//     server.send(200, "text/plain", "LED 2 toggled.");
//   } else if (led == "animationON") {  // Animation des LEDs
//     ledAnimationActive = true;
//     server.send(200, "text/plain", "LED Animation ON.");
//     while (ledAnimationActive) {
//       digitalWrite(LED1, HIGH);
//       digitalWrite(LED2, LOW);
//       delay(500);
//       digitalWrite(LED1, LOW);
//       digitalWrite(LED2, HIGH);
//       delay(500);
//       String led = server.arg("led");
//       if (led == "animationOFF") {
//         ledAnimationActive = false;
//         server.send(200, "text/plain", "LED Animation OFF.");
//         break;  // Arrêter l'animation
//       }
//     }
//     digitalWrite(LED1, LOW);
//     digitalWrite(LED2, LOW);
//   }

// }

void gererLEDs() {
  String led = server.arg("led");
  if (led == "blue") {
    digitalWrite(LED1, !digitalRead(LED1));
    //server.send(200, "text/plain", "LED 1 toggled.");
    sendJsonResponse("LED 1 toggled.", true);
  } else if (led == "white") {
    digitalWrite(LED2, !digitalRead(LED2));
    //server.send(200, "text/plain", "LED 2 toggled.");
    sendJsonResponse("LED 2 toggled.", true);
  } else if (led == "animationON") {
    ledAnimationActive = true;
    animationStartMillis = millis(); // Start animation timer
    //server.send(200, "text/plain", "LED Animation ON.");
    sendJsonResponse("LED Animation ON.", true);
  } else if (led == "animationOFF") {
    ledAnimationActive = false;
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);
    //server.send(200, "text/plain", "LED Animation OFF.");
    sendJsonResponse("LED Animation OFF.", true);

  }
}

// Function to toggle the sound sensor
void toggleSoundSensor() {
  capteurSonActif = !capteurSonActif;
  if(DEBUG){
    Serial.println(capteurSonActif ? "Sound sensor activated." : "Sound sensor deactivated.");
  }
  //server.send(200, "text/plain", capteurSonActif ? "Sound sensor activated." : "Sound sensor deactivated.");
  sendJsonResponse(capteurSonActif ? "Sound sensor activated" : "Sound sensor deactivated", capteurSonActif);
}

// Function to toggle the movement sensor
void toggleMovementSensor() {
  capteurMouvementActif = !capteurMouvementActif;
  if(DEBUG){
    Serial.println(capteurMouvementActif ? "Movement sensor activated." : "Movement sensor deactivated.");
  }
  //server.send(200, "text/plain", capteurMouvementActif ? "Movement sensor activated." : "Movement sensor deactivated.");
  sendJsonResponse(capteurMouvementActif ? "Movement sensor activated" : "Movement sensor deactivated", capteurMouvementActif);
}


// Fonction pour surveiller le capteur PIR
void surveillerCapteurPIR() {
  int etatPIR = digitalRead(PIR_PIN);
  if (etatPIR == LOW && !babyPresent) {
    //server.send(200, "text/plain", "Mouvement détecté : Bébé présent !");
    sendJsonResponse("Mouvement détecté : Bébé présent !", true);
    babyPresent = true;
    dernierEnvoi = millis();
    if(DEBUG){
      Serial.println("Mouvement détecté : Bébé présent !");
    }
  } else if (etatPIR == HIGH && babyPresent && millis() - dernierEnvoi > delaiEnvoi) {
    //server.send(200, "text/plain", "Aucune présence détectée.");
    sendJsonResponse("Aucune présence détectée.", true);
    babyPresent = false;
    desactiverServo();
    dernierEnvoi = millis();
    if(DEBUG){
      Serial.println("Aucune présence détectée.");
    }
  }
   delay(100);
}

//Send status to the server
// void sendStatus() {
//   String status = "Servo: " + String(servoActive ? "Active" : "Inactive") + "\n" +
//                   "LED1: " + String(digitalRead(LED1)==0 ? "ON" : "OFF") + "\n" +
//                   "LED2: " + String(digitalRead(LED2)==0 ? "ON" : "OFF") + "\n" +
//                   "Sound Sensor: " + String(capteurSonActif ? "Active" : "Inactive") + "\n" +
//                   "Movement Sensor: " + String(capteurMouvementActif ? "Active" : "Inactive") + "\n" + 
//                   "Baby Cry : " + String(babyCryDetected ? "Baby crying" : "No baby cry") + "\n" + 
//                   "Baby Presence : " + String(babyPresent ? "Baby present" : "No baby present") + "\n" ;
//   server.send(200, "text/plain", status);
// }

void sendStatus() {
  // Create a JSON document
  StaticJsonDocument<200> jsonDoc;

  // Populate the JSON object
  jsonDoc["Servo"] = servoActive ? "Active" : "Inactive";
  jsonDoc["LED1"] = digitalRead(LED1) == 0 ? "ON" : "OFF";
  jsonDoc["LED2"] = digitalRead(LED2) == 0 ? "ON" : "OFF";
  jsonDoc["Sound Sensor"] = capteurSonActif ? "Active" : "Inactive";
  jsonDoc["Movement Sensor"] = capteurMouvementActif ? "Active" : "Inactive";
  jsonDoc["Baby Cry"] = babyCryDetected ? "Baby crying" : "No baby cry";
  jsonDoc["Baby Presence"] = babyPresent ? "Baby present" : "No baby present";

  // Serialize JSON to a string
  String response;
  serializeJson(jsonDoc, response);

  // Send the response as JSON
  server.send(200, "application/json", response);
}

// Boucle principale
void loop() {
  // Handle HTTP requests
  server.handleClient();

  if (ledAnimationActive) {
    unsigned long currentMillis = millis();
    if (currentMillis - animationStartMillis >= animationInterval) {
      animationStartMillis = currentMillis;

      // Toggle LEDs
      static bool toggleState = false;
      toggleState = !toggleState;
      digitalWrite(LED1, toggleState ? HIGH : LOW);
      digitalWrite(LED2, toggleState ? LOW : HIGH);
    }
  }

  if (capteurSonActif) babyCryDetector();
  if (capteurMouvementActif) surveillerCapteurPIR();
  if (servoActive) swing();
}
