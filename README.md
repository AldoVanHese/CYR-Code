# CYR-Code

Code voor Touchbox:

```C++
#include <Wire.h>
#include <Adafruit_MPR121.h>
#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>

Adafruit_MPR121 cap = Adafruit_MPR121();

// Let op: DFPlayer TX → pin 11, RX ← pin 10
SoftwareSerial mySerial(10, 11); // RX, TX
DFRobotDFPlayerMini myDFPlayer;
bool dfPlayerOk = false;

uint16_t previousState = 0; // Houdt vorige aanraakstatus bij

void setup() {
    Serial.begin(9600);
    mySerial.begin(9600);

    if (!cap.begin(0x5B)) { // Controleer het I2C-adres (kan ook 0x5A of 0x5C zijn)
        Serial.println("MPR121 niet gevonden!");
        while (1);
    }
    Serial.println("MPR121 gestart 🤖");

    dfPlayerOk = myDFPlayer.begin(mySerial);
    if (!dfPlayerOk) {
        Serial.println("DFPlayer Mini niet gevonden!");
    } else {
        myDFPlayer.volume(25);  // Volume 0–30
        Serial.println("DFPlayer gestart 🤖");
    }
}

void loop() {
    uint16_t currentState = cap.touched(); // Lees de huidige status

    for (uint8_t i = 0; i < 12; i++) {
        bool previousTouch = (previousState & (1 << i));
        bool currentTouch = (currentState & (1 << i));

        if (currentTouch && !previousTouch) { // Alleen een nieuwe aanraking registreren als de toets nu wordt aangeraakt maar eerder niet
            if (isMetalTouched()) {
                Serial.print("TOUCH_");
                Serial.println(i);

                if (dfPlayerOk) {
                    myDFPlayer.play(i + 1);  // Speel bestand 0001.mp3 t/m 0012.mp3
                } else {
                    Serial.println("DFPlayer niet actief, geen geluid afgespeeld.");
                }
            }
        }
    }

    previousState = currentState; // Update vorige status
    //delay(100); // Optioneel: afvlakken van input
}

bool isMetalTouched() {
    delay(5);  // Stabilisatie
    uint16_t secondCheck = cap.touched();
    return secondCheck != 0;
}
```


Code Boombox:
Geschreven voor Arduino IDE
```C++
/*
 * PIN AANSLUITINGEN
 * GND --> GND (zwart)
 *  +  --> 5V  (rood)
 * SW  --> 12  (geel)
 * DT  --> 3   (groen, data)
 * CLK --> 2   (blauw, clock)
 */

int aantalClicks = 3;
int i = 1;

// Rotary encoder pinnen
const int pinA = 2;
const int pinB = 3;
volatile int pinAstateCurrent = LOW;
volatile int pinAStateLast = LOW;
int countLinks = 0;
int countRechts = 0;

// Schakelaar
const int switchPin = 12;
int switchState = HIGH;

// Double-click detectie variabelen
unsigned long lastClickTime = 0;
bool wachtOpTweedeKlik = false;
const int doubleClickTijd = 250;  // Strengere double-click tijd (ms)
bool debounceActief = false;
unsigned long debounceTimer = 0;
const int debounceTijd = 50;

// Toggle vlag: als deze true is, worden schakelaarIngedrukt én encoder (Links/Rechts) geactiveerd
bool enkeleKlikActief = false;

void setup() {
  Serial.begin(115200);

  pinMode(switchPin, INPUT_PULLUP);
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);

  // Interrupt aan encoder PinA
  attachInterrupt(digitalPinToInterrupt(pinA), update, CHANGE);
}

void loop() {
  switchState = digitalRead(switchPin);
  unsigned long huidigeTijd = millis();

  // Schakelaar-debounce
  if (debounceActief && (huidigeTijd - debounceTimer > debounceTijd)) {
    debounceActief = false;
  }

  if (switchState == LOW && !debounceActief) {
    debounceActief = true;
    debounceTimer = huidigeTijd;

    if (!wachtOpTweedeKlik) {
      // Eerste klik: start timer
      wachtOpTweedeKlik = true;
      lastClickTime = huidigeTijd;
    } else {
      // Tweede klik binnen doubleClickTijd → toggle modus
      if (huidigeTijd - lastClickTime <= doubleClickTijd) {
        // Toggle de actieve modus; als deze true is, worden encoder- en schakelaarhandelingen uitgevoerd
        enkeleKlikActief = !enkeleKlikActief;
        Serial.println("dubbelSchakelaarIngedrukt");
        wachtOpTweedeKlik = false;
      }
    }
    // Wacht tot de schakelaar weer losgelaten is
    while (digitalRead(switchPin) == LOW)
      ;
  }

  // Als er een enkele klik is gedetecteerd en de doubleclick tijd verstreken is
  if (wachtOpTweedeKlik && (huidigeTijd - lastClickTime > doubleClickTijd)) {
    // Voer de actie alleen uit als de modus actief is
    if (enkeleKlikActief) {
      switch (i) {
        case 1: Serial.println("schakelaarIngedrukt1"); break;
        case 2: Serial.println("schakelaarIngedrukt2"); break;
        case 3: Serial.println("schakelaarIngedrukt3"); break;
        case 4: Serial.println("schakelaarIngedrukt4"); break;
      }
    }
    wachtOpTweedeKlik = false;
  }
}

void update() {
  pinAstateCurrent = digitalRead(pinA);

  if ((pinAStateLast == LOW) && (pinAstateCurrent == HIGH)) {
    if (digitalRead(pinB) == HIGH) {
      countLinks++;
    } else {
      countRechts++;
    }

    // Als voldoende linker pulsen geteld zijn
    if (countLinks >= aantalClicks) {
      // Alleen verwerken als toggle actief is en er binnen de grenzen gegaan kan worden
      if (enkeleKlikActief && (i > 1)) {
        i--;
        Serial.println("Links");
      }
      countLinks = 0;
      countRechts = 0;
    }

    // Als voldoende rechter pulsen geteld zijn
    if (countRechts >= aantalClicks) {
      // Alleen verwerken als toggle actief is en er binnen de grenzen gegaan kan worden
      if (enkeleKlikActief && (i < 4)) {
        i++;
        Serial.println("Rechts");
      }
      countLinks = 0;
      countRechts = 0;
    }
  }

  pinAStateLast = pinAstateCurrent;
}
```

Connectieschema Momenteel:
<p align="left">
  <img src="Pictures/Schema Boombox 1.png" width="33%">
  <img src="Pictures/Schema Touchbox 1.png" width="33%">


Nadeel:
-     Laptop vereist
-     Wifi vereist
 
<p align="left">
  <img src="Pictures/Schema Boombox 2.png" width="50%">
