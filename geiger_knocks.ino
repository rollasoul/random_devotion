/*
**** random_devotion ****
based on http://www.arduino.cc/en/Tutorial/ButtonStateChange and https://learn.adafruit.com/secret-knock-activated-drawer-lock/code
*/

// this constant won't change:
const int  buttonPin = 12;    // the pin that the pushbutton is attached to
const int ledPin = 13;       // the pin that the LED is attached to
const int maximumKnocks = 20;      // Maximum number of knocks to listen for.
const int maximumHumanKnocks = 20;

const int knockSensor = A1; // listens to piezo-knocks
const int threshold = 3; // sensitivity of knock-sensor

// Variables will change:
int buttonPushCounter = 0;   // counter for the number of button presses
int buttonPushCounterOld = 0;

int knockSensorValue = 0; // counter for human knocks
int lastKnockSensorValue = 0;

int knockCounter = 0;
int knockCounterOld = 0;

int buttonState = 0;         // current state of the button
int lastButtonState = 0;     // previous state of the button
int startTime = millis();
int endTime = millis();
int betweenTime = endTime - startTime;
int knockReadings[maximumKnocks];


int endKnocksTime = millis();
int startKnocksTime = millis();
int betweenKnocksTime = endKnocksTime - startKnocksTime;
int knockHumanReadings[maximumHumanKnocks];

void setup() {
  // initialize the button pin as a input:
  pinMode(buttonPin, INPUT);
  // initialize the LED as an output:
  pinMode(ledPin, OUTPUT);
  // initialize serial communication:
  Serial.begin(9600);
}


void loop() {
  // read the pushbutton input pin
  buttonState = digitalRead(buttonPin);

  //read piezo for knocks
  knockSensorValue = analogRead(knockSensor);

  ////////// listen to human knockSensor ///////////
  if (knockSensorValue != lastKnockSensorValue) {
    // if the state has changed, increment the counter
    if (knockSensorValue > threshold) {
      // if the current state is HIGH then the button went from off to on:
      knockCounter++;
      //startTime = millis();
      if (knockCounter > knockCounterOld) {
        endKnocksTime = millis();
        betweenKnocksTime = endKnocksTime - startKnocksTime;
        knockHumanReadings[knockCounter - 1] = betweenKnocksTime;
        startKnocksTime = millis();
      }
      //Serial.println("on");
      Serial.print("absolute number of knocks: ");
      Serial.println(knockCounter);
      Serial.println("time difference between knocks: ");
      Serial.println(betweenTime);
      Serial.println(sizeof(knockHumanReadings));
    } else {
      // if the current state is LOW then the button went from on to off:
      //Serial.println("off");
      knockCounterOld = knockCounter;

    }
    // Delay a little bit to avoid bouncing
    delay(50);
    if (knockCounter ==  maximumHumanKnocks) {
      for (int i = 0; i < maximumHumanKnocks; i++) {
        Serial.println(knockHumanReadings[i]);
      }
      for (int i=0; i < maximumKnocks; i++){
        knockReadings[i] = 0;
      }
      knockCounter = 0;
    }
  }
  // save the current state as the last state, for next time through the loop
  lastKnockSensorValue = knockSensorValue;


  ////////// listen to geiger counter //////////////

  // compare the buttonState to its previous state
  if (buttonState != lastButtonState) {
    // if the state has changed, increment the counter
    if (buttonState == HIGH) {
      // if the current state is HIGH then the button went from off to on:
      buttonPushCounter++;
      //startTime = millis();
      if (buttonPushCounter > buttonPushCounterOld) {
        endTime = millis();
        betweenTime = endTime - startTime;
        knockReadings[buttonPushCounter - 1] = betweenTime;
        startTime = millis();
      }
      //Serial.println("on");
      Serial.print("absolute number of particles decayed: ");
      Serial.println(buttonPushCounter);
      Serial.println("time difference between particles: ");
      Serial.println(betweenTime);
      Serial.println(sizeof(knockReadings));
    } else {
      // if the current state is LOW then the button went from on to off:
      //Serial.println("off");
      buttonPushCounterOld = buttonPushCounter;

    }
    // Delay a little bit to avoid bouncing
    delay(50);
    if (buttonPushCounter ==  maximumKnocks) {
      for (int i = 0; i < maximumKnocks; i++) {
        Serial.println(knockReadings[i]);
      }
      for (int i=0; i < maximumKnocks; i++){
        knockReadings[i] = 0;
      }
      buttonPushCounter = 0;
    }
  }
  // save the current state as the last state, for next time through the loop
  lastButtonState = buttonState;


  // turns on the LED every four button pushes by checking the modulo of the
  // button push counter. the modulo function gives you the remainder of the
  // division of two numbers:
  if (buttonPushCounter % 4 == 0) {
    digitalWrite(ledPin, HIGH);
  } else {
    digitalWrite(ledPin, LOW);
  }

}
