/*
**** random_devotion ****
listens to a geiger counter and compares them with human knocks, knocks back if both match
based on http://www.arduino.cc/en/Tutorial/ButtonStateChange and https://learn.adafruit.com/secret-knock-activated-drawer-lock/code
*/

// this constant won't change:
const int  buttonPin = 12;    // the pin that the pushbutton is attached to
const int ledPin = 13;       // the pin that the LED is attached to
const int maximumKnocks = 5;      // Maximum number of geiger pulses to listen for
const int maximumHumanKnocks = 5; // max amount of human knocks to listen for

const int minmumMatchPerCent = 6;

const int knockSensor = A1; // listens to piezo-knocks
const int threshold = 13; // sensitivity of knock-sensor

// Variables will change:
int buttonPushCounter = 0;   // counter for the number of button presses
int buttonPushCounterOld = 0;

int knockSensorValue = 0; // counter for human knocks
int lastKnockSensorValue = 0;
int firstKnockValue = 0; // starts function that adds values to array from geiger counter

int knockCounter = 0;
int knockCounterOld = 0;

int buttonState = 0;         // current state of the button
int lastButtonState = 0;     // previous state of the button
unsigned long currentTime; //this variable will be overwritten by millis() each iteration of loop
//unsigned long pastTime     = 0; //no time has passed yet
unsigned long real_startTime = millis();
unsigned long absTime;
int startTime = absTime;
int endTime = absTime;
int betweenTime = endTime - startTime;
int knockReadings[maximumKnocks];


int endKnocksTime = absTime;
int startKnocksTime = absTime;
int betweenKnocksTime = endKnocksTime - startKnocksTime;
int knockHumanReadings[maximumHumanKnocks];

int matchCounter = 0;
unsigned long startKnocking = absTime;
int endKnocking = 0;
//set up a counter and timerfor the solenoid knocks-interval timing
int ucounter = 0;
unsigned long currentTimeInterval = 0;
unsigned long real_startTimeInterval;
unsigned long absTimeInterval = currentTimeInterval - real_startTimeInterval;

void setup() {
  unsigned long real_startTime = millis();
  // intitialize solenoid pin
  pinMode(4, OUTPUT);
  // initialize the button pin as a input:
  pinMode(buttonPin, INPUT);
  // initialize the LED as an output:
  pinMode(ledPin, OUTPUT);
  // initialize serial communication:
  Serial.begin(9600);
  // get some values in both arrays (pulses and knocks)
  for (int i = 0; i < 5; i++){
    knockReadings[i] = 0;
    knockHumanReadings[i] = 0;
  }
}


void loop() {
  // set an absolute time
  unsigned long currentTime = millis();
  unsigned long absTime = currentTime - real_startTime;
  //Serial.println(absTime);
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
      startTime = absTime;
      if (knockCounter > knockCounterOld) {
        endKnocksTime = absTime;
        betweenKnocksTime = (endKnocksTime - startKnocksTime) / 100;
        if (knockCounterOld == 0) {
          knockHumanReadings[knockCounter - 1] = betweenKnocksTime;
        }
        else { 
          knockHumanReadings[knockCounter - 1] = knockHumanReadings[knockCounter - 2] + betweenKnocksTime;  
        }
        startKnocksTime = absTime;
        firstKnockValue = 1;
      }
      //Serial.println("on");
      Serial.print("absolute number of knocks: ");
      Serial.println(knockCounter);
      Serial.println("time difference between knocks: ");
      Serial.println(betweenKnocksTime);
      //Serial.println(sizeof(knockHumanReadings));
    } else {
      // if the current state is LOW then the button went from on to off:
      //Serial.println("off");
      knockCounterOld = knockCounter;

    }
    // Delay a little bit to avoid bouncing
    //delay(50);
  }
  // save the current state as the last state, for next time through the loop
      if (knockCounter ==  maximumHumanKnocks || knockCounter >  maximumHumanKnocks) {
      Serial.println("time stamps for last 5 knocks and last 5 particles");
      for (int i = 0; i < maximumHumanKnocks; i++) {
        Serial.print(knockHumanReadings[i]); 
        Serial.print(" ");
        Serial.print(knockReadings[i]);
        if (buttonPushCounter > 0){
          if (knockHumanReadings[i] - knockReadings[i] > - 10 && knockHumanReadings[i] - knockReadings[i] <  10){
            Serial.println(" -> close match");
            matchCounter ++;
            }
          else {
            Serial.println("");
          }
        }
        else {
          Serial.println("");
        }
      }
      // overwrite array when maximum number of knocks or particles is reached
      for (int i=0; i < maximumKnocks; i++){
        knockReadings[i] = 0;
        knockHumanReadings[i] = 0;
      }
      knockCounter = 0;
      firstKnockValue = 0;
      buttonPushCounter = 0;
      Serial.println("CloseMatches:");
      Serial.println(matchCounter);
      if (matchCounter > 2) {
        Serial.println("You're in sync - the Stone will knock back at you"); 
        //Serial.println(absTime);
        //startKnocking = absTime;
        real_startTimeInterval = millis();
        while (absTimeInterval < 15000) {
          //Serial.println(absTimeInterval);
          //measure the time again
          currentTimeInterval = millis();
          absTimeInterval = currentTimeInterval - real_startTimeInterval;
          //Serial.println(endKnocking);
          if (digitalRead(12) == HIGH) {
            digitalWrite(4, HIGH);   // turn the LED on (HIGH is the voltage level)
            delay(100);                       // wait for a moment
            digitalWrite(4, LOW);
            delay(50);
          } 
          else {
            digitalWrite(4, LOW);    // turn the LED off by making the voltage LOW
          }
        }
        matchCounter=0;
        absTimeInterval = 0;
        //endKnocking = 0;
        //startKnocking = 0;
        //Serial.println("yo");
      }
      else {
        matchCounter = 0;
        Serial.println("yo");
      }
    }
  lastKnockSensorValue = knockSensorValue;


  ////////// listen to geiger counter //////////////

  // compare the buttonState to its previous state
  if (buttonState != lastButtonState && firstKnockValue == 1) {
    // if the state has changed, increment the counter
    if (buttonState == HIGH) {
      // if the current state is HIGH then the button went from off to on:
      buttonPushCounter++;
      //startTime = millis();
      if (buttonPushCounter > buttonPushCounterOld) {
        endTime = absTime;
        betweenTime = (endTime - startTime) / 100;
        if (buttonPushCounter == 0) {
          knockReadings[buttonPushCounter - 1] = betweenTime;
        }
        else {
          knockReadings[buttonPushCounter - 1] = knockReadings[buttonPushCounter - 2] + betweenTime;
        }
        startTime = absTime;
        digitalWrite(ledPin, HIGH);
      }
      //Serial.println("on");
      Serial.print("absolute number of particles decayed: ");
      Serial.println(buttonPushCounter);
      Serial.println("time difference between particles: ");
      Serial.println(betweenTime);
      //Serial.println(sizeof(knockReadings));
    } else {
      // if the current state is LOW then the button went from on to off:
      //Serial.println("off");
      buttonPushCounterOld = buttonPushCounter;
      digitalWrite(ledPin, LOW);
    }
    // Delay a little bit to avoid bouncing
    //delay(50);
//    if (buttonPushCounter ==  maximumKnocks) {
//      Serial.println("time stamps for last 5 knocks and last 5 particles");
//      for (int i = 0; i < maximumHumanKnocks; i++) {
//        Serial.print(knockHumanReadings[i]); 
//        Serial.print(" ");
//        Serial.print(knockReadings[i]);
//        if (knockHumanReadings[i] - knockReadings[i] > - 30 && knockHumanReadings[i] - knockReadings[i] <  30){
//          Serial.println(" -> close match");
//          matchCounter ++;
//        }
//        else {
//          Serial.println("");
//        }
//      }
//      buttonPushCounter = 0;
//      Serial.println("CloseMatches:");
//      Serial.println(matchCounter);
//      if (matchCounter > 0) {
//        matchCounter = 0;
//        Serial.println("You're in sync - the Stone will knock back at you"); 
//        startKnocking = millis();
//        endKnocking = startKnocking + 15000;
//        while (millis() < endKnocking) {
//          Serial.println("yo");
//          if (digitalRead(12) == HIGH) {
//            digitalWrite(4, HIGH);   // turn the LED on (HIGH is the voltage level)
//            delay(100);                       // wait for a moment
//            digitalWrite(4, LOW);
//            delay(50);
//          } 
//          else {
//            digitalWrite(4, LOW);    // turn the LED off by making the voltage LOW
//            //delay(50); // wait for a second
//          }
//        }
//      }
//      else {
//        matchCounter = 0;
//      }
//    }
  }
  // save the current state as the last state, for next time through the loop
  lastButtonState = buttonState;
}


  // turns on the LED every four button pushes by checking the modulo of the
  // button push counter. the modulo function gives you the remainder of the
  // division of two numbers:
//   if (buttonState == HIGH) {
//     digitalWrite(ledPin, HIGH);
//   } else {
//     digitalWrite(ledPin, LOW);
//   }
//
// }
