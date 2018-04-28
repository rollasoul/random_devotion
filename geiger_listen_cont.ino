/*
**** random_devotion ****
listens to a geiger counter and compares them with human knocks, knocks back if both match
based on http://www.arduino.cc/en/Tutorial/ButtonStateChange and https://learn.adafruit.com/secret-knock-activated-drawer-lock/code
*/

// this constant won't change:
const int  buttonPin = 12;    // the pin that the pushbutton is attached to
const int ledPin = 13;       // the pin that the LED is attached to

const int knockSensor = A1; // listens to piezo-knocks
const int threshold = 20; // sensitivity of knock-sensor

// Variables will change:
int buttonPushCounter = 0;   // counter for the number of button presses
int buttonPushCounterOld = 0;

int knockSensorValue = 0; // counter for human knocks
int lastKnockSensorValue = 0;

int knockCounter = 0;
int knockCounterOld = 0;

int buttonState = 0;         // current state of the button
int lastButtonState = 0;     // previous state of the button
unsigned long currentTime; //this variable will be overwritten by millis() each iteration of loop
//unsigned long pastTime     = 0; //no time has passed yet
unsigned long real_startTime = millis();
unsigned long absTime;
unsigned long knockReadings;
unsigned long knockHumanReadings;

// declare a few things to set up a time interval for the knocks of the solenoid
unsigned long  absTimeInterval = 0;
unsigned long  currentTimeInterval = 0;

int matchCounter = 0;

void setup() {
  unsigned long real_startTime = millis();
  // intitialize solenoid pin
  pinMode(4, OUTPUT);
  // initialize the button pin as a input:
  pinMode(buttonPin, INPUT);
  // initialize the LED as an output:
  pinMode(ledPin, OUTPUT);
  // initialize serial communication:
  Serial.begin(115200);
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
  // if we detect knock, dave it into array
  if (knockSensorValue != lastKnockSensorValue) {
    // if the state has changed, increment the counter
    if (knockSensorValue > threshold) {
      // if the current state is HIGH then the button went from off to on:
      knockCounter++;
      if (knockCounter > knockCounterOld) {
          knockHumanReadings = absTime;  
        }
      Serial.print("number of knock: ");
      Serial.println(knockCounter);
      Serial.println("time of knock: ");
      Serial.println(knockHumanReadings);
      //Serial.println(sizeof(knockHumanReadings));
    } 
    else {
      // if the current state is LOW then the button went from on to off:
      knockCounterOld = knockCounter;
    }
  }
  lastKnockSensorValue = knockSensorValue;

  ////////// listen to geiger counter //////////////

  // compare the buttonState to its previous state
  if (buttonState != lastButtonState) {
    // if the state has changed, increment the counter
    if (buttonState == HIGH) {
      // if the current state is HIGH then the button went from off to on:
      buttonPushCounter++;
      if (buttonPushCounter > buttonPushCounterOld) {
        knockReadings = absTime;
        //digitalWrite(ledPin, HIGH);
      }
      Serial.print("number of particle decayed: ");
      Serial.println(buttonPushCounter);
      Serial.println("time of decay of particle: ");
      Serial.println(knockReadings);
    } 
    else {
      // if the current state is LOW then the button went from on to off:
      //Serial.println("off");
      buttonPushCounterOld = buttonPushCounter;
      //digitalWrite(ledPin, LOW);
    }
  }
  // save the current state as the last state, for next time through the loop
  lastButtonState = buttonState;
//  Serial.println("difference: ");
//  Serial.println(knockReadings);
//  Serial.println(knockHumanReadings);
  /////////// compare the knocks and particles ////////////
  if (knockHumanReadings - knockReadings > - 1000){
    if (knockHumanReadings - knockReadings < 1000){
            Serial.println(" -> close match");
            Serial.println(knockHumanReadings - knockReadings);
            matchCounter ++;
            if (matchCounter > 0) {
              Serial.println("You're in sync - the Stone will knock back at you");
              Serial.println(knockReadings); 
              Serial.println(knockHumanReadings);
              //Serial.println(absTime);
              //startKnocking = absTime;
              unsigned long real_startTimeInterval = millis();
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
             }
      else {
        matchCounter = 0;
      }
  }
  }
 }

