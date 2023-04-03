/* "Roboo" - ENGR 1L Final Project
 * By Team 19: Ethan Sychangco, Tiffany Nguyen, Henrik Evers
 * Tuesday, March 21, 2023
 * 
 * The "Roboo" is a car-like robot inspired by the "Boo" character
 * from the Mario Bros. series.
 *
 * The robot starts in an all-off state.
 * After pushing the start button, the robot will react to light. 
 * When it is bright (shining a flashlight), it will act completely
 * still and rotate a servo to "hide its eyes". But when it is dark 
 * (no flashlight), it will make noise, show its red eyes, and drive 
 * forward.
 *
 * The "Roboo" has a game involved. The goal is stop the car on a 
 * specific win zone, without having it reach the end of the track
 * where a figurine stands. If either conditions are met, corresponding
 * effects will play and the robot will return to an all-off state.
 *
 * Outline of this Arduino Sketch:
 * 1. Definitions
 * 2. Setup
 * 3. Functions
 * 4. Main Logic
 */


//=============//
// DEFINITIONS //
//=============//

#include <Servo.h>                      // include the Servo library

Servo myservo1;                         // initialize first servo object
Servo myservo2;                         // initialize second servo object

int photoPin = A0;                      // pin A0 is for photoresistor sensing
int ledPin1 = 2;                        // pin 2 is first RED LED
int ledPin2 = 7;                        // pin 7 is second RED LED
int servoPin1 = 4;                      // pin 4 is first servo motor (arm)
int servoPin2 = 8;                      // pin 8 is second servo motor (arm)
int usTrigPin = 5;                      // pin 5 is ultrasonic sensor "trigger"
int usReadPin = 6;                      // pin 6 is ultrasonic sensor "reading"
int speakerPin = 10;                    // pin 10 is speaker 
int buttonPin = 3;                      // pin 3 is button
int motorPin = 11;                      // pin 11 is motor

enum notes {NOTE_C1 = 33, NOTE_E1 = 41, NOTE_D2 = 73, NOTE_E2 = 82, 
  NOTE_A2 = 110, NOTE_A3 = 220, NOTE_AS3 = 233, NOTE_C4 = 262,
  NOTE_A4 = 440, NOTE_AS4 = 466, NOTE_C5 = 523, NOTE_CS5 = 554,
  NOTE_E5 = 659, NOTE_F5 = 698, NOTE_G5 = 784, NOTE_A5 = 880,
  NOTE_C6 = 1047, NOTE_D6 = 1175, NOTE_F6 = 1397, NOTE_G6 = 1568,
  NOTE_A6 = 1760, NOTE_B6 = 1976, NOTE_C7 = 2093, NOTE_D7 = 2349,
  NOTE_DS7 = 2489, REST = 0};
enum notes noteEnum;
// Enum object holding selected values of "pitches.h" 
// [CREDIT]: Original pitch compilation by Brett Hagman

enum sounds {START_SOUND = 1, LOSE_SOUND = 2, WIN_SOUND = 3, AMBIENT_SOUND = 4};
enum sounds soundEnum;
// Enum object holding sound IDs

int start[] = {NOTE_E5,8, NOTE_E5,8, REST,8, NOTE_E5,8, REST,8, NOTE_C5,8, 
  NOTE_E5,8, REST,8, NOTE_G5,4, REST,4};

int lose[] = {NOTE_C5,16, NOTE_C5,16, REST,4, NOTE_CS5,8, NOTE_F6,8, 
  REST,8, NOTE_F6,8, NOTE_F6,-8, NOTE_D6,8, NOTE_C6,8, NOTE_A5,8, NOTE_F5,8, 
  REST,8, NOTE_E1,8, REST,16, NOTE_C1,8};

int win[] = {NOTE_F6,8, NOTE_G6,8, NOTE_A6,16, NOTE_B6,16, NOTE_C7,16,
  NOTE_D7,16, NOTE_DS7,2};

int ambience[] = {NOTE_C4,8, NOTE_C5,8, NOTE_A3,8, NOTE_A4,8, NOTE_AS3,8, 
  NOTE_AS4,8};
// Note charts for robot sound effects
// [CREDIT]: Super Mario Bros. Overworld, Underworld, 1UP, & Death by Koji Kondo.
// [CREDIT]: Charts by GitHub user "robsoncouto",
//   code repo: github.com/robsoncouto/arduino-songs/blob/master/supermariobros

int soundDebounce = 0;
// Variable used to ensure dark ambience sound
// only plays once per light state change

enum states {OFF = 0, DARK = 1, LIGHT = 2, WIN = 3, LOSE = 4};
enum states stateEnum;
// Enum object holding state IDs

int state = 0;                          
// State variable controls what the robot is doing

int winZones[] = {10, 11, 12};
// Specific ultrasonic distances, in centimeters,
// that result in a win stored in this array

#define MOTOR_SPEED 175
// "PWM cycle speed" that controls how fast our wheels rotate

#define LIGHT_THRESHOLD 600               
// "Analog input value" that is our threshold between DARK and LIGHT state

#define MIN_DIST 5
// Ultrasonic sensor should return higher than 5 centimeters,
// else it has hit the figurine

#define DELAY_LENGTH 250
// Engine loops every 250 milliseconds


//=======//
// SETUP //
//=======//

void setup(){
  pinMode(photoPin, INPUT);             // set the photoresistor as input
  pinMode(ledPin1, OUTPUT);             // set first LED as output
  pinMode(ledPin2, OUTPUT);             // set second LED as output
  pinMode(speakerPin, OUTPUT);          // set speaker as output
  pinMode(buttonPin, INPUT);            // set button as input  
  pinMode(usTrigPin, OUTPUT);           // set ultrasonic trigger pin as input
  pinMode(usReadPin, INPUT);            // set ultrasonic read pin as output
  pinMode(motorPin, OUTPUT);            // set motor pin as output

  myservo1.attach(servoPin1); 		      // attach servo pin to a servo object
  myservo1.write(0); 		                // servo begins with no movement
  myservo2.attach(servoPin2); 		      // attach servo pin to a servo object
  myservo2.write(0); 		                // servo begins with no movement

  Serial.begin(9600);                   // initialize serial for feedback
}


//===========//
// FUNCTIONS //
//===========//

void playAudio(int selectedAudio[], int size){
  /* Helper function called by "selectAudio()",
   * Contains logic/loop to play the notes in that function
   * [CREDIT]: robsoncouto
   */

  // change this to make the song slower or faster
  int tempo = 200;

  // sizeof gives the number of bytes, each int value is two bytes (16 bits)
  // two values per note (pitch & duration), for each note there are four bytes
  int notes = size / sizeof(selectedAudio[0]) / 2;

  // this calculates the duration of a whole note in ms
  int wholenote = (60000 * 4) / tempo;

  // base duration of a note, in ms
  int noteDuration = 5;

  // used to differentiate regular and dotted notes
  int divider;

  // play selected melody
  for (int thisNote = 0; thisNote < notes * 2; thisNote = thisNote + 2) {
    Serial.print("Note ");
    Serial.print(selectedAudio[thisNote]);
    Serial.print(" ");

    // calculates the duration of each note
    divider = selectedAudio[thisNote + 1];
    if (divider > 0) {
      // regular note, just proceed
      noteDuration = (wholenote) / divider;
    } else if (divider < 0) {
      // dotted notes are represented with negative durations
      noteDuration = (wholenote) / abs(divider);
      noteDuration *= 1.5; // increases the duration in half for dotted notes
    }

    // we only play the note for 90% of the duration, leaving 10% as a pause
    tone(speakerPin, selectedAudio[thisNote], noteDuration * 0.9);
  
    // wait for the specied duration before playing the next note
    delay(noteDuration);

    // stop the speaker before the next note
    noTone(speakerPin);
  }
}


void selectAudio(int which){
  /* Function: Holds the different sounds of the robot, when called it 
   * will invoke playAudio() to play the corresponding sound
   */
  
  switch (which) {    // call playAudio() with the corresponding note array
    case START_SOUND:
      playAudio(start, sizeof(start));
      break;
    case LOSE_SOUND:
      playAudio(lose, sizeof(lose));
      break;
    case WIN_SOUND:
      playAudio(win, sizeof(win));
      break;
    case AMBIENT_SOUND:
      playAudio(ambience, sizeof(ambience));
  }
}


int buttonCheck(){
  /* Function: Get button press value from pushbutton,
   * return corresponding "ALL-ON" or "ALL-OFF" state value
   */
  
  int buttonRead = digitalRead(buttonPin);      // read button pin (bool)

  if (buttonRead && state == 0){                // if: PRESS and CURRENTLY OFF? 
    Serial.println("[!] ALL ON");               // return the on state
    selectAudio(START_SOUND);                   // play start sound
    soundDebounce = 0;                          // allow playing ambience
    return 1;
  }
  else if (buttonRead && state != 0){           // if: PRESS and CURRENTLY ON?
    Serial.println("[!] ALL OFF");              // return the off state
    return 0;                         
  }
  else {                                        // else, NOT PRESSED
    return state;                               // do not change state
  }
}


int lightCheck(){
  /* Function: Get brightness value from photoresistor,
   * return corresponding "DARK" or "LIGHT" state value
   */
  
  int photoRead = analogRead(photoPin);         // read photoresistor pin (number)
  Serial.print("Light Value: ");
  Serial.println(photoRead);

  if (photoRead < LIGHT_THRESHOLD){             // if: DARK?
    Serial.println(">> NO LIGHT");              // return dark state
    if (!soundDebounce) {                       // if debounce permits,
      selectAudio(AMBIENT_SOUND);                     // play dark ambience sound
      soundDebounce = 1;
    }
    return 1;
  }
  else {                                        // else, LIGHT
    Serial.println(">> LIGHT");                 // return light state
    soundDebounce = 0;
    return 2;
  }
}


int distanceCheck(){
  /* Function: Get values from ultrasonic sensor, 
   * return a state value if hit corresponding to win/lose
   */
  
  // [CREDIT]: HowToMechatronics.com
  // Send a ultrasonic pulse
  digitalWrite(usTrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(usTrigPin, HIGH);
  delayMicroseconds(20);
  digitalWrite(usTrigPin, LOW);

  // Get distance value from pulse
  unsigned long duration = pulseIn(usReadPin, HIGH);  // time till pulse return
  unsigned long distance = (duration / 2) / 29.1;     // distance in centimeters
  Serial.print("Distance Value: ");
  Serial.println(distance);

  // Distance logic
  if (state == 1 && distance < MIN_DIST && distance > 0){  // if: DARK & CLOSE?
    Serial.println("[X] RIP MARIO");              // return game over state
    return 4;
  }
  else if (state == 2){                           // else, test for wins - loop
    for (int i = 0; i < sizeof(winZones) / sizeof(winZones[0]); i++){   
      if (distance == winZones[i]){               // if: WINZONE?
        Serial.println("[W] YOU ARE WINNER");     // return win state
        return 3;
      }
    }
  }  
  return state;                         // do not change state if no matches
}


void performActions(int state){
  /* Function: Control red LEDs, servo rotation, dark audio effects
   */
  
  if (state == 0){                      // if: ALL OFF?
    myservo1.write(90);                 // eyes off, hands neutral
    myservo2.write(90);                 // and don't drive
    digitalWrite(ledPin1, LOW);
    digitalWrite(ledPin2, LOW);
    analogWrite(motorPin, 0);
  }
  else if (state == 1){                 // else if: DARK?
    myservo1.write(0);                  // show eyes
    myservo2.write(180);                // and drive
    digitalWrite(ledPin1, HIGH);
    digitalWrite(ledPin2, HIGH);
    analogWrite(motorPin, MOTOR_SPEED);
  }
  else if (state == 2){                 // else if: LIGHT?
    myservo1.write(180);                // hide eyes
    myservo2.write(0);                  // and don't drive
    digitalWrite(ledPin1, LOW);
    digitalWrite(ledPin2, LOW);
    analogWrite(motorPin, 0);
  }
  else {                                // else, no change
    return;
  }
}


//============//
// MAIN LOGIC //
//============//

void loop(){
  // Roboo Sensing
  state = buttonCheck();                // "ALL-ON / ALL-OFF" test > pushbutton
  if (state > 0){                       // if: ALL-ON?
    state = lightCheck();               // "LIGHT / DARK" test > photoresistor
    state = distanceCheck();            // "WIN / LOSE" test > ultrasonic
  }
  
  // Roboo Actions
  performActions(state);
  
  // On any stop conditions
  if (state == 3){                      // if: WIN?
    selectAudio(WIN_SOUND);                     // play win noise
    state = 0;                          // set to all-off
    Serial.println("[!] ALL OFF");
  }
  if (state == 4){                      // if: LOSE? 
    selectAudio(LOSE_SOUND);                     // play game over noise
    state = 0;                          // set to all-off
    Serial.println("[!] ALL OFF");
  }

  // Delay before next loop
  delay(DELAY_LENGTH);
}
