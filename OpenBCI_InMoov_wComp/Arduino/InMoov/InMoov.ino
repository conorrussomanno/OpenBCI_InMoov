#include <Servo.h>

Servo servos[5];
int numFingers = 5;

int openThreshold[] = {
  40, //thumb
  15, //pointer
  10, //middle
  50, //ring
  10 //pinky
};
int closedThreshold[] = {
  180, //thumb
  175, //pointer
  180, //middle
  180, //ring
  170 //pinky
};

int pos[5]; //array of current finger positions ... this gets written to the hand
float normalizedPos;

int led = 3;
int targetValue = 0;
int actualValue = 0;
int fadeAmount = 5;    // how many points to fade the LED by
int addToValue = 0;
float actualValueFloat = 0.0;
float numberToHand = 0.0;

void setup() { 
  Serial.begin(115200);
//  for(int i = 0; i < numFingers; i++){
//    servos[i].attach(i + 2);
//    pos[i] = openThreshold[i]; //set all fingers to open by default
//  }
  servos[0].attach(A1);
  servos[1].attach(A2);
  servos[2].attach(A3);
  servos[3].attach(A4);
  servos[4].attach(A5);
  
  pinMode(led, OUTPUT);
} 


void loop() {            // Loop through motion tests
//  allOpen();           // Uncomment to use this
//  delay(4000);           // Uncomment to use this
//  allClosed();            // Uncomment to use this
//  delay(2000);           // Uncomment to use this
  

//======= Basic Open/Close Animation ======= //

//close hand
//  for(float i = 0.0; i < 1.0; i+=0.01){
//    setAllFingers(i);
//    Serial.println(i);
//    delay(20);
//  }
//  
//  delay(2000);
//  
//  //open hand
//  for(float i = 1.0; i > 0.0; i-=0.01){
//    setAllFingers(i); 
//    Serial.println(i);
//    delay(20);
//  }
//  delay(2000);
  
  


//------ Responding To OpenBCI EMG ------//
//
  addToValue = (int)(((float)targetValue - (float)actualValue)/10.0);
  actualValue += addToValue;

  if(actualValue > 255) actualValue = 255;
  if(actualValue < 0) actualValue = 0;
  
  analogWrite(led, actualValue);
  
//  actualValueFloat = (float)actualValue;
  numberToHand = float(map(actualValue, 0, 255, 0, 1000))/1000.0;
  
//  analogWrite(led, (int)numberToHand*255);
 
  setAllFingers(numberToHand);
  delay(20); 
// 
//------ 
//  setAllFingers(0);
//  delay(20);
   
//  setFingerDegree(4, 40);
//  delay(30);
  
//----- For Resetting Servos to 90 ----- //  
//  setAllServosTo(20);
//  delay(1000);
//  setAllServosTo(90);
//  delay(3000);
//  setAllServosTo(160);
//  delay(1000);
 
}

void serialEvent() {
  while (Serial.available()) {
    int newData = Serial.read();
    targetValue = (int)newData;
  }
}


// Motion to set the servo into "virtual" 0 position: alltovirtual
void allMiddle() {   
  for(int i = 0; i < numFingers; i++){
    servos[i].write(90);
  }
}
// Motion to set the servo into "rest" position: alltorest
void allOpen() {    
  for(int i = 0; i < numFingers; i++){
    servos[i].write(openThreshold[i]);
  }  
}
// Motion to set the servo into "max" position: alltomax
void allClosed() {
  for(int i = 0; i < numFingers; i++){
    servos[i].write(closedThreshold[i]);
  }  
}

void setAllFingers(float _normalizedPos){
  for(int i = 0; i < numFingers; i++){
    pos[i] = (int)((_normalizedPos * (closedThreshold[i] - openThreshold[i])) + openThreshold[i]);  //map the normalized value onto the full range or each finger
    servos[i].write(pos[i]);
  }
}

void setFinger (int _finger, float _normalizedPos){
  pos[_finger] = (int)((_normalizedPos * (closedThreshold[_finger] - openThreshold[_finger])) + openThreshold[_finger]);
  servos[_finger].write(pos[_finger]);
}

void setFingerDegree (int _finger, int _degree){
  servos[_finger].write(_degree);
}

void setAllServosTo(int _degree){  //set all servos to the middle
  for(int i = 0; i < numFingers; i++){  
    servos[i].write(_degree);
  }
}
