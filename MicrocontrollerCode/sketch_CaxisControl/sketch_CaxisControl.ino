#include <AccelStepper.h>
#include <ArduinoJson.h>
#include <AsyncServoLib.h>

// C-Axis Motor 
const int stepPin = 7;
const int directionPin = 6;
const int enablePin = 5;


// Motor max speed and acceleration 
const int maxSpeed = 20000;
float maxAccelC = 1000;
float lastAccelC = maxAccelC;


// Stepper motot control objects
AccelStepper stepper(AccelStepper::DRIVER, stepPin, directionPin);

//Digital output 
const int select = 3; //select between fluidnc and MCU stepper control 

// variables to hold input data
JsonDocument doc;
float C_speed = 0;
boolean newData = false;
boolean newacc = false;
boolean newposspeed = false;



void setup() {
  // put your setup code here, to run once:
   Serial.begin(115200);
   delay(500); //give it some time to init serial
  // C-Axis
   stepper.setEnablePin(enablePin);
   stepper.setPinsInverted(true,false,true);
   stepper.enableOutputs();
   stepper.setMaxSpeed(maxSpeed);
   stepper.setAcceleration(maxAccelC);
   stepper.setSpeed(0);

  // slect output 
  pinMode(select, OUTPUT);
  digitalWrite(select, LOW); // set low => select fluidnc as default

  // wait shortly to get everyhing ready
  delay(250);


}


void loop() {
  // adapted from: https://arduinojson.org/v6/how-to/do-serial-communication-between-two-boards/
  if (Serial.available()){
    StaticJsonDocument<300> doc;

    DeserializationError err = deserializeJson(doc, Serial);
    
    if (err == DeserializationError::Ok){
      if (doc.containsKey("c_axis_acc")) { 
        maxAccelC = doc["c_axis_acc"];
        stepper.setAcceleration(maxAccelC);
      }

      if (doc.containsKey("c_axis_speed")) { 
        C_speed = doc["c_axis_speed"];
        stepper.setSpeed(C_speed);
        if (C_speed != 0){
          digitalWrite(select, HIGH); // select/switch steppermotor to MCU control  
        } else {
          digitalWrite(select, LOW); // select/switch steppermotor to FluidNC control  
        }
      }
    }
  }

  // call stepper functions to update their outputs/steps 
  stepper.runSpeed();
//Serial.pri;ntln("loop");
}


