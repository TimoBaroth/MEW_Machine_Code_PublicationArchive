

// constants won't change. They're used here to set pin numbers:
const int LS1 = 22;  // light switch 1
const int LS2 = 23;    // light switch 2
const int LS3 = 24;    // light switch 3
const int LS4 = 25;    // light switch 4
const int IN1 = 27;    // microtic opto click 3, input 1
const int IN2 = 26;    // microtic opto click 3, input 2
const int OUT1 = 29;   // microtic opto click 3, Output 1
const int OUT2 = 28;   // microtic opto click 3, Output 2

// variables will change:
int buttonState = 0;  // variable for reading the pushbutton status

void setup() {
  // initialize pins
  pinMode(LS1, INPUT_PULLUP);
  pinMode(LS2, INPUT_PULLUP);
  pinMode(LS3, INPUT_PULLUP);
  pinMode(LS4, INPUT_PULLUP);
  pinMode(IN1, INPUT);
  pinMode(IN2, INPUT);  
  pinMode(OUT1, OUTPUT);
  pinMode(OUT2, OUTPUT);

  digitalWrite(OUT1, LOW);
  digitalWrite(OUT2, LOW);
}

void loop() {
  static int state = 0; // state variable for flow of program 
  int LS_status = 0; // status variable for overall light switch status, 0 = all off/not triggerd, 1 = at least one on/triggerd 

  if (digitalRead(LS1) == HIGH || digitalRead(LS2) == HIGH || digitalRead(LS3) == HIGH || digitalRead(LS4) == HIGH) {
        LS_status = 1; // if any light switch triggerd, save in variable
    }


  switch (state) {
    case 0: //initial state 
      if (LS_status == 0) { // if no light switch is triggerd, go to state 1, normal operation 
        state = 1;
      } 
      else {               // if a light switch is triggerd, go to state 2, first error state 
        state = 2;      
      }
      break;

    case 1: // no light switch triggerd 
      digitalWrite(OUT2, HIGH); // turn on output to allow for power reset 
      digitalWrite(OUT1, LOW); // turn off error output 
      if (LS_status == 1) { // if light switch is triggerd, go to state 2, first error state 
        state = 2;
      }
      break;

    case 2: // process error routine 
      digitalWrite(OUT2, LOW); // turn off output to shut off motors 
      digitalWrite(OUT1, HIGH); // turn on error output to signal error condition
      delay(5000); // wait 5s before reset is allowed 
      digitalWrite(OUT2, HIGH); // allow reset of motor supply 
      state = 3; 
      break;

    case 3: // wait for error to be resolved to go back to normal operation 
      if (LS_status == 0){  // if no light switch is triggerd anymore, go back to normal operation
        state = 1;
      }
      break;     
  }
}
