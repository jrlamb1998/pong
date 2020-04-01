//stepper matches displacement of potentiometer

//dip switch settings:
//1 down: 800 pulses per rotation
//1 and 2 down: 200 pulses per rotation (gets very hot, but is fastest)


//create pin values
int pinA0 = A0;
const int stepPin = 5; 
const int dirPin = 2; 
const int enPin = 8;

//initialize pot position values at zero
int old_pos = 0;
int new_pos = 0;
int delta_pos = 0;

//initialize other values
int noise_amp = 1;
int step_delay = 500; //delay in microseconds

//initialize pins
void setup(){
  Serial.begin(9600);
  
  pinMode(stepPin,OUTPUT);
  pinMode(dirPin,OUTPUT);
  pinMode(enPin,OUTPUT);
  
  digitalWrite(enPin,LOW);
}

void loop(){
  new_pos = analogRead(pinA0); //measure new position

  delta_pos = new_pos-old_pos; //calculate change in position
  Serial.println(delta_pos);

  //ignore changes in position less than/equal to the noise amplitude
  if (abs(delta_pos) <= noise_amp){
    delta_pos = 0;
  }

  //if change in position is positive, move CW
  if (delta_pos > 0){
    digitalWrite(dirPin,HIGH); //set direction CW

    //this loop makes delta_pos number of pulses, can be replaced with a PWM signal that runs for delta_pos cycles
    for (int x = 0; x < delta_pos; x++){
      digitalWrite(stepPin,HIGH); 
      delayMicroseconds(step_delay); 
      digitalWrite(stepPin,LOW); 
      delayMicroseconds(step_delay);
    }

    old_pos = new_pos; //reset position after moving
  }

  //if change in position is negative, move CCW
  else if (delta_pos < 0){
    digitalWrite(dirPin,LOW); //set direction CCW

    //this loop makes delta_pos number of pulses
    for (int x = 0; x < abs(delta_pos); x++){
      digitalWrite(stepPin,HIGH);
      delayMicroseconds(step_delay);
      digitalWrite(stepPin,LOW);
      delayMicroseconds(step_delay);
    }

    old_pos = new_pos; //reset position after moving
  }
}
