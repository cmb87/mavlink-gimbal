// Define pin connections & motor's steps per revolution
const int dirPin = 2;
const int stepPin = 4;
const int stepsPerRevolution = 500;
long incomingByte = 0; // for incoming serial data

int ctr = 0;

void setup()
{
  // Declare pins as Outputs
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  Serial.begin(115200);
}
void loop()
{

  if (Serial.available() > 0) {

    // read the incoming byte:
    incomingByte = Serial.read();
    // say what you got:

    Serial.print("I received: ");
    Serial.println(incomingByte);

  }



  //clockwise
  digitalWrite(dirPin, HIGH);

  if (ctr < 100) {
    incomingByte = 1000;
  } else {
    incomingByte = 4000;
  }

  

  // Spin motor
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(incomingByte);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(incomingByte);

  if (ctr > 200){
    ctr = 0;
  }
  ctr += 1;
}