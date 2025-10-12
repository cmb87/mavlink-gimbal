// Define pin connections & motor's steps per revolution
const int dirPin = 2;
const int stepPin = 4;
const int stepsPerRevolution = 500;
int stepDelay=2000;
void setup()
{
  // Declare pins as Outputs
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
}
void loop()
{
  //clockwise
  digitalWrite(dirPin, HIGH);
  // Spin motor
  for(int x = 0; x < stepsPerRevolution; x++)
  {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(stepDelay);
  }
  delay(1000); // Wait a second
  
  //counterclockwise
  digitalWrite(dirPin, LOW);
  // Spin motor
  for(int x = 0; x < stepsPerRevolution; x++)
  {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(stepDelay);
  }
  delay(1000); // Wait a second
}