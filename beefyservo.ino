#include <Servo.h>
Servo myservo;
int pos = 0;

void setup() {
  Serial.begin(460800);
  while (!Serial)
    ;
  Serial.println("Setting up BEEF servo");
  delay(10);
  myservo.attach(9);

  Serial.println("Type in angle with downscale of 1.5, ex if you want 135 degrees, input 90 or if you want 270 degrees, input 179");
}

void loop() {

  for (pos = 0; pos <= 180; pos += 1)
    if (Serial.available())

    {
      int state = Serial.parseInt();

      // if (state < 10)
      if (state >= 1 && state < 180) {
        Serial.print(">");
        Serial.println(state);
        Serial.print("turning servo to ");
        Serial.print(state * 1.5);
        Serial.println(" degrees");
        myservo.write(state);
      }
    }
}