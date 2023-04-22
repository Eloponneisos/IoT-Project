#define lineSensor1 8
#define lineSensor2 7
#define lineSensor3 6
#define lineSensor4 5
#define lineSensor5 4

#define collisionSensor 3

#define distanceSensor 2

bool value[6] = { false, false, false, false, false, false };

int distance = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("REBOOT");

  pinMode(lineSensor1, INPUT);
  pinMode(lineSensor2, INPUT);
  pinMode(lineSensor3, INPUT);
  pinMode(lineSensor4, INPUT);
  pinMode(lineSensor5, INPUT);

  pinMode(collisionSensor, INPUT);

  pinMode(distanceSensor, INPUT);

  Serial.println("END BOOT");
}

void loop() {
  /*if (lineSensor1) {
    value[0] = true;
  } else {
    value[0] = false;
  }

  if (lineSensor2) {
    value[1] = true;
  } else {
    value[1] = false;
  }

  if (lineSensor3) {
    value[2] = true;
  } else {
    value[2] = false;
  }

  if (lineSensor4) {
    value[3] = true;
  } else {
    value[3] = false;
  }

  if (lineSensor5) {
    value[4] = true;
  } else {
    value[4] = false;
  }

  if (collisionSensor) {
    value[5] = true;
  } else {
    value[5] = false;
  }*/

  Serial.print(!digitalRead(lineSensor1));
  Serial.print(" | ");

  Serial.print(!digitalRead(lineSensor2));
  Serial.print(" | ");

  Serial.print(!digitalRead(lineSensor3));
  Serial.print(" | ");

  Serial.print(!digitalRead(lineSensor4));
  Serial.print(" | ");

  Serial.print(!digitalRead(lineSensor5));
  Serial.print(" | ");

  Serial.print(digitalRead(collisionSensor));
  Serial.print(" | ");

  distance = !digitalRead(distanceSensor);

  Serial.println(distance);

  /*for (int i = 0; i < 6; i++) {
    Serial.print(value[i]);
    Serial.print(" | ");
  }*/

}
