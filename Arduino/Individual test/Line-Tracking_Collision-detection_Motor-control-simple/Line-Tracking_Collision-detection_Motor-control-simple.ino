#define left 10
#define right 9

#define lineSensorLL 8
#define lineSensorL 7
#define lineSensorC 6
#define lineSensorR 5
#define lineSensorRR 4

#define collisionSensor 3

int sensors[6] = { lineSensorLL, lineSensorL, lineSensorC, lineSensorR, lineSensorRR, collisionSensor };

String sensornames[7] = { "LL", "L", "C", "R", "RR", "Col", "Line" };

bool value[7] = { false, false, false, false, false, false, false };

void setup() {
  Serial.begin(115200);
  Serial.println("REBOOT");

  pinMode(left, OUTPUT);
  pinMode(right, OUTPUT);

  pinMode(lineSensorLL, INPUT);
  pinMode(lineSensorL, INPUT);
  pinMode(lineSensorC, INPUT);
  pinMode(lineSensorR, INPUT);
  pinMode(lineSensorRR, INPUT);

  pinMode(collisionSensor, INPUT);

  Serial.println("END BOOT");
}

void loop() {
  for (int i = 0; i < 6; i++) {
    value[i] = !digitalRead(sensors[i]);
  }

  value[5] = !value[5];

  for (int i = 0; i < 6; i++) {
    Serial.print(String(sensornames[i]));
    Serial.print(": ");
    Serial.print(value[i]);
    Serial.print(" | ");
  }
  Serial.println();

  if (value[5] == HIGH) {
    analogWrite(left, 0);
    analogWrite(right, 0);
    value[6] = false;
  }

  else if ((value[0] == HIGH) && (value[1] == HIGH) && (value[2] == HIGH) && (value[3] == HIGH) && (value[4] == HIGH) && (value[5] == LOW)) {
    analogWrite(left, 0);
    analogWrite(right, 0);
    Serial.println("Halt point reached");
    value[6] = false;
  }

  else if ((value[0] == LOW) && (value[1] == LOW) && (value[2] == HIGH) && (value[3] == LOW) && (value[4] == LOW) && (value[5] == LOW)) {
    analogWrite(left, 127);
    analogWrite(right, 127);
    value[6] = false;
  }

  else if (((value[0] == HIGH) || (value[1] == HIGH)) && (value[2] == LOW) && (value[3] == LOW) && (value[4] == LOW) && (value[5] == LOW)) {
    analogWrite(left, 0);
    if (value[0] == HIGH) {
      analogWrite(right, 255);
    } else {
      analogWrite(right, 127);
    }
    value[6] = false;
  }

  else if ((value[0] == LOW) && (value[1] == LOW) && (value[2] == LOW) && ((value[3] == HIGH) || (value[4] == HIGH)) && (value[5] == LOW)) {
    if (value[4] == HIGH) {
      analogWrite(left, 255);
    } else {
      analogWrite(left, 127);
    }
    analogWrite(right, 0);
    value[6] = false;
  }

  else if ((value[0] == LOW) && (value[1] == LOW) && (value[2] == LOW) && ((value[3] == LOW) || (value[4] == LOW)) && (value[5] == LOW)) {
    analogWrite(left, 0);
    analogWrite(right, 0);
    value[6] = true;
    Serial.println("Lost the line");
  }

  else {
    Serial.println("ERROR");
    value[6] = false;
  }
}
