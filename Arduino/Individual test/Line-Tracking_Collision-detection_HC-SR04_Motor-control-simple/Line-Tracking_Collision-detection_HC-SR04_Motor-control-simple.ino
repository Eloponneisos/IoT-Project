//-----Simple Motor Control-----
#define left 10
#define right 9

#define SpeedControl 0.75
#define SpeedDifference 1.25

//-----Line Sensors-----
#define lineSensorLL 8
#define lineSensorL 7
#define lineSensorC 6
#define lineSensorR 5
#define lineSensorRR 4

//-----Collision Sensor-----
#define collisionSensor 3

//-----Distance Sensor-----
#define trigPin 11
#define echoPin 12
long duration, cm;
int triggerDistance = 10;

//-----Sensor Data Handling-----
int sensors[6] = { lineSensorLL, lineSensorL, lineSensorC, lineSensorR, lineSensorRR, collisionSensor };

String sensornames[8] = { "LL", "L", "C", "R", "RR", "Col", "Dist", "Line" };

bool value[8] = { false, false, false, false, false, false, false, false };

//-----Time keeping for Line losing-----
int startTime = 0;
int elapsedTime = 0;
int allowedTime = 2000;
bool check = false;

void setup() {
  //-----Serial Communications Setup-----
  Serial.begin(115200);
  Serial.println("REBOOT");

  //-----Motor Pins Configuration-----
  pinMode(left, OUTPUT);
  pinMode(right, OUTPUT);

  //-----Line Sensors Configuration-----
  pinMode(lineSensorLL, INPUT);
  pinMode(lineSensorL, INPUT);
  pinMode(lineSensorC, INPUT);
  pinMode(lineSensorR, INPUT);
  pinMode(lineSensorRR, INPUT);

  //-----Collision Sensor Configuration-----
  pinMode(collisionSensor, INPUT);

  //-----Distance Sensor Configuration-----
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  //-----Serial Communications Debug-----
  Serial.println("END BOOT");
}

void loop() {
  //-----Sensor Data Collection-----
  for (int i = 0; i < 6; i++) {
    value[i] = !digitalRead(sensors[i]);
  }

  value[5] = !value[5];

  //-----Printing of the Data for Debug-----
  for (int i = 0; i < 8; i++) {
    Serial.print(String(sensornames[i]));
    Serial.print(": ");
    Serial.print(value[i]);
    Serial.print(" | ");
  }
  Serial.print(cm);
  Serial.print("cm");
  Serial.println();

  distanceCalc();

  if (cm < triggerDistance) {
    value[6] = true;
  } else {
    value[6] = false;
  }

  if (value[5] == HIGH) {
    analogWrite(left, 0);
    analogWrite(right, 0);
    value[7] = false;
    elapsedTime = 0;
  }

  else if (value[6] == HIGH) {
    analogWrite(left, 0);
    analogWrite(right, 0);
    value[7] = false;
    elapsedTime = 0;
  }

  else if ((value[0] == HIGH) && (value[1] == HIGH) && (value[2] == HIGH) && (value[3] == HIGH) && (value[4] == HIGH) && (value[5] == LOW)) {
    analogWrite(left, 0);
    analogWrite(right, 0);
    Serial.println("Halt point reached");
    value[7] = false;
    elapsedTime = 0;
  }

  else if ((value[0] == LOW) && (value[1] == LOW) && (value[2] == HIGH) && (value[3] == LOW) && (value[4] == LOW) && (value[5] == LOW)) {
    analogWrite(left, SpeedControl*255);
    analogWrite(right, SpeedControl*255);
    value[7] = false;
    elapsedTime = 0;
  }

  else if (((value[0] == HIGH) || (value[1] == HIGH)) && (value[2] == LOW) && (value[3] == LOW) && (value[4] == LOW) && (value[5] == LOW)) {
    analogWrite(left, 0);
    if (value[0] == HIGH) {
      analogWrite(right, SpeedDifference*(SpeedControl*255));
    } else {
      analogWrite(right, SpeedControl*255);
    }
    elapsedTime = 0;
    value[7] = false;
  }

  else if ((value[0] == LOW) && (value[1] == LOW) && (value[2] == LOW) && ((value[3] == HIGH) || (value[4] == HIGH)) && (value[5] == LOW)) {
    analogWrite(right, 0);
    if (value[4] == HIGH) {
      analogWrite(left, SpeedDifference*(SpeedControl*255));
    } else {
      analogWrite(left, SpeedControl*255);
    }
    elapsedTime = 0;
    value[7] = false;
  }

  else if ((value[0] == LOW) && (value[1] == LOW) && (value[2] == LOW) && ((value[3] == LOW) || (value[4] == LOW)) && (value[5] == LOW)) {
    analogWrite(left, 0);
    analogWrite(right, 0);

    if (!check) {
      startTime = millis();
      check = true;
    } else {
      elapsedTime = millis() - startTime;
    }

    if (elapsedTime > allowedTime) {
      value[7] = true;
      Serial.println("Lost the line");
      check = false;
    } else {
      value[7] = false;
    }
  }

  else {
    Serial.println("ERROR");
    value[7] = false;
  }
}

//-----Calculating the distance to the nearest object infront of the sensor-----
void distanceCalc() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);

  cm = (duration / 2) * 0.0343;
}
