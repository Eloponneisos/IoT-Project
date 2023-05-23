//------MCP23016------
#include <MCP23016.h>

MCP23016 MCP;

#define GPA 0x00  //port A data register address
#define GPB 0x01  //port B data register address

uint8_t mcpA;  //internal variable of the library - required to be declared as is
uint8_t mcpB;  //internal variable of the library - required to be declared as is

//------Status lights-----
#define problem a0
#define obstacle a1
#define line a2
#define onOFF a3

//------sensor------
#define lineSensorLL b0
#define lineSensorL b1
#define lineSensorC b2
#define lineSensorR b3
#define lineSensorRR b4

#define collisionSensor b5

int sensors[6] = { lineSensorLL, lineSensorL, lineSensorC, lineSensorR, lineSensorRR, collisionSensor };

String sensornames[7] = { "LL", "L", "C", "R", "RR", "Col", "Line" };

bool value[7] = { false, false, false, false, false, false, false };

bool middle = false;
bool left = false;
bool right = false;

//------Motor Controls------
#define M1_EN 15
#define M1_F 2
#define M1_R 4

#define M2_EN 23
#define M2_F 18
#define M2_R 19

#define high 200
#define low 0

//------Line Controls------
bool whiteLine = true;

void setup() {
  Serial.begin(115200);
  Serial.println("REBOOT");

  pinMode(M1_EN, OUTPUT);
  pinMode(M1_F, OUTPUT);
  pinMode(M1_R, OUTPUT);

  pinMode(M2_EN, OUTPUT);
  pinMode(M2_F, OUTPUT);
  pinMode(M2_R, OUTPUT);

  Wire.begin();
  Wire.setClock(100000);

  MCP.init(0x20);

  MCP.pinMode(lineSensorLL, INPUT);
  MCP.pinMode(lineSensorL, INPUT);
  MCP.pinMode(lineSensorC, INPUT);
  MCP.pinMode(lineSensorR, INPUT);
  MCP.pinMode(lineSensorRR, INPUT);

  MCP.pinMode(collisionSensor, INPUT);

  Serial.println("END BOOT");
}

void loop() {

  if (whiteLine) {
    //------Sensor data acquisition for a white line----
    for (int i = 0; i < 6; i++) {
      value[i] = MCP.digitalRead(sensors[i]);
    }
  } else {
    //------Sensor data acquisition for a black line----
    for (int i = 0; i < 6; i++) {
      value[i] = !MCP.digitalRead(sensors[i]);
    }
      value[5] = !value[5];
  }



  for (int i = 0; i < 6; i++) {
    Serial.print(String(sensornames[i]));
    Serial.print(": ");
    Serial.print(value[i]);
    Serial.print(" | ");
  }
  Serial.println();

  if (value[5] == HIGH) {
    digitalWrite(M1_EN, LOW);
    analogWrite(M1_F, low);
    analogWrite(M1_R, low);
    digitalWrite(M2_EN, LOW);
    analogWrite(M2_F, low);
    analogWrite(M2_R, low);
    middle = false;
    left = false;
    right = false;
    value[6] = false;
  }

  else if ((value[0] == HIGH) && (value[1] == HIGH) && (value[2] == HIGH) && (value[3] == HIGH) && (value[4] == HIGH) && (value[5] == LOW)) {
    digitalWrite(M1_EN, LOW);
    analogWrite(M1_F, low);
    analogWrite(M1_R, low);
    digitalWrite(M2_EN, LOW);
    analogWrite(M2_F, low);
    analogWrite(M2_R, low);
    Serial.println("Halt point reached");
    middle = false;
    left = false;
    right = false;
    value[6] = false;
  }

  else if ((value[0] == LOW) && (value[1] == LOW) && (value[2] == HIGH) && (value[3] == LOW) && (value[4] == LOW) && (value[5] == LOW)) {
    digitalWrite(M1_EN, HIGH);
    analogWrite(M1_F, high);
    analogWrite(M1_R, low);
    digitalWrite(M2_EN, HIGH);
    analogWrite(M2_F, high);
    analogWrite(M2_R, low);
    middle = true;
    left = false;
    right = false;
    value[6] = false;
  }

  else if (((value[0] == HIGH) || (value[1] == HIGH)) && (value[2] == LOW) && (value[3] == LOW) && (value[4] == LOW) && (value[5] == LOW)) {
    digitalWrite(M1_EN, HIGH);
    analogWrite(M1_F, high);
    analogWrite(M1_R, low);
    digitalWrite(M2_EN, HIGH);
    analogWrite(M2_F, low);
    analogWrite(M2_R, high);
    middle = false;
    left = true;
    right = false;
    value[6] = false;
  }

  else if ((value[0] == LOW) && (value[1] == LOW) && (value[2] == LOW) && ((value[3] == HIGH) || (value[4] == HIGH)) && (value[5] == LOW)) {
    digitalWrite(M1_EN, HIGH);
    analogWrite(M1_F, low);
    analogWrite(M1_R, high);
    digitalWrite(M2_EN, HIGH);
    analogWrite(M2_F, high);
    analogWrite(M2_R, low);
    middle = false;
    left = false;
    right = true;
    value[6] = false;
  }

  else if ((value[0] == LOW) && (value[1] == LOW) && (value[2] == LOW) && (value[3] == LOW) && (value[4] == LOW) && (value[5] == LOW)) {
    if (middle == true) {
      digitalWrite(M1_EN, HIGH);
      analogWrite(M1_F, high);
      analogWrite(M1_R, low);
      digitalWrite(M2_EN, HIGH);
      analogWrite(M2_F, high);
      analogWrite(M2_R, low);
    } else if (left == true) {
      digitalWrite(M1_EN, HIGH);
      analogWrite(M1_F, low);
      analogWrite(M1_R, high);
      digitalWrite(M2_EN, HIGH);
      analogWrite(M2_F, high);
      analogWrite(M2_R, low);
    } else if (right == true) {
      digitalWrite(M1_EN, HIGH);
      analogWrite(M1_F, high);
      analogWrite(M1_R, low);
      digitalWrite(M2_EN, HIGH);
      analogWrite(M2_F, low);
      analogWrite(M2_R, high);
    } else {
      digitalWrite(M1_EN, LOW);
      analogWrite(M1_F, low);
      analogWrite(M1_R, low);
      digitalWrite(M2_EN, LOW);
      analogWrite(M2_F, low);
      analogWrite(M2_R, low);
      middle = false;
      left = false;
      right = false;
      value[6] = true;
      Serial.println("Lost the line");
    }
  }

  else {
    Serial.println("ERROR");
    value[6] = false;
  }

  if (middle == true) {
    MCP.digitalWrite(obstacle, HIGH);
    Serial.println("Middle");
  } else if (left == true) {
    MCP.digitalWrite(problem, HIGH);
    Serial.println("left");
  } else if (right == true) {
    MCP.digitalWrite(line, HIGH);
    Serial.println("right");
  } else {
    Serial.println("No line");
  }
}
