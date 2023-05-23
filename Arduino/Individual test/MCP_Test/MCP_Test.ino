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

//------Line Sensor------
#define lineSensorLL b0
#define lineSensorL b1
#define lineSensorC b2
#define lineSensorR b3
#define lineSensorRR b4

#define collisionSensor b5

int sensors[6] = { lineSensorLL, lineSensorL, lineSensorC, lineSensorR, lineSensorRR, collisionSensor };

String sensornames[7] = { "LL", "L", "C", "R", "RR", "Col", "Line" };

bool value[7] = { false, false, false, false, false, false, false };

//------Motor controls------
#define M1_EN 15
#define M1_F 2
#define M1_R 4

#define M2_EN 23
#define M2_F 18
#define M2_R 19

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

  MCP.pinMode(onOFF, OUTPUT);
  MCP.pinMode(line, OUTPUT);
  MCP.pinMode(obstacle, OUTPUT);
  MCP.pinMode(problem, OUTPUT);

  MCP.pinMode(lineSensorLL, INPUT);
  MCP.pinMode(lineSensorL, INPUT);
  MCP.pinMode(lineSensorC, INPUT);
  MCP.pinMode(lineSensorR, INPUT);
  MCP.pinMode(lineSensorRR, INPUT);

  MCP.pinMode(collisionSensor, INPUT);

  Serial.println("END BOOT");
}

void loop() {
  Serial.println("Start Light-test");

  MCP.digitalWrite(onOFF, HIGH);
  MCP.digitalWrite(line, LOW);
  MCP.digitalWrite(obstacle, LOW);
  MCP.digitalWrite(problem, LOW);
  delay(500);
  MCP.digitalWrite(onOFF, LOW);
  MCP.digitalWrite(line, HIGH);
  MCP.digitalWrite(obstacle, LOW);
  MCP.digitalWrite(problem, LOW);
  delay(500);
  MCP.digitalWrite(onOFF, LOW);
  MCP.digitalWrite(line, LOW);
  MCP.digitalWrite(obstacle, HIGH);
  MCP.digitalWrite(problem, LOW);
  delay(500);
  MCP.digitalWrite(onOFF, LOW);
  MCP.digitalWrite(line, LOW);
  MCP.digitalWrite(obstacle, LOW);
  MCP.digitalWrite(problem, HIGH);
  delay(500);
  MCP.digitalWrite(onOFF, HIGH);
  MCP.digitalWrite(line, LOW);
  MCP.digitalWrite(obstacle, HIGH);
  MCP.digitalWrite(problem, LOW);
  delay(500);
  MCP.digitalWrite(onOFF, LOW);
  MCP.digitalWrite(line, HIGH);
  MCP.digitalWrite(obstacle, LOW);
  MCP.digitalWrite(problem, HIGH);
  delay(500);
  MCP.digitalWrite(onOFF, HIGH);
  MCP.digitalWrite(line, HIGH);
  MCP.digitalWrite(obstacle, HIGH);
  MCP.digitalWrite(problem, HIGH);
  delay(500);
  MCP.digitalWrite(onOFF, LOW);
  MCP.digitalWrite(line, LOW);
  MCP.digitalWrite(obstacle, LOW);
  MCP.digitalWrite(problem, LOW);
  delay(500);

  Serial.println("END Lights-test");

  Serial.println("Begin Sensor Read test");
  for (int u = 0; u < 5; u++) {
    for (int i = 0; i < 6; i++) {
      value[i] = !MCP.digitalRead(sensors[i]);
    }

    value[5] = !value[5];

    for (int i = 0; i < 6; i++) {
      Serial.print(String(sensornames[i]));
      Serial.print(": ");
      Serial.print(value[i]);
      Serial.print(" | ");
    }
    Serial.println();
    if (value[2] == HIGH) {
      MCP.digitalWrite(onOFF, HIGH);
      MCP.digitalWrite(line, LOW);
    } else {
      MCP.digitalWrite(onOFF, LOW);
      MCP.digitalWrite(line, HIGH);
    }
    delay(1000);
  }
  Serial.println("End Sensor Read test");

  /*Serial.println("Begin Motor Test");
  digitalWrite(M1_EN, LOW);
  digitalWrite(M1_F, LOW);
  digitalWrite(M1_R, LOW);
  digitalWrite(M2_EN, LOW);
  digitalWrite(M2_F, LOW);
  digitalWrite(M2_R, LOW);

  digitalWrite(M1_EN, HIGH);
  digitalWrite(M1_F, HIGH);
  digitalWrite(M1_R, LOW);
  digitalWrite(M2_EN, LOW);
  digitalWrite(M2_F, LOW);
  digitalWrite(M2_R, LOW);
  delay(1000);
  digitalWrite(M1_EN, LOW);
  digitalWrite(M1_F, LOW);
  digitalWrite(M1_R, LOW);
  digitalWrite(M2_EN, HIGH);
  digitalWrite(M2_F, HIGH);
  digitalWrite(M2_R, LOW);
  delay(1000);
  digitalWrite(M1_EN, HIGH);
  digitalWrite(M1_F, HIGH);
  digitalWrite(M1_R, LOW);
  digitalWrite(M2_EN, HIGH);
  digitalWrite(M2_F, LOW);
  digitalWrite(M2_R, HIGH);
  delay(1000);
  digitalWrite(M1_EN, HIGH);
  digitalWrite(M1_F, LOW);
  digitalWrite(M1_R, HIGH);
  digitalWrite(M2_EN, HIGH);
  digitalWrite(M2_F, HIGH);
  digitalWrite(M2_R, LOW);
  delay(1000);
  digitalWrite(M1_EN, HIGH);
  digitalWrite(M1_F, HIGH);
  digitalWrite(M1_R, LOW);
  digitalWrite(M2_EN, HIGH);
  digitalWrite(M2_F, HIGH);
  digitalWrite(M2_R, LOW);
  delay(1000);
  digitalWrite(M1_EN, HIGH);
  digitalWrite(M1_F, LOW);
  digitalWrite(M1_R, HIGH);
  digitalWrite(M2_EN, HIGH);
  digitalWrite(M2_F, LOW);
  digitalWrite(M2_R, HIGH);
  delay(1000);

  digitalWrite(M1_EN, LOW);
  digitalWrite(M1_F, LOW);
  digitalWrite(M1_R, LOW);
  digitalWrite(M2_EN, LOW);
  digitalWrite(M2_F, LOW);
  digitalWrite(M2_R, LOW);

  Serial.println("End Motor Test");*/

}
