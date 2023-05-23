//------MCP23016------
#include <MCP23016.h>

MCP23016 MCP;

#define GPA 0x00  //port A data register address
#define GPB 0x01  //port B data register address

uint8_t mcpA;  //internal variable of the library - required to be declared as is
uint8_t mcpB;  //internal variable of the library - required to be declared as is


//------sensor------
#define left b0
#define right b1

#define lineSensorLL a0
#define lineSensorL a1
#define lineSensorC a2
#define lineSensorR a3
#define lineSensorRR a4

#define collisionSensor a5

int sensors[6] = { lineSensorLL, lineSensorL, lineSensorC, lineSensorR, lineSensorRR, collisionSensor };

String sensornames[7] = { "LL", "L", "C", "R", "RR", "Col", "Line" };

bool value[7] = { false, false, false, false, false, false, false };

void setup() {
  Serial.begin(115200);
  Serial.println("REBOOT");

  Wire.begin();
  Wire.setClock(100000);

  MCP.init(0x20);

  MCP.pinMode(left, OUTPUT);
  MCP.pinMode(right, OUTPUT);

  MCP.pinMode(lineSensorLL, INPUT);
  MCP.pinMode(lineSensorL, INPUT);
  MCP.pinMode(lineSensorC, INPUT);
  MCP.pinMode(lineSensorR, INPUT);
  MCP.pinMode(lineSensorRR, INPUT);

  MCP.pinMode(collisionSensor, INPUT);

  Serial.println("END BOOT");
}

void loop() {
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

  if (value[5] == HIGH) {
    MCP.digitalWrite(left, LOW);
    MCP.digitalWrite(right, LOW);
    value[6] = false;
  }

  else if ((value[0] == HIGH) && (value[1] == HIGH) && (value[2] == HIGH) && (value[3] == HIGH) && (value[4] == HIGH) && (value[5] == LOW)) {
    MCP.digitalWrite(left, LOW);
    MCP.digitalWrite(right, LOW);
    Serial.println("Halt point reached");
    value[6] = false;
  }

  else if ((value[0] == LOW) && (value[1] == LOW) && (value[2] == HIGH) && (value[3] == LOW) && (value[4] == LOW) && (value[5] == LOW)) {
    MCP.digitalWrite(left, HIGH);
    MCP.digitalWrite(right, HIGH);
    value[6] = false;
  }

  else if (((value[0] == HIGH) || (value[1] == HIGH)) && (value[2] == LOW) && (value[3] == LOW) && (value[4] == LOW) && (value[5] == LOW)) {
    MCP.digitalWrite(left, LOW);
    if (value[0] == HIGH) {
      MCP.digitalWrite(right, HIGH);
    } else {
      MCP.digitalWrite(right, HIGH);
    }
    value[6] = false;
  }

  else if ((value[0] == LOW) && (value[1] == LOW) && (value[2] == LOW) && ((value[3] == HIGH) || (value[4] == HIGH)) && (value[5] == LOW)) {
    if (value[4] == HIGH) {
      MCP.digitalWrite(left, HIGH);
    } else {
      MCP.digitalWrite(left, HIGH);
    }
    MCP.digitalWrite(right, LOW);
    value[6] = false;
  }

  else if ((value[0] == LOW) && (value[1] == LOW) && (value[2] == LOW) && ((value[3] == LOW) || (value[4] == LOW)) && (value[5] == LOW)) {
    MCP.digitalWrite(left, LOW);
    MCP.digitalWrite(right, LOW);
    value[6] = true;
    Serial.println("Lost the line");
  }

  else {
    Serial.println("ERROR");
    value[6] = false;
  }
}
