#include <MCP23016.h>

MCP23016 MCP;

#define GPA 0x00 //port A data register address
#define GPB 0x01 //port B data register address

uint8_t mcpA; //internal variable of the library - required to be declared as is
uint8_t mcpB; //internal variable of the library - required to be declared as is


#define LedG_1 a0
#define LedG_2 a1

#define LedY_1 a2

#define LedR_1 a3
#define LedR_2 a4

int looptime = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();    
  Wire.setClock(100000);

  MCP.init(0x20);

  MCP.pinMode(LedG_1, OUTPUT);
  MCP.pinMode(LedG_2, OUTPUT);

  MCP.pinMode(LedY_1, OUTPUT);

  MCP.pinMode(LedR_1, OUTPUT);
  MCP.pinMode(LedR_2, OUTPUT);

  MCP.digitalWrite(LedG_1, HIGH); //set A0
  Serial.print("A0 state: ");
  Serial.println(MCP.digitalRead(LedG_1));
  Serial.print("A1 state: ");
  Serial.println(MCP.digitalRead(LedG_2)); 
  delay(2000); 
  MCP.digitalWrite(LedG_1, LOW); //reset A0
}

void loop() {

  looptime += 1;
  Serial.println("Looptime: " + String(looptime));

  MCP.digitalWrite(LedG_1, HIGH);
  MCP.digitalWrite(LedG_2, LOW);

  MCP.digitalWrite(LedY_1, LOW);

  MCP.digitalWrite(LedR_1, LOW);
  MCP.digitalWrite(LedR_2, HIGH);

  delay(1000);

  MCP.digitalWrite(LedG_1, LOW);
  MCP.digitalWrite(LedG_2, HIGH);

  MCP.digitalWrite(LedY_1, LOW);

  MCP.digitalWrite(LedR_1, HIGH);
  MCP.digitalWrite(LedR_2, LOW);

  delay(1000);

  MCP.digitalWrite(LedG_1, LOW);
  MCP.digitalWrite(LedG_2, LOW);

  MCP.digitalWrite(LedY_1, HIGH);

  MCP.digitalWrite(LedR_1, LOW);
  MCP.digitalWrite(LedR_2, LOW);

  delay(1000);

  MCP.digitalWrite(LedG_1, LOW);
  MCP.digitalWrite(LedG_2, LOW);

  MCP.digitalWrite(LedY_1, LOW);

  MCP.digitalWrite(LedR_1, LOW);
  MCP.digitalWrite(LedR_2, LOW);

  delay(500);

  MCP.digitalWrite(LedG_1, HIGH);
  MCP.digitalWrite(LedG_2, LOW);

  MCP.digitalWrite(LedY_1, LOW);

  MCP.digitalWrite(LedR_1, LOW);
  MCP.digitalWrite(LedR_2, LOW);

  delay(500);

  MCP.digitalWrite(LedG_1, LOW);
  MCP.digitalWrite(LedG_2, LOW);

  MCP.digitalWrite(LedY_1, LOW);

  MCP.digitalWrite(LedR_1, LOW);
  MCP.digitalWrite(LedR_2, HIGH);

  delay(500);

  MCP.digitalWrite(LedG_1, LOW);
  MCP.digitalWrite(LedG_2, HIGH);

  MCP.digitalWrite(LedY_1, LOW);

  MCP.digitalWrite(LedR_1, LOW);
  MCP.digitalWrite(LedR_2, LOW);

  delay(500);

  MCP.digitalWrite(LedG_1, LOW);
  MCP.digitalWrite(LedG_2, LOW);

  MCP.digitalWrite(LedY_1, LOW);

  MCP.digitalWrite(LedR_1, HIGH);
  MCP.digitalWrite(LedR_2, LOW);

  delay(500);

  MCP.digitalWrite(LedG_1, LOW);
  MCP.digitalWrite(LedG_2, LOW);

  MCP.digitalWrite(LedY_1, HIGH);

  MCP.digitalWrite(LedR_1, LOW);
  MCP.digitalWrite(LedR_2, LOW);

  delay(500);
}
