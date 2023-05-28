#define BAT 26

float VBAT;
float R1 = 3259.0;
float R2 = 983.0;

void setup() {
  Serial.begin(115200);
  Serial.println("REBOOT");

  pinMode(BAT, INPUT);

  Serial.println("END BOOT");

}

void loop() {
  VBAT = ((R1+R2)/R2) * (3.30f / 4095.0f) * analogRead(BAT);
  Serial.print("Battery Voltage = "); Serial.print(VBAT, 2); Serial.println(" V");  
  //Serial.print("Value = "); Serial.print(value, 2); Serial.println(" V");  

}
