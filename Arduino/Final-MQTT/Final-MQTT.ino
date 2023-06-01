//------WiFi and MQTT Setup------
#include <WiFi.h>
#include <PubSubClient.h>

const char* ssid = "embed";
const char* password = "weareincontrol";
const char* mqttServer = "192.168.1.17";
const int mqttPort = 1883;
const char* mqttUser = "Nick";
const char* mqttPassword = "odroid";
const char* clientID = "client_livingroom";

WiFiClient espClient;
PubSubClient client(espClient);

#define Node_ID "0x01"
#define Broad_ID "0x00"

int MQTTHalt = 50;
int MQTTHaltLast;
int MQTTHaltCurrent;

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

String sensornames[8] = { "LL", "L", "C", "R", "RR", "Col", "Line", "Dist" };

bool value[8] = { false, false, false, false, false, false, false, false };

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

#define high 120
#define low 0

//------Line Controls------
bool whiteLine = true;

bool haltControl = true;
int haltLast = 0;
int elapsedHalt = 0;
int currentHalt = 0;

#define haltButton 32

//-----Distance Sensor-----
#define trigPin 33
#define echoPin 25
long duration, cm;
int triggerDistance = 10;

//------Battery Voltage------
#define BAT 26

float VBAT;
float R1 = 3259.0;
float R2 = 983.0;

#define Full 8
#define Medium 5

#define PIN_RED 14
#define PIN_GREEN 13
#define PIN_BLUE 12

//------Startup-Tune------
#define NOTE_CS4 277
#define NOTE_D4 294
#define NOTE_E4 330
#define NOTE_FS4 370
#define NOTE_GS4 415
#define NOTE_A4 440
#define NOTE_B4 494
#define NOTE_CS5 554
#define NOTE_D5 587
#define NOTE_DS5 622
#define NOTE_E5 659
#define REST 0

int tempo = 180;

int buzzer = 27;

int melody[] = {
  NOTE_E5,8,NOTE_D5,8,NOTE_FS4,4,NOTE_GS4,4,
  NOTE_CS5,8,NOTE_B4,8,NOTE_D4,4,NOTE_E4,4,
  NOTE_B4,8,NOTE_A4,8,NOTE_CS4,4,NOTE_E4,4,
  NOTE_A4,2,
};

int notes = sizeof(melody) / sizeof(melody[0]) / 2;

int wholenote = (60000 * 4) / tempo;

int divider = 0, noteDuration = 0;

void setup() {
  //------Serial Communication------
  Serial.begin(115200);
  Serial.println("REBOOT");

  //------Connecting to the WiFi------
  Serial.println();
  Serial.print("Connecting to ");
  WiFi.begin(ssid, password);  // Connectie met het netwerk beginnen

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");

  //------Connecting to the MQTT Server------
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);

  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
    if (client.connect(Node_ID, mqttUser, mqttPassword)) {
      Serial.println("Connected");
      Serial.println("User: " + String(mqttUser) + " | Pass: " + String(mqttPassword));
    } else {
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
    }
  }

  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  //------Motor pins------
  pinMode(M1_EN, OUTPUT);
  pinMode(M1_F, OUTPUT);
  pinMode(M1_R, OUTPUT);

  pinMode(M2_EN, OUTPUT);
  pinMode(M2_F, OUTPUT);
  pinMode(M2_R, OUTPUT);

  //-----Distance Sensor Configuration-----
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  //------Halt Button------
  pinMode(haltButton, INPUT_PULLUP);

  //------MCP23016 I/O Expander------
  Wire.begin();
  Wire.setClock(100000);

  MCP.init(0x20);

  MCP.pinMode(lineSensorLL, INPUT);
  MCP.pinMode(lineSensorL, INPUT);
  MCP.pinMode(lineSensorC, INPUT);
  MCP.pinMode(lineSensorR, INPUT);
  MCP.pinMode(lineSensorRR, INPUT);

  MCP.pinMode(collisionSensor, INPUT);

  MCP.pinMode(problem, OUTPUT);
  MCP.pinMode(obstacle, OUTPUT);
  MCP.pinMode(line, OUTPUT);
  MCP.pinMode(onOFF, OUTPUT);

  //------VBAT------
  pinMode(BAT, INPUT);

  pinMode(PIN_RED, OUTPUT);
  pinMode(PIN_GREEN, OUTPUT);
  pinMode(PIN_BLUE, OUTPUT);

  analogWrite(PIN_RED, 0);
  analogWrite(PIN_GREEN, 0);
  analogWrite(PIN_BLUE, 0);
  
  //------Startup-Tune-----
  for (int thisNote = 0; thisNote < notes * 2; thisNote = thisNote + 2) {
    divider = melody[thisNote + 1];
    if (divider > 0) {
      noteDuration = (wholenote) / divider;
    } else if (divider < 0) {
      noteDuration = (wholenote) / abs(divider);
      noteDuration *= 1.5;
    }
    tone(buzzer, melody[thisNote], noteDuration * 0.9);
    delay(noteDuration);
    noTone(buzzer);

  }
  Serial.println("END BOOT");
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }

  MQTTHaltCurrent = millis();
  if ((MQTTHaltCurrent - MQTTHaltLast) >= MQTTHalt) {
    MQTTHaltLast = MQTTHaltCurrent;
    Serial.println("MQTT SEND");
    sendDataMQTT();
  }

  batteryCheck();

  if (VBAT >= Full) {
    analogWrite(PIN_RED, 0);
    analogWrite(PIN_GREEN, 255);
    analogWrite(PIN_BLUE, 0);
  } else if (VBAT < Full && VBAT >= Medium) {
    analogWrite(PIN_RED, 0);
    analogWrite(PIN_GREEN, 0);
    analogWrite(PIN_BLUE, 255);
  } else if (VBAT < Medium) {
    analogWrite(PIN_RED, 255);
    analogWrite(PIN_GREEN, 0);
    analogWrite(PIN_BLUE, 0);
  }

  distanceCalc();
  //Serial.println("Dist: " + String(cm) + " cm");

  if (cm < triggerDistance) {
    value[7] = true;
  } else {
    value[7] = false;
  }

  MCP.digitalWrite(onOFF, HIGH);

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



  /*for (int i = 0; i < 8; i++) {
    Serial.print(String(sensornames[i]));
    Serial.print(": ");
    Serial.print(value[i]);
    Serial.print(" | ");
  }
  Serial.println();*/

  if (value[5] == false && value[6] == false && value[7] == false) {
    MCP.digitalWrite(problem, HIGH);
  } else {
    MCP.digitalWrite(problem, LOW);
  }

  if (value[5] == true || value[7] == true) {
    MCP.digitalWrite(obstacle, HIGH);
  } else {
    MCP.digitalWrite(obstacle, LOW);
  }

  if (value[6] == true) {
    MCP.digitalWrite(line, HIGH);
  } else {
    MCP.digitalWrite(line, LOW);
  }

  //------Stand still at obstacle------
  if (value[5] == HIGH || value[7] == HIGH) {
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

  //------White line controll------
  else if (((value[1] == HIGH) && (value[2] == HIGH) && (value[3] == HIGH) && (value[5] == LOW) && (value[7] == LOW)) || haltControl == false) {
    digitalWrite(M1_EN, LOW);
    analogWrite(M1_F, low);
    analogWrite(M1_R, low);
    digitalWrite(M2_EN, LOW);
    analogWrite(M2_F, low);
    analogWrite(M2_R, low);
    //Serial.println("Halt point reached");
    if (haltControl) {
      haltControl = false;
      haltLast = millis();
    }
    currentHalt = millis();
    if (((currentHalt - haltLast) >= 5000) || digitalRead(haltButton) == LOW) {
      currentHalt = haltLast + 5500;
      haltControl = true;
      digitalWrite(M1_EN, HIGH);
      analogWrite(M1_F, high);
      analogWrite(M1_R, low);
      digitalWrite(M2_EN, HIGH);
      analogWrite(M2_F, high);
      analogWrite(M2_R, low);
      delay(200);
    }
    middle = false;
    left = false;
    right = false;
    value[6] = false;
  }

  //------Line following: center white------
  else if ((value[1] == LOW) && (value[2] == HIGH) && (value[3] == LOW) && (value[5] == LOW) && (value[7] == LOW)) {
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

  //------Line following: Left white------
  else if ((value[1] == HIGH) && (value[2] == LOW) && (value[3] == LOW) && (value[5] == LOW) && (value[7] == LOW)) {
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

  //------Line following: Right white------
  else if ((value[1] == LOW) && (value[2] == LOW) && (value[3] == HIGH) && (value[5] == LOW) && (value[7] == LOW)) {
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

  //------Line lost check------
  else if ((value[1] == LOW) && (value[2] == LOW) && (value[3] == LOW) && (value[5] == LOW) && (value[7] == LOW)) {
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
      //Serial.println("Lost the line");
    }
  }

  //------Unforeseen state------
  else {
    //Serial.println("ERROR");
    value[6] = false;
  }

  //------Printing the states------
  /*if (middle == true) {
    Serial.println("Middle");
  } else if (left == true) {
    Serial.println("left");
  } else if (right == true) {
    Serial.println("right");
  } else {
    Serial.println("No line");
  }*/
}

void reconnect() {
  client.disconnect();
  while (!client.connected()) {
    //Serial.println("Attempting MQTT Connection...");

    if (client.connect(Node_ID, mqttUser, mqttPassword)) {
      //Serial.println("Connected");
      break;
    } else {
      /*Serial.print("Connection failed, rc= ");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");*/

      delay(5000);
    }
  }
  return;
}

void sendDataMQTT() {
  for (int i = 0; i < 8; i++) {
    String sensValue = "0";
    if (value[i] == true) {
      sensValue = "1";
    } else {
      sensValue = "0";
    }
    String(tempS) = String(sensValue);
    char message1[2];
    tempS.toCharArray(message1, 2);
    String(topic) = "IoT_Car/sensor/" + String(i);
    char topicTemp[17];
    topic.toCharArray(topicTemp, 17);
    client.publish(topicTemp, message1);
    //Serial.print(topicTemp);
    //Serial.println(" | MQTT Send: " + String(i) + " | With value of: " + String(sensValue));
  }
}

void callback(char* topic, byte* payload, unsigned int length) {

  //Serial.print("Message arrived in topic: ");
 // Serial.println(topic);

  String messageTemp;

  for (int i = 0; i < length; i++) {
    messageTemp += (char)payload[i];
  }

  /*Serial.println("Msg: " + messageTemp);

  Serial.println("-----------------------");
  Serial.println();*/

  delay(10);
}

//------Checking the battery voltage------
void batteryCheck() {
  VBAT = (((R1 + R2) / R2) * (3.30f / 4095.0f) * analogRead(BAT)) + 0.5f;
  /*Serial.print("Battery Voltage = ");
  Serial.print(VBAT, 2);
  Serial.println(" V");*/
}

//------Calculating the distance to the nearest object infront of the sensor------
void distanceCalc() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);

  cm = (duration / 2) * 0.0343;
}
