#include <WiFi.h>
#include <PubSubClient.h>

const char* Subscribe_topic = "home/GET-Request";
const char* Publish_topic = "home/PUSH-Response";

const char* ssid = "telenet-37303";
const char* password = "nQP4VC0K7T8M";
const char* mqttServer = "192.168.0.162";
const int mqttPort = 1883;
const char* mqttUser = "Nick";
const char* mqttPassword = "odroid";
const char* clientID = "client_livingroom";

WiFiClient espClient;
PubSubClient client(espClient);

#define Node_ID "0x01"
#define Broad_ID "0x00"

//-----------------------------------------

//------MCP23016------
#include <MCP23016.h>

MCP23016 MCP;

#define GPA 0x00  //port A data register address
#define GPB 0x01  //port B data register address

uint8_t mcpA;  //internal variable of the library - required to be declared as is
uint8_t mcpB;  //internal variable of the library - required to be declared as is

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

//------Line Controls------
bool whiteLine = true;

void setup() {
  Serial.begin(115200);
  Serial.println("REBOOT");

  Serial.println();
  Serial.print("Connecting to ");
  WiFi.begin(ssid, password);  // Connectie met het netwerk beginnen

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");

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

  client.subscribe(Subscribe_topic);

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
  if (!client.connected()) {
    reconnect();
  }
  readSensorData();
  sendDataMQTT();

  client.loop();
  delay(1000);
}

void reconnect() {
  client.disconnect();
  while (!client.connected()) {
    Serial.println("Attempting MQTT Connection...");

    if (client.connect(Node_ID, mqttUser, mqttPassword)) {
      Serial.println("Connected");
      client.subscribe(Subscribe_topic);
      break;
    } else {
      Serial.print("Connection failed, rc= ");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");

      delay(5000);
    }
  }
  return;
}

void readSensorData() {
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
}

void sendDataMQTT() {
  /*String(tempS) = String(Temp);
  char message1[12];
  tempS.toCharArray(message1, 12);
  client.publish(topicTemp, message1);

  String(humS) = String(Hum);
  char message2[16];
  humS.toCharArray(message2, 16);
  client.publish(topicHum, message2);

  String(CO2S) = String(MQ135SensorValue);
  char message3[16];
  CO2S.toCharArray(message3, 16);
  client.publish(topicCO2, message3);*/

  /*String topic = "";
  for (int i = 0; i < 6; i++) {
    topic = "home/sensor/" + String(i);
    Serial.println("MQTT Send: " + String(i));
    Serial.print(topic);
  }*/


  for (int i = 0; i < 6; i++) {
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
    Serial.print(topicTemp);
    Serial.println(" | MQTT Send: " + String(i) + " | With value of: " + String(sensValue));
  }
}

void serialprintData() {
  for (int i = 0; i < 6; i++) {
    Serial.print(String(sensornames[i]));
    Serial.print(": ");
    Serial.print(value[i]);
    Serial.print(" | ");
  }
  Serial.println();
}

void callback(char* topic, byte* payload, unsigned int length) {

  Serial.print("Message arrived in topic: ");
  Serial.println(topic);

  String messageTemp;

  for (int i = 0; i < length; i++) {
    messageTemp += (char)payload[i];
  }

  Serial.println("Msg: " + messageTemp);

  Serial.println("-----------------------");
  Serial.println();

  delay(10);
}
