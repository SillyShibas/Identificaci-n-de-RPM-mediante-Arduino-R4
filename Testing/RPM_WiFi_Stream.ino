// Testing Wifi-Stream for Arduino R4 WIFI (non functional code, do not load to Arduino)
#include <Arduino_CAN.h>
#include <WiFiS3.h>

// Wifi Settings
char ssid[] = "";       // Name of the wifi network
char pass[] = "";     // Password of the wifi network

WiFiServer server(8080);
WiFiClient client;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  // Start-up of the CAN module (set 500k or the speed the car uses)
  if (!CAN.begin(CanBitRate::BR_500k)) {
    Serial.println("Error inicializando el modulo CAN");
    while (1);
  }

  // Wifi start-up
  Serial.print("Conectando a la red WiFi...");
  while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConectado a la red WiFi.");
  Serial.print("IP del Arduino R4: ");
  Serial.println(WiFi.localIP()); 

  server.begin(); // Begin wifi server at 8080 port
}

void loop() {
  // Handle incoming wifi connections
  WiFiClient newClient = server.accept();
  if (newClient) {
    client = newClient;
  }

  // Reading and sending the CAN data
  if (CAN.available()) {
    CanMsg const msg = CAN.read();

    // Structuring the message: ID,Lenght,Byte0,Byte1,Byte2...
    String dataString = String(msg.id) + "," + String(msg.data_length);
    for (int i = 0; i < msg.data_length; i++) {
      dataString += "," + String(msg.data[i]);
    }

    // 1. Send via Serial Port
    Serial.println(dataString);

    // 2. Send via Wifi (if a program is connected listening)
    if (client && client.connected()) {
      client.println(dataString);
    }
  }
}