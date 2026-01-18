#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

const char* ssid = "androprojects";
const char* password = "4959538539";

ESP8266WebServer server(80);


void setup() {
  Serial.begin(115200);
  
  pinMode(D0, OUTPUT);
  pinMode(D1, OUTPUT);
  pinMode(D2, OUTPUT);
  resetPins();

  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  
  Serial.println();
  Serial.println(WiFi.localIP());

  server.on("/", handleRoot);
  server.on("/esp8266/roofopen", handleRoofOpen);
  server.on("/esp8266/roofclose", handleRoofClose);
  
  server.begin();
}


void resetPins() {
  digitalWrite(D0, LOW);
  digitalWrite(D1, LOW);
  digitalWrite(D2, LOW);
}

void handleRoot() {
  String html = "<html><body>";
  html += "<h1>Roof Control</h1>";
  html += "<p><a href='/esp8266/roofopen'>Open Roof</a></p>";
  html += "<p><a href='/esp8266/roofclose'>Close Roof</a></p>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void handleRoofOpen() {
  server.send(200, "text/plain", "Opening Roof...");
  digitalWrite(D0, HIGH);
  digitalWrite(D1, HIGH);
  digitalWrite(D2, HIGH);
  delay(11000);
  resetPins();
}

void handleRoofClose() {
  server.send(200, "text/plain", "Closing Roof...");
  digitalWrite(D0, HIGH);
  delay(11000);
  resetPins();
}

void loop() {
  server.handleClient();
}
