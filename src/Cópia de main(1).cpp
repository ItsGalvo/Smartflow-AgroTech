#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define RELAY_PIN 25
#define SOIL_MOISTURE_PIN A0

const char* ssid = "SEU_SSID";
const char* password = "SUA_SENHA";

Adafruit_BME280 bme;
WiFiServer server(80);

int umidadePercentual = 0;
bool bombaLigada = false;

void setup() {
  Serial.begin(115200);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW); // Bomba inicialmente desligada

  if (!bme.begin(0x76)) {
    Serial.println("Não foi possível inicializar o sensor BME280.");
    while (1);
  }

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi conectado!");
  Serial.println(WiFi.localIP());

  server.begin();
}

void loop() {
  WiFiClient client = server.available();
  if (client) {
    String request = client.readStringUntil('\r');
    client.flush();

    if (request.indexOf("/ligar_bomba") != -1) {
      ligarBomba();
    }
    if (request.indexOf("/desligar_bomba") != -1) {
      desligarBomba();
    }

    umidadePercentual = calcularUmidade();
    float temperatura = bme.readTemperature();
    float umidadeBME = bme.readHumidity();
    float pressao = bme.readPressure() / 100.0F;

    String data = "{\"umidade_solo\":" + String(umidadePercentual) + ", \"temperatura\":" + String(temperatura) + ", \"umidade\":" + String(umidadeBME) + ", \"pressao\":" + String(pressao) + "}";
    
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: application/json");
    client.println("Connection: close");
    client.println();
    client.println(data);

    delay(1);
    client.stop();
  }
}

int calcularUmidade() {
  int valorSensor = analogRead(SOIL_MOISTURE_PIN);
  return map(valorSensor, 0, 4095, 0, 100);
}

void ligarBomba() {
  digitalWrite(RELAY_PIN, HIGH);
  bombaLigada = true;
}

void desligarBomba() {
  digitalWrite(RELAY_PIN, LOW);
  bombaLigada = false;
}
