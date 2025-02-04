#include <Wire.h>
#include <LoRa.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <esp_sleep.h>
#include <BluetoothSerial.h>
#include <vector>
#include <cmath>

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme;
BluetoothSerial SerialBT;

#define SOIL_MOISTURE_PIN A0
#define RELAY_PIN 25

#define SCK_LORA           5
#define MISO_LORA          19
#define MOSI_LORA          27
#define RESET_PIN_LORA     14
#define SS_PIN_LORA        18

#define HIGH_GAIN_LORA     20
#define BAND               915E6

#define DEBUG_SERIAL_BAUDRATE 115200

int valorSensor = 0;
int umidadePercentual = 0;

double w = 0.0;
double b = 0.0;

bool init_comunicacao_lora(void);

double meanSquaredError(const std::vector<double>& y_true, const std::vector<double>& y_pred) {
    double error = 0.0;
    for (size_t i = 0; i < y_true.size(); ++i) {
        error += pow(y_true[i] - y_pred[i], 2);
    }
    return error / y_true.size();
}

std::vector<double> predict(const std::vector<double>& X, double w, double b) {
    std::vector<double> predictions;
    for (double x : X) {
        predictions.push_back(w * x + b);
    }
    return predictions;
}

void trainModel(const std::vector<double>& X, const std::vector<double>& y, double& w, double& b, double learning_rate, int epochs) {
    size_t n = X.size();
    
    if (n == 0) {
        Serial.println("Erro: dados de treinamento ausentes.");
        return;
    }
    
    for (int i = 0; i < epochs; ++i) {
        double dw = 0.0;
        double db = 0.0;

        for (size_t j = 0; j < n; ++j) {
            double y_pred = w * X[j] + b;
            dw += (y_pred - y[j]) * X[j];
            db += (y_pred - y[j]);
        }

        w -= learning_rate * dw / n;
        b -= learning_rate * db / n;

        if (isnan(w) || isnan(b)) {
            Serial.println("Erro: w ou b resultaram em NaN durante o treinamento.");
            return;
        }

        if (i % 100 == 0) {
            std::vector<double> y_pred = predict(X, w, b);
            Serial.print("Epoch ");
            Serial.print(i);
            Serial.print(" - MSE: ");
            Serial.println(meanSquaredError(y, y_pred));
            Serial.print(" - w: ");
            Serial.print(w);
            Serial.print(", b: ");
            Serial.println(b);
        }
    }
}

bool init_comunicacao_lora(void) {
    bool status_init = false;
    Serial.println("[LoRa Sender] Tentando iniciar comunicação com o rádio LoRa...");
    SPI.begin(SCK_LORA, MISO_LORA, MOSI_LORA, SS_PIN_LORA);
    LoRa.setPins(SS_PIN_LORA, RESET_PIN_LORA, LORA_DEFAULT_DIO0_PIN);
     
    if (!LoRa.begin(BAND)) {
        Serial.println("[LoRa Sender] Comunicação com o rádio LoRa falhou. Nova tentativa em 1 segundo...");        
        delay(1000);
        status_init = false;
    } else {
        LoRa.setTxPower(HIGH_GAIN_LORA); 
        Serial.println("[LoRa Sender] Comunicação com o rádio LoRa ok");
        status_init = true;
    }
 
    return status_init;
}

void sendLoRaData(int umidadePercentual, float tempBME, float humidity, float pressure) {
    LoRa.beginPacket();
    LoRa.print("{\"umidade_solo\":");
    LoRa.print(umidadePercentual);
    LoRa.print(", \"temperatura\":");
    LoRa.print(tempBME);
    LoRa.print(", \"umidade\":");
    LoRa.print(humidity);
    LoRa.print(", \"pressao\":");
    LoRa.print(pressure);
    LoRa.print("}");
    LoRa.endPacket();
}

void setup() {
    Serial.begin(DEBUG_SERIAL_BAUDRATE);
    SerialBT.begin("SmartFlow-ESP"); // Nome do Bluetooth
    Serial.println("Bluetooth inicializado!");

    // Configura o pino do relé como saída
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW);

    if (!bme.begin(0x76)) {
        Serial.println("Não foi possível encontrar o sensor BME280! Verifique as conexões.");
        while (1);
    }

    Serial.println("Sensor BME280 inicializado!");
    Wire.begin();
    Serial.println("Comunicação I2C inicializada!");

    while (init_comunicacao_lora() == false);
    Serial.println("LoRa inicializado!");

    std::vector<double> umidade = {15.0, 25.0, 35.0, 45.0, 55.0};
    std::vector<double> irrigacao = {1, 1, 1, 0, 0};
    double taxa_aprendizado = 0.001;
    int epocas = 5000;
    trainModel(umidade, irrigacao, w, b, taxa_aprendizado, epocas);

    Serial.println("Treinamento da IA concluído!");
}

void loop() {
    valorSensor = analogRead(SOIL_MOISTURE_PIN);
    umidadePercentual = map(valorSensor, 0, 4095, 0, 100);

    // Leitura dos sensores
    float humidity = bme.readHumidity();
    float tempBME = bme.readTemperature();
    float pressure = bme.readPressure() / 100.0F;

    // Decisão baseada no modelo de IA
    double irrigacaoPredita = w * umidadePercentual + b;
    Serial.print("Decisão de irrigação predita (baseada no modelo de IA): ");
    Serial.println(irrigacaoPredita);

    // Verifica se a irrigação é necessária
    if (irrigacaoPredita > 0.5) {
        digitalWrite(RELAY_PIN, HIGH);  // Ligar bomba
        Serial.println("Bomba ligada pela decisão da IA!");
    } else {
        digitalWrite(RELAY_PIN, LOW);  // Desligar bomba
        Serial.println("Bomba desligada pela decisão da IA!");
    }

    // Enviar dados para o LoRa
    sendLoRaData(umidadePercentual, tempBME, humidity, pressure);

    // Enviar dados para o Bluetooth
    if (SerialBT.available()) {
        String command = SerialBT.readStringUntil('\n');
        command.trim();

        if (command == "GET_DATA") {
            String data = "{\"umidade_solo\":" + String(umidadePercentual) +
                          ", \"temperatura\":" + String(tempBME) +
                          ", \"umidade\":" + String(humidity) +
                          ", \"pressao\":" + String(pressure) + "}";
            SerialBT.println(data);
        } else if (command == "LIGAR_BOMBA") {
            digitalWrite(RELAY_PIN, HIGH);
            SerialBT.println("Bomba ligada");
        } else if (command == "DESLIGAR_BOMBA") {
            digitalWrite(RELAY_PIN, LOW);
            SerialBT.println("Bomba desligada");
        }
    }

    // Espera para o próximo ciclo
    Serial.println("Entrando em modo de deep sleep por 10 segundos...");
    esp_sleep_enable_timer_wakeup(10 * 1000000);
    esp_deep_sleep_start();
}
