#include <Wire.h>
#include <LoRa.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <esp_sleep.h>

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // Cria um objeto do sensor BME280

#define SOIL_MOISTURE_PIN A0 // Pino analógico para o sensor de umidade do solo
#define RELAY_PIN 25 // Pino digital para controle do relé (bomba de aquário)

// Definições para o Sensor de pH
#define PH_PIN 36 // Pino analógico para o sensor de pH (GPIO 36 - VP)
#define PH_OFFSET 0.0 // Ajuste este valor após calibração

// Definições para LoRa
#define SCK_LORA           5
#define MISO_LORA          19
#define MOSI_LORA          27
#define RESET_PIN_LORA     14
#define SS_PIN_LORA        18

#define HIGH_GAIN_LORA     20  /* dBm */
#define BAND               915E6  /* 915MHz de frequência */

#define DEBUG_SERIAL_BAUDRATE 115200

int valorSensor = 0;
int umidadePercentual = 0;

bool init_comunicacao_lora(void);

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

void setup() {
    Serial.begin(DEBUG_SERIAL_BAUDRATE);
    while (!Serial);

    Serial.println("Inicializando...");

    // Configura o pino do relé como saída
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW); // Garante que a bomba está desligada inicialmente

    // Inicializa o sensor BME280
    if (!bme.begin(0x76)) {
        Serial.println("Não foi possível encontrar o sensor BME280! Verifique as conexões.");
        while (1);
    }

    Serial.println("Sensor BME280 inicializado!");

    Wire.begin(); // Inicializa a comunicação I2C
    Serial.println("Comunicação I2C inicializada!");

    // Inicializa a comunicação LoRa
    while (init_comunicacao_lora() == false);

    Serial.println("LoRa inicializado!");

    // Configura o pino do sensor de pH como entrada analógica
    pinMode(PH_PIN, INPUT);
}

void loop() {
    valorSensor = analogRead(SOIL_MOISTURE_PIN);  // Leitura do valor analógico
    umidadePercentual = map(valorSensor, 0, 4095, 0, 100);
    Serial.print("Umidade do Solo lida: ");
    Serial.println(umidadePercentual);

    // Lendo dados do BME280
    float humidity = bme.readHumidity();
    float tempBME = bme.readTemperature();
    float pressure = bme.readPressure() / 100.0F; // Converte de Pa para hPa

    // Lendo o sensor de pH
    int phValue = analogRead(PH_PIN);
    float voltage = phValue * (3.3 / 4095.0); // Conversão do valor analógico para tensão (ESP32 ADC é de 12 bits: 0-4095)
    float pH = voltage * 3.5 + PH_OFFSET; // Ajuste a constante 3.5 conforme a calibração

    // Checa se falhou alguma leitura
    if (isnan(humidity) || isnan(tempBME) || isnan(pressure)) {
        Serial.println("Falha na leitura de algum sensor!");
    } else {
        // Imprime os valores lidos no Serial Monitor
        Serial.print("Umidade do Solo: ");
        Serial.print(umidadePercentual);
        Serial.print("%, Umidade BME280: ");
        Serial.print(humidity);
        Serial.print("%, Temp BME280: ");
        Serial.print(tempBME);
        Serial.print("C, Pressão BME280: ");
        Serial.print(pressure);
        Serial.print(" hPa, pH: ");
        Serial.println(pH);

        // Envia os dados via LoRa
        String data = String(umidadePercentual) + "," + String(humidity) + "," + String(tempBME) + "," + String(pressure) + "," + String(pH);
        LoRa.beginPacket();
        LoRa.print(data);
        LoRa.endPacket();
    }

    // Ativa a bomba de aquário dependendo da umidade do solo
    if (umidadePercentual < 40) {
        Serial.println("Umidade do solo abaixo de 40%. Ativando bomba de aquário por 10 segundos...");
        digitalWrite(RELAY_PIN, HIGH); // Ativa a bomba
        delay(10000); // Mantém a bomba ativada por 10 segundos
        digitalWrite(RELAY_PIN, LOW); // Desativa a bomba
        Serial.println("Bomba de aquário desativada.");
    } else {
        Serial.println("Umidade do solo suficiente. Bomba de aquário permanece desligada.");
        digitalWrite(RELAY_PIN, LOW); // Garante que a bomba está desligada
    }

    // Entrando em modo de deep sleep por 10 segundos
    Serial.println("Entrando em modo de deep sleep por 10 segundos...");
    esp_sleep_enable_timer_wakeup(10 * 1000000); // Define o tempo de deep sleep para 10 segundos (10 milhões de microsegundos)
    esp_deep_sleep_start(); // Coloca o ESP32 em deep sleep
}
