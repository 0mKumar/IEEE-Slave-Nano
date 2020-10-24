#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <dht.h>
#include <RH_NRF24.h>

#define DHT11_PIN 7
#define ONE_WIRE_BUS 2
#define VALVE_RELAY_PIN 6

dht DHT;

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature dallasTemperature(&oneWire);
const int AirValue = 620;
const int WaterValue = 310;
RH_NRF24 nrf24;

bool isValveOpened = false;

#define PACKET_TYPE_SENSOR_DATA 1
#define PACKET_TYPE_MESSAGE 2
#define PACKET_TYPE_RESPONSE 3
#define PACKET_TYPE_INSTRUCTION 4

#define INSTRUCTION_SEND_SENSOR_DATA 1
#define INSTRUCTION_OPEN_VALVE 2
#define INSTRUCTION_CLOSE_VALVE 3

#define RESPONSE_OPENED_VALVE 5
#define RESPONSE_CLOSED_VALVE 6

unsigned long counter = 0;

void reInitialiseNRF() {
    if (!nrf24.init())
        Serial.println("initialization failed");
    if (!nrf24.setChannel(5))
        Serial.println("Channel set failed");
    if (!nrf24.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPowerm18dBm))
        Serial.println("RF set failed");
    nrf24.setModeRx();
}

union DataPacket {
    struct {
        long packet_type;
        unsigned long id;
        union {
            struct {
                long moisture_percent;
                float atmospheric_temperature;
                float humidity;
                float soil_temperature;

                void print() const {
                    Serial.print(F("Sensor Data | "));
                    Serial.print(F("moisture : "));
                    Serial.print(moisture_percent);
                    Serial.print(F(" %\t atm_temp : "));
                    Serial.print(atmospheric_temperature);
                    Serial.print(F(" C\t humidity : "));
                    Serial.print(humidity);
                    Serial.print(F(" \t soil_temp : "));
                    Serial.print(soil_temperature);
                    Serial.println(F(" C"));
                }
            } sensor;

            struct {
                long code;
                char text[RH_NRF24_MAX_MESSAGE_LEN - 12];

                void print() {
                    Serial.print(F("Response "));
                    Serial.print(code);
                    Serial.print(F(" | "));
                    Serial.println(text);
                }
            } response;

            struct {
                long code;
                char text[RH_NRF24_MAX_MESSAGE_LEN - 12];

                void print() {
                    Serial.print(F("Instruction "));
                    Serial.print(code);
                    Serial.print(F(" | "));
                    Serial.println(text);
                }
            } instruction;

            char message[RH_NRF24_MAX_MESSAGE_LEN - 8];
        } data;

        void print() {
            Serial.print("#");
            Serial.print(id);
            Serial.print(": ");
            switch (packet_type) {
                case PACKET_TYPE_SENSOR_DATA:
                    data.sensor.print();
                    break;
                case PACKET_TYPE_MESSAGE:
                    Serial.print(F("Master >> "));
                    Serial.println(data.message);
                    break;
                case PACKET_TYPE_RESPONSE:
                    data.response.print();
                    break;
                case PACKET_TYPE_INSTRUCTION:
                    data.instruction.print();
                    break;
            }
        }
    } packet;

    byte bytes[RH_NRF24_MAX_MESSAGE_LEN];
};

void readAndConsumeDataPacket();

void sendData(uint8_t *bytes, byte length) {
    Serial.println("Sending data...");
    for (int i = 0; i < length; i++) {
        Serial.print(bytes[i]);
        Serial.print(" ");
    }
    Serial.println();
    nrf24.setModeTx();
    nrf24.send(bytes, length);
    if (!nrf24.waitPacketSent()) {
        Serial.println(F("Transmission Failed!!"));
        reInitialiseNRF();
    }
    nrf24.setModeRx();
}

DataPacket createDataPacket(int type) {
    DataPacket packet{{type}};
    packet.packet.id = counter++;
    return packet;
}

void sendMessage(String text) {
    Serial.println(text);

    DataPacket packet = createDataPacket(PACKET_TYPE_MESSAGE);
    memcpy(packet.packet.data.message, text.begin(), text.length());

    Serial.println("Sending message");
    packet.packet.print();

    sendData(packet.bytes, sizeof(packet.bytes));
}

void sendResponse(int responseCode, String text) {
    Serial.println(text);

    DataPacket packet = createDataPacket(PACKET_TYPE_RESPONSE);
    packet.packet.data.response.code = responseCode;
    memcpy(packet.packet.data.response.text, text.begin(), text.length());

    Serial.println("Sending response");
    packet.packet.print();

    sendData(packet.bytes, sizeof(packet.bytes));
}

byte readSoilMoisture() {
    int soilMoistureValue = analogRead(A0);
    int soilMoisturePercent = (int) map(soilMoistureValue, AirValue, WaterValue, 0, 100);
    soilMoisturePercent = constrain(soilMoisturePercent, 0, 100);
    Serial.println(soilMoistureValue);
    Serial.println(soilMoisturePercent);
    return (byte) soilMoisturePercent;
}

int updateDHT() {
    int chk = DHT.read11(DHT11_PIN);
    switch (chk) {
        case DHTLIB_OK:
            Serial.println(F("DHT read success"));
            break;
        case DHTLIB_ERROR_CHECKSUM:
            Serial.println(F("DHT read error checksum"));
            break;
        case DHTLIB_ERROR_TIMEOUT:
            Serial.println(F("DHT read error timeout"));
            break;
        default:
            Serial.println(F("DHT read error"));
    }
//    return chk;
    return DHTLIB_OK;
}

float readSoilTemperature() {
    dallasTemperature.requestTemperatures();
    return dallasTemperature.getTempCByIndex(0);
}

void sendSensorData() {
    DataPacket packet = createDataPacket(PACKET_TYPE_SENSOR_DATA);
    packet.packet.data.sensor.moisture_percent = readSoilMoisture();
    packet.packet.data.sensor.atmospheric_temperature = (float) DHT.temperature;
    packet.packet.data.sensor.humidity = (float) DHT.humidity;
    packet.packet.data.sensor.soil_temperature = readSoilTemperature();
    Serial.println("Sending sensor data...");
    packet.packet.print();
    sendData(packet.bytes, sizeof(packet.bytes));
}


void openValve() {
    if (isValveOpened) return;
    else {
        isValveOpened = true;
        digitalWrite(VALVE_RELAY_PIN, HIGH);
    }
}

void closeValve() {
    if (!isValveOpened) return;
    else {
        isValveOpened = false;
        digitalWrite(VALVE_RELAY_PIN, LOW);
    }
}

void readAndConsumeDataPacket() {
    DataPacket dataPacket{};
    uint8_t len = sizeof(dataPacket.bytes);
    if (nrf24.recv(dataPacket.bytes, &len)) {
        dataPacket.packet.print();
        for (int i = 0; i < len; i++) {
            Serial.print(dataPacket.bytes[i]);
            Serial.print(" ");
        }
        Serial.println();
        switch (dataPacket.packet.packet_type) {
            case PACKET_TYPE_INSTRUCTION:
                switch (dataPacket.packet.data.instruction.code) {
                    case INSTRUCTION_SEND_SENSOR_DATA: {
                        Serial.println(dataPacket.packet.data.instruction.text);
                        switch (updateDHT()) {
                            case DHTLIB_OK:
                                sendSensorData();
                                break;
                        }
                        break;
                    }
                    case INSTRUCTION_OPEN_VALVE: {
                        openValve();
                        sendResponse(RESPONSE_OPENED_VALVE, F("Valve Opened"));
                        break;
                    }
                    case INSTRUCTION_CLOSE_VALVE: {
                        closeValve();
                        sendResponse(RESPONSE_CLOSED_VALVE, F("Valve Closed"));
                        break;
                    }
                    default:
                        Serial.println(F("Unknown Instruction Received"));
                }
        }
    } else {
        Serial.println(F("Receive failed"));
    }
}

void setup() {
    Serial.begin(9600);
    dallasTemperature.begin();
    while (!Serial);
    reInitialiseNRF();
    delay(1000);
}

//unsigned long count = 0;

void loop() {
    while (nrf24.available()) {
        readAndConsumeDataPacket();
    }

//    if(count == 500000){
//        sendMessage("Hi");
//        count = 0;
//    }
//    count++;
}