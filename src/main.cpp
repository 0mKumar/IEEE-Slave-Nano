#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <dht.h>
#include <RH_NRF24.h>

unsigned long StartTime, CurrentTime, ElapsedTime;

dht DHT;
#define DHT11_PIN 7
#define ONE_WIRE_BUS 2

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
const int AirValue = 620;   //you need to replace this value with Value_1
const int WaterValue = 310;  //you need to replace this value with Value_2
int soilMoistureValue = 0;
int soilmoisturepercent = 0;
RH_NRF24 nrf24;
int flag = false;
bool opened = false;
String data = "";
int relayPin = 6;
int cnt = 0;

bool stringCompare(String a, uint8_t *b) {
    for (int i = 0; i < a.length(); i++) {
        if (a[i] != b[i]) {
            return false;
        }
    }
    return true;
}

void sendData(uint8_t data[]) {
    nrf24.send(data, sizeof(data));
    if (!nrf24.waitPacketSent()) {
        Serial.println("Tramission Failed!!");
    }
}


void sendSensorData(byte moisture, float dht_temp, float humidity, float soil_temp) {
    uint8_t arr[14];
    arr[0] = moisture;
    byte *b = (byte *) &dht_temp;
    arr[1] = b[0];
    arr[2] = b[1];
    arr[3] = b[2];
    arr[4] = b[3];
    byte *c = (byte *) &humidity;
    arr[5] = c[0];
    arr[6] = c[1];
    arr[7] = c[2];
    arr[8] = c[3];
    byte *d = (byte *) &soil_temp;
    arr[9] = d[0];
    arr[10] = d[1];
    arr[11] = d[2];
    arr[12] = d[3];
    arr[13] = 0;
    sendData(arr);
}

void sendData(String data) {
    Serial.println(data);
    flag = true;
    StartTime = millis();
    uint8_t dataArray[data.length()];
    data.getBytes(dataArray, data.length());
    nrf24.send(dataArray, sizeof(dataArray));
    if (!nrf24.waitPacketSent()) {
        Serial.println("Tramission Failed!!");
    }
}

void setup() {
    Serial.begin(9600);
    sensors.begin();
    while (!Serial);
    if (!nrf24.init())
        Serial.println("initialization failed");
    if (!nrf24.setChannel(5))
        Serial.println("Channel Set failed");
    if (!nrf24.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm))
        Serial.println("RF set failed");
    delay(1000);
}

byte moisture_percent() {
    soilMoistureValue = analogRead(A0);  //put Sensor insert into soil
    soilmoisturepercent = map(soilMoistureValue, AirValue, WaterValue, 0, 100);
    soilmoisturepercent = constrain(soilmoisturepercent, 0, 100);
    Serial.println(soilMoistureValue);
    Serial.println(soilmoisturepercent);
    return (byte) soilmoisturepercent;
}

String atmospheric_temp_n_humid() {
    int chk = DHT.read11(DHT11_PIN);
    return String(DHT.temperature) + String(" ") + String(DHT.humidity) + String(" ");
}

float soil_temp() {
    sensors.requestTemperatures();
    return sensors.getTempCByIndex(0);
    /*//print the temperature in Celsius
      Serial.print("Temperature: ");
      Serial.print(sensors.getTempCByIndex(0));
      Serial.println("°C");//shows degrees character*/
}

void OpenValve() {
    if (opened)
        return;
    else {
        opened = true;
        digitalWrite(relayPin, HIGH);
    }
}

void CloseValve() {
    if (!opened)
        return;
    else {
        opened = false;
        digitalWrite(relayPin, LOW);
    }
}

void loop() {
    Serial.println("loop");
    Serial.println(flag);
    if (flag == false && nrf24.available()) {
        data = "Data ";
        uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];
        uint8_t len = sizeof(buf);
        if (nrf24.recv(buf, &len)) {
            Serial.println((char *) buf);
            String str1 = "Attempt to connect";
            String str2 = "Open Valve";
            String str3 = "Close Valve";
            bool attempt, openvalve, closevalve;
            int i = 0;

            attempt = stringCompare(str1, buf);
            openvalve = stringCompare(str2, buf);
            closevalve = stringCompare(str3, buf);

            if (attempt) {
                Serial.print("Received: ");
                Serial.println((char *) buf);
                int chk = DHT.read11(DHT11_PIN);
                sendSensorData(moisture_percent(), DHT.humidity, DHT.temperature, soil_temp());
            } else if (openvalve) {
                OpenValve();
                data = String("Valve Opened ");
                sendData(data);
            } else if (closevalve) {
                CloseValve();
                data = String("Valve Closed ");
                sendData(data);
            } else {
                Serial.println("Entire data is not received");
            }
        } else {
            Serial.println("recv failed");
        }
    }
    Serial.println(flag);
    if (flag) {
        cnt = 0;
        Serial.println(data);
        uint8_t dataArray[data.length()];
        data.getBytes(dataArray, data.length());
        nrf24.send(dataArray, sizeof(dataArray));
        if (!nrf24.waitPacketSent()) {
            Serial.println("Tramission Failed!!");
        }
    }


    CurrentTime = millis();
    ElapsedTime = CurrentTime - StartTime;
    Serial.println(ElapsedTime);

    if (ElapsedTime > 5000) {
        if (!nrf24.init())
            Serial.println("initialization failed");
        if (!nrf24.setChannel(5))
            Serial.println("Channel Set failed");
        if (!nrf24.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm))
            Serial.println("RF set failed");
        cnt = 0;

        flag = false;
        StartTime = millis();
    }


}