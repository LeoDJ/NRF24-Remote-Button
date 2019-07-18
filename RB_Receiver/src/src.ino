#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <printf.h>

#define OUTPUT_PIN  2
#define LISTEN_ADDR 0x00
#define NRF24_CE    7
#define NRF24_CS    8
#define COMMS_PIPE  (uint8_t*)"CfHRB"

RF24 radio(NRF24_CE, NRF24_CS);

typedef struct {
    uint8_t header;
    uint8_t address;
    uint8_t btnStates;
} btnData_t;

void setup() {
    Serial.begin(115200);
    Serial.println(F("\nNRF24-Remote-Button Receiver\n\n"));

    pinMode(OUTPUT_PIN, OUTPUT);

    radio.begin();
    radio.setDataRate(RF24_250KBPS); // hopefully increase range a bit
    radio.openReadingPipe(1, COMMS_PIPE);
    radio.printDetails();
}

void pulseOutput() {
    digitalWrite(OUTPUT_PIN, HIGH);
    delay(50);
    digitalWrite(OUTPUT_PIN, LOW);
}

void loop() {
    if(radio.available()) {
        btnData_t data;
        radio.read(&data, sizeof(btnData_t));
        printf("Received packet: [%02x %02X %02X]\n", data.header, data.address, data.btnStates);

        if(data.header == 0x42) {
            if(data.address == LISTEN_ADDR) {
                if(data.btnStates & 0x01) {
                    Serial.println("Got valid packet, pulsing output");
                    pulseOutput();
                }
            }
        }
    }
}