#include <Arduino.h>
#include <RF24.h>
#include <SPI.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <printf.h>

#define BUTTON_PIN  2
#define LED_PIN     9   // status LED
#define NRF24_CE    7
#define NRF24_CS    8
#define COMMS_PIPE (uint8_t *)"CfHRB"
const uint8_t dipSwitchPins[] = {3, 4, 5, 6}; // set the address of the button by pulling the pins to GND (inverted logic)

const uint8_t dipSwitchPinCount = sizeof(dipSwitchPins) / sizeof(dipSwitchPins[0]);
RF24 radio(NRF24_CE, NRF24_CS);

typedef struct {
    uint8_t header = 0x42;
    uint8_t address;
    uint8_t btnStates;
} btnData_t;

void wakeInterrupt() {
    detachInterrupt(digitalPinToInterrupt(BUTTON_PIN)); // detach interrupt so it won't fire continuosly
    sleep_disable();
}

void sleep() {
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), wakeInterrupt, LOW);
    delay(100);

    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sleep_mode();

    // program now sleeps and will continue from here

    sleep_disable(); // so first disable sleep
}

uint8_t getDipSwitchAddress() {
    uint8_t addr;
    for (uint8_t i = 0; i < dipSwitchPinCount; i++) {
        addr |= !digitalRead(dipSwitchPins[i]) << i;
    }
    return addr;
}

void setup() {
    Serial.begin(115200);
    Serial.println(F("\nNRF24-Remote-Button Transmitter\n\n"));
    btnData_t d;
    Serial.println(d.header);

    for (uint8_t i = 0; i < dipSwitchPinCount; i++) {
        pinMode(dipSwitchPins[i], INPUT_PULLUP);
    }
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(LED_PIN, OUTPUT);

    radio.begin();
    radio.setDataRate(RF24_250KBPS); // hopefully increase range a bit
    radio.setPALevel(RF24_PA_MAX);
    radio.openWritingPipe(COMMS_PIPE);
    radio.printDetails();
    radio.stopListening();
}

bool sendPacket(uint8_t address, uint8_t btnStates) {
    btnData_t data;
    data.address = address;
    data.btnStates = btnStates;

    printf("Sending packet: [%02x %02X %02X]\n", data.header, data.address, data.btnStates);
    radio.stopListening();  // stop rx to tx
    bool state = radio.write(&data, sizeof(btnData_t));
    if (!state) {
        Serial.println("Error: Sending failed");
    }
    return state;
}

void loop() {
    radio.powerDown();
    sleep(); // will hang here until button is pressed
    digitalWrite(LED_PIN, HIGH);
    // radio.powerUp(); // should automatically wake up on next write()

    bool ok = sendPacket(getDipSwitchAddress(), 0x01);

    if (!ok) {
        // blink LED to show error
        radio.powerDown(); // but power down radio beforehand
        for (uint8_t i = 0; i < 3; i++) {
            digitalWrite(LED_PIN, HIGH);
            delay(100);
            digitalWrite(LED_PIN, LOW);
            delay(100);
        }
    } else {
        digitalWrite(LED_PIN, LOW);
    }
}