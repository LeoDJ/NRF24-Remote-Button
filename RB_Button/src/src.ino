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
const uint8_t dipSwitchPins[] = {3, 4, 5, 6}; // set the address of the button by pulling the pins to GND (inverted logic)
#define LOW_BAT     3150    //mV when battery is considered empty
#define COMMS_PIPE (uint8_t *)"CfHRB"

const uint8_t dipSwitchPinCount = sizeof(dipSwitchPins) / sizeof(dipSwitchPins[0]);
RF24 radio(NRF24_CE, NRF24_CS);

typedef struct {
    uint8_t header = 0x42;
    uint8_t address;
    uint8_t btnStates;
    uint8_t vcc20;      // voltage in mV/20
} btnData_t;

long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

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
    radio.powerDown(); // power down radio, because we'll first sleep in loop anyways
}

bool sendPacket(uint8_t address, uint8_t btnStates, long vcc) {
    btnData_t data;
    data.address = address;
    data.btnStates = btnStates;
    data.vcc20 = vcc / 20;

    radio.stopListening();  // stop rx to tx
    bool state = radio.write(&data, sizeof(btnData_t));
    printf("Sent packet: [%02x %02X %02X %02X]\n", data.header, data.address, data.btnStates, data.vcc20);
    if (!state) {
        Serial.println("Error: Sending failed");
    }
    radio.powerDown(); // but power radio right afterwards
    return state;
}

void loop() {
    sleep(); // will hang here until button is pressed
    digitalWrite(LED_PIN, HIGH);
    // radio.powerUp(); // should automatically wake up on next write()

    long vcc = readVcc();
    printf("VCC: %ld mV\n", vcc);

    bool ok = sendPacket(getDipSwitchAddress(), 0x01, vcc);

    if (!ok) {
        // blink LED to show error
        for (uint8_t i = 0; i < 3; i++) {
            digitalWrite(LED_PIN, HIGH);
            delay(100);
            digitalWrite(LED_PIN, LOW);
            delay(100);
        }
    } else {
        digitalWrite(LED_PIN, LOW);
    }

    if(vcc < LOW_BAT) {
        // two slow blinks for low battery
        for (uint8_t i = 0; i < 2; i++) {
            digitalWrite(LED_PIN, HIGH);
            delay(250);
            digitalWrite(LED_PIN, LOW);
            delay(250);
        }
    }
}