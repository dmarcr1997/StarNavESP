#include "screen_press.h"

static const uint8_t TOUCH_I2C_ADDR = 0x38;

void touchInit() {
    Wire.begin();

    pinMode(TOUCH_RST_PIN, OUTPUT);
    digitalWrite(TOUCH_RST_PIN, LOW);
    delay(10);
    digitalWrite(TOUCH_RST_PIN, HIGH);
    delay(50);

    pinMode(TOUCH_INT_PIN, INPUT_PULLUP);
    Serial.println("I2C Touch init done");
}
uint8_t readRegister(uint8_t register) {
    Wire.beginTransmission(TOUCH_I2C_ADDR);
    Wire.write(register);
    Wire.endTransmission(false);

    Wire.requestFrom((uint8_t)TOUCH_I2C_ADDR, (uint8_t)1);
    if(Wire.available()) {
        return Wire.read();
    }
    return 0;
}
bool screenPressedPoints() {
    uint8_t points = readRegister(0x02);
    /*
    | Register | Meaning                |
    | -------- | ---------------------- |
    | `0x02`   | Number of touch points |
    | `0x03`   | X high byte            |
    | `0x04`   | X low byte             |
    | `0x05`   | Y high byte            |
    | `0x06`   | Y low byte             |
    */
    return (points > 0);
}
bool screenPressed() {
    int v = digitalRead(TOUCH_INT_PIN);
    return (v == LOW );
}