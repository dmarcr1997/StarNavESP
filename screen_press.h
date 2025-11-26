#include <Wire.h>

#define TOUCH_INT_PIN   D7
#define TOUCH_RST_PIN   D2

void touchInit();
uint8_t touchRegisterRead(uint8_t register);
bool screenPressed();