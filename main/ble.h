
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

void bleInit(void);
void ble_set_attr_value(uint16_t length, const uint8_t *value);
void ble_send_indicate(uint16_t length, const uint8_t *value);