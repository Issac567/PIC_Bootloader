#ifndef FLASH_H
#define FLASH_H

void sendHandShakeBytes();
void sendConfigBytes();
void sendFirmwareBytes();
bool isOperationFailed();
void disableFunction();

extern unsigned long previousMillis;
extern const long millisNoDelayValue;
#endif