#ifndef FLASH_H
#define FLASH_H

void sendHandShakeBytes();
void continueHandshake();
void sendConfigBytes();
void sendFirmwareBytes();
bool getBooleanStatus();
bool handleStatus();
void disableFunction();

extern unsigned long previousMillis;
extern const long millisNoDelayValue;
#endif