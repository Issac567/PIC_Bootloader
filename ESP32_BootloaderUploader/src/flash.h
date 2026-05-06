#ifndef FLASH_H
#define FLASH_H

void sendHandShakeBytes();
void sendConfigBytes();
void sendFirmwareBytes();
bool getBooleanStatus();
void disableFunction();

#endif