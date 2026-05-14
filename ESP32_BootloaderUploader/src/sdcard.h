#ifndef SDCARD_H
#define SDCARD_H

#include <Arduino.h>

void initSDSystem();
void verifyStatus();
bool compareFiles(const char* path1, const char* path2);
String GetConfigInfo();

extern const char* FLASH_FILE;
extern const char* CONFIG_FILE;
extern const char* VERIFY_FILE;


#endif