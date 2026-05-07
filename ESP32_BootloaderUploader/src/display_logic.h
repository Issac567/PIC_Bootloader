#ifndef DISPLAY_LOGIC_H
#define DISPLAY_LOGIC_H

#include <Arduino.h>

enum MenuState { MAIN, FLASH, FIRMWARESTART, BT_CHECK, SYSTEM, ABOUT };

// UI Prototypes
void initDisplaySystem();
void handleTouch();
void handleFlashBack();
void handleBack2();
void handleFlashStart();
void drawUI();
void drawMainMenu();
void drawFlashMenu();
void drawBTMenu();
void drawSystemMenu();
void drawAboutMenu();
void drawFlashFirmwareMenu();
void createButton(int x, int y, int w, int h, uint32_t color, const char *label);
void drawBackButton();
void drawBack2Button();
void drawFlashButton();
void changeMenu(MenuState next);
bool checkTouchMount();
bool checkSDMount();
void updateProgressBar(float progress);
void updateProgressLabel(String msg) ;
void updateBlockSizeLabel(int intBlockSize, size_t fileSize, size_t i);
void updateVerifyBytesLabel();
void updateCriticalLabel(String msg, bool isSuccess) ;
void ResetProgressBar();

#endif