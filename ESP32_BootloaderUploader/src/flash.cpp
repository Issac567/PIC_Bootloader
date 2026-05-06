#include "globals.h"
#include "flash.h"
#include "display_logic.h"
#include "ble.h"
#include "sdcard.h"

PicStatus myPicStatus;
uint16_t intMTUSize = 20;

//-----------------------------------------------------------
//Firmware uploader handler
//-----------------------------------------------------------
void sendHandShakeBytes()
{
    bool blnToggle = false;
    uint8_t b_55[1] = {0x55};
    uint8_t b_AA[1] = {0xAA};

    disableFunction();
    updateProgressLabel("Please wait...");

    // Loop until handshake succeeds or user cancels
    while (true)
    {
        // 1. Success Check (Set by notifyCallback/handleMessage)
        if (myPicStatus.blnHandShakeSuccess == true)
        {
            delay(300); // Give firmware time to stabilize
            sendConfigBytes();
            return;
        }
        else
        {
            // 2. Send Toggle Bytes via NimBLE
            if (blnToggle == false)
            {
                // Write 0x55 to the PIC
                pRemoteCharacteristic->writeValue(b_55, 1, false);
                Serial.println("HANDSHAKE: Sending: 0x55");
            }
            else
            {
                // Write 0xAA to the PIC
                pRemoteCharacteristic->writeValue(b_AA, 1, false);
                Serial.println("HANDSHAKE: Sending: 0xAA");
            }
        }

        // 3. Delay to avoid flooding the UART/PIC buffer
        uint16_t getHSDelay = myConfig.intHandShakeDelayMS;
        while(getHSDelay --)
        {
            handleTouch();

            // 4. Status Check (User abort/System status)
            if (getBooleanStatus() == true) 
            {
                updateCriticalLabel("Failed!", false);
                return;
            }

            if (myPicStatus.blnUserCancel == true) 
            {
                changeMenu(MAIN);
                return;
            }
            
            delay(1);
        }

        // 5. Flip the toggle
        blnToggle = !blnToggle;
    }
}

void sendConfigBytes()
{
    // Use your global or defined UUID for the characteristic
    uint8_t byteONE[1];
    uint8_t byteTWO[1];
    uint8_t byteTHREE[1];

    // 1. Set the BLE Flag (0x01)
    byteONE[0] = 0x01;

    // 2. Extract High Byte (MSB) 
    // Shift right by 8 bits to move the top 8 bits into the bottom
    byteTWO[0] = (uint8_t)((intMTUSize & 0xFF00) >> 8);

    // 3. Extract Low Byte (LSB)
    // Mask to keep only the bottom 8 bits
    byteTHREE[0] = (uint8_t)(intMTUSize & 0x00FF);

    // Write packets with 50ms gaps as per your B4J logic
    pRemoteCharacteristic->writeValue(byteONE, 1, false);
    delay(50);
    
    pRemoteCharacteristic->writeValue(byteTWO, 1, false);
    delay(50);
    
    pRemoteCharacteristic->writeValue(byteTHREE, 1, false);
    delay(50);

    // Debugging output to Serial Monitor
    Serial.printf("CFG BYTES: Sending: 0x%02X, 0x%02X, 0x%02X\n", byteONE[0], byteTWO[0], byteTHREE[0]);

    // Wait for the PIC to confirm configuration via notifyCallback
    while (myPicStatus.blnConfigOK == false)
    {
        handleTouch();
        
        if (getBooleanStatus() == true) 
        {
            updateCriticalLabel("Failed!", false);
            return;
        }
        if (myPicStatus.blnUserCancel == true) 
        {
            changeMenu(MAIN);
            return;
        }
        
        delay(10);
    }

    while (myPicStatus.blnEndFlashErase == false)
    {
        handleTouch();

        if (getBooleanStatus() == true) 
        {
            updateCriticalLabel("Failed!", false);
            return;
        }
        if (myPicStatus.blnUserCancel == true) 
        {
            changeMenu(MAIN);
            return;
        }
        
        delay(10);
    }

    ResetProgressBar();
    delay(300);             // give PIC time to enter Receive firmware mode!
    sendFirmwareBytes();
}

void sendFirmwareBytes()
{
    // 1. Calculate Block Size based on PIC Architecture
    int intBlockSize;
    if (myConfig.blnUse4Padding == true)
    {
        intBlockSize = myConfig.intInstructionPacket * 4;
    }
    else
    {
        intBlockSize = myConfig.intInstructionPacket * 2;
    }

    // 2. Open the binary file from the SD card
    File dataFile = SD.open(FLASH_FILE, FILE_READ);
    if (!dataFile)
    {
        updateCriticalLabel("Error opening flash.bin!", false);
        Serial.println("FIRMWAREUPLOAD: Error opening flash.bin");
        return;
    }

    size_t fileSize = dataFile.size();
    uint8_t block[intBlockSize];
    
    Serial.printf("FIRMWAREUPLOAD: size: %d, bytes/block: %d\n", fileSize, intBlockSize);

    size_t i = 0;
    while (i < fileSize)
    {
        handleTouch();

        // 3. Read Block from SD and Pad with 0xFF if necessary (REMOVE THIS NOT NECCESSARY!!  ALSO IN B4J??)
        size_t bytesRead = dataFile.read(block, intBlockSize);
        if (bytesRead < intBlockSize)
        {
            for (size_t j = bytesRead; j < intBlockSize; j++)
            {
                block[j] = 0xFF; // Padding
            }
        }

        // 4. Check for Global Abort
        if (getBooleanStatus() == true)
        {
            updateCriticalLabel("Failed!", false);
            dataFile.close();
            return;
        }
        if (myPicStatus.blnUserCancel == true) 
        {
            dataFile.close();
            changeMenu(MAIN);
            return;
        }
              
        
        // 5. Wait for ACK or handle ISR Timeout
        // Note: The ACK is set to true in your handleMessage()
        while (myPicStatus.blnWriteACK == false)
        {
            handleTouch();
 
            if (getBooleanStatus() == true)
            {
                updateCriticalLabel("Failed!", false);
                dataFile.close();
                return;
            }
            if (myPicStatus.blnUserCancel == true) 
            {
                dataFile.close();
                changeMenu(MAIN);
                return;
            }

            if (myPicStatus.blnISRTimeOut == true)
            {
                Serial.printf("FIRMWAREUPLOAD: Timeout detected, retrying at byte #%d\n", i);
                myPicStatus.blnISRTimeOut = false;
                //dataFile.seek(i); // Move SD file pointer back to start of this block
                //dataFile.read(block, intBlockSize); // Reload the block
                // Continue loop logic occurs naturally here
            }
            delay(1); // Small yield for background tasks
        }
        
        myPicStatus.blnWriteACK = false;

        // 6. Fragmentation Logic (BLE MTU Handling)
        size_t chunkSize = (intMTUSize < 20) ? 20 : intMTUSize;

        if (intBlockSize <= chunkSize)
        {
            bool blnStatus = pRemoteCharacteristic->writeValue(block, intBlockSize, false);
            delay(10);
        }
        else
        {
            for (size_t x = 0; x < intBlockSize; x += chunkSize)
            {
                size_t currentChunkSize = (intBlockSize - x < chunkSize) ? (intBlockSize - x) : chunkSize;
                bool blnStatus = pRemoteCharacteristic->writeValue(&block[x], currentChunkSize, false);
                delay(10);
                // No delay required as per B4J testing!  We need it here??  It tested success without delays??
            }
        }

        delay(myConfig.intPacketDelayMS);

        // 7. Update Progress (Map this to your ILI9488 UI)
        float progress = (float)(i + intBlockSize) / fileSize;
        updateProgressBar(progress); 

        // 8. Update the Block Status Text
        updateBlockSizeLabel(intBlockSize, fileSize, i);
  
        i += intBlockSize;
    }

    dataFile.close();
    Serial.println("FIRMWAREUPLOAD: Firmware upload completed!");

    ResetProgressBar();  // for verifyflash!
}

bool getBooleanStatus()
{
    // 3 timeouts = this
    if (myPicStatus.blnTimeoutError == true) 
    {
        return true;
    }

    // Exit if BLE is NOT connected (connection lost)
    if (bleIsConnected() == false)
    {
        Serial.println("STATUS: Connection Lost!");
        return true;
    }

    // If everything is fine, return false (don't exit the loop)
    return false;
}

void disableFunction()
{
	myPicStatus.blnHandShakeSuccess = false;
    myPicStatus.blnConfigOK = false;
	myPicStatus.blnTimeoutError = false;
	myPicStatus.blnISRTimeOut = false;
    myPicStatus.blnWriteACK = false;
	myPicStatus.blnStartVerifyRequest = false;
    myPicStatus.blnEndFlashErase = false;
	myPicStatus.cntVerify = 0;
    myPicStatus.blnUserCancel = false;
}

