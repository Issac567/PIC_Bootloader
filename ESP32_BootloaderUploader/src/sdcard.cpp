#include "globals.h"
#include "sdcard.h"
#include "display_logic.h"

ConfigMap myConfig;
const char* FLASH_FILE = "/flash.bin";
const char* CONFIG_FILE = "/config.map";

void initSDSystem() 
{
    // --- IMPORTANT: QUIET THE SD CARH ---
    pinMode(SD_CS, OUTPUT);
    digitalWrite(SD_CS, HIGH);      // SD Idle

    // INITIALIZE SD CARD (Pass the shared SPI) ---
    // Note: SD.begin takes (SS_PIN, SPI_BUS, FREQUENCY)
    if (!SD.begin(SD_CS, touchSPI, 16000000)) { // 16MHz is stable for S3
        Serial.println("SD Card mount failed!");
    } else {
        Serial.println("SD Card mounted.");
    }
}

void verifyStatus()
{
    if (compareFiles("/verify.bin", FLASH_FILE) == true)
    {
        updateCriticalLabel("Passed!", true);
        Serial.println("File Success same!!!");
    } else {
        updateCriticalLabel("Failed!", false);
        Serial.println("File not same!!!");
    }
}

bool compareFiles(const char* path1, const char* path2)
{
    File file1 = SD.open(path1, FILE_READ);
    File file2 = SD.open(path2, FILE_READ);

    if (!file1 || !file2 || file1.size() != file2.size())
    {
        if (file1) file1.close();
        if (file2) file2.close();
        return false; // Sizes don't match or file missing
    }

    while (file1.available())
    {
        if (file1.read() != file2.read())
        {
            file1.close();
            file2.close();
            return false; // Found a mismatch!
        }
    }

    file1.close();
    file2.close();
    return true; // Files are identical
}

String GetConfigInfo() 
{
    // Check for mount
    if (!SD.begin(SD_CS)) 
    {
        Serial.println("SD Card mount failed!");
        return "SD Card mount failed!";
    }

    // Check flash.bin
    File binFile = SD.open(FLASH_FILE);
    if (!binFile) 
    {
        Serial.println("flash.bin is missing!");
        return "flash.bin is missing!";
    }

    // Check config.map
    File configFile = SD.open(CONFIG_FILE);
    if (!configFile) 
    {
        Serial.println("config.map is missing!");
        return "config.map is missing!";
    }

    // Parse the config.map into variables
    while (configFile.available()) 
    {
        String line = configFile.readStringUntil('\n');
        line.trim();

        // Skip comments or empty lines
        if (line.startsWith("#") || line.length() == 0) continue;

        // Split Key and Value
        int separatorIndex = line.indexOf('=');
        if (separatorIndex == -1) continue;

        String key = line.substring(0, separatorIndex);
        String val = line.substring(separatorIndex + 1);
        key.trim();
        val.trim();

        // Assign to struct based on Key
        if (key == "StartAddrFlash")      myConfig.intStartAddrFlash = strtoul(val.c_str(), NULL, 10);
        else if (key == "EndAddrFlash")    myConfig.intEndAddrFlash = strtoul(val.c_str(), NULL, 10);
        else if (key == "EmptyFlashValue") myConfig.intEmptyFlashValue = val.toInt();
        else if (key == "InstructionPacket") myConfig.intInstructionPacket = val.toInt();
        else if (key == "PacketDelayMS")    myConfig.intPacketDelayMS = val.toInt();
        else if (key == "HandShakeDelayMS") myConfig.intHandShakeDelayMS = val.toInt();
        else if (key == "StopBit")          myConfig.intStopBit = val.toInt();
        else if (key == "Notes")            myConfig.strNotes = val;
        else if (key == "PicName")          myConfig.strPicName = val;
        else if (key == "ExpectedBytes")    myConfig.intExpectedFirmwareBytes = val.toInt();
        else if (key == "UseWriteBurst")    myConfig.blnUseWriteBurst = (val == "true");
        else if (key == "UseDoubleHexAddr") myConfig.blnUseDoubleHexAddr = (val == "true");
        else if (key == "Use4Padding")      myConfig.blnUse4Padding = (val == "true");
    }

    configFile.close();
    Serial.println("Config successfully loaded.");

    return "OK";
}
