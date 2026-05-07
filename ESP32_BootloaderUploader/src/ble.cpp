#include "globals.h"
#include "ble.h"
#include "display_logic.h"
#include "sdcard.h"

bool doConnect = false;
bool doScan = true;
String rxBufferString;
File verifyFile;

// Shared variables (extern from header)
NimBLEAdvertisedDevice* myDevice = nullptr;
NimBLERemoteCharacteristic* pRemoteCharacteristic = nullptr;

// Internal (file-local) variables
static String targetDeviceName = "HM10";
static NimBLEUUID serviceUUID("0000ffe0-0000-1000-8000-00805f9b34fb");
static NimBLEUUID charUUID((uint16_t)0xFFE1);

static NimBLEClient* pClient = nullptr;
static NimBLERemoteService* pRemoteService = nullptr;

//-----------------------------------------------------------
//Connect/Disconnect event
//-----------------------------------------------------------
class MyClientCallback : public NimBLEClientCallbacks 
{
    void onConnect(NimBLEClient* pClient) override
    {
        Serial.println(" - onConnect");
    }

    void onDisconnect(NimBLEClient* pClient, int reason) override
    {
        myPicStatus.blnUserCancel = true;
        Serial.println(" - onDisconnect");
        doScan = true; 
    }
};

//-----------------------------------------------------------
// * Scan for BLE servers and find the first one that advertises the service uuid or name we are looking for.
//-----------------------------------------------------------
class MyAdvertisedDeviceCallbacks : public NimBLEScanCallbacks 
{
    void onResult(const NimBLEAdvertisedDevice* advertisedDevice) override 
    {
        Serial.println("------------------------------");
        Serial.println("BLE Advertised Device found:");

        if (advertisedDevice->haveName()) 
        {
            Serial.println(advertisedDevice->getName().c_str());
        } else {
            Serial.println("(none)");
        }

        Serial.println(advertisedDevice->getAddress().toString().c_str());
        Serial.println(advertisedDevice->getRSSI());

        bool foundByUUID = advertisedDevice->isAdvertisingService(serviceUUID);
        bool foundByName = false;
        if (advertisedDevice->haveName()) {
            foundByName = (String(advertisedDevice->getName().c_str()) == targetDeviceName);
        }

        if (foundByUUID || foundByName) 
        {
            NimBLEDevice::getScan()->stop();
            
            if (myDevice) 
            {
                delete myDevice;
                myDevice = nullptr;
            }

            myDevice = new NimBLEAdvertisedDevice(*advertisedDevice);  
            doConnect = true;
            Serial.println("Ready to connect...");
        }
    }
};

//-----------------------------------------------------------
// Callback function to handle notifications
//-----------------------------------------------------------
static void notifyCallback(NimBLERemoteCharacteristic* pRemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) 
{
    //Serial.print("Notify callback for characteristic ");
    //Serial.print(pRemoteCharacteristic->getUUID().toString().c_str());
    //Serial.print(" of data length ");
    //Serial.println(length);
    //Serial.print("data: ");
    // NimBLE is very efficient; pData points directly to the received buffer
    //Serial.write(pData, length);
    //Serial.println();

    rxBufferString += String((char*)pData, length);

    if (rxBufferString.indexOf(">") > -1 || myPicStatus.blnStartVerifyRequest == true)
    {
        // Call your message handler (passing the string and your data buffer)
        handleMessage(rxBufferString, pData, length);
        
        // Clear the string for the next message (equivalent to rxBufferString = "")
        rxBufferString = "";
    }
}

//-----------------------------------------------------------
//Connection to server + BLE Scan
//-----------------------------------------------------------
bool bleconnectToServer() 
{
    doConnect = false;          // prevent repeated attempts

    // 1. Important to delete client or it will crash anytime you reconnect.
    Serial.println("Cleaning up old client memory...");
    if (pClient != nullptr) 
    {
        pClient->disconnect();
        NimBLEDevice::deleteClient(pClient); 
        pClient = nullptr; 
    }
    // Clear old references
    pRemoteService = nullptr;
    pRemoteCharacteristic = nullptr;
        
    Serial.println("Starting Pre-Flight Check...");
    Serial.print("Forming a connection to ");
    Serial.println(myDevice->getAddress().toString().c_str());

    // 2. NimBLE uses NimBLEDevice
    pClient = NimBLEDevice::createClient();
    Serial.println(" - Created client");

    pClient->setClientCallbacks(new MyClientCallback()); // 'true' handles automatic deletion

    // 3. Connect to the remote server. 
    // pClient->setConnectionParams(12, 24, 0, 400);
    if (!pClient->connect(myDevice)) 
    {
        Serial.println(" - Failed to connect to server");
        NimBLEDevice::deleteClient(pClient);
        return false;
    }
    pClient->updateConnParams(16, 32, 0, 100);  // 1 seconds timeout.  when powered off it takes 1 seconds to update the status of connection
    Serial.println(" - Connected to server");
    
    // 4. Obtain a reference to the service
    pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) 
    {
        Serial.print("Failed to find our service UUID: ");
        Serial.println(serviceUUID.toString().c_str());
        pClient->disconnect();
        return false;
    }
    Serial.println(" - Found our service");
    
    // 5. Obtain a reference to the characteristic
    pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
    if (pRemoteCharacteristic == nullptr) 
    {
        Serial.print("Failed to find our characteristic UUID: ");
        Serial.println(charUUID.toString().c_str());
        pClient->disconnect();
        return false;
    }
    Serial.println(" - Found our characteristic");

    
    // 6. Subscription Logic
    if (pRemoteCharacteristic->canNotify()) 
    {
        // NimBLE uses subscribe() instead of registerForNotify()
        // First param is true for notifications, false for indications
        if (pRemoteCharacteristic->subscribe(true, notifyCallback)) 
        {
            Serial.println(" - Subscribed to notifications");
        } else {
            Serial.println(" - Failed to subscribe");
            pClient->disconnect();
            return false;
        }
    } else {
        Serial.println(" - Characteristic does not support notifications");
        pClient->disconnect();
        return false;
    }
    
    return true;
}

void handleConnection()
{
    if (doConnect == true && doScan == false) 
    {
        if (bleconnectToServer()) {
            Serial.println("We are now connected to the BLE Server.");
        } else {
            Serial.println("We have failed to connect to the server; there is nothing more we will do.");
            doScan = true; 
        }
    }

    if (bleIsConnected() == false) 
    {
        myPicStatus.blnStartVerifyRequest = false;      // just in case verify is running and you power off the HM10!
    }
}

void handleBleScan()
{
    if (!bleIsConnected() && doConnect == false && doScan == true) 
    {
        Serial.println("BLE is not connected, attempting to reconnect...");
        bleDoScan();
    }
}

void bleDoScan() 
{
    Serial.println("Starting Arduino NimBLE Client application...");
    doScan = false;     // prevent repeated attempts

    NimBLEDevice::init("");
    NimBLEScan* pBLEScan = NimBLEDevice::getScan();

    pBLEScan->setScanCallbacks(new MyAdvertisedDeviceCallbacks(), true);
    pBLEScan->setActiveScan(true);
    pBLEScan->setInterval(100);
    pBLEScan->setWindow(90);
    pBLEScan->start(0, false);
}

bool bleIsConnected()
{
    if (pClient && pClient->isConnected())
        return true;

    return false;
}

void bleDisconnect()
{
    if (pClient) {
        pClient->disconnect();
    }
}

// Used with flash.cpp
void handleMessage(String msg, uint8_t* rawBytes, size_t length)
{
    // We dont want to log Incoming <ACK> while Firmware upload!
	// We dont want to log Incoming PIC VERIFY bytes
    if (myPicStatus.blnStartVerifyRequest != true && msg != "<ACK>")
    {
        // Print to the PlatformIO Serial Monitor
        Serial.print("PIC: ");
        Serial.println(msg);
    }  

    // Check if we are in Verification Mode
    if (myPicStatus.blnStartVerifyRequest == true) {
        myPicStatus.cntVerify += length;

        // Check for completion based on byte count
        if (myPicStatus.cntVerify >= myConfig.intExpectedFirmwareBytes)
        {
            // Without this, Last iteration will cause issues! Due to low 150ms delay in PIC after msg send!
            myPicStatus.blnStartVerifyRequest = false;

            // we let <EndFlashVerify> handle the UI transition.
            Serial.println("STATUS: All verify bytes received.");
        }

        // Append incoming bytes directly to SD to save RAM
        if (verifyFile)
        {
            verifyFile.write(rawBytes, length);
        }

        //if ((myPicStatus.blnUserCancel == true) && (currentMenu != FIRMWARESTART))
        if (myPicStatus.blnUserCancel == true)
        {
            // Can't do much with verify when its going.  it has to exhaust all sent bytes from PIC.
            // Currently, no shutting off Verify_Flash!!
            return;
        } else {
            // Update progress bar
            float progress = (float)myPicStatus.cntVerify / myConfig.intExpectedFirmwareBytes;
            updateProgressBar(progress);

            // Update total bytes
            updateVerifyBytesLabel();
        }

    // Handle PIC System Messages
    } else {
        if (msg.indexOf("<HandShakeTimeout>") > -1)
        {
            Serial.println("STATUS: Timeout exiting bootloader --> Entering application...");
        }
        else if (msg.indexOf("<InitFromApp>") > -1)
        {
            Serial.println("STATUS: App responded --> Entering bootloader...");
        }
        else if (msg.indexOf("<InitReceived>") > -1)
        {
            myPicStatus.blnHandShakeSuccess = true;
            Serial.println("STATUS: Bootloader responded.");
        }
        else if (msg.indexOf("<ConfigTimeout>") > -1)
        {
            myPicStatus.blnTimeoutError = true;
            Serial.println("STATUS: PIC reported timeout error, try again");
        }
        else if (msg.indexOf("<ConfigOK>") > -1)
        {
            myPicStatus.blnConfigOK = true;
        }
        else if (msg.indexOf("<EndFlashErase>") > -1)
        {
            myPicStatus.blnEndFlashErase = true;
        }
        else if (msg.indexOf("<ACK>") > -1)
        {
            myPicStatus.blnWriteACK = true;
        }
        else if (msg.indexOf("<ISR Timeout>") > -1)
        {
            myPicStatus.blnISRTimeOut = true;
        }
        else if (msg.indexOf("<ErrorTimeout>") > -1)
        {
            myPicStatus.blnTimeoutError = true;
            Serial.println("STATUS: PIC reported timeout error, try again");
        }
        else if (msg.indexOf("<StartFlashVerify>") > -1)
        {
            myPicStatus.cntVerify = 0;
            myPicStatus.blnStartVerifyRequest = true;
            SD.remove("/verify.bin");
            verifyFile = SD.open("/verify.bin", FILE_WRITE);         // OPEN ONCE
            Serial.println("STATUS: Waiting for verification...");
        }
        else if (msg.indexOf("<EndFlashVerify>") > -1)
        {
            myPicStatus.blnEndFlashVerify = true;

            if (verifyFile) 
            {
                verifyFile.close();
                // Important: Zero out the file object so we don't use it again
                verifyFile = File(); 
            }
            verifyStatus();             // Display the result on your ILI9488
        }
    }

}



