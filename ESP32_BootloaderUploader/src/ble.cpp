#include "globals.h"
#include "ble.h"
#include "display_logic.h"
#include "sdcard.h"

bool doConnect = false;
bool doScan = true;
String rxBufferString;
File verifyFile;
uint16_t intMTUSize = 20;                         

// Shared variables (extern from header)
NimBLEAdvertisedDevice* myDevice = nullptr;
NimBLERemoteCharacteristic* pRemoteCharacteristic = nullptr;

// Internal (file-local) variables
static String targetDeviceName = "HM10";
static NimBLEUUID serviceUUID("0000ffe0-0000-1000-8000-00805f9b34fb");    // HM-10 + HM-20 Service UUID
static NimBLEUUID charUUID((uint16_t)0xFFE1);

static NimBLEClient* pClient = nullptr;
static NimBLERemoteService* pRemoteService = nullptr;

//-----------------------------------------------------------
//Initialize BLE system
//-----------------------------------------------------------
void initBLESystem() 
{
    NimBLEDevice::init("");
}

//-----------------------------------------------------------
//Connect/Disconnect event
//-----------------------------------------------------------
class MyClientCallback : public NimBLEClientCallbacks 
{
    void onConnect(NimBLEClient* pClient) override
    {  
        uint16_t RawMTU = pClient->getMTU();
        setMTUSize(RawMTU);   // Call this for HM-20 Workaround. HM-20 does not properly handle MTU exchange, so we need to set the MTU size manually to ensure we can send larger packets. The MTU size will be negotiated in the onMTUChange callback, but we need to set it here first to ensure that the negotiation happens with the correct size. By default, NimBLE starts with a small MTU (23 bytes), which is not sufficient for our needs, so we set it to a larger value (e.g., 512) to allow for larger packets. The actual MTU size will be determined by the negotiation process, but this ensures that we start with a reasonable size for our application.

        Serial.println(" - onConnect");
        Serial.println(" - Requesting larger MTU..."); 
        pClient->exchangeMTU();
    }

    void onDisconnect(NimBLEClient* pClient, int reason) override
    {
        myPicStatus.blnUserCancel = true;
        Serial.println(" - onDisconnect: " + String(reason));
        doScan = true; 
    }

    // This is the event that triggers when negotiation finishes
    void onMTUChange(NimBLEClient* pClient, uint16_t MTU) override 
    {
        Serial.println(" - onMTUChange -");
        setMTUSize(MTU);
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

    if (rxBufferString.indexOf(">") > -1 || myPicStatus.blnStartFlashVerify == true)
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
    doConnect = false;          // prevent repeated attempts to connect

    // 1. Important to delete client or it will crash anytime you reconnect. This is because the NimBLE library does not allow multiple clients to exist at the same time, so we need to make sure to clean up the old client before creating a new one.
    Serial.println("Cleaning up old client memory...");
    if (pClient != nullptr) 
    {
        pClient->disconnect();
        NimBLEDevice::deleteClient(pClient); 
        pClient = nullptr; 
    }
    // Clear old references to service and characteristic since we are creating a new client and connecting to a new server, so we need to make sure to clear out the old references to avoid any issues with dangling pointers.
    pRemoteService = nullptr;
    pRemoteCharacteristic = nullptr;
        
    Serial.println("Starting Pre-Flight Check...");
    Serial.print("Forming a connection to ");
    Serial.println(myDevice->getAddress().toString().c_str());

    // 2. NimBLE uses NimBLEDevice to create clients instead of directly instantiating them. This is because NimBLE manages the lifecycle of clients internally to ensure proper cleanup and resource management, which is especially important in embedded environments with limited resources.
    pClient = NimBLEDevice::createClient();
    Serial.println(" - Created client");

    pClient->setClientCallbacks(new MyClientCallback()); // 'true' handles automatic deletion of the callback object when the client is deleted. This is important to prevent memory leaks since the NimBLE library does not allow multiple clients to exist at the same time, so we need to make sure to clean up the old client and its callbacks before creating a new one.

    // 3. Connect to the remote server. 
    if (!pClient->connect(myDevice)) 
    {
        Serial.println(" - Failed to connect to server");
        NimBLEDevice::deleteClient(pClient);
        return false;
    }
    pClient->updateConnParams(16, 32, 0, 100);  // 1 seconds timeout.  when powered off it takes 1 seconds to update the status of connection
    Serial.println(" - Connected to server");
    
    // 4. Obtain a reference to the service we are after in the remote BLE server. We will need this to find the characteristic we want to read from and write to.
    pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) 
    {
        Serial.print("Failed to find our service UUID: ");
        Serial.println(serviceUUID.toString().c_str());
        pClient->disconnect();
        return false;
    }
    Serial.println(" - Found our service");
    
    // 5. Obtain a reference to the characteristic in the service of the remote BLE server that we are interested in. We will need this to read and write to the characteristic.
    pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
    if (pRemoteCharacteristic == nullptr) 
    {
        Serial.print("Failed to find our characteristic UUID: ");
        Serial.println(charUUID.toString().c_str());
        pClient->disconnect();
        return false;
    }
    Serial.println(" - Found our characteristic");

    
    // 6. Subscription Logic - Check if the characteristic supports notifications, and if so subscribe to it with our notifyCallback function. This will allow us to receive asynchronous updates from the BLE server whenever the characteristic value changes, which is essential for receiving messages from the PIC in real-time without having to constantly poll for updates.
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

void setMTUSize(uint16_t RawMTU)
{
    // Step 1: Remove ATT header (always 3 bytes)
    int PayloadMTU = RawMTU - 3;
    
    // Step 2: Align for PIC (Multiple of 4)
    // Bitwise AND with 0xFFFC clears the last two bits
    int UniversalMTU = PayloadMTU & 0xFFFC; 
    
    if (UniversalMTU > 0) {
        intMTUSize = (uint16_t)UniversalMTU;
    }

    Serial.printf(" - Negotiated MTU: %d\n", RawMTU);
    Serial.printf(" - Payload MTU (MTU-3): %d\n", PayloadMTU);
    Serial.printf(" - Universal MTU for PIC: %d\n", UniversalMTU);
}

void bleDoScan() 
{
    Serial.println("Starting Arduino NimBLE Client application...");
    doScan = false;     // prevent repeated attempts to scan

    NimBLEScan* pBLEScan = NimBLEDevice::getScan();

    pBLEScan->setScanCallbacks(new MyAdvertisedDeviceCallbacks(), true);
    pBLEScan->setActiveScan(true);
    pBLEScan->setInterval(100);
    pBLEScan->setWindow(90);
    pBLEScan->start(0, false);
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
        myPicStatus.blnStartFlashVerify = false;      // just in case verify is running and you power off the HM10, we want to make sure to turn off verify mode since it relies on the connection to know when to end.
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
    if (myPicStatus.blnStartFlashVerify != true && msg != "<ACK>")
    {
        // Print to the PlatformIO Serial Monitor
        Serial.print("PIC: ");
        Serial.println(msg);
    }  

    // Check if we are in Verification Mode (we know this because PIC will start sending raw bytes instead of messages, but it will still send a message to trigger the start of verify mode)
    if (myPicStatus.blnStartFlashVerify == true) {
        myPicStatus.cntVerify += length;

        // Check for completion based on byte count 
        if (myPicStatus.cntVerify >= myConfig.intExpectedFirmwareBytes)
        {
            // Without this, Last iteration will cause issues! Due to low 150ms delay in PIC after msg send!
            myPicStatus.blnStartFlashVerify = false;
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
            myPicStatus.blnStartFlashVerify = true;
            if (verifyFile) 
			{
                verifyFile.close();
			}
			SD.remove(VERIFY_FILE);
            verifyFile = SD.open(VERIFY_FILE, FILE_WRITE);          // OPEN ONCE AND APPEND ALL BYTES TO IT.  This is to save RAM since we are receiving bytes in chunks and not as a whole file.
            Serial.println("STATUS: Waiting for verification...");
        }
        else if (msg.indexOf("<EndFlashVerify>") > -1)
        {
            myPicStatus.blnEndFlashVerify = true;                   // used in display_logic.cpp where it deals with millis
            if (verifyFile) 
            {
                verifyFile.close();
                verifyFile = File();                                // Important: Zero out the file object so we don't use it again
            }
            verifyStatus();                                         // Display the result on your ILI9488 UI (you will need to implement this function in sdcard.cpp or wherever you want since it just needs to read the VERIFY_FILE and compare it to flash.bin)
        }
    }

}



