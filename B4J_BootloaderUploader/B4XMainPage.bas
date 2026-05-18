B4A=true
Group=Default Group
ModulesStructureVersion=1
Type=Class
Version=9.85
@EndOfDesignText@
' Features: Bluetooth SSP 2.0, Bluetooth BLE 4.0, WIFI TCP/IP and USB TTL Serial (COM)

'------------------------------------------------------------------------------------------------------
' DT-06 NOTES:
' The factory DT-06 firmware proved incompatible with the B4J uploader’s timing 
' requirements. While DT06 Flashing custom firmware successfully optimized the B4J-To-WiFi 
' downlink and PIC-To-WIFI uploadlink.

'------------------------------------------------------------------------------------------------------
'BUILD STANALONE INSTRUCTIONS!
' Using .Exe from Build Standalone Package you must include the .map files in 
' \BootloaderUploader\Objects\temp\build\bin\configs

'------------------------------------------------------------------------------------------------------
'BLEAK DEPENDENCIES INSTRUCTIONS!
' HM10 requires Bleak Library and its dependencies.  You need to install Bleak dependencies manually.
' 1. Locate "C:\Program Files\Anywhere Software\B4J\Libraries\Python\python\python.exe"
' 2. Open Terminal by right click and type the following
' 3. ".\python.exe -m pip install bleak" without quotes

'------------------------------------------------------------------------------------------------------
' HC05/HM10 needs To be programmed To 57600 Baud. All PICS firmware communicates at 57600!  PIC Communicates at 57600 baud.
' Use AT Mode to configure for HC05 and HM10!


'Ctrl + click to export as zip: ide://run?File=%B4X%\Zipper.jar&Args=Project.zip

Sub Class_Globals
	Private Const VERSION As String = "12.51"
	
	Private Const CONFIG_MAP As String = "config.map"		' For ESP32 Uploader
	Private Const FLASH_BIN As String = "flash.bin"			' For ESP32 Uploader
	
	Private Const DEVICE_NONE As Int = 0
	Private Const DEVICE_BLE As Int = 1
	Private Const DEVICE_CLASSIC_BT As Int = 2
	Private Const DEVICE_WIFI As Int = 3
	Private Const DEVICE_TTLSERIAL As Int = 4
	Private WhichDeviceConnection As Int = 0
	
	Private Const BUTTON_FLASH As Int = 1
	Private Const BUTTON_VERIFY As Int = 2
	Private Const BUTTON_BLE As Int = 3
	Private Const BUTTON_CLASSIC_BT As Int = 4
	Private Const BUTTON_WIFI As Int = 5
	Private Const BUTTON_TTLSERIAL As Int = 6
	Private Const BUTTON_STOP As Int = 7
	
	' If Button labels are changed, update them here!
	Private Const txt_CONNECT As String = "Connect"
	Private Const txt_DISCONNECT As String = "Disconnect"
	Private Const txt_OPEN_PORT As String = "Open Port"
	Private Const txt_CLOSE_PORT As String = "Close Port"
	Private Const txt_STOP As String = "Stop"
	Private Const txt_FLASH As String = "Flash"
	
	'---------------------------------------
	' Map Config Variables
	'---------------------------------------
	Type ConfigMap( _
		intStartAddrFlash As Int, _							' Used with Intel Hex Conversion
		intEndAddrFlash   As Int, _							' Used with Intel Hex Conversion
		intEmptyFlashValue As Int, _						' 16F = 0x3F, 18F = 0xFF, 24F = 0x00 Phantom 24bit
		intInstructionPacket As Int, _						' Number of Instruction per block
		intPacketDelayMS As Int, _							' Delay for Tx Write Packets
		intHandShakeDelayMS As Int, _						' Delay for 0x55 and 0xAA in between	
		intStopBit As Int, _ 								' Default
		strNotes As String, _ 
		intExpectedFirmwareBytes As Int, _					' Total Firmware bytes
		blnUseWriteBurst  As Boolean, _						' True = Tx Write Packet as whole, no delays in between!
		blnUseDoubleHexAddr As Boolean, _					' 16F, 24F = Increment 2 Hex Address, 18F = Increment 1 Hex Address. Used with Intel Hex Conversion
		blnUse4Padding As Boolean, _						' 24 Bit need step 4, others step 2. Used with Intel Hex Conversion
		blnUseCheckSum As Boolean, _						' Will enable or disable checksum usage
		strPicName As String)								' Name of PIC
	Private myConfigMap As ConfigMap

	'---------------------------------------
	' PIC Status
	'---------------------------------------
	Type PicStatus( _
		cntVerify As Int, _									' Counter detection of incoming verify bytes from PIC
		blnHandShakeSuccess As Boolean, _					' <InitReceived> from PIC
		blnConfigOK As Boolean, _							' <ConfigOK> from PIC exit function (Success bytes received)
		blnWriteACK As Boolean, _							' <ACK> from PIC used in Firmware Upload.  Needs this <ACK> from PIC to continue next Block Write bytes
		blnISRTimeOut As Boolean, _							' <ISR Timeout> Timeout Detected from PIC
		blnTimeoutError As Boolean, _						' <TimeoutError> 3 ISR Timeout from PIC
		blnStartFlashVerify As Boolean, _					' <StartFlashVerify> from PIC
		blnAstreamError	As Boolean, _						' Astream error exit loop
		blnUserCancel As Boolean)							' Exit loop for Stop Flash, Disconnect and Quit App
	Private myPicStatus As PicStatus
	
	'---------------------------------------
	' BLE (Bleak), jBluetooth and Serial Library + Astream
	'---------------------------------------
	Private btHC05 As Bluetooth								' SSP Bluetooth
	Private btHC05Connection As BluetoothConnection			' SSP Bluetooth Connection
	Private serialUSBTTL As Serial							' UART Serial COM
	Private WIFIClient As Socket							' WIFI Socket
	Private astream As AsyncStreams							' Read/Write Stream
	
	Public Py As PyBridge
	Private btHM10 As Bleak									' BLE Bluetooth
	Private bkHM10Client As BleakClient
	
	'---------------------------------------
	' UI Elements
	'---------------------------------------
	Private Root As B4XView
	Private xui As XUI
	Public ATCommandMode As ATMode
	
	'Common Elements
	Private MenuBar1 As MenuBar
	Private Pane1 As Pane
	Private TabPane1 As TabPane
	Private btnFlash As Button
	Private btnLoadFile As Button
	Private btnVerify As Button
	Private txtLog As TextArea
	Private prgBar As ProgressBar
	Private cmbPicList As ComboBox
	Private chkCheckSum As CheckBox
	
	'Tab1 Elements (Bluetooth SSP)
	Private ListView1HC05 As ListView							
	Private btnSearchHC05 As Button								
	Private btnConnectHC05 As Button						

	'Tab2 Elements (Virtual Serial Com)
	Private btnOpenUSBTTL As Button
	Private cmbPortUSBTTL As ComboBox
	Private btnRefreshComUSBTTL As Button
	
	'Tab3 Elements (Bluetooth BLE)
	Private btnConnectHM10 As Button
	Private btnStartScanHM10 As Button
	Private btnStopScanHM10 As Button
	Private ListView1HM10 As ListView
	
	'Tab4 Elements (WIFI)
	Private btnConnectWIFI As Button
	Private txtHostIPWIFI As TextField
	Private txtPortWIFI As TextField
	
	'File integrity checker
	Private firmwareFile() As Byte							' firmware binary from FILE
	Private firmwareVerify() As Byte						' firmware binary from PIC
	Private intFileTotalChecksum As Int 					' firmware Checksum from FILE
	Private intVerifyTotalChecksum As Int					' Verify Checksum from PIC
	Private strLastFilePath As String						' Reloads firmware from FILE when PIC name changed so Firmware array be corrected

	'For BLE and SSP
	Private foundDevices As Map								' Bluetooth list for both SSP and BLE		
	Private BLE_useUUID As String							' What Characteristic for BLE
	Private BLE_useMTUSize As Int = 20						' Write Flash support, Verify Flash in firmware will default at 20 now!
	
	Private rxBufferString As String						' Buffer Newdata in string format PIC Message only
	

End Sub

Public Sub Initialize
	foundDevices.Initialize
	btHC05.Initialize("btHC05")
	serialUSBTTL.Initialize("serial")
	WIFIClient.Initialize("WifiClient")
	ATCommandMode.Initialize
	myConfigMap.Initialize
	myConfigMap.intStopBit = 1
	myPicStatus.Initialize
End Sub

'--------------------------------------------------------
'This event will be called once, before the page becomes visible.
'--------------------------------------------------------
Private Sub B4XPage_Created (Root1 As B4XView)
	Dim f As Form = B4XPages.GetNativeParent(Me)
	f.Stylesheets.Add(File.GetUri(File.DirAssets, "style.css"))
	
	Root = Root1
	Root.LoadLayout("MainPage")
	
	TabPane1.LoadLayout("TabPane1", "Bluetooth HC-05")
	TabPane1.LoadLayout("TabPane3", "Bluetooth HM-10")
	TabPane1.LoadLayout("TabPane2", "Serial Com - TTL USB")
	TabPane1.LoadLayout("TabPane4", "WIFI DT-06")	
	
	'B4XPages.AddPageAndCreate("AT Command Mode", ATCommandMode)
	B4XPages.AddPage("AT Command Mode", ATCommandMode)
	
	B4XPages.SetTitle(Me, "PIC Bootloader Upload Deluxe")
		
	' Python Init
	Py.Initialize(Me, "Py")
	Dim opt As PyOptions = Py.CreateOptions("Python/python/python.exe")
	Py.Start(opt)
	Wait For Py_Connected (Success As Boolean)
	If Success = False Then
		LogError("Failed to start Python process.")
	End If
	btHM10.Initialize(Me, "btHM10", Py)
	
	' Form dimensions
	Dim wMap As Map
	wMap.Initialize
	If File.Exists(File.DirApp, "Settings.map") Then
		wMap = File.ReadMap(File.DirApp, "Settings.map")
	End If
	
	B4XPages.GetNativeParent(Me).WindowHeight = wMap.GetDefault("FormHeight", 700)
	B4XPages.GetNativeParent(Me).WindowWidth = wMap.GetDefault("FormWidth", 600)
	B4XPages.GetNativeParent(Me).WindowTop = wMap.GetDefault("FormTop", 0)
	B4XPages.GetNativeParent(Me).WindowLeft = wMap.GetDefault("FormLeft", 0)
	
	' Load all available COM Ports
	cmbPortUSBTTL.Items.AddAll(serialUSBTTL.ListPorts)
	
	' Load PIC names from map files
	Dim getList As List = LoadAllPicNames
	For Each name As String In getList
		cmbPicList.Items.Add(name)
	Next
		
	' Set prompt text for combo box
	Dim jo As JavaObject = cmbPortUSBTTL
	jo.RunMethod("setPromptText", Array("Choose Port"))
	Dim jo As JavaObject = cmbPicList
	jo.RunMethod("setPromptText", Array("PIC List"))
End Sub
Private Sub B4XPage_CloseRequest As ResumableSub
	
	' Flash in progress alert user!
	If btnFlash.Text = txt_STOP Then
		Dim sf4 As Object = xui.Msgbox2Async("Flash in progress!", "Quit?", "Yes", "", "No", Null)
		Wait For (sf4) Msgbox_Result(ret3 As Int)
				
		If ret3 = xui.DialogResponse_Negative Then
			Return False  ' will not exit
		End If
	End If
	
	' Close them when exiting!
	If astream.IsInitialized Then 		
		astream.Close
		
		' Close Serial Com
		serialUSBTTL.Close
		
		' Disconnect HC05 Bluetooth
		btHC05Connection.Disconnect
		
		'Close WIFI 
		If WIFIClient.Connected = True Then 
			WIFIClient.Close
		End If
	End If

	' Disconnect HM10 Bluetooth
	If bkHM10Client.IsInitialized Then
		bkHM10Client.Disconnect
	End If
			
	' Save screen dimension
	Dim wMap As Map
	wMap.Initialize
	wMap.Put("FormHeight", B4XPages.GetNativeParent(Me).WindowHeight)
	wMap.Put("FormWidth", B4XPages.GetNativeParent(Me).WindowWidth)
	wMap.Put("FormTop", B4XPages.GetNativeParent(Me).WindowTop)
	wMap.Put("FormLeft", B4XPages.GetNativeParent(Me).WindowLeft)
	File.WriteMap(File.DirApp, "Settings.map", wMap)
	
	Return True	
End Sub
Private Sub B4XPage_Background
	Py.KillProcess
End Sub

'--------------------------------------------------------
' Astream Functions
'--------------------------------------------------------
Sub astream_NewData (Buffer() As Byte)
	' Note: For Bluetooth reliability, firmware MSG_MS_DELAY should be at least 150 ms.
	' There must be a high delay between <EndFlashErase> and <StartFlashWrite> or between
	' 2 proceeding messages from PIC.
	' At a 50 ms delay with Bluetooth SSP only, both messages were observed in a single poll
	' (e.g., "<EndFlashErase><St"), and the parser currently cannot handle
	' concatenated or mixed incoming messages correctly.
	
	' Option display Hex for Debugging!
	'Dim AddHexView As String = BytesToHexString(Buffer).ToUpperCase
	'LogMessage("HEX", AddHexView)
	
	' Buffer for Messages, PIC Sends
	rxBufferString = rxBufferString & BytesToString(Buffer, 0, Buffer.Length, "UTF8") 
	
	' PIC sends with > as last byte to confirm end of message or bytes
	' When VerifyRequest = true, it does not received ">". Sticktly bytes only!
	' Buffer is meant for Verify Bytes only.
	If rxBufferString.Contains(">") Or myPicStatus.blnStartFlashVerify = True Then
		HandleMessage(rxBufferString, Buffer)	
		rxBufferString = ""
	End If
	
	' Private Sub btHM10_CharNotify (Notification As BleakNotification)  BLE...Same as here!
End Sub
Sub astream_Error
	LogMessage("STATUS", "Error: " & LastException)
	astream_Terminated
End Sub
Sub astream_Terminated
	myPicStatus.blnAstreamError = True
	
	If astream.IsInitialized Then
		astream.Close
		If WIFIClient.Connected = True Then 
			WIFIClient.Close
		End If
		serialUSBTTL.Close
		btHC05Connection.Disconnect
	End If
	EnableFunction(True)
	LogMessage("STATUS", "Astream is terminated!")
End Sub

'--------------------------------------------------------
' HC-05 Bluetooth SSP Routine 
'--------------------------------------------------------
Private Sub btHC05_DeviceFound (Name As String, MacAddress As String)
	Dim description As String = Name & ": " & MacAddress
	ListView1HC05.Items.Add(description)
	foundDevices.Put(description, MacAddress)
End Sub
Private Sub btHC05_DiscoveryFinished
	btnSearchHC05.Enabled = True
	If ListView1HC05.Items.Size > 0 Then 
		btnConnectHC05.Enabled = True
	End If
	LogMessage("BT SPP", "Discovery completed")
End Sub

'--------------------------------------------------------
' HM-10/20 Bluetooth BLE Routine
'--------------------------------------------------------
Private Sub btHM10_DeviceFound (Device As BleakDevice)
	'Log($"${Device.DeviceId}, Name=${Device.Name}, Services=${Device.ServiceUUIDS}, ServiceData=${Device.ServiceData}"$)
	Dim description As String = Device.Name & ": " & Device.DeviceId
	
	' Add if not in FoundDevice map!
	If foundDevices.ContainsKey(description) = False Then
		ListView1HM10.Items.Add(description)
		If ListView1HM10.Items.Size > 0 Then
			btnConnectHM10.Enabled = True
		Else
			btnConnectHM10.Enabled = False
		End If
	End If
	
	' Add to map.  Existing will overwrite
	foundDevices.Put(description, Device)
	
End Sub
Private Sub btHM10_DeviceDisconnected (DeviceId As String)
	EnableFunction(True)
	LogMessage("BLE", "BLE Disconnected! @ " & DeviceId)
End Sub
Private Sub btHM10_CharNotify (Notification As BleakNotification)
	Dim Buffer() As Byte = Notification.Value

	' Option display Hex for Debugging!
	'Dim AddHexView As String = BytesToHexString(Buffer).ToUpperCase
	'LogMessage("HEX", AddHexView)
		
	' Buffer for Messages, PIC Sends
	rxBufferString = rxBufferString & BytesToString(Buffer, 0, Buffer.Length, "UTF8")
	
	' PIC sends with > as last byte to confirm end of message
	' When VerifyRequest = true, it does not received ">". Sticktly bytes only!
	' Buffer is meant for Verify Bytes only.
	If rxBufferString.Contains(">") Or myPicStatus.blnStartFlashVerify = True Then
		HandleMessage(rxBufferString, Buffer)
		rxBufferString = ""
	End If
	
End Sub

Private Sub Py_Disconnected
	Log("PyBridge disconnected")
End Sub

'--------------------------------------------------------
' Handle incomming buffer and messages
'--------------------------------------------------------
Sub HandleMessage(msg As String, buffer() As Byte)
	
	' We dont want to log Incoming <ACK> while Firmware upload!
	' We dont want to log Incoming PIC VERIFY bytes, but just <VerifyCancelled>!
	If msg <> "<ACK>" Then
		If myPicStatus.blnStartFlashVerify = True Then
			If msg = "<VerifyCancelled>" Or msg.Contains("Cancelled") Then
				'workaround For this! rxBufferString does Not concatenate due To blnStartVerifyFlash = true!
				LogMessage("PIC", msg)
			End If
		Else
			LogMessage("PIC", msg)
		End If
	End If
	
	' This is triggered by <StartFlashVerify> from PIC after Flash Write is completed
	If myPicStatus.blnStartFlashVerify = True Then
		'LogMessage("INCOMMING", BytesToHexString(buffer))  ' debugging only!!!	
		
		' Checksum comparison performed here
		If chkCheckSum.Checked = True Then
			myPicStatus.blnStartFlashVerify = False
			If buffer.Length = 1 Then	' expecting no more then 1 byte returned!
				intVerifyTotalChecksum = Bit.And(buffer(0), 0xFF)
			End If
			' Let <EndFlashVerify> display the status of Verify!
			' Just enable button here
			EnableFunction(False)
		' Byte for Byte comparison performed here				
		Else
			For x = 0 To buffer.Length - 1  
				' This array will compare to firmware() which is Converted FILE binary
				firmwareVerify(myPicStatus.cntVerify) = buffer(x)
											
				' Update progress bar
				prgBar.Progress = Min(1, myPicStatus.cntVerify / myConfigMap.intExpectedFirmwareBytes)
				myPicStatus.cntVerify = myPicStatus.cntVerify + 1
						
				' Check if we reached the expected firmware size
				If myPicStatus.cntVerify >= myConfigMap.intExpectedFirmwareBytes Then
					myPicStatus.blnStartFlashVerify = False
					' Let <EndFlashVerify> display the status of Verify!
					' Just enable button here
					EnableFunction(False)
					Exit
				End If
			Next
		End If
	' This is PIC MESSAGES excluding blnVerifyRequest (Verify bytes)!
	Else
			' Bootloader firmware confirmed handshake received
		If msg.Contains("<HandShakeTimeout>") Then
			LogMessage("STATUS", "Timeout exiting bootloader --> Entering application...")

			' Application firmware confirmed handshake received
		Else If msg.Contains("<InitFromApp>") Then
			LogMessage("STATUS", "App responded --> Entering bootloader...")
			
			' Bootloader start handshake timeout.  if no handshake it will enter application
		Else If msg.Contains("<InitReceived>") Then
			myPicStatus.blnHandShakeSuccess = True	' Must do this first. Exit handshake no more 0x55 0xAA.
			LogMessage("STATUS", "Bootloader responded.")
			
		Else If msg.Contains("<ConfigTimeout>") Then
			myPicStatus.blnTimeoutError = True
			LogMessage("STATUS", "PIC reported timeout error, try again")
			
		Else If msg.Contains("<ConfigOK>") Then
			myPicStatus.blnConfigOK = True
								
			' End of Erase Flash. When Pic sends this, delay a bit and start the flash upload
		Else If msg.Contains("<EndFlashErase>") Then
			Sleep(300)
			SendFirmwareBytes(WhichDeviceConnection)
			
			' B4J expects <ACK> from PIC so B4J sends next packets in Firmware Upload routine
		Else If msg.Contains("<ACK>") Then
			myPicStatus.blnWriteACK = True

			' Timeout occurred with Flash Write.  This will attempt a redo send
		Else If msg.Contains("<ISR Timeout>") Then
			myPicStatus.blnISRTimeOut = True
			
			' 3 ISR timeout = error by PIC
		Else If msg.Contains("<ErrorTimeout>") Then
			myPicStatus.blnTimeoutError = True
			LogMessage("STATUS", "PIC reported timeout error, try again")

			' Start of verify flash program code
		Else If msg.Contains("<StartFlashVerify>") Then
			myPicStatus.cntVerify = 0		' not needed?? its in disableFunction?
			myPicStatus.blnStartFlashVerify = True
			LogMessage("STATUS", "Waiting for verification...")
				
		'Else If msg.Contains("Cancelled>") Then
			' This will never trigger! due to blnStartVerifyFlash
			
			
			' End of verify flash program code
		Else If msg.Contains("<EndFlashVerify>") Then
			If chkCheckSum.Checked = True Then
				VerifyStatusChecksum
			Else
				VerifyStatus
			End If
			EnableFunction(False)
		End If
	
	End If
			
End Sub

'--------------------------------------------------------
' Buttons and Combo Box functions in TabPane
'--------------------------------------------------------
Private Sub btnSearchHC05_Click 
	If btHC05.IsEnabled = True Then
		Dim res As Boolean = btHC05.StartDiscovery
		If res Then
			ListView1HC05.Items.Clear
			foundDevices.Clear
			btnSearchHC05.Enabled = False
			btnConnectHC05.Enabled = False
			LogMessage("BT SPP", "Searching, please wait...")
		Else
			Log("Error starting discovery")
		End If
	Else
		xui.Msgbox2Async("Bluetooth is disabled. Please turn it on!", "Bluetooth", "Ok", "", "", Null)
	End If
End Sub
Private Sub btnConnectHC05_Click
	If btnConnectHC05.Text = txt_CONNECT Then
		ConnectHC05
	Else
		PerformUserAbort(BUTTON_CLASSIC_BT, WhichDeviceConnection, False)
	End If
End Sub
Private Sub ConnectHC05
	If ListView1HC05.SelectedIndex <> - 1 Then
		If btHC05.IsEnabled = False Then
			xui.Msgbox2Async("Bluetooth is disabled. Please turn it on!", "Bluetooth", "Ok", "", "", Null)
			Return
		End If
			
		If CloseOtherConnection(WhichDeviceConnection) = True Then
			Sleep(500)	' Min 400 ms. Let other open connection close properly!
		End If
		
		btHC05.CancelDiscovery
			
		' Connect Bluetooth with Address selected from listview
		Dim address As String = foundDevices.Get(ListView1HC05.SelectedItem)
		btHC05.Connect(address)
		LogMessage("BT SPP", "Connecting to " & ListView1HC05.SelectedItem & "...")
		wait for btHC05_Connected (Success As Boolean, connection As BluetoothConnection)
		btHC05Connection = connection
		If Success Then
			If astream.IsInitialized Then astream.Close
			astream.Initialize(connection.InputStream, connection.OutputStream, "astream")
			WhichDeviceConnection = DEVICE_CLASSIC_BT
			btnConnectHC05.Text = txt_DISCONNECT
			LogMessage("BT SPP", "Bluetooth Connected! @ " & address)
			LogMessage("BT SPP", "Ready Flash")
		Else
			LogMessage("BT SPP", "Bluetooth Failed! @ " & address)
		End If
	Else
		xui.Msgbox2Async("Please select Bluetooth fom the list!", "Select Bluetooth", "Ok", "", "", Null)
	End If
End Sub

Private Sub btnStartScanHM10_Click
	If btHC05.IsEnabled = True Then
		If bkHM10Client.IsInitialized Then bkHM10Client.Disconnect
		
		Wait For (btHM10.Scan(Null)) Complete (Success As Boolean)
		If Success Then
			ListView1HM10.Items.Clear
			foundDevices.Clear
			btnStartScanHM10.Enabled = False
			btnStopScanHM10.Enabled = True
			LogMessage("BLE", "Searching, please wait...")
		Else
			LogMessage("Python", Py.PyLastException)
		End If
	Else
		xui.Msgbox2Async("Bluetooth is disabled. Please turn it on!", "Bluetooth", "Ok", "", "", Null)		
	End If
End Sub
Private Sub btnStopScanHM10_Click
	btnStartScanHM10.Enabled = True
	btnStopScanHM10.Enabled = False
	If btHM10.IsScanning = True Then
		btHM10.StopScan
		LogMessage("BLE", "Scan has stopped")
	End If
End Sub
Private Sub btnConnectHM10_Click
	If btnConnectHM10.Text = txt_CONNECT Then
		ConnectHM10
	Else
		PerformUserAbort(BUTTON_BLE, WhichDeviceConnection, False)
	End If

End Sub
Private Sub ConnectHM10	
	If ListView1HM10.SelectedIndex <> - 1 Then
		If btHC05.IsEnabled = False Then
			xui.Msgbox2Async("Bluetooth is disabled. Please turn it on!", "Bluetooth", "Ok", "", "", Null)
			Return
		End If
			
		' Close any open connections
		If CloseOtherConnection(WhichDeviceConnection)  = True Then
			Sleep(500)	' Min 400 ms. Let other open connection close properly!
		End If
		
		BLE_useUUID = ""
			
		' Connect Bluetooth with Address selected from listview
		Dim address As String = ListView1HM10.SelectedItem
		bkHM10Client = btHM10.CreateClient(foundDevices.Get(address))
		LogMessage("BLE", "Connecting to " & ListView1HM10.SelectedItem & "...")
		Wait For (bkHM10Client.Connect) Complete (Success As Boolean)
		If Success Then
				
			'----------------------------------------------------------------
			' Check MTU amount allowed
			Dim mtu As PyWrapper = bkHM10Client.client.GetField("mtu_size")
			Wait For (mtu.Fetch) Complete (mtu As PyWrapper)
			' Step 1: Get MTU Size
			Dim RawMTU As Int = mtu.Value
			' Step 2: Remove ATT header
			Dim PayloadMTU As Int = RawMTU - 3
			' Step 3: Align for PIC (multiple of 4 and compatible with 2 in Verify_Flash Firmware (Future reserved))
			Dim UniversalMTU As Int = Bit.And(PayloadMTU, 0xFFFC)
			If UniversalMTU > 0 Then
				BLE_useMTUSize = UniversalMTU
			End If
			Log("Negotiated MTU: " & RawMTU)
			Log("Payload MTU (MTU-3): " & PayloadMTU)
			Log("Universal MTU for PIC: " & UniversalMTU)
			LogMessage("BLE", "MTU Size = " & UniversalMTU)
			'-----------------------------------------------------------------
				
			WhichDeviceConnection = DEVICE_BLE
			btnConnectHM10.Text = txt_DISCONNECT
			btnStopScanHM10_Click
			LogMessage("BLE", "Bluetooth Connected! @ " & address)
			For Each service As BleakService In bkHM10Client.Services.Values
				Log("---------------------------------")
				Log("Service: " & service.UUID)
				For Each Chara As BleakCharacteristic In service.Characteristics
					Log(Chara.Properties)
					Log(Chara.UUID)
					If Chara.UUID.ToLowerCase.Contains("ffe1") = True And Chara.Properties.IndexOf("notify") <> -1 Then
						BLE_useUUID = Chara.UUID
						Wait For (bkHM10Client.SetNotify(BLE_useUUID)) Complete (Result As PyWrapper)
						If Result.IsSuccess = False Then
							LogMessage("BLE", "Failed to set notify!")
						Else
							LogMessage("BLE", "Notify set @ " & BLE_useUUID)
							LogMessage("BLE", "Ready Flash")
						End If
					End If
				Next
			Next
				
			If BLE_useUUID = "" Then
				LogMessage("BLE", "Characteristic UUID not found!")
			End If
		Else
			Log(Py.PyLastException)
			LogMessage("BLE", "Bluetooth Failed! @ " & address)
		End If
	Else
		xui.Msgbox2Async("Please select Bluetooth fom the list!", "Select Bluetooth", "Ok", "", "", Null)
	End If
End Sub

Private Sub btnConnectWIFI_Click
	If btnConnectWIFI.Text = txt_CONNECT Then
		ConnectWIFI
	Else
		PerformUserAbort(BUTTON_WIFI, WhichDeviceConnection, False)
	End If

End Sub
Private Sub ConnectWIFI
	If CloseOtherConnection(WhichDeviceConnection)  = True Then
		Sleep(500)	' Min 400 ms. Let other open connection close properly!
	End If
	
	Dim c As Socket
	c.Initialize("client")
	c.Connect(txtHostIPWIFI.text, txtPortWIFI.text, 5000)
	LogMessage("WIFI", "Connecting @ " & txtHostIPWIFI.Text & ":" & txtPortWIFI.Text)
	Wait For client_Connected (Successful As Boolean)
	If Successful Then
		WIFIClient = c
		astream.Initialize(WIFIClient.InputStream, WIFIClient.OutputStream, "astream")
		WhichDeviceConnection = DEVICE_WIFI
		btnConnectWIFI.Text = txt_DISCONNECT
		LogMessage("WIFI", "WIFI connected!")
		LogMessage("WIFI", "Ready Flash")
	Else
		LogMessage("WIFI", "WIFI connection failed!")
	End If
End Sub

Private Sub btnRefreshComUSBTTL_Click
	' Load all available COM Ports
	cmbPortUSBTTL.Items.Clear
	cmbPortUSBTTL.Items.AddAll(serialUSBTTL.ListPorts)
End Sub
Private Sub cmbPortUSBTTL_SelectedIndexChanged(Index As Int, Value As Object)
	btnOpenUSBTTL.Enabled = Index > -1 'enable the button if there is a selected item
End Sub
Private Sub btnOpenUSBTTL_Click
	If btnOpenUSBTTL.Text = txt_OPEN_PORT Then
		OpenUSBTLL
	Else
		PerformUserAbort(BUTTON_TTLSERIAL, WhichDeviceConnection, False)
	End If
End Sub
Private Sub OpenUSBTLL
	If CloseOtherConnection(WhichDeviceConnection)  = True Then
		Sleep(500)	' Min 400 ms. Let other open connection close properly!
	End If
	
	Try
		Dim serial1 As Serial
		serial1.Initialize("serial1")
		serial1.Open(cmbPortUSBTTL.Value)
		serial1.SetParams(serial1.BAUDRATE_57600, serial1.DATABITS_8, myConfigMap.intStopBit, serial1.PARITY_NONE)  ' Set baud=57600, 8 data bits, config value, no parity
		If astream.IsInitialized Then astream.Close
		astream.Initialize(serial1.GetInputStream, serial1.GetOutputStream, "astream")
		serialUSBTTL = serial1
	Catch
		LogMessage("COM", "Error opening port" & LastException)
		Return
	End Try
	WhichDeviceConnection = DEVICE_TTLSERIAL
	btnOpenUSBTTL.Text = txt_CLOSE_PORT
	btnRefreshComUSBTTL.Enabled = False
	LogMessage("COM", "Serial COM opened")
	LogMessage("COM", "Ready Flash")
End Sub

Private Sub cmbPicList_SelectedIndexChanged(Index As Int, Value As Object)
	If LoadConfiguration(Value) = False Then
		LogMessage("STATUS", "Error loading configuration for " & Value)
	End If
End Sub

Private Sub MenuBar1_Action
	Dim mi As MenuItem = Sender
	Select mi.Text
		' Help
		Case "Help TTL _USB"
			Dim fx As JFX
			fx.ShowExternalDocument("https://github.com/Issac567/PIC_Bootloader/blob/main/B4J_BootloaderUploader/Help/TTLUSBReadme.md")
		Case "Help HC-0_5"
			Dim fx As JFX
			fx.ShowExternalDocument("https://github.com/Issac567/PIC_Bootloader/blob/main/B4J_BootloaderUploader/Help/HC05Readme.md")
		Case "Help HM-1_0"
			Dim fx As JFX
			fx.ShowExternalDocument("https://github.com/Issac567/PIC_Bootloader/blob/main/B4J_BootloaderUploader/Help/HC08Readme.md")
		Case "Help DT-0_6"
			Dim fx As JFX
			fx.ShowExternalDocument("https://github.com/Issac567/PIC_Bootloader/blob/main/B4J_BootloaderUploader/Help/DT06Readme.md")
		Case "Help _Bleak"
			Dim fx As JFX
			fx.ShowExternalDocument("https://github.com/Issac567/PIC_Bootloader/blob/main/B4J_BootloaderUploader/Help/BleakReadme.md")
		Case "Help AT _Mode"
			Dim fx As JFX
			fx.ShowExternalDocument("https://github.com/Issac567/PIC_Bootloader/blob/main/B4J_BootloaderUploader/Help/ATMode.md")
		Case "_About"
			xui.Msgbox2Async("Bootloader Uploader Deluxe" & Chr(10) & "Version: " & VERSION, "About", "Ok", "", "", Null)
	
	
		' Tools
		Case "AT _Mode"
			B4XPages.ShowPage("AT Command Mode")
		Case "_Export for ESP32"
			btnLoadFile_Click
	End Select

End Sub

Private Sub TabPane1_TabChanged (SelectedTab As TabPage)
	' Switching from BLE to SSP requires scan or Search, if not it will crash!
	ListView1HC05.Items.Clear
	ListView1HM10.Items.Clear
End Sub

Sub CloseOtherConnection(WhichDevice As Int) As Boolean
	' You cant use this when flashing is in progress due to tabpane1 disabled! 
	' Only useful for switching from one connection device to another. Making sure all connection is closed 
	' before opening new connection. 
	
	' Close TTL USB Serial
	If WhichDevice = DEVICE_TTLSERIAL Then
		PerformUserAbort(BUTTON_TTLSERIAL, WhichDevice, False)
		Return True
	
	' Close SSP
	Else If WhichDevice = DEVICE_CLASSIC_BT Then
		PerformUserAbort(BUTTON_CLASSIC_BT, WhichDevice, False)
		Return True

	' Close BLE
	Else If WhichDevice = DEVICE_BLE Then
		PerformUserAbort(BUTTON_BLE, WhichDevice, False)
		Return True

	' Close WIFI
	Else If WhichDevice = DEVICE_WIFI Then
		PerformUserAbort(BUTTON_WIFI, WhichDevice, False)
		Return True
		
	End If
	
	Return False
End Sub
Sub PerformUserAbort(WhichButton As Int, WhichDevice As Int, useMSGBox As Boolean)
	' 1. --- Confirm with User ---
	' msgbox triggers only for Stop button pressed!
	If WhichButton = BUTTON_STOP And useMSGBox = True Then
		Dim sf3 As Object = xui.Msgbox2Async("Flash in progress! Do you want to stop?", "Flashing", "Yes", "", "No", Null)
		Wait For (sf3) Msgbox_Result(ret2 As Int)
		If ret2 = xui.DialogResponse_Positive Then
			LogMessage("STATUS", "User stop flash!")
			If myPicStatus.blnStartFlashVerify = False Then
				LogMessage("WARNING!", "Minimum 20 seconds delay before another flash attempt!")
			End If
		Else
			Return
		End If
	End If
	
	' 2. --- If in blnStartFlashVerify then send 0xCA byte to cancel in PIC ---
	If myPicStatus.blnStartFlashVerify Then
		Dim CancelByte(1) As Byte
		CancelByte(0) = 0xCA		
		Select Case WhichDevice
			Case DEVICE_BLE
				Dim rs As Object
				rs = bkHM10Client.Write(BLE_useUUID, CancelByte)
				Wait For (rs) Complete (Result2 As PyWrapper)
				Sleep(50)
				rs = bkHM10Client.Write(BLE_useUUID, CancelByte)
				Wait For (rs) Complete (Result2 As PyWrapper)
				
			Case DEVICE_CLASSIC_BT
				astream.Write(CancelByte)
				Sleep(50)
				astream.Write(CancelByte)
				
			Case DEVICE_WIFI
				astream.Write(CancelByte)
				Sleep(50)
				astream.Write(CancelByte)
			Case DEVICE_TTLSERIAL
				astream.Write(CancelByte)
				Sleep(50)
				astream.Write(CancelByte)
		End Select
		
	End If

	' 3. --- Close the connection! ---
	' Only closes with Connection Device Button not Stop Button!
	Select Case WhichButton
		Case BUTTON_BLE
			If bkHM10Client.IsInitialized Then
				bkHM10Client.Disconnect
				LogMessage("BLE", "BLE Disconnected")
			End If
			
		Case BUTTON_CLASSIC_BT
			If astream.IsInitialized Then
				astream.Close
				btHC05Connection.Disconnect
				LogMessage("BT SPP", "BT SSP Disconnected")
			End If
		
		Case BUTTON_WIFI
			If astream.IsInitialized Then
				astream.Close
				If WIFIClient.Connected = True Then
					WIFIClient.Close
				End If
				LogMessage("WIFI", "WIFI Disconnected")
			End If
		
		Case BUTTON_TTLSERIAL
			If astream.IsInitialized Then
				astream.Close
				serialUSBTTL.Close
				LogMessage("COM", "Serial COM closed")
			End If
				
	End Select
	
	
	' 4. --- Enable the functions ---
	If WhichButton = BUTTON_STOP Then
		EnableFunction(False)
	Else
		EnableFunction(True)
	End If
	
End Sub

'--------------------------------------------------------
' Load Intel Hex file and convert it to binary (LSB->MSB Little Endian)
'--------------------------------------------------------
Private Sub btnLoadFile_Click
	If cmbPicList.SelectedIndex = -1 Then
		xui.Msgbox2Async("Please select PIC Name from the list!  Conversion requires PIC configuration input!", "PIC Name?", "Ok", "", "", Null)
		Return
	End If
	
	' Init File Chooser
	Dim fc As FileChooser
	fc.Initialize
	fc.Title = "Intel Hex File Only!"
	
	' Add Intel Hex filter only!
	Dim l As List
	l.Initialize
	l.Add("*.hex")
	fc.SetExtensionFilter("Intel Hex", l)
	
	' Get File Path
	Dim filepath As String
	filepath = fc.ShowOpen(B4XPages.GetNativeParent(Me))
   
	If filepath <> "" Then
		strLastFilePath = filepath
		LogMessage("STATUS", "File Path: " & filepath)
		firmwareFile = ConvertHexIntelToBinaryRange(filepath, myConfigMap.intStartAddrFlash, myConfigMap.intEndAddrFlash)
	Else
		strLastFilePath = ""
		LogMessage("STATUS", "No file selected.")
	End If
End Sub
Sub ConvertHexIntelToBinaryRange(filepath As String, startAddr As Int, endAddr As Int) As Byte()
	Try
		Dim lines As List = File.ReadList("", filepath)
        
		' Intel Double Hex Byte Addressed
		Dim startByte, endByte As Int
		If myConfigMap.blnUseDoubleHexAddr = True Then
			startByte = startAddr * 2       ' eg. PIC 16F88, PIC 24F
			endByte = endAddr * 2
		' Non Intel Double Hex Byte Addressed
		Else
			startByte = startAddr           ' eg. PIC 18F27K42
			endByte = endAddr
		End If
        
		' Create binary array relative to startAddr
		Dim firmwareData(myConfigMap.intExpectedFirmwareBytes) As Byte
        
		' --- FIRMWARE INITIALIZATION (Don't leave this out!) ---
		If myConfigMap.blnUse4Padding = True Then
			'LSB, Mid, MSB, Phantom format for PIC24
			For i = 0 To firmwareData.Length - 1 Step 4
				firmwareData(i) = 0xFF                  ' Low Byte
				firmwareData(i+1) = 0xFF                ' Mid Byte
				firmwareData(i+2) = 0xFF                ' High Byte
				firmwareData(i+3) = myConfigMap.intEmptyFlashValue  ' Phantom (Void) Byte
			Next
		Else
			' LSB(0xFF) Then MSB(0x3F or 0xFF) format for 18F and 16F
			For i = 0 To firmwareData.Length - 1 Step 2
				firmwareData(i) = 0xFF					' Low Byte
				firmwareData(i+1) = myConfigMap.intEmptyFlashValue	' High Byte
			Next
		End If
        
		Dim blnDetectRecord As Boolean = False
		Dim upperAddr As Long = 0 ' <--- Essential for addresses > 0xFFFF
        
		' Process each HEX line
		For Each line As String In lines
			line = line.Trim
			If line.Length = 0 Or line.CharAt(0) <> ":" Then Continue
            
			Dim byteCount As Int = Bit.ParseInt(line.SubString2(1, 3), 16)
			Dim byteAddr As Int = Bit.ParseInt(line.SubString2(3, 7), 16)
			Dim recordType As Int = Bit.ParseInt(line.SubString2(7, 9), 16)
            
			' --- HANDLE EXTENDED ADDRESSING (Record Type 04) ---
			If recordType = 4 Then
				' This tells us we are moving to a new 64KB block
				Dim highBits As Int = Bit.ParseInt(line.SubString2(9, 13), 16)
				upperAddr = Bit.ShiftLeft(highBits, 16)
				Continue
			End If
            			
			' Only process Data Records (Type 00)
			If recordType <> 0 Then Continue
            
			' The TRUE address on the chip
			Dim absoluteAddr As Long = upperAddr + byteAddr
            
			' Filter out bootloader/low code
			If absoluteAddr < startAddr Then 
				Log("Low Memory Data Detected at: " & Bit.ToHexString(absoluteAddr).ToUpperCase)
				Continue
			End If

			' Filter out config/higher code
			If absoluteAddr >= endByte Then
				Log("High Memory Data Detected at: " & Bit.ToHexString(absoluteAddr).ToUpperCase)
				Continue
			End If
			
			' Correct range code
			Log("Valid Memory: " & Bit.ToHexString(absoluteAddr).ToUpperCase)
            
			' Map to our array index
			Dim arrayIndex As Long = absoluteAddr - startByte
            
			For i = 0 To byteCount - 1
				Dim b As Int = Bit.ParseInt(line.SubString2(9 + i * 2, 11 + i * 2), 16)
                
				Dim currentIndex As Long = arrayIndex + i
                
				If currentIndex >= 0 And currentIndex < firmwareData.Length Then
					firmwareData(currentIndex) = b
					blnDetectRecord = True
				End If
			Next
		Next
        
		If blnDetectRecord Then
			btnFlash.Enabled = True
			btnVerify.Enabled = True
			LogMessage("STATUS", "Conversion success.")
			ExportBinaryFile(firmwareData)		' This will be used for esp32 project.  Use binary instead of Intel Hex
			ExportConfigFile					' This will be used for esp32 project. 
			intFileTotalChecksum = CalculateSum8(firmwareData)
		Else
			btnFlash.Enabled = False
			btnVerify.Enabled = False
			LogMessage("STATUS", "Error: No valid data found above start address.")
		End If
		        
		Return firmwareData
        
	Catch
		LogMessage("STATUS", "Error:  " & LastException & " in ConvertHexIntelToBinaryRange")
		Dim EmptyArr() As Byte = Array As Byte()
		Return EmptyArr
	End Try
End Sub
Sub ExportBinaryFile(binData() As Byte)
	If File.Exists(File.DirApp, FLASH_BIN) Then
		File.Delete(File.DirApp, FLASH_BIN)
	End If
	File.WriteBytes(File.DirApp, FLASH_BIN, binData)
	LogMessage("STATUS", "export @ " & File.DirApp & "\" & FLASH_BIN)
End Sub
Sub ExportConfigFile
	If File.Exists(File.DirApp, CONFIG_MAP) Then
		File.Delete(File.DirApp, CONFIG_MAP)
	End If
	
	Dim cfg As Map
	cfg.Initialize
	
	cfg.Put("StartAddrFlash", myConfigMap.intStartAddrFlash)
	cfg.Put("EndAddrFlash", myConfigMap.intEndAddrFlash)
	cfg.Put("EmptyFlashValue", myConfigMap.intEmptyFlashValue)
	cfg.Put("InstructionPacket", myConfigMap.intInstructionPacket)
	cfg.Put("PacketDelayMS", myConfigMap.intPacketDelayMS)
	cfg.Put("HandShakeDelayMS", myConfigMap.intHandShakeDelayMS)
	cfg.Put("StopBit", myConfigMap.intStopBit)
	cfg.Put("Notes", myConfigMap.strNotes)
	cfg.Put("ExpectedBytes", myConfigMap.intExpectedFirmwareBytes)
	cfg.Put("UseWriteBurst", myConfigMap.blnUseWriteBurst)
	cfg.Put("UseDoubleHexAddr", myConfigMap.blnUseDoubleHexAddr)
	cfg.Put("Use4Padding", myConfigMap.blnUse4Padding)
	cfg.Put("PicName", myConfigMap.strPicName)
	cfg.Put("UseCheckSum", myConfigMap.blnUseCheckSum)
	
	File.WriteMap(File.DirApp, CONFIG_MAP, cfg)
	LogMessage("STATUS", "export @ " & File.DirApp & "\" & CONFIG_MAP)
End Sub
Sub CalculateSum8(binData() As Byte) As Int
	Dim checksum As Int = 0
	For Each b As Byte In binData
		' Convert signed byte to unsigned 0-255 and add
		checksum = (checksum + Bit.And(b, 0xFF))
	Next
	Dim toHexStr As String
	toHexStr = Bit.ToHexString(Bit.And(checksum, 0xFF)).ToUpperCase
	If toHexStr.Length = 1 Then toHexStr = "0" & toHexStr
	LogMessage("STATUS", "File Checksum: 0x" & toHexStr & " (" & Bit.And(checksum, 0xFF) & ")")
	
	Return Bit.And(checksum, 0xFF) ' Keep only the last 8 bits (0-255); return int
End Sub

'--------------------------------------------------------
' Start Handshake and Firmware Upload
'--------------------------------------------------------
Private Sub btnFlash_Click
	If btnFlash.Text = txt_FLASH Then
		ProcessFlashType(BUTTON_FLASH, WhichDeviceConnection)
	Else
		ProcessFlashType(BUTTON_STOP, WhichDeviceConnection)
	End If
End Sub
Private Sub btnVerify_Click
	ProcessFlashType(BUTTON_VERIFY, WhichDeviceConnection)
End Sub
Sub ProcessFlashType(WhichButton As Int, WhichDevice As Int)
		' Check if configuration .map available
	If cmbPicList.Items.Size = 0 Then
		xui.Msgbox2Async("Configuration is missing!", "Configuration", "Ok", "", "", Null)
		' Make sure there is connection established
	Else If WhichDevice = DEVICE_NONE Then
		xui.Msgbox2Async("Please connect or open port!", "Connection Required!", "Ok", "", "", Null)
		' Flashing/Verifying Logic
	Else If WhichButton = BUTTON_FLASH Or WhichButton = BUTTON_VERIFY Then
		Dim sf2 As Object = xui.Msgbox2Async("If the PIC application is running, it will enter bootloader mode automatically. " & _
  											 "If not, please click OK and then power cycle.", _
                                      		 "Attention!", "Ok", "Cancel", "", Null)
		Wait For (sf2) Msgbox_Result(ret As Int)
		If ret = xui.DialogResponse_Positive Then
			SendHandShakeBytes(WhichButton, WhichDevice)
		End If
		' Flash Stop Logic
	Else If WhichButton = BUTTON_STOP Then 
		PerformUserAbort(WhichButton, WhichDevice, True)
	End If
End Sub
Sub SendHandShakeBytes(WhichButton As Int, WhichDevice As Int)
	Dim rs As Object
	Dim blnToggle As Boolean
	Dim b() As Byte = Array As Byte(0x55)
	Dim b2() As Byte = Array As Byte(0xAA)
	
	DisableFunction
		
	Do While True
		' Status boolean
		If isOperationFailed = True Then 
			Return
		End If
		
		' Exit and Start Config upload
		If myPicStatus.blnHandShakeSuccess = True Then
			Sleep(300) ' give firmware time to call ReceiveConfig.
			SendConfigBytes(WhichButton, WhichDevice)
			Return
		Else
			If blnToggle = False Then
				' BLE
				If WhichDevice = DEVICE_BLE Then
					rs = bkHM10Client.Write(BLE_useUUID, b)
					Wait For (rs) Complete (Result2 As PyWrapper)
				' OTHERS (SSP, WIFI and TTL USB)
				Else
					If astream.IsInitialized Then astream.Write(b)
				End If
				LogMessage("HANDSHAKE", "Sending: 0x55")
			Else
				' BLE
				If WhichDevice = DEVICE_BLE Then
					rs = bkHM10Client.Write(BLE_useUUID, b2)
					Wait For (rs) Complete (Result2 As PyWrapper)
				' OTHERS (SSP, WIFI and TTL USB)
				Else
					If astream.IsInitialized Then astream.Write(b2)
				End If

				LogMessage("HANDSHAKE", "Sending: 0xAA")
			End If
		End If
				
		' Avoid flooding UART
		Sleep(myConfigMap.intHandShakeDelayMS)		' Recommend 200 ms minimum in .map file
		
		blnToggle = Not(blnToggle)
	Loop
End Sub
Sub SendConfigBytes(WhichButton As Int, WhichDevice As Int)
	' One time shot. The PIC will be ready to poll!
	' Byte 1: 		0x01 = BLE
	'				0x02 = BT Classic
	' 				0x03 = WIFI
	' 				0x04 = TTL Serial
	' Byte 2 and 3: 0x00 and 0x14 = 16Bit Number MTU Size
	' Byte 4: 		0x00 = Flash and Verify Byte for Byte
	'				0x01 = Flash and Verify Checksum
	' 				0x02 = Verify Byte for Byte only
	' 				0x03 = Verify Checksum only
	
	Dim rs As Object
	Dim byteONE(1), byteTWO(1), byteTHREE(1), byteFourth(1) As Byte

	' 1. --- Set the Flag ---
	Select Case WhichDeviceConnection
		Case DEVICE_BLE:		byteONE(0) = 0x01		
		Case DEVICE_CLASSIC_BT: byteONE(0) = 0x02	
		Case DEVICE_WIFI:		byteONE(0) = 0x03	
		Case DEVICE_TTLSERIAL:  byteONE(0) = 0x04
	End Select

	' 2. --- Extract High Byte (Most Significant Byte) ---
	' Shift right by 8 bits to move the top 8 bits into the bottom 8 bits
	byteTWO(0) = Bit.ShiftRight(Bit.And(BLE_useMTUSize, 0xFF00), 8)

	' 3. --- Extract Low Byte (Least Significant Byte) ---
	' Use a mask to keep only the bottom 8 bits
	byteTHREE(0) = Bit.And(BLE_useMTUSize, 0xFF)

	' 4. --- Which Flash Type using ---
	If WhichButton = BUTTON_FLASH Then
		If chkCheckSum.Checked = True Then
			byteFourth(0) = 0x01
		Else
			byteFourth(0) = 0x00
		End If
	Else
		If chkCheckSum.Checked = True Then
			byteFourth(0) = 0x03
		Else
			byteFourth(0) = 0x02				
		End If
	End If

	' BLE
	If WhichDevice = DEVICE_BLE Then
		rs = bkHM10Client.Write(BLE_useUUID, byteONE)
		Wait For (rs) Complete (Result2 As PyWrapper)
		Sleep(50)
		rs = bkHM10Client.Write(BLE_useUUID, byteTWO)
		Wait For (rs) Complete (Result2 As PyWrapper)
		Sleep(50)
		rs = bkHM10Client.Write(BLE_useUUID, byteTHREE)
		Wait For (rs) Complete (Result2 As PyWrapper)
		Sleep(50)
		rs = bkHM10Client.Write(BLE_useUUID, byteFourth)
		Wait For (rs) Complete (Result2 As PyWrapper)
		Sleep(50)
	' OTHERS (SSP, WIFI and TTL USB)
	Else
		If astream.IsInitialized Then 
			astream.Write(byteONE)
			Sleep(50)
			astream.Write(byteTWO)
			Sleep(50)
			astream.Write(byteTHREE)
			Sleep(50)
			astream.Write(byteFourth)
			Sleep(50)
		End If
	End If
	LogMessage("CFG BYTES", "Sending: 0x" & Bit.ToHexString(byteONE(0)).ToUpperCase & ", " & "0x" & Bit.ToHexString(byteTWO(0)).ToUpperCase & ", " & "0x" & Bit.ToHexString(byteTHREE(0)).ToUpperCase& ", " & "0x" & Bit.ToHexString(byteFourth(0)).ToUpperCase)
					
	Do While myPicStatus.blnConfigOK = False
		If isOperationFailed = True Then
			Return
		End If
		Sleep(50)
	Loop
		
End Sub
Sub SendFirmwareBytes(WhichDevice As Int)
	' Firmware Binary file must include all flash data including empty addresses!
	Dim intBlockSize As Int
	
	If myConfigMap.blnUse4Padding = True Then
		intBlockSize = myConfigMap.intInstructionPacket * 4 ' eg. 64 words = 256 intBlockSize 24FJ64GA102
	Else
		intBlockSize = myConfigMap.intInstructionPacket * 2 ' eg. 4 words = 8 intBlockSize 16F88
	End If
	
	Dim block(intBlockSize) As Byte
	Dim totalBlocks As Int = Ceil(firmwareFile.Length / intBlockSize)
	Dim rs As Object
	
	LogMessage("FIRMWAREUPLOAD", "Firmware size: " & firmwareFile.Length & " bytes, total blocks: " & totalBlocks & ", bytes/block: " & intBlockSize)
	
	' Loop through file buffer and send them to PIC
	Dim i As Int = 0
	Do While i <= firmwareFile.Length

		' Copy bytes into block, pad with 0xFF if last block is smaller
		Dim remaining As Int = firmwareFile.Length - i
		Dim currentBlockSize As Int = Min(intBlockSize, remaining)
		For j = 0 To intBlockSize - 1
			If j < currentBlockSize Then
				block(j) = firmwareFile(i + j)
			Else
				block(j) = 0xFF   ' padding (Should never trigger here! cause problems if it does?)
			End If
		Next

		' Check for global abort
		If isOperationFailed = True Then
			Return
		End If

		' Wait for ACK or handle timeout
		Do While myPicStatus.blnWriteACK = False
			If isOperationFailed = True Then
				Return
			End If

			If myPicStatus.blnISRTimeOut = True Then
				LogMessage("FIRMWAREUPLOAD", "Timeout detected, retrying at byte #" & i)
				myPicStatus.blnISRTimeOut = False
				Continue    ' retry immediately
			End If

			Sleep(0)
		Loop
		
		' Reset ACK for next block
		myPicStatus.blnWriteACK = False
		
		'--------------------------------------------------------------------------------
		' Burst = False
		'--------------------------------------------------------------------------------
		If myConfigMap.blnUseWriteBurst = False Then
			For x = 0 To intBlockSize - 1
				Dim b(1) As Byte
				b(0) = block(x)
				'------------------------------------------------------------------------
				' BLE (Never configure .map to use this!
				'------------------------------------------------------------------------
				If WhichDevice = DEVICE_BLE Then
					rs = bkHM10Client.WriteWithResponse(BLE_useUUID, b, False)
					Wait For (rs) Complete (Result2 As PyWrapper)
				'------------------------------------------------------------------------
				' OTHERS (SSP, WIFI and TTL USB)
				'------------------------------------------------------------------------
				Else
					If astream.IsInitialized Then astream.Write(b)
				End If
				
				Sleep(myConfigMap.intPacketDelayMS)
				
			Next
		'-----------------------------------------------------------------------------
		' Burst = True (Common)
		'-----------------------------------------------------------------------------
		Else
			'-----------------------------------------------------------------------------
			' BLE
			'-----------------------------------------------------------------------------
			If WhichDevice = DEVICE_BLE Then
				' BLE Flash Write is supported by MTU Size requested
				' Need to test HM-20 supports over 400 mtu size!  
				' HM-10 tested at 20 mtu really sucks!
				Dim bc As ByteConverter
				Dim chunkSize As Int = Max(20, BLE_useMTUSize)
				
				' Write intBlockSize
				If intBlockSize <= chunkSize Then
					' Send block at once if it fits within MTU Size limits
					rs = bkHM10Client.WriteWithResponse(BLE_useUUID, block, False)
					Wait For (rs) Complete (Result2 As PyWrapper)
				' Write fragment of intBlockSize
				Else
					' Fragment the block because its larger than the MTU Size
					For x = 0 To intBlockSize - 1 Step chunkSize
						Dim currentChunkSize As Int = Min(chunkSize, intBlockSize - x)
						Dim tempChunk(currentChunkSize) As Byte
						bc.ArrayCopy(block, x, tempChunk, 0, currentChunkSize)      
						rs = bkHM10Client.WriteWithResponse(BLE_useUUID, tempChunk, False)
						Wait For (rs) Complete (Result2 As PyWrapper)
						
						' No delay required. Tested successfully!
					Next
				End If
			'-------------------------------------------------------------------------
			' OTHERS (SSP, WIFI and TTL USB)
			'-------------------------------------------------------------------------
			Else
				If astream.IsInitialized Then astream.Write(block)
			End If
			
			Sleep(myConfigMap.intPacketDelayMS)
		End If

		' Update progress bar
		prgBar.Progress = Min(1, (i + intBlockSize) / firmwareFile.Length)

		' Move to next block
		i = i + intBlockSize
	Loop

	LogMessage("FIRMWAREUPLOAD", "Firmware upload completed!")
End Sub
Sub isOperationFailed As Boolean
	
	' Astream error or terminated or BLE Disconnected!
	If myPicStatus.blnAstreamError = True Then
		EnableFunction(True)
		Return True
	End If
	
	' PIC reported timeout error (Handshake does not have this!)
	If myPicStatus.blnTimeoutError = True Then
		EnableFunction(False)
		Return True
	End If
		
	' Stop or Quit Detected
	If myPicStatus.blnUserCancel = True Then
		EnableFunction(False)
		Return True
	End If
	
	Return False
End Sub

'--------------------------------------------------------
' Verify Firmware
'--------------------------------------------------------
Sub VerifyStatus
	If CompareFirmware = True Then
		LogMessage("STATUS", "Programming/Verify Success")
	Else
		LogMessage("STATUS", "Programming/Verify Failed!")
	End If
End Sub
Sub CompareFirmware As Boolean
	' Make sure both arrays are the same length
	If firmwareFile.Length <> firmwareVerify.Length Then
		Return False
	End If

	' Compare byte by byte
	For i = 0 To firmwareFile.Length - 1
		If firmwareFile(i) <> firmwareVerify(i) Then
			LogMessage("STATUS", "Mismatch at byte " & i & _
                       ": firmware file = " & BytesToHexString2(firmwareFile(i)) & _
                       " vs firmware verify = " & BytesToHexString2(firmwareVerify(i)))
			Return False
		End If
	Next

	' All bytes match
	Return True
End Sub
Sub VerifyStatusChecksum
	If CompareChecksum = True Then
		LogMessage("STATUS", "Programming/Verify Success")
	Else
		LogMessage("STATUS", "Programming/Verify Failed!")
	End If
End Sub
Sub CompareChecksum As Boolean
	Dim toHexStr, toHexStr2 As String
	toHexStr = Bit.ToHexString(intFileTotalChecksum).ToUpperCase
	toHexStr2 = Bit.ToHexString(intVerifyTotalChecksum).ToUpperCase
	If toHexStr.Length = 1 Then toHexStr = "0" & toHexStr
	If toHexStr2.Length = 1 Then toHexStr2 = "0" & toHexStr2
	LogMessage("STATUS", "File Checksum: 0x" & toHexStr & " (" & intFileTotalChecksum & ")" & " / " & "Verify Checksum: 0x" & toHexStr2 & " (" & intVerifyTotalChecksum & ")")

	' If not same return false
	If intFileTotalChecksum <> intVerifyTotalChecksum Then
		Return False
	End If

	' All bytes match
	Return True
End Sub

'--------------------------------------------------------
' Disable/Enable
'--------------------------------------------------------
Sub DisableFunction
	' Buttons disabled
	MenuBar1.Enabled = False
	TabPane1.Enabled = False
	btnLoadFile.Enabled = False
	cmbPicList.Enabled = False
	chkCheckSum.Enabled = False
	btnVerify.Visible = False
	btnFlash.Text = txt_STOP

	' Reset myPicStatus
	myPicStatus.blnUserCancel = False
	myPicStatus.blnHandShakeSuccess = False
	myPicStatus.blnTimeoutError = False
	myPicStatus.blnAstreamError = False
	myPicStatus.blnISRTimeOut = False
	myPicStatus.blnConfigOK = False
	myPicStatus.blnStartFlashVerify = False
	myPicStatus.blnWriteACK = False
	
	txtLog.Text = ""
	prgBar.Progress = 0		
	
End Sub
Sub EnableFunction(ResetConnectionState As Boolean)
	' Buttons Enabled
	MenuBar1.Enabled = True
	TabPane1.Enabled = True
	btnLoadFile.Enabled = True
	cmbPicList.Enabled = True
	chkCheckSum.Enabled = True
	btnVerify.Visible = True
	btnFlash.Text = txt_FLASH
	
	myPicStatus.blnUserCancel = True
	
	If ResetConnectionState = True Then
		WhichDeviceConnection = DEVICE_NONE
		btnConnectHM10.Text = txt_CONNECT
		btnConnectHC05.Text = txt_CONNECT
		btnConnectWIFI.Text = txt_CONNECT
		btnOpenUSBTTL.Text = txt_OPEN_PORT
		btnRefreshComUSBTTL.Enabled = True
	End If
End Sub

'--------------------------------------------------------
' Load all Map files from /configs and return PicName list
'--------------------------------------------------------
Sub LoadAllPicNames As List
	Dim picList As List
	picList.Initialize
    
	' Make sure folder exists
	If File.Exists(File.DirApp, "configs") = False Then
		LogMessage("STATUS", "Missing configs directory, creating one!")
		File.MakeDir(File.DirApp, "configs")
		Return picList ' empty list
	End If
    
	' Get all files in configs folder
	Dim cfgDir As String = File.Combine(File.DirApp, "configs")
	Dim files As List = File.ListFiles(cfgDir)
    
	For Each f As String In files
		If f.ToLowerCase.EndsWith(".map") Then
			Try
				Dim cfg As Map = File.ReadMap(cfgDir, f)
				If cfg.ContainsKey("PicName") Then
					picList.Add(cfg.Get("PicName"))
				Else
					' fallback to filename if PicName missing
					LogMessage("STATUS", "Missing Pic Name - " & f)
				End If
			Catch
				' skip files that fail to load
				Log(LastException.Message)
			End Try
		End If
	Next
    
	Return picList
End Sub
Sub LoadConfiguration(SelectedPicName As String) As Boolean
	
	' Make sure folder exists
	If File.Exists(File.DirApp, "configs") = False Then
		Return False
	End If
	
	Dim cfgDir As String = File.Combine(File.DirApp, "configs")
	
	' Get all files in configs folder
	Dim files As List = File.ListFiles(cfgDir)
    
	For Each f As String In files
		If f.ToLowerCase.EndsWith(".map") Then
			Try
				Dim cfg As Map = File.ReadMap(cfgDir, f)
				If cfg.ContainsKey("PicName") Then
					Dim CheckName As String = cfg.Get("PicName")
					If CheckName = SelectedPicName Then
						txtLog.Text = ""
						
						myConfigMap.intStartAddrFlash = cfg.Get("StartAddrFlash")		' Start Address of Flash. For Intel Hex Conversion
						myConfigMap.intEndAddrFlash = cfg.Get("EndAddrFlash")			' End Address of Flash
						myConfigMap.intEmptyFlashValue = cfg.Get("EmptyFlashValue")		' Empty Flash Value (eg. 3FFF = 3F)
						myConfigMap.intInstructionPacket = cfg.Get("InstructionPacket")	' Total Instruction Words Per Packet for Write Block
						myConfigMap.intPacketDelayMS = cfg.Get("PacketDelayMS")			' Write Block Packet Delay
						myConfigMap.intHandShakeDelayMS = cfg.Get("HandShakeDelayMS")	' Handshake Delay (0x55 and 0xAA in between delay)
						myConfigMap.intStopBit = cfg.Get("StopBit")						' 1 or 2 stop bits, older PIC need 2 so it buys time processing other instructions
						myConfigMap.strNotes = cfg.Get("Notes")							' Special Notes
						myConfigMap.intExpectedFirmwareBytes = cfg.Get("ExpectedBytes") ' Total Bytes need flash and erase
						myConfigMap.blnUseWriteBurst = cfg.Get("UseWriteBurst")			' No delays in between bytes if True!
						myConfigMap.blnUseDoubleHexAddr = cfg.Get("UseDoubleHexAddr") 	' For Intel Hex Conversion
						myConfigMap.blnUse4Padding = cfg.Get("Use4Padding")				' For Intel Hex conversion
						myConfigMap.strPicName = cfg.Get("PicName")						' PIC Name
						myConfigMap.blnUseCheckSum = cfg.Get("UseCheckSum")				' Checksum usage
						
						' Set Proper Arrays to FirmwareVerfiy()
						firmwareVerify = Array As Byte()
						Dim temp(myConfigMap.intExpectedFirmwareBytes) As Byte
						firmwareVerify = temp
		
						' Make sure reload the Intel Hex file to new firmwareFile() array
						If strLastFilePath <> "" Then
							firmwareFile = ConvertHexIntelToBinaryRange(strLastFilePath, myConfigMap.intStartAddrFlash, myConfigMap.intEndAddrFlash)
						End If
						
						' Enable/Disable Checksum based on UseCheckSum
						chkCheckSum.Checked = myConfigMap.blnUseCheckSum
						
						' If port already open update stop bit parameter!
						If WhichDeviceConnection = DEVICE_TTLSERIAL Then
							LogMessage("Status", "Reapplying Parameters to Serial Com")
							PerformUserAbort(BUTTON_TTLSERIAL, WhichDeviceConnection, False) ' close
							OpenUSBTLL ' open
						End If
						
						LogMessage("", "---------------------------------------------------------")
						LogMessage("", "CONFIGURATION FOR " & CheckName)
						LogMessage("", "---------------------------------------------------------")
						LogMessage(":::", "Notes: " & myConfigMap.strNotes)
						LogMessage(":::", "Start Address = 0x" & Bit.ToHexString(myConfigMap.intStartAddrFlash).ToUpperCase)
						LogMessage(":::", "End Address = 0x" & Bit.ToHexString(myConfigMap.intEndAddrFlash).ToUpperCase)
						LogMessage(":::", "Empty Flash Value = 0x" & Bit.ToHexString(myConfigMap.intEmptyFlashValue).ToUpperCase)
						LogMessage(":::", "HandShake Delay = " & myConfigMap.intHandShakeDelayMS & " ms")
						LogMessage(":::", "Packet Delay = " & myConfigMap.intPacketDelayMS & " ms")
						LogMessage(":::", "Stop Bit = " & myConfigMap.intStopBit)
						If myConfigMap.blnUse4Padding = True Then
							LogMessage(":::", "Instruction Write Size = " & (myConfigMap.intInstructionPacket * 4) & " bytes w/padding (" &  myConfigMap.intInstructionPacket & " instructions)" )
						Else
							LogMessage(":::", "Instruction Write Size = " & (myConfigMap.intInstructionPacket * 2) & " bytes w/padding (" &  myConfigMap.intInstructionPacket & " instructions)" )
						End If
						LogMessage(":::", "Expected Firmware Size = " & NumberFormat2(myConfigMap.intExpectedFirmwareBytes, 1, 0, 0, True) & " bytes")
						LogMessage(":::", "Use Write Burst = " & myConfigMap.blnUseWriteBurst)
						LogMessage(":::", "Use Double Hex Addressed = " & myConfigMap.blnUseDoubleHexAddr)
						LogMessage(":::", "Use 4 Padding = " & myConfigMap.blnUse4Padding)
						LogMessage("", "---------------------------------------------------------")
  						
						Return True
					End If			
				End If
			Catch
				' skip files that fail to load
				Log(LastException.Message)
			End Try
		End If
	Next
	
	Return False
    
End Sub

'--------------------------------------------------------
' Log Message
'--------------------------------------------------------
Sub LogMessage(From As String, Msg As String)
	' Clear if exceeded
	'If txtLog.Text.Length > 15000 Then
	'	txtLog.Text = ""
	'End If
	txtLog.Text = txtLog.Text & From & ": " & Msg & CRLF
	txtLog.SetSelection(txtLog.Text.Length, txtLog.Text.Length)
End Sub
Sub BytesToHexString(b() As Byte) As String
	Dim sb As StringBuilder
	sb.Initialize
	For Each bt As Byte In b
		Dim byteString As String
		byteString = Bit.ToHexString(Bit.And(bt, 0xFF))
		If byteString.Length = 1 Then byteString = "0" & byteString
		sb.Append(byteString).Append(" ")
	Next
	Return sb.ToString.Trim.ToUpperCase
End Sub
Sub BytesToHexString2(b As Byte) As String
	Dim byteString As String

	byteString = Bit.ToHexString(Bit.And(b, 0xFF))
	If byteString.Length = 1 Then byteString = "0" & byteString
	
	Return byteString.ToUpperCase
End Sub



