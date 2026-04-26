B4A=true
Group=Default Group
ModulesStructureVersion=1
Type=Class
Version=9.85
@EndOfDesignText@
' Author: Issac
' Features: Bluetooth SSP 2.0, Bluetooth BLE 4.0, WIFI TCP/IP and USB TTL Serial (COM)

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
	Private Version As String = "10.07"
	
	'---------------------------------------
	' Map Config Variables
	'---------------------------------------
	Private intStartAddrFlash As Int 						' Used with Intel Hex Conversion
	Private intEndAddrFlash   As Int						' Used with Intel Hex Conversion
	Private intEmptyFlashValue As Int						' 16F = 0x3F, 18F = 0xFF, 24F = 0x00 Phantom 24bit
	Private intInstructionPacket As Int						' Number of Instruction per block
	Private intPacketDelayMS As Int							' Delay for Tx Write Packets
	Private intHandShakeDelayMS As Int						' Delay for 0x55 and 0xAA in between	
	Private intStopBit As Int = 1							' Default
	Private strNotes As String 
	Private intExpectedFirmwareBytes As Int					' Total Firmware bytes
	Private blnUseWriteBurst  As Boolean					' True = Tx Write Packet as whole, no delays in between!
	Private blnUseDoubleHexAddr As Boolean					' 16F, 24F = Increment 2 Hex Address, 18F = Increment 1 Hex Address. Used with Intel Hex Conversion
	Private blnUse4Padding As Boolean						' 24 Bit need step 4, others step 2. Used with Intel Hex Conversion
	
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
	Private txtLog As TextArea
	Private prgBar As ProgressBar
	Private cmbPicList As ComboBox
	
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
	
	
	Private firmwareFile() As Byte							' firmware binary from FILE
	Private firmwareVerify() As Byte						' firmware binary from PIC
	Private cntVerify As Int								' Counter detection of incoming verify bytes from PIC
	
	Private blnHandShakeSuccess As Boolean					' <InitReceived> from PIC
	Private blnConfigOK As Boolean							' <ConfigOK> from PIC exit function (Success bytes received)
	Private blnWriteACK As Boolean							' <ACK> from PIC used in Firmware Upload.  Needs this <ACK> from PIC to continue next Block Write bytes
	Private blnISRTimeOut As Boolean						' <ISR Timeout> Timeout Detected from PIC
	Private blnTimeoutError As Boolean						' <TimeoutError> 3 ISR Timeout from PIC
	Private blnStartVerifyRequest As Boolean				' <StartFlashVerify> from PIC
	Private blnAstreamError	As Boolean						' Astream error exit loop
	Private blnAppStopQuit As Boolean						' Exit loop for Stop Flash, Disconnect and Quit App
	
	Private rxBufferString As String						' Buffer Newdata in string format PIC Message only
	Private strLastFilePath As String						' Reloads firmware from FILE when PIC name changed so Firmware array be corrected
	
	Private foundDevices As Map								' Bluetooth list for both SSP and BLE		
	Private BLE_useUUID As String							' What Characteristic for BLE
	Private BLE_useMTUSize As Int = 20						' Write Flash support, Verify Flash in firmware will default at 20 now!
	
	
End Sub

Public Sub Initialize
	foundDevices.Initialize
	btHC05.Initialize("btHC05")
	serialUSBTTL.Initialize("serial")
	WIFIClient.Initialize("WifiClient")
	ATCommandMode.Initialize
End Sub

'--------------------------------------------------------
'This event will be called once, before the page becomes visible.
'--------------------------------------------------------
Private Sub B4XPage_Created (Root1 As B4XView)
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
	If btnFlash.Text = "Stop" Then
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
	If rxBufferString.Contains(">") Or blnStartVerifyRequest = True Then
		HandleMessage(rxBufferString, Buffer)
		rxBufferString = ""
	End If
	
End Sub
Sub astream_Error
	LogMessage("STATUS", "Error: " & LastException)
	astream_Terminated
End Sub
Sub astream_Terminated
	blnAstreamError = True
	
	If astream.IsInitialized Then
		astream.Close
		If WIFIClient.Connected = True Then 
			WIFIClient.Close
		End If
		serialUSBTTL.Close
		btHC05Connection.Disconnect
		btnConnectHC05.Text = "Connect"
		btnOpenUSBTTL.Text = "Open Port"
		btnConnectWIFI.Text = "Connect"
	End If
	EnableFunction
	LogMessage("STATUS", "Connection is terminated!")
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
	If ListView1HC05.Items.Size > 0 Then btnConnectHC05.Enabled = True
	LogMessage("BLUETOOTH", "Discovery completed")
End Sub

'--------------------------------------------------------
' HM-10 Bluetooth BLE Routine
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
	btnConnectHM10.Text = "Connect"
	EnableFunction
	LogMessage("STATUS", "Bluetooth Disconnected! @ " & DeviceId)
End Sub
Private Sub btHM10_CharNotify (Notification As BleakNotification)
	Dim Buffer() As Byte = Notification.Value

	' Option display Hex for Debugging!
	'Dim AddHexView As String = BytesToHexString(Buffer).ToUpperCase
	'LogMessage("HEX", AddHexView)
		
	' Buffer for Messages, PIC Sends
	rxBufferString = rxBufferString & BytesToString(Buffer, 0, Buffer.Length, "UTF8")
	
	'-------------------------------------------------------------------------------------
	' Temp Workaround until firmware solution (BLE ONLY!) (NOT NEEDED!! Fixed in HandleMessage)
	'-------------------------------------------------------------------------------------
	' FILTER: Find the first valid starting bracket
	' 24F causes bad char at initial power up.  always the first character PIC sends!
'	If rxBufferString.Contains("<") Then
'		Dim StartPos As Int = rxBufferString.IndexOf("<")
'	        
'		' If there is any junk before the "<", discard it immediately
'		If StartPos > 0 Then
'			rxBufferString = rxBufferString.SubString(StartPos)
'		End If
'	End If
	
	' UPDATE:
	' I changed HandleMessage routine to handle this by using contain property!
	'-------------------------------------------------------------------------------------
	
	' PIC sends with > as last byte to confirm end of message
	' When VerifyRequest = true, it does not received ">". Sticktly bytes only!
	' Buffer is meant for Verify Bytes only.
	If rxBufferString.Contains(">") Or blnStartVerifyRequest = True Then
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
	' We dont want to log Incoming PIC VERIFY bytes
	If blnStartVerifyRequest <> True And msg <> "<ACK>" Then
		LogMessage("PIC", msg)
	End If
	
	' This is triggered by <StartFlashVerify> from PIC after Flash Write is completed
	If blnStartVerifyRequest = True Then
		'LogMessage("INCOMMING", BytesToHexString(buffer))  ' debugging only!!!					
		For x = 0 To buffer.Length - 1  ' This method is better.  Newdata does not guarantee all block in one event
			' This array will compare to firmware() which is Converted FILE binary
			firmwareVerify(cntVerify) = buffer(x)
										
			' Update progress bar
			prgBar.Progress = Min(1, cntVerify / intExpectedFirmwareBytes)
			cntVerify = cntVerify + 1
					
			' Check if we reached the expected firmware size
			If cntVerify >= intExpectedFirmwareBytes Then
				' Let <EndFlashVerify> display the status of Verify!
				' Just enable button here
				EnableFunction
				Exit
			End If
			Next
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
			blnHandShakeSuccess = True	' Must do this first. Exit handshake no more 0x55 0xAA. 
			LogMessage("STATUS", "Bootloader responded.")
			
		Else If msg.Contains("<ConfigTimeout>") Then
			blnTimeoutError = True
			EnableFunction
			LogMessage("STATUS", "PIC reported timeout error, try again")
			
		Else If msg.Contains("<ConfigOK>") Then
			blnConfigOK = True
								
			' End of Erase Flash. When Pic sends this, delay a bit and start the flash upload
		Else If msg.Contains("<EndFlashErase>") Then
			Sleep(300)
			SendFirmwareBytes
			
			' B4J expects <ACK> from PIC so B4J sends next packets in Firmware Upload routine
		Else If msg.Contains("<ACK>") Then
			blnWriteACK = True

			' Timeout occurred with Flash Write.  This will attempt a redo send
		Else If msg.Contains("<ISR Timeout>") Then
			blnISRTimeOut = True
			
			' 3 ISR timeout = error by PIC
		Else If msg.Contains("<ErrorTimeout>") Then
			blnTimeoutError = True
			EnableFunction
			LogMessage("STATUS", "PIC reported timeout error, try again")

			' Start of verify flash program code
		Else If msg.Contains("<StartFlashVerify>") Then
			cntVerify = 0
			blnStartVerifyRequest = True
			LogMessage("STATUS", "Waiting for verification...")
				
			' End of verify flash program code
		Else If msg.Contains("<EndFlashVerify>") Then
			EnableFunction
			VerifyStatus
			
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
			LogMessage("BLUETOOTH", "Searching, please wait...")
		Else
			Log("Error starting discovery")
		End If
	Else
		xui.Msgbox2Async("Bluetooth is disabled. Please turn it on!", "Bluetooth", "Ok", "", "", Null)
	End If
End Sub
Private Sub btnConnectHC05_Click
	' USE (57600 Baud) from HC05 to PIC
	If btnConnectHC05.Text = "Connect" Then
		If ListView1HC05.SelectedIndex <> - 1 Then
			If btHC05.IsEnabled = False Then
				xui.Msgbox2Async("Bluetooth is disabled. Please turn it on!", "Bluetooth", "Ok", "", "", Null)
				Return
			End If
			
			CloseConnection(True, False, True, True)
		
			btHC05.CancelDiscovery
			
			' Connect Bluetooth with Address selected from listview
			Dim address As String = foundDevices.Get(ListView1HC05.SelectedItem)
			btHC05.Connect(address)
			LogMessage("STATUS", "Connecting to " & ListView1HC05.SelectedItem & "...")
			wait for btHC05_Connected (Success As Boolean, connection As BluetoothConnection)
			btHC05Connection = connection
			If Success Then
				If astream.IsInitialized Then astream.Close
				astream.Initialize(connection.InputStream, connection.OutputStream, "astream")
				btnConnectHC05.Text = "Disconnect"
				LogMessage("STATUS", "Bluetooth Connected! @ " & address)
			Else
				LogMessage("STATUS", "Bluetooth Failed! @ " & address)
			End If
		Else
			xui.Msgbox2Async("Please select Bluetooth fom the list!", "Select Bluetooth", "Ok", "", "", Null)
		End If
	' Disconnect
	Else
		' If its flashing inform with msgbox
		If btnFlash.Text = "Stop" Then
			Dim sf3 As Object = xui.Msgbox2Async("Flash in progress! Do you want to stop?", "Flashing", "Yes", "", "No", Null)
			Wait For (sf3) Msgbox_Result(ret2 As Int)
			If ret2 = xui.DialogResponse_Positive Then
				LogMessage("STATUS", "User stop flash!")
				EnableFunction
			Else
				Return
			End If
		End If
		
		LogMessage("STATUS", "Disconnect called")
		
		' close Astream
		If astream.IsInitialized Then
			astream.Close
			btHC05Connection.Disconnect
		End If
		
		btnConnectHC05.Text = "Connect"
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
			LogMessage("BLUETOOTH", "Searching, please wait...")
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
		LogMessage("BLUETOOTH", "Scan has stopped")
	End If
End Sub
Private Sub btnConnectHM10_Click
	' USE (57600 Baud) from HM10 to PIC
	If btnConnectHM10.Text = "Connect" Then
		If ListView1HM10.SelectedIndex <> - 1 Then
			If btHC05.IsEnabled = False Then
				xui.Msgbox2Async("Bluetooth is disabled. Please turn it on!", "Bluetooth", "Ok", "", "", Null)
				Return
			End If
			
			' Close any open connections
			CloseConnection(False, True, True, True)
			
			BLE_useUUID = ""
			
			' Connect Bluetooth with Address selected from listview
			Dim address As String = ListView1HM10.SelectedItem
			bkHM10Client = btHM10.CreateClient(foundDevices.Get(address))
			LogMessage("STATUS", "Connecting to " & ListView1HM10.SelectedItem & "...")
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
				'-----------------------------------------------------------------
				
				btnConnectHM10.Text = "Disconnect"
				btnStopScanHM10_Click
				LogMessage("STATUS", "Bluetooth Connected! @ " & address)
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
								LogMessage("BLUETOOTH", "Failed to set notify!")
							Else
								LogMessage("BLUETOOTH", "Notify set @ " & BLE_useUUID)
							End If
						End If					
					Next
				Next
				
				If BLE_useUUID = "" Then
					LogMessage("BLUETOOTH", "Characteristic UUID not found!")
				End If
				LogMessage("BLUETOOTH", "MTU Size = " & UniversalMTU)
			Else
				Log(Py.PyLastException)
				LogMessage("STATUS", "Bluetooth Failed! @ " & address)
			End If
		Else
			xui.Msgbox2Async("Please select Bluetooth fom the list!", "Select Bluetooth", "Ok", "", "", Null)
		End If
	' Disconnect
	Else
		' If its flashing inform with msgbox
		If btnFlash.Text = "Stop" Then
			Dim sf3 As Object = xui.Msgbox2Async("Flash in progress! Do you want to stop?", "Flashing", "Yes", "", "No", Null)
			Wait For (sf3) Msgbox_Result(ret2 As Int)
			If ret2 = xui.DialogResponse_Positive Then
				LogMessage("STATUS", "User stop flash!")
				EnableFunction
			Else
				Return
			End If
		End If
		
		LogMessage("STATUS", "Disconnect called")
		bkHM10Client.Disconnect
		btnConnectHM10.Text = "Connect"
	End If

End Sub

Private Sub btnConnectWIFI_Click
	If btnConnectWIFI.Text = "Connect" Then
		
		CloseConnection(True, True, True, False)
		
		Dim c As Socket
		c.Initialize("client")
		c.Connect(txtHostIPWIFI.text, txtPortWIFI.text, 5000)
		LogMessage("STATUS", "Connecting @ " & txtHostIPWIFI.Text & ":" & txtPortWIFI.Text)
		Wait For client_Connected (Successful As Boolean)
		If Successful Then
			WIFIClient = c
			astream.Initialize(WIFIClient.InputStream, WIFIClient.OutputStream, "astream")
			btnConnectWIFI.Text = "Disconnect"
			LogMessage("STATUS", "WIFI connected!")
		Else
			LogMessage("STATUS", "WIFI connection failed!")
		End If
	Else
		LogMessage("STATUS", "Disconnect called")
		If astream.IsInitialized Then
			astream.Close
			If WIFIClient.Connected = True Then 
				WIFIClient.Close
			End If
		End If
		btnConnectWIFI.Text = "Connect"
	End If

End Sub

Private Sub btnOpenUSBTTL_Click
	' Open Port (57600 Baud)
	If btnOpenUSBTTL.Text = "Open Port" Then

		CloseConnection(True, True, False, True)
		
		Try
			Dim serial1 As Serial
			serial1.Initialize("serial1")
			serial1.Open(cmbPortUSBTTL.Value)
			serial1.SetParams(serial1.BAUDRATE_57600, serial1.DATABITS_8, intStopBit, serial1.PARITY_NONE)  ' Set baud=57600, 8 data bits, config value, no parity
			If astream.IsInitialized Then astream.Close
			astream.Initialize(serial1.GetInputStream, serial1.GetOutputStream, "astream")
			serialUSBTTL = serial1
		Catch
			LogMessage("STATUS", "Error opening port" & LastException)
			Return
		End Try
		btnOpenUSBTTL.Text = "Close Port"
		btnRefreshComUSBTTL.Enabled = False
		LogMessage("STATUS", "Serial COM opened")
		
		' Close Port
	Else
		If astream.IsInitialized Then
			astream.Close
			serialUSBTTL.Close
		End If
		btnOpenUSBTTL.Text = "Open Port"
		btnRefreshComUSBTTL.Enabled = True
		LogMessage("STATUS", "Serial COM closed")
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
			xui.Msgbox2Async("Bootloader Uploader Deluxe" & Chr(10) & "Version: " & Version, "About", "Ok", "", "", Null)
	
	
		' Tools
		Case "AT _Mode"
			B4XPages.ShowPage("AT Command Mode")
	End Select

End Sub

Sub CloseConnection(BLE As Boolean, SSP As Boolean, TTLUSB As Boolean, WIFI As Boolean)
	
	If TTLUSB = True Then
		' Close TTL USB Serial
		If btnOpenUSBTTL.Text = "Close Port" Then
			btnOpenUSBTTL_Click
			Sleep(200)
		End If
	End If
	
	If SSP = True Then
		' Close SSP
		If btnConnectHC05.Text = "Disconnect" Then
			btnConnectHC05_Click
			Sleep(200)
		End If
	End If
	
	If BLE = True Then	
		' Close BLE
		If btnConnectHM10.Text = "Disconnect" Then
			btnConnectHM10_Click
			Sleep(200)
		End If
	End If
	
	If WIFI = True Then
		' Close BLE
		If btnConnectWIFI.Text = "Disconnect" Then
			btnConnectWIFI_Click
			Sleep(200)
		End If
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
		firmwareFile = ConvertHexIntelToBinaryRange(filepath, intStartAddrFlash, intEndAddrFlash)
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
		If blnUseDoubleHexAddr = True Then
			startByte = startAddr * 2       ' eg. PIC 16F88, PIC 24F
			endByte = endAddr * 2
		' Non Intel Double Hex Byte Addressed
		Else
			startByte = startAddr           ' eg. PIC 18F27K42
			endByte = endAddr
		End If
        
		' Create binary array relative to startAddr
		Dim firmwareData(intExpectedFirmwareBytes) As Byte
        
		' --- FIRMWARE INITIALIZATION (Don't leave this out!) ---
		If blnUse4Padding = True Then
			'LSB, Mid, MSB, Phantom format for PIC24
			For i = 0 To firmwareData.Length - 1 Step 4
				firmwareData(i) = 0xFF                  ' Low Byte
				firmwareData(i+1) = 0xFF                ' Mid Byte
				firmwareData(i+2) = 0xFF                ' High Byte
				firmwareData(i+3) = intEmptyFlashValue  ' Phantom (Void) Byte
			Next
		Else
			' LSB(0xFF) Then MSB(0x3F or 0xFF) format for 18F and 16F
			For i = 0 To firmwareData.Length - 1 Step 2
				firmwareData(i) = 0xFF					' Low Byte
				firmwareData(i+1) = intEmptyFlashValue	' High Byte
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
			LogMessage("STATUS", "Conversion success.")
		Else
			btnFlash.Enabled = False
			LogMessage("STATUS", "Error: No valid data found above start address.")
		End If
        
		Return firmwareData
        
	Catch
		LogMessage("STATUS", "Error:  " & LastException & " in ConvertHexIntelToBinaryRange")
		Dim EmptyArr() As Byte = Array As Byte()
		Return EmptyArr
	End Try
End Sub

'--------------------------------------------------------
' Start Handshake and Firmware Upload
'--------------------------------------------------------
Private Sub btnFlash_Click
	' Check if configuration .map available
	If cmbPicList.Items.Size = 0 Then
		xui.Msgbox2Async("Configuration is missing!", "Configuration", "Ok", "", "", Null)
	' Validation Checks
	Else If btnConnectHC05.Text = "Connect" And btnOpenUSBTTL.Text = "Open Port" And btnConnectHM10.Text = "Connect" Then
		xui.Msgbox2Async("Please connect or open port!", "Connection Required!", "Ok", "", "", Null)
	' Flashing Logic
	Else If btnFlash.Text = "Flash" Then
		Dim sf2 As Object = xui.Msgbox2Async("If the PIC application is running, it will enter bootloader mode automatically. " & _
  											 "If not, please click OK and then power cycle.", _
                                      		 "Attention!", "Ok", "Cancel", "", Null)
		Wait For (sf2) Msgbox_Result(ret As Int)
		If ret = xui.DialogResponse_Positive Then
			SendHandShakeBytes
		End If
	' Flash Stop Logic
	Else If btnFlash.Text = "Stop" Then
		Dim sf3 As Object = xui.Msgbox2Async("Flash in progress! Do you want to stop?", "Flashing", "Yes", "", "No", Null)
		Wait For (sf3) Msgbox_Result(ret2 As Int)		
		If ret2 = xui.DialogResponse_Positive Then
			LogMessage("STATUS", "User stop flash!")
			EnableFunction
		End If
	End If
End Sub
Sub SendHandShakeBytes
	Dim rs As Object
	Dim blnToggle As Boolean
	Dim b() As Byte = Array As Byte(0x55)
	Dim b2() As Byte = Array As Byte(0xAA)
	
	DisableFunction
		
	Do While True
		' Status boolean
		If GetBooleanStatus = True Then 
			EnableFunction
			Return
		End If
		
		' Exit and Start Config upload
		If blnHandShakeSuccess = True Then
			Sleep(300) ' give firmware time to call ReceiveConfig.
			SendConfigBytes
			Return
		Else
			If blnToggle = False Then
				' BLE
				If btnConnectHM10.Text = "Disconnect" Then
					rs = bkHM10Client.Write(BLE_useUUID, b)
					Wait For (rs) Complete (Result2 As PyWrapper)
				' OTHERS (SSP, WIFI and TTL USB)
				Else
					If astream.IsInitialized Then astream.Write(b)
				End If
				LogMessage("HANDSHAKE", "Sending: 0x55")
			Else
				' BLE
				If btnConnectHM10.Text = "Disconnect" Then
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
		Sleep(intHandShakeDelayMS)		' Recommend 200 ms minimum in .map file
		
		blnToggle = Not(blnToggle)
	Loop
End Sub
Sub SendConfigBytes
	' One time shot. The PIC will be ready to poll!
	
	Dim rs As Object
	Dim byteONE(1), byteTWO(1), byteTHREE(1) As Byte

	' 1. Set the BLE Flag
	If btnConnectHM10.Text = "Disconnect" Then
		byteONE(0) = 0x01	'BLE
	Else
		byteONE(0) = 0x00   'OTHERS
	End If

	' 2. Extract High Byte (Most Significant Byte)
	' Shift right by 8 bits to move the top 8 bits into the bottom 8 bits
	byteTWO(0) = Bit.ShiftRight(Bit.And(BLE_useMTUSize, 0xFF00), 8)

	' 3. Extract Low Byte (Least Significant Byte)
	' Use a mask to keep only the bottom 8 bits
	byteTHREE(0) = Bit.And(BLE_useMTUSize, 0xFF)

	' BLE
	If btnConnectHM10.Text = "Disconnect" Then
		rs = bkHM10Client.Write(BLE_useUUID, byteONE)
		Wait For (rs) Complete (Result2 As PyWrapper)
		Sleep(50)
		rs = bkHM10Client.Write(BLE_useUUID, byteTWO)
		Wait For (rs) Complete (Result2 As PyWrapper)
		Sleep(50)
		rs = bkHM10Client.Write(BLE_useUUID, byteTHREE)
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
		End If
	End If
	LogMessage("CFG BYTES", "Sending: 0x" & Bit.ToHexString(byteONE(0)).ToUpperCase & ", " & "0x" & Bit.ToHexString(byteTWO(0)).ToUpperCase & ", " & "0x" & Bit.ToHexString(byteTHREE(0)).ToUpperCase)
					
	Do While blnConfigOK = False
		If GetBooleanStatus = True Then 
			EnableFunction
			Return
		End If
		Sleep(50)
	Loop
		
End Sub
Sub SendFirmwareBytes
	' Firmware Binary file must include all flash data including empty addresses!
	Dim intBlockSize As Int

	If blnUse4Padding = True Then
		intBlockSize = intInstructionPacket * 4 ' eg. 64 words = 256 intBlockSize 24FJ64GA102
	Else
		intBlockSize = intInstructionPacket * 2 ' eg. 4 words = 8 intBlockSize 16F88
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
		If GetBooleanStatus = True Then 
			EnableFunction
			Return
		End If

		' Wait for ACK or handle timeout
		Do While blnWriteACK = False
			If GetBooleanStatus = True Then 
				EnableFunction
				Return
			End If

			If blnISRTimeOut = True Then
				LogMessage("FIRMWAREUPLOAD", "Timeout detected, retrying at byte #" & i)
				blnISRTimeOut = False
				Continue    ' retry immediately
			End If

			Sleep(0)
		Loop
		
		' Reset ACK for next block
		blnWriteACK = False
		
		'--------------------------------------------------------------------------------
		' Burst = False
		'--------------------------------------------------------------------------------
		If blnUseWriteBurst = False Then
			For x = 0 To intBlockSize - 1
				Dim b(1) As Byte
				b(0) = block(x)
				'------------------------------------------------------------------------
				' BLE (Never configure .map to use this!
				'------------------------------------------------------------------------
				If btnConnectHM10.Text = "Disconnect" Then
					rs = bkHM10Client.Write(BLE_useUUID, b)
					Wait For (rs) Complete (Result2 As PyWrapper)
				'------------------------------------------------------------------------
				' OTHERS (SSP, WIFI and TTL USB)
				'------------------------------------------------------------------------
				Else
					If astream.IsInitialized Then astream.Write(b)
				End If
				
				Sleep(intPacketDelayMS)
				
			Next
		'-----------------------------------------------------------------------------
		' Burst = True (Common)
		'-----------------------------------------------------------------------------
		Else
			'-----------------------------------------------------------------------------
			' BLE
			'-----------------------------------------------------------------------------
			If btnConnectHM10.Text = "Disconnect" Then
				' BLE Flash Write is supported by MTU Size requested
				' Need to test HM-20 supports over 400 mtu size!  
				' HM-10 tested at 20 mtu really sucks!
				Dim bc As ByteConverter
				Dim chunkSize As Int = Max(20, BLE_useMTUSize)

				If intBlockSize <= chunkSize Then
					' Send block at once if it fits within MTU Size limits
					rs = bkHM10Client.Write(BLE_useUUID, block)
					Wait For (rs) Complete (Result2 As PyWrapper)
				Else
					' Fragment the block because its larger than the MTU Size
					For x = 0 To intBlockSize - 1 Step chunkSize
						Dim currentChunkSize As Int = Min(chunkSize, intBlockSize - x)
						Dim tempChunk(currentChunkSize) As Byte
						bc.ArrayCopy(block, x, tempChunk, 0, currentChunkSize)
        
						rs = bkHM10Client.Write(BLE_useUUID, tempChunk)
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
			
			Sleep(intPacketDelayMS)
			
		End If

		' Update progress bar
		prgBar.Progress = Min(1, (i + intBlockSize) / firmwareFile.Length)

		' Move to next block
		i = i + intBlockSize
	Loop

	LogMessage("FIRMWAREUPLOAD", "Firmware upload completed!")
End Sub
Sub GetBooleanStatus As Boolean
	' PIC reported timeout error (Handshake does not have this!)
	If blnTimeoutError = True Then
		Return True
	End If
		
	' Astream error or terminated
	If blnAstreamError = True Then
		Return True
	End If
		
	' Stop or Quit Detected
	If blnAppStopQuit = True Then
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
Sub CompareFirmware() As Boolean
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

'--------------------------------------------------------
' Disable/Enable
'--------------------------------------------------------
Sub DisableFunction
	MenuBar1.Enabled = False
	ListView1HC05.Enabled = False  
	btnSearchHC05.Enabled = False  
	ListView1HM10.Enabled = False
	btnStartScanHM10.Enabled = False
	btnOpenUSBTTL.Enabled = False
	cmbPortUSBTTL.Enabled = False
	btnLoadFile.Enabled = False
	cmbPicList.Enabled = False
	blnAppStopQuit = False
	btnFlash.Text = "Stop"

	blnHandShakeSuccess = False
	blnTimeoutError = False
	blnAstreamError = False
	blnISRTimeOut = False
	blnConfigOK = False
	txtLog.Text = ""
	prgBar.Progress = 0		
	cntVerify = 0
End Sub
Sub EnableFunction
	MenuBar1.Enabled = True
	ListView1HC05.Enabled = True 
	btnSearchHC05.Enabled = True 
	ListView1HM10.Enabled = True
	btnStartScanHM10.Enabled = True
	btnOpenUSBTTL.Enabled = True
	cmbPortUSBTTL.Enabled = True
	btnLoadFile.Enabled = True
	cmbPicList.Enabled = True
	blnAppStopQuit = True
	btnFlash.Text = "Flash"
	
	blnWriteACK = False
	blnStartVerifyRequest = False
	
	
End Sub

'--------------------------------------------------------
' Load all Map files from /configs and return PicName list
'--------------------------------------------------------
Sub LoadAllPicNames() As List
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
						
						intStartAddrFlash = cfg.Get("StartAddrFlash")		' Start Address of Flash. For Intel Hex Conversion
						intEndAddrFlash = cfg.Get("EndAddrFlash")			' End Address of Flash
						intEmptyFlashValue = cfg.Get("EmptyFlashValue")		' Empty Flash Value (eg. 3FFF = 3F)
						intInstructionPacket = cfg.Get("InstructionPacket")	' Total Instruction Words Per Packet for Write Block
						intPacketDelayMS = cfg.Get("PacketDelayMS")			' Write Block Packet Delay
						intHandShakeDelayMS = cfg.Get("HandShakeDelayMS")	' Handshake Delay (0x55 and 0xAA in between delay)
						intStopBit = cfg.Get("StopBit")						' 1 or 2 stop bits, older PIC need 2 so it buys time processing other instructions
						strNotes = cfg.Get("Notes")							' Special Notes
						intExpectedFirmwareBytes = cfg.Get("ExpectedBytes") ' Total Bytes need flash and erase
						blnUseWriteBurst = cfg.Get("UseWriteBurst")			' No delays in between bytes if True!
						blnUseDoubleHexAddr = cfg.Get("UseDoubleHexAddr") 	' For Intel Hex Conversion
						blnUse4Padding = cfg.Get("Use4Padding")				' For Intel Hex conversion
							
						' Set Proper Arrays to FirmwareVerfiy()
						firmwareVerify = Array As Byte()
						Dim temp(intExpectedFirmwareBytes) As Byte
						firmwareVerify = temp
		
						' Make sure reload the Intel Hex file to new firmwareFile() array
						If strLastFilePath <> "" Then
							firmwareFile = ConvertHexIntelToBinaryRange(strLastFilePath, intStartAddrFlash, intEndAddrFlash)
						End If
						
						' If port already open update stop bit parameter!
						'If btnConnect.Text = "Disconnect" Then
						'	serial1.SetParams(serial1.BAUDRATE_57600, serial1.DATABITS_8, intStopBit, serial1.PARITY_NONE)  ' Set baud=57600, 8 data bits, config value, no parity
						'End If
						
						LogMessage("", "---------------------------------------------------------")
						LogMessage("", "CONFIGURATION FOR " & CheckName)
						LogMessage("", "---------------------------------------------------------")
						LogMessage(":::", "Notes: " & strNotes)
						LogMessage(":::", "Start Address = 0x" & Bit.ToHexString(intStartAddrFlash).ToUpperCase)
						LogMessage(":::", "End Address = 0x" & Bit.ToHexString(intEndAddrFlash).ToUpperCase)
						LogMessage(":::", "Empty Flash Value = 0x" & Bit.ToHexString(intEmptyFlashValue).ToUpperCase)
						LogMessage(":::", "HandShake Delay = " & intHandShakeDelayMS & " ms")
						LogMessage(":::", "Packet Delay = " & intPacketDelayMS & " ms")
						LogMessage(":::", "Stop Bit = " & intStopBit)
						If blnUse4Padding = True Then
							LogMessage(":::", "Instruction Write Size = " & (intInstructionPacket * 4) & " bytes w/padding (" &  intInstructionPacket & " instructions)" )
						Else
							LogMessage(":::", "Instruction Write Size = " & (intInstructionPacket * 2) & " bytes w/padding (" &  intInstructionPacket & " instructions)" )
						End If
						LogMessage(":::", "Expected Firmware Size = " & NumberFormat2(intExpectedFirmwareBytes, 1, 0, 0, True) & " bytes")
						LogMessage(":::", "Use Write Burst = " & blnUseWriteBurst)
						LogMessage(":::", "Use Double Hex Addressed = " & blnUseDoubleHexAddr)
						LogMessage(":::", "Use 4 Padding = " & blnUse4Padding)
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

