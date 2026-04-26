B4J=true
Group=Default Group
ModulesStructureVersion=1
Type=Class
Version=10.5
@EndOfDesignText@
Sub Class_Globals
	Private Root As B4XView 'ignore
	Private xui As XUI 'ignore
		
	' Pane1
	Private Pane1 As Pane
	Private cmbBaud As ComboBox
	Private txtDeviceName As TextField
	Private btnSetBaud As Button
	Private btnSetDeviceName As Button
	Private btnQueryBaud As Button
	Private btnQueryName As Button
	Private optHC05 As RadioButton
	Private optHM10 As RadioButton
	Private optHM20 As RadioButton
	
	' Pane2
	Private Pane2 As Pane
	Private btnOpen As Button
	Private cmbBaudMode As ComboBox
	Private cmbPort As ComboBox

	' Pane3
	Private Pane3 As Pane
	Private txtCommand As TextField
	Private btnSend As Button
	Private txtLog As TextArea
	Private cmbListATCommand As ComboBox
	
	Private serialATMode As Serial							' UART Serial COM
	Private astream As AsyncStreams							' Read/Write Stream



End Sub

Public Sub Initialize As Object
	serialATMode.Initialize("serial")
	Return Me
End Sub

Private Sub B4XPage_Created (Root1 As B4XView)
	Root = Root1
	Root.LoadLayout("ATMode")
	
	Dim lstBaud As List
	lstBaud.Initialize
	lstBaud.Add("1200")
	lstBaud.Add("2400")
	lstBaud.Add("4800")
	lstBaud.Add("9600")
	lstBaud.Add("19200")
	lstBaud.Add("38400")
	lstBaud.Add("57600")
	lstBaud.Add("115200")
	
	' Baud for Uart to PIC
	cmbBaud.Items.AddAll(lstBaud)

	' AT Mode Baud
	cmbBaudMode.Items.AddAll(lstBaud)
			
	' Load all available COM Ports
	cmbPort.Items.AddAll(serialATMode.ListPorts)
	
	' Set prompt text for combo box
	Dim jo As JavaObject = cmbBaud
	jo.RunMethod("setPromptText", Array("Baud"))
	Dim jo As JavaObject = cmbBaudMode
	jo.RunMethod("setPromptText", Array("AT Mode Baud"))
	Dim jo As JavaObject = cmbPort
	jo.RunMethod("setPromptText", Array("Choose Port"))
	Dim jo As JavaObject = cmbListATCommand
	jo.RunMethod("setPromptText", Array("List Commands"))
	
	optHC05.Selected = True
	cmbBaudMode.RequestFocus
	

End Sub
Private Sub B4XPage_CloseRequest As ResumableSub
	' Close them when exiting!
	If astream.IsInitialized Then
		astream.Close
		serialATMode.Close
	End If
	
	If btnOpen.Text = "Close" Then
		btnOpen_Click
	End If

	Return True
End Sub

'--------------------------------------------------------
' Pane1
'--------------------------------------------------------
Private Sub btnSetDeviceName_Click
	If txtDeviceName.Text.Length > 0 Then
		If optHC05.Selected = True Then
			txtCommand.Text = "AT+NAME=" & txtDeviceName.Text
		Else
			txtCommand.Text = "AT+NAME" & txtDeviceName.Text
		End If
		btnSend_Click
	End If
End Sub
Private Sub btnQueryName_Click
	If optHC05.Selected = True Then
		txtCommand.Text = "AT+NAME?"
	Else
		txtCommand.Text = "AT+NAME?"
	End If
	btnSend_Click
End Sub

Private Sub btnSetBaud_Click
	If cmbBaud.SelectedIndex <> -1 Then
		If optHC05.Selected = True Then
			txtCommand.Text = "AT+UART=" & cmbBaud.Items.Get(cmbBaud.SelectedIndex) & ",0,0"
		Else if optHM10.Selected = True Then
			Dim intSet As Int
			Dim Value As String = cmbBaud.Items.Get(cmbBaud.SelectedIndex)
			Select Case Value
				Case "1200":	intSet = 7
				Case "2400":	intSet = 6
				Case "4800":	intSet = 5
				Case "9600":	intSet = 0
				Case "19200":	intSet = 1
				Case "38400":	intSet = 2
				Case "57600":	intSet = 3
				Case "115200":  intSet = 4
			End Select
			txtCommand.Text = "AT+BAUD" & intSet
		Else 
			Dim intSet As Int
			Dim Value As String = cmbBaud.Items.Get(cmbBaud.SelectedIndex)
			Select Case Value
				Case "1200":	intSet = 0
				Case "2400":	intSet = 1
				Case "4800":	intSet = 2
				Case "9600":	intSet = 3
				Case "19200":	intSet = 4
				Case "38400":	intSet = 5
				Case "57600":	intSet = 6
				Case "115200":  intSet = 7
			End Select
			txtCommand.Text = "AT+BAUD" & intSet
		End If
		btnSend_Click
	End If
End Sub
Private Sub btnQueryBaud_Click
	If optHC05.Selected = True Then
		txtCommand.Text = "AT+UART?"
	Else
		txtCommand.Text = "AT+BAUD?"
	End If
	btnSend_Click
End Sub

Private Sub optHM10_SelectedChange(Selected As Boolean)
	cmbListATCommand.Items.Clear
	cmbListATCommand.Items.Add("AT")
	cmbListATCommand.Items.Add("AT+ADDR?")
	cmbListATCommand.Items.Add("AT+ADVI?")
	cmbListATCommand.Items.Add("AT+ADTY?")
	cmbListATCommand.Items.Add("AT+ANCS?")
	cmbListATCommand.Items.Add("AT+ALLO?")
	cmbListATCommand.Items.Add("AT+BEFC?")
	cmbListATCommand.Items.Add("AT+AFTC?")
	cmbListATCommand.Items.Add("AT+BATC?")
	cmbListATCommand.Items.Add("AT+BATT?")
	cmbListATCommand.Items.Add("AT+BIT7?")
	cmbListATCommand.Items.Add("AT+COMI?")
	cmbListATCommand.Items.Add("AT+COMA?")
	cmbListATCommand.Items.Add("AT+COLA?")
	cmbListATCommand.Items.Add("AT+COSU?")
	cmbListATCommand.Items.Add("AT+COUP?")
	cmbListATCommand.Items.Add("AT+CHAR?")
	cmbListATCommand.Items.Add("AT+CLEAR")
	cmbListATCommand.Items.Add("AT+CONNL")
	cmbListATCommand.Items.Add("AT+COL??")
	cmbListATCommand.Items.Add("AT+CYC??")
	cmbListATCommand.Items.Add("AT+COMP?")
	cmbListATCommand.Items.Add("AT+DISC?")
	cmbListATCommand.Items.Add("AT+DISI?")
	cmbListATCommand.Items.Add("AT+ERASE")
	cmbListATCommand.Items.Add("AT+FILT?")
	cmbListATCommand.Items.Add("AT+FLOW?")
	cmbListATCommand.Items.Add("AT+GAIN?")
	cmbListATCommand.Items.Add("AT+HELP?")
	cmbListATCommand.Items.Add("AT+IBEA?")
	cmbListATCommand.Items.Add("AT+IBE0?")
	cmbListATCommand.Items.Add("AT+IBE1?")
	cmbListATCommand.Items.Add("AT+IBE2?")
	cmbListATCommand.Items.Add("AT+IBE3?")
	cmbListATCommand.Items.Add("AT+MARJ?")
	cmbListATCommand.Items.Add("AT+MINO?")
	cmbListATCommand.Items.Add("AT+MEAS?")
	cmbListATCommand.Items.Add("AT+MODE?")
	cmbListATCommand.Items.Add("AT+NOTI?")
	cmbListATCommand.Items.Add("AT+NOTP?")
	cmbListATCommand.Items.Add("AT+PCTL?")
	cmbListATCommand.Items.Add("AT+PASS?")
	cmbListATCommand.Items.Add("AT+POWE?")
	cmbListATCommand.Items.Add("AT+PWRM?")
	cmbListATCommand.Items.Add("AT+RELI?")
	cmbListATCommand.Items.Add("AT+RENEW")
	cmbListATCommand.Items.Add("AT+RESET")
	cmbListATCommand.Items.Add("AT+ROLE?")
	cmbListATCommand.Items.Add("AT+RSSI?")
	cmbListATCommand.Items.Add("AT+RADD?")
	cmbListATCommand.Items.Add("AT+RAT??")
	cmbListATCommand.Items.Add("AT+STOP?")
	cmbListATCommand.Items.Add("AT+START")
	cmbListATCommand.Items.Add("AT+SLEEP")
	cmbListATCommand.Items.Add("AT+SAVE?")
	cmbListATCommand.Items.Add("AT+SCAN?")
	cmbListATCommand.Items.Add("AT+SENS?")
	cmbListATCommand.Items.Add("AT+SHOW?")
	cmbListATCommand.Items.Add("AT+TEHU?")
	cmbListATCommand.Items.Add("AT+TEMP?")
	cmbListATCommand.Items.Add("AT+TCON?")
	cmbListATCommand.Items.Add("AT+TYPE?")
	cmbListATCommand.Items.Add("AT+UUID?")
	cmbListATCommand.Items.Add("AT+VERR?")
	cmbListATCommand.SelectedIndex = 0
End Sub

Private Sub optHC05_SelectedChange(Selected As Boolean)
	cmbListATCommand.Items.Clear
	cmbListATCommand.Items.Add("AT")
	cmbListATCommand.Items.Add("AT+RESET")
	cmbListATCommand.Items.Add("AT+VERSION?")
	cmbListATCommand.Items.Add("AT+ORGL")
	cmbListATCommand.Items.Add("AT+ADDR?")
	cmbListATCommand.Items.Add("AT+ROLE?")
	cmbListATCommand.Items.Add("AT+CLASS?")
	cmbListATCommand.Items.Add("AT+IAC?")
	cmbListATCommand.Items.Add("AT+INQM?")
	cmbListATCommand.Items.Add("AT+PSWD?")
	cmbListATCommand.Items.Add("AT+CMODE?")
	cmbListATCommand.Items.Add("AT+BIND?")
	cmbListATCommand.Items.Add("AT+MPIO?")
	cmbListATCommand.Items.Add("AT+IPSCAN?")
	cmbListATCommand.Items.Add("AT+SENM?")
	cmbListATCommand.Items.Add("AT+ADCN?")
	cmbListATCommand.Items.Add("AT+MRAD?")
	cmbListATCommand.Items.Add("AT+STATE?")
	cmbListATCommand.Items.Add("AT+INIT")
	cmbListATCommand.Items.Add("AT+INQ")
	cmbListATCommand.Items.Add("AT+DISC")	
	cmbListATCommand.SelectedIndex = 0
End Sub

Private Sub optHM20_SelectedChange(Selected As Boolean)
	' Same as HM-10 but Setting Baud is different
	cmbListATCommand.Items.Clear
	cmbListATCommand.Items.Add("AT")
	cmbListATCommand.Items.Add("AT+ADDR?")
	cmbListATCommand.Items.Add("AT+ADVI?")
	cmbListATCommand.Items.Add("AT+ADTY?")
	cmbListATCommand.Items.Add("AT+ANCS?")
	cmbListATCommand.Items.Add("AT+ALLO?")
	cmbListATCommand.Items.Add("AT+BEFC?")
	cmbListATCommand.Items.Add("AT+AFTC?")
	cmbListATCommand.Items.Add("AT+BATC?")
	cmbListATCommand.Items.Add("AT+BATT?")
	cmbListATCommand.Items.Add("AT+BIT7?")
	cmbListATCommand.Items.Add("AT+COMI?")
	cmbListATCommand.Items.Add("AT+COMA?")
	cmbListATCommand.Items.Add("AT+COLA?")
	cmbListATCommand.Items.Add("AT+COSU?")
	cmbListATCommand.Items.Add("AT+COUP?")
	cmbListATCommand.Items.Add("AT+CHAR?")
	cmbListATCommand.Items.Add("AT+CLEAR")
	cmbListATCommand.Items.Add("AT+CONNL")
	cmbListATCommand.Items.Add("AT+COL??")
	cmbListATCommand.Items.Add("AT+CYC??")
	cmbListATCommand.Items.Add("AT+COMP?")
	cmbListATCommand.Items.Add("AT+DISC?")
	cmbListATCommand.Items.Add("AT+DISI?")
	cmbListATCommand.Items.Add("AT+ERASE")
	cmbListATCommand.Items.Add("AT+FILT?")
	cmbListATCommand.Items.Add("AT+FLOW?")
	cmbListATCommand.Items.Add("AT+GAIN?")
	cmbListATCommand.Items.Add("AT+HELP?")
	cmbListATCommand.Items.Add("AT+IBEA?")
	cmbListATCommand.Items.Add("AT+IBE0?")
	cmbListATCommand.Items.Add("AT+IBE1?")
	cmbListATCommand.Items.Add("AT+IBE2?")
	cmbListATCommand.Items.Add("AT+IBE3?")
	cmbListATCommand.Items.Add("AT+MARJ?")
	cmbListATCommand.Items.Add("AT+MINO?")
	cmbListATCommand.Items.Add("AT+MEAS?")
	cmbListATCommand.Items.Add("AT+MODE?")
	cmbListATCommand.Items.Add("AT+NOTI?")
	cmbListATCommand.Items.Add("AT+NOTP?")
	cmbListATCommand.Items.Add("AT+PCTL?")
	cmbListATCommand.Items.Add("AT+PASS?")
	cmbListATCommand.Items.Add("AT+POWE?")
	cmbListATCommand.Items.Add("AT+PWRM?")
	cmbListATCommand.Items.Add("AT+RELI?")
	cmbListATCommand.Items.Add("AT+RENEW")
	cmbListATCommand.Items.Add("AT+RESET")
	cmbListATCommand.Items.Add("AT+ROLE?")
	cmbListATCommand.Items.Add("AT+RSSI?")
	cmbListATCommand.Items.Add("AT+RADD?")
	cmbListATCommand.Items.Add("AT+RAT??")
	cmbListATCommand.Items.Add("AT+STOP?")
	cmbListATCommand.Items.Add("AT+START")
	cmbListATCommand.Items.Add("AT+SLEEP")
	cmbListATCommand.Items.Add("AT+SAVE?")
	cmbListATCommand.Items.Add("AT+SCAN?")
	cmbListATCommand.Items.Add("AT+SENS?")
	cmbListATCommand.Items.Add("AT+SHOW?")
	cmbListATCommand.Items.Add("AT+TEHU?")
	cmbListATCommand.Items.Add("AT+TEMP?")
	cmbListATCommand.Items.Add("AT+TCON?")
	cmbListATCommand.Items.Add("AT+TYPE?")
	cmbListATCommand.Items.Add("AT+UUID?")
	cmbListATCommand.Items.Add("AT+VERR?")
	cmbListATCommand.SelectedIndex = 0
End Sub

'--------------------------------------------------------
' Pane2
'--------------------------------------------------------
Private Sub btnOpen_Click	
	' Open Port
	If btnOpen.Text = "Open" Then
		If cmbBaudMode.SelectedIndex = -1 Then
			xui.Msgbox2Async("Select Baud to open from list", "Baud Required", "Ok", "", "", Null)
			Return
		End If
		
		Dim serial1 As Serial
		serial1.Initialize("serial1")
		
		Dim UseBaud As Int
		Dim getBaudValue As String = cmbBaudMode.Items.Get(cmbBaudMode.SelectedIndex)
		Select Case getBaudValue
			Case "1200": 	UseBaud = serial1.BAUDRATE_1200
			Case "2400": 	UseBaud = serial1.BAUDRATE_2400
			Case "4800": 	UseBaud = serial1.BAUDRATE_4800
			Case "9600": 	UseBaud = serial1.BAUDRATE_9600
			Case "19200": 	UseBaud = serial1.BAUDRATE_19200
			Case "38400": 	UseBaud = serial1.BAUDRATE_38400
			Case "57600": 	UseBaud = serial1.BAUDRATE_57600
			Case "115200": 	UseBaud = serial1.BAUDRATE_115200
			Case Else:  	UseBaud = serial1.BAUDRATE_9600
		End Select
		
		LogMessage("Status", "AT Mode Baud = " & getBaudValue )
		
		Try
			serial1.Open(cmbPort.Value)
			serial1.SetParams(UseBaud, serial1.DATABITS_8, serial1.STOPBITS_1, serial1.PARITY_NONE) 
			If astream.IsInitialized Then astream.Close
			astream.Initialize(serial1.GetInputStream, serial1.GetOutputStream, "astream")
			serialATMode = serial1
		Catch
			LogMessage("Status", "Error opening port" & LastException)
			Return
		End Try

		cmbBaud.Enabled = True
		btnSetBaud.Enabled = True
		btnSetDeviceName.Enabled = True
		btnQueryBaud.Enabled = True
		btnQueryName.Enabled = True
		cmbBaudMode.Enabled = False
		cmbPort.Enabled = False
		btnSend.Enabled = True
		cmbListATCommand.Enabled = True
		btnOpen.Text = "Close"
		
		LogMessage("Status", "Serial COM opened")
		
	' Close Port
	Else
		If astream.IsInitialized Then
			astream.Close
			serialATMode.Close
		End If
		
		cmbBaud.Enabled = False
		btnSetBaud.Enabled = False
		btnSetDeviceName.Enabled = False
		btnQueryBaud.Enabled = False
		btnQueryName.Enabled = False
		cmbBaudMode.Enabled = True
		cmbPort.Enabled = True
		btnSend.Enabled = False
		cmbListATCommand.Enabled = False
		btnOpen.Text = "Open"

		LogMessage("Status", "Serial COM closed")
	End If
End Sub

Private Sub cmbPort_SelectedIndexChanged(Index As Int, Value As Object)
	btnOpen.Enabled = Index > -1 'enable the button if there is a selected item
End Sub

'--------------------------------------------------------
' Pane3
'--------------------------------------------------------
Private Sub btnSend_Click
	Dim DataOut As String 
	
	If optHC05.Selected = True Then
		DataOut = txtCommand.Text & Chr(13) & Chr(10)	' With CRLF
	Else
		txtLog.Text = txtLog.Text & Chr(10) & Chr(13) & Chr(10) & Chr(13)
		txtLog.SetSelection(txtLog.Text.Length, txtLog.Text.Length)
		DataOut = txtCommand.Text						' Without CRLF
	End If
	astream.Write(DataOut.GetBytes("utf8"))
	LogMessage("Send", txtCommand.Text)
End Sub

Private Sub cmbListATCommand_SelectedIndexChanged(Index As Int, Value As Object)
	If Index <> -1 Then	txtCommand.Text = Value
End Sub

'--------------------------------------------------------
' Astream Functions
'--------------------------------------------------------
Sub astream_NewData (Buffer() As Byte)
	Dim dataIn As String = BytesToString(Buffer, 0, Buffer.Length, "UTF8")
	txtLog.Text = txtLog.text & dataIn
	txtLog.SetSelection(txtLog.Text.Length, txtLog.Text.Length)
End Sub
Sub astream_Error
	LogMessage("Status", "Error: " & LastException)
	astream_Terminated
End Sub
Sub astream_Terminated
	If astream.IsInitialized Then
		astream.Close
	End If
	LogMessage("Status", "Connection is terminated!")
End Sub


Sub LogMessage(From As String, Msg As String)
	txtLog.Text = txtLog.Text & From & ": " & Msg & CRLF
	txtLog.SetSelection(txtLog.Text.Length, txtLog.Text.Length)
End Sub



