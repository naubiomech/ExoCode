const char* UARTUUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
const char* txUUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
const char* rxUUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";
const int BUFFER_SIZE = 512;
bool BUFFERS_FIXED_LENGTH = false;
//Add Nordics UART Service
BLEService UARTService(UARTUUID);   //Instantiate UART service
BLECharacteristic TXChar(rxUUID, BLERead | BLENotify | BLEBroadcast,             BUFFER_SIZE, BUFFERS_FIXED_LENGTH); //TX characteristic of service
BLECharacteristic RXChar(txUUID, BLEWriteWithoutResponse | BLEWrite | BLENotify, BUFFER_SIZE, BUFFERS_FIXED_LENGTH); //RX characteristic of service

void setupBLE()
{
  if (BLE.begin())
  {
    //Advertised name and service
    BLE.setLocalName("EXOBLE_Dur2"); //SET DEVICE ID HERE "EXOBLE_#"
    BLE.setDeviceName("EXOBLE_Dur2"); //SET DEVICE ID HERE "EXOBLE_#"
    BLE.setAdvertisedService(UARTService);
    //Add chars to service
    UARTService.addCharacteristic(TXChar);
    UARTService.addCharacteristic(RXChar);
    //Add service to BLE
    BLE.addService(UARTService);

    //Add callbacks
    BLE.setEventHandler(BLEConnected,    onBLEConnected);
    BLE.setEventHandler(BLEDisconnected, onBLEDisconnected);
    RXChar.setEventHandler(BLEWritten,   onRxCharValueUpdate);
    BLE.advertise();
    //config_ble_regs();
    digitalWrite(GREEN,HIGH);
  }
  else {
    Serial.println("Error Setting up BLE");
    digitalWrite(GREEN, LOW);
    digitalWrite(BLUE, LOW);
    digitalWrite(RED, HIGH);
  }
}

void onRxCharValueUpdate(BLEDevice central, BLECharacteristic characteristic) {
  static bool collecting = false;
  char data[32] = {0};
  //callback_thread.set_priority(osPriorityNormal);
  int val_len = RXChar.valueLength();
  RXChar.readValue(data, val_len);
  if (data[0] == '!') {
    //Serial.println("Got matlab message");
    if (!handle_matlab_message(data, val_len)) {
      receive_and_transmit();
      return;
    }
  } else {
    //Serial.println("Got mobile message");
    if (!handle_mobile_message(data, val_len)) {
      receive_and_transmit();
      return;
    }
  }
  //callback_thread.set_priority(osPriorityAboveNormal);
}//End onRxCharValueUpdate

void onBLEConnected(BLEDevice central)
{
  digitalWrite(GREEN,HIGH);
  digitalWrite(BLUE, HIGH);
  BLE.stopAdvertise();
}

void onBLEDisconnected(BLEDevice central)
{
  digitalWrite(GREEN,HIGH);
  digitalWrite(BLUE, LOW);
  BLE.advertise();
}
