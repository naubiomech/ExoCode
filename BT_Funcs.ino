const char* UARTUUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
const char* txUUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
const char* rxUUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";
const int BUFFER_SIZE = 256;
bool BUFFERS_FIXED_LENGTH = false;
//Add Nordics UART Service
BLEService UARTService(UARTUUID);   //Instantiate UART service
BLECharacteristic TXChar(rxUUID, BLERead | BLENotify | BLEBroadcast,             BUFFER_SIZE, BUFFERS_FIXED_LENGTH); //TX characteristic of service
BLECharacteristic RXChar(txUUID, BLEWriteWithoutResponse | BLEWrite | BLENotify, BUFFER_SIZE, BUFFERS_FIXED_LENGTH); //RX characteristic of service

//Used for receiving multi byte commands
bool collecting = false;
int bytesReceived = 0;
int bytesExpected = 0;

void setupBLE()
{
  if (BLE.begin())
  {
    //Advertised name and service
    BLE.setLocalName("EXOBLE");
    BLE.setDeviceName("EXOBLE");
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
  }
  else
  {
    if (DEBUG) {
      Serial.println("Problem Starting BLE!");
    }
  }
}

void onRxCharValueUpdate(BLEDevice central, BLECharacteristic characteristic)
{
  byte tmp[32];
  int dataLength = RXChar.readValue(tmp, 32);
  int transLength = RXChar.valueLength();
  int valLength = 0;
  switch (dataLength)
  {
    case sizeof(char):
      cmd_from_Gui = tmp[0]; //Chars are only used for commands
      collecting = map_expected_bytes();
      if (collecting)
      {
        if (DEBUG) {
          Serial.println("Start Collecting");
        }
      }
      else
      {
        if (DEBUG) {
          Serial.println("Send to receive_and_transmit()");
        }
        receive_and_transmit();
        return;
      }
      break;
    case sizeof(double):
    case sizeof(int):
      if (collecting)
      {
        valLength = ((dataLength == sizeof(double)) ? sizeof(double) : sizeof(int));  //Only working with doubles and ints. '?' is the "ternary operator"
        for (int i = 0; i < valLength; ++i)
        {
          holdon[bytesReceived + i] = tmp[i];
        }
        bytesReceived += valLength;
      }
      else
      {
        if (DEBUG) {
          Serial.println("Got int || double when not collecting!");
        }
      }
      break;
    default:
      if (DEBUG) {
        Serial.println("Not char, int, or double!");
      }
  }//End switch
  if (bytesReceived == bytesExpected)
  {
    if (DEBUG) {
      Serial.println("Done Collecting");
    }
    bytesReceived = 0;
    collecting = false;
    receive_and_transmit();
  }
}//End onRxCharValueUpdate

void onBLEConnected(BLEDevice central)
{
  if (DEBUG)
  {
    Serial.print("Connected event, central: ");
    Serial.println(central.address());
  }
  digitalWrite(BLUE, LOW);
  BLE.stopAdvertise();
}

void onBLEDisconnected(BLEDevice central)
{
  if (DEBUG)
  {
    Serial.print("Disconnected event, central: ");
    Serial.println(central.address());
  }
  digitalWrite(BLUE, HIGH);
  BLE.advertise();
}
