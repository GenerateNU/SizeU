/*********************************************************************
Generate Product Develop Studio
Project: SizeU
Spring 2017
Christopher Tannock - Product Engineer
**********************************************************************
Header File Info
*********************************************************************/
#include <SPI.h>
#include "Adafruit_BLE_UART.h"

// Connect CLK/MISO/MOSI to hardware SPI
// e.g. On UNO & compatible: CLK = 13, MISO = 12, MOSI = 11
#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 2     // This should be an interrupt pin, on Uno thats #2 or #3
#define ADAFRUITBLE_RST 9
#define BLUE_LED 8
#define GREEN_LED 7
#define RED_LED 6
#define INTERNAL_LED_PIN 13
#define BUTTON_PIN 4


Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);
/**************************************************************************/
/*!
    Configure the Arduino and start advertising with the radio
*/
/**************************************************************************/

// variables will change.
int buttonState = 0;
int lastButtonState = 0;

void setup(void)
{ 
  configureLights();
  pinMode(BUTTON_PIN, INPUT);
  pinMode(INTERNAL_LED_PIN, OUTPUT);
  
  Serial.begin(9600);
  while(!Serial); // Leonardo/Micro should wait for serial init
  Serial.println(F("Generate Hardware Prototype Demo"));

  BTLEserial.setDeviceName("SizeU");

  BTLEserial.begin();
}

/**************************************************************************/
/*!
    Constantly checks for new events on the nRF8001
*/
/**************************************************************************/
aci_evt_opcode_t laststatus = ACI_EVT_DISCONNECTED;

void loop()
{ 
  while(1){
    successfulSend();
  }

  
  // Tell the nRF8001 to do whatever it should be working on.
  BTLEserial.pollACI();
  
  // Ask what is our current status
  aci_evt_opcode_t status = BTLEserial.getState();



  if (status != laststatus) {
    if (status == ACI_EVT_DEVICE_STARTED) {
        Serial.println(F("* Advertising started"));
    }
    if (status == ACI_EVT_CONNECTED) {
        Serial.println(F("* Connected!"));
    }
    if (status == ACI_EVT_DISCONNECTED) {
        Serial.println(F("* Disconnected or advertising timed out"));
    }
    laststatus = status;
  }

  if (status == ACI_EVT_CONNECTED) {
    bluetoothConnected();
    
    // Lets see if there's any data for us!
    if (BTLEserial.available()) {
      Serial.print("* "); Serial.print(BTLEserial.available()); Serial.println(F(" bytes available from BTLE"));
    }
    // OK while we still have something to read, get a character and print it out
    while (BTLEserial.available()) {
      char c = BTLEserial.read();
      Serial.print(c);
    }

    // We need to convert the line to bytes
    buttonState = digitalRead(BUTTON_PIN);
    String s = "Neck: 16 Chest: 39 Sleeve: 33.5 Waist: 33 Hip: 39 Inseam: 32";
    uint8_t sendbuffer[70];
    s.getBytes(sendbuffer, 70);
    char sendbuffersize = min(70, s.length());
    
    if (buttonState != lastButtonState) {
      if (buttonState == HIGH) {
        BTLEserial.write(sendbuffer, sendbuffersize);
      }
      delay(50);
    }
    lastButtonState = buttonState;
 

    /*
    // Next up, see if we have any data to get from the Serial console
    if (Serial.available()) {
      // Read a line from Serial
      Serial.setTimeout(100); // 100 millisecond timeout
      String s = Serial.readString();

      // We need to convert the line to bytes, no more than 20 at this time
      uint8_t sendbuffer[20];
      s.getBytes(sendbuffer, 20);
      char sendbuffersize = min(20, s.length());

      Serial.print(F("\n* Sending -> \"")); Serial.print((char *)sendbuffer); Serial.println("\"");

      // write the data
      BTLEserial.write(sendbuffer, sendbuffersize);
    }
    */
  }

  if (status == ACI_EVT_DEVICE_STARTED) {
    bluetoothAdvertising();
  }

  if (status == ACI_EVT_DISCONNECTED) {

  }
  
}
/*********************************************************************
Bluetooth nRF8001 Functions
*********************************************************************/
aci_evt_opcode_t getConnectionState(void)
{
  aci_evt_opcode_t status = BTLEserial.getState();
}
/*********************************************************************
RGB LED Functions
*********************************************************************/
void configureLights(void)
{
  pinMode(RED_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
}

void lightsOff(void)
{
  digitalWrite(RED_LED, LOW);
  digitalWrite(BLUE_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
}

void bluetoothConnected(void)
{
  digitalWrite(BLUE_LED, HIGH);
}

void bluetoothAdvertising(void)
{
  digitalWrite(BLUE_LED, HIGH);
  delay(1000);
  digitalWrite(BLUE_LED, LOW);
  delay(1000);
}

void successfulSend(void)
{
  lightsOff();
  digitalWrite(GREEN_LED, HIGH);
  delay(200);
  digitalWrite(GREEN_LED, LOW);
  delay(200);
  digitalWrite(GREEN_LED, HIGH);
  delay(200);
  digitalWrite(GREEN_LED, LOW);
}

void failedSend(void)
{
  lightsOff();
  digitalWrite(RED_LED, HIGH);
  delay(200);
  digitalWrite(RED_LED, LOW);
  delay(200);
  digitalWrite(RED_LED, HIGH);
  delay(200);
  digitalWrite(RED_LED, LOW);
}






