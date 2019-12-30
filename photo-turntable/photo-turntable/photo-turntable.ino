// Include Libraries
#include "Arduino.h"
#include "LiquidCrystal_PCF8574.h"
#include "IRremote.h"
#include "Encoder.h"
#include "Button.h"
#include "math.h"
#include <Wire.h>

// Pin Definitions
#define RotaryEncoder_PIN_D  4
#define RotaryEncoder_PIN_CLK 2
#define RotaryEncoder_PIN_S1  5
#define STEPPER_PIN_STEP  6
#define STEPPER_PIN_DIR 7

#define SLOW 5000
#define FAST 500

// set default parameters
int speed = 100;
int angles = 8;
const int fullRotation = 18300;

// Define LCD characteristics
#define LCD_ROWS 2
#define LCD_COLUMNS 16
#define SCROLL_DELAY 150
#define ZURUECKLIGHT 255
long rotaryEncDOldPosition  = 0;

// object initialization
Encoder rotaryEncD(RotaryEncoder_PIN_D, RotaryEncoder_PIN_CLK);
Button rotaryEncDButton(RotaryEncoder_PIN_S1);
LiquidCrystal_PCF8574 lcdI2C;
IRsend ir_led;

//IR Receiver
#define RECV_PIN 8
IRrecv irrecv(RECV_PIN);
decode_results results;

enum menuState {START, VIDEO, FOTOS, VIDSTART, FOTOSTART, GESCHWINDIGKEIT, ANZAHL, VIDZURUECK, STILZURUECK, CHANGEGESCHWINDIGKEIT, CHANGEANZAHL} state = FOTOS, oldState = VIDEO;

// Global variables and defines
// There are several different versions of the LCD I2C adapter, each might have a different address.
// Try the given addresses by Un/commenting the following rows until LCD works follow the serial monitor prints.
// To find your LCD address go to: http://playground.arduino.cc/Main/I2cScanner and run example.
//#define LCD_ADDRESS 0x3F
//#define LCD_ADDRESS 0x27
//aka #define LCD_ADDRESS 39
//#define LCD_ADDRESS 0x23

// Setup the essentials for your circuit to work. It runs first every time your circuit is powered with electricity.
void setup()
{
  Wire.begin();
  // Setup Serial which is useful for debugging
  // Use the Serial Monitor to view printed messages
  Serial.begin(9600);
  while (!Serial);             // Leonardo: wait for serial monitor
  Serial.println("\nI2C Scanner");
  byte error, address;
  int nDevices;
  int addessI2C;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address ");
      if (address < 16)
        Serial.print("0");
      Serial.print(address);
      Serial.println("!");
      addessI2C = address;
      Serial.println(addessI2C);
      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address ");
      if (address < 16)
        Serial.print("0");
      Serial.println(address);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  // initialize the lcd
  lcdI2C.begin(LCD_COLUMNS, LCD_ROWS, addessI2C, ZURUECKLIGHT);
  lcdI2C.print("Alles ready?");
  delay(50);
  lcdI2C.selectLine(2);
  for (int i = 0 ; i < 17 ; i++)
  {
    lcdI2C.print("\333");
    delay(600 - i * 35);
  }
  delay(250);


  rotaryEncDButton.init();
  pinMode(RotaryEncoder_PIN_S1, INPUT_PULLUP);

  pinMode(STEPPER_PIN_STEP, OUTPUT);

  //IR Receiver strart
  irrecv.enableIRIn();
  irrecv.blink13(true);
}
