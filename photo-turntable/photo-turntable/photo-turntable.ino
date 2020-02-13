// Include Libraries
#include "Arduino.h"
#include "LiquidCrystal_PCF8574.h"
#include "IRremote.h"
#include "Encoder.h"
#include "Button.h"
#include "math.h"
#include <Wire.h>

// Pin Definitions
#define RotaryEncoder_PIN_CLK 2
#define RotaryEncoder_PIN_D  4
#define RotaryEncoder_PIN_S1  5
#define RECV_PIN 6
#define EN 8
#define DIR 9
#define PUL 10

// set default parameters
const long fullRotation = 102400;
const int accSteps = ((fullRotation) / 100);
const int basicSpeed = 20;
int angles = 8;
int picpause = 1;
int speed = 100;

// Define LCD characteristics
#define LCD_ROWS 2
#define LCD_COLUMNS 16
#define SCROLL_DELAY 150
#define ZURUECKLIGHT 255

// object initialization
Encoder rotaryEncD(RotaryEncoder_PIN_D, RotaryEncoder_PIN_CLK); //Rotary Encoder
Button rotaryEncDButton(RotaryEncoder_PIN_S1); //Rotary Encoder Button
LiquidCrystal_PCF8574 lcdI2C; //LCD Display
IRsend ir_led; //IR Sender
IRrecv irrecv(RECV_PIN); //IR Receiver
decode_results results;

long rotaryEncDOldPosition  = 0;

enum menuState {START, VIDEO, FOTOS, VIDSTART, FOTOSTART, GESCHWINDIGKEIT, ANZAHL, VIDZURUECK, STILZURUECK, CHANGEGESCHWINDIGKEIT, CHANGEANZAHL, FOTOPAUSE, CHANGEPAUSE} state = FOTOS, oldState = VIDEO;

// Global variables and defines
// There are several different versions of the LCD I2C adapter, each might have a different address.
// Try the given addresses by Un/commenting the following rows until LCD works follow the serial monitor prints.
// To find your LCD address go to: http://playground.arduino.cc/Main/I2cScanner and run example.
//#define LCD_ADDRESS 0x3F
//#define LCD_ADDRESS 0x27 //aka #define LCD_ADDRESS 39
//#define LCD_ADDRESS 0x23

//declare reset function @ address 0
void(* resetFunc) (void) = 0;

// Setup the essentials for your circuit to work. It runs first every time your circuit is powered with electricity.
void setup()
{
  delay(500);
  Wire.begin();
  // Setup Serial which is useful for debugging
  // Use the Serial Monitor to view printed messages
  Serial.begin(9600);
  while (!Serial);             // Leonardo: wait for serial monitor
  //Serial.println("\nI2C Scan");
  byte error, address;
  int nDevices;
  int addessI2C;

  //Serial.println("Scanning...");

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
      {
        Serial.print("0");
      }
      Serial.print(address);
      //Serial.println("!");
      addessI2C = address;
      //Serial.println(addessI2C);
      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address ");
      if (address < 16)
      {
        Serial.print("0");
      }
      //Serial.println(address);
    }
  }
  if (nDevices == 0)
  {
    //Serial.println("No I2C devices found\n");
  }
  else
  {
    //Serial.println("done\n");
  }

  delay(10);
  // initialize the lcd
  lcdI2C.begin(LCD_COLUMNS, LCD_ROWS, addessI2C, ZURUECKLIGHT);
  // lcdI2C.begin(LCD_COLUMNS, LCD_ROWS, 39, ZURUECKLIGHT);
  delay(50);
  lcdI2C.print("Alles ready?");
  delay(50);
  lcdI2C.selectLine(2);
  for (int i = 0 ; i < 17 ; i++)
  {
    lcdI2C.print("\333");
    delay(300 - i * 18);
  }
  delay(10);

  rotaryEncDButton.init();
  pinMode(RotaryEncoder_PIN_S1, INPUT_PULLUP);

  //IR Receiver strart
  irrecv.enableIRIn();
  irrecv.blink13(true);

  pinMode(EN, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(PUL, OUTPUT);
}

// Main logic of your circuit. It defines the interaction between the components you selected.
//After setup, it runs over and over again, in an eternal loop.
void loop()
{
  char menuDir = ' ';
  bool select = rotaryEncDButton.onPress();

  //IR Receiver actions
  // playpause D7E84B1B 3622325019
  // prievious 52A3D41F 1386468383
  // down F076C13B 4034314555
  // next 20FE4DBB 553536955
  // up E5CFBD7F 3855596927
  if (irrecv.decode(&results)) {
    irrecv.resume();
    if (results.value == 1386468383 or results.value == 4034314555)
    {
      menuDir = 'L';
      delay(5);
    }
    if (results.value == 553536955 or results.value == 3855596927)
    {
      menuDir = 'R';
      delay(5);
    }
    if (results.value == 3622325019)
    {
      select = true;
      delay(5);
    }
    if (results.value == 3810010651)
    {
      //Serial.println("resetting");
      delay(15);
      resetFunc();
      delay(5);
      //Serial.println("resetted");
    }
  }


  //Read encoder new position
  long rotaryEncDNewPosition = rotaryEncD.read() / 4;
  if (rotaryEncDNewPosition != rotaryEncDOldPosition) {
    //Serial.println(rotaryEncDNewPosition);
    if (rotaryEncDNewPosition < rotaryEncDOldPosition)
    {
      menuDir = 'L';
      delay(5);
    }
    else if (rotaryEncDNewPosition >= rotaryEncDOldPosition)
    {
      menuDir = 'R';
      delay(5);
    }
    rotaryEncDOldPosition = rotaryEncDNewPosition;
  }

  if (state != oldState)
  {
    lcdI2C.clear();
    delay(5);
    oldState = state;
  }

  lcdI2C.selectLine(1);
  lcdI2C.print(printState(state));

  switch (state)
  {
    case VIDEO:
      if (menuDir == 'L' || menuDir == 'R')
        state = FOTOS;

      if (select)
        state = VIDSTART ;

      break;

    case FOTOS:
      if (menuDir == 'L' || menuDir == 'R')
        state = VIDEO;

      if (select)
        state = FOTOSTART;

      break;

    case VIDSTART:
      if (menuDir == 'R')
        state = GESCHWINDIGKEIT;

      else if (menuDir == 'L')
        state = VIDZURUECK;

      if (select) {
        video();

      }
      break;

    case GESCHWINDIGKEIT:
      if (menuDir == 'L')
        state = VIDSTART;

      else if (menuDir == 'R')
        state = VIDZURUECK;

      if (select)

        state = CHANGEGESCHWINDIGKEIT;

      break;

    case CHANGEGESCHWINDIGKEIT:
      if (menuDir == 'L')
      {
        speed -= 5;
        lcdI2C.selectLine(2);
        lcdI2C.print("     ");
      }
      else if (menuDir == 'R')
      {
        speed += 5;
        lcdI2C.selectLine(2);
        lcdI2C.print("     ");
      }
      speed = constrain(speed, 0, 100);
      lcdI2C.selectLine(2);
      lcdI2C.print(speed);
      lcdI2C.print("%");


      if (select)
        state = VIDSTART;

      break;
    case VIDZURUECK:
      if (menuDir == 'L')
        state = GESCHWINDIGKEIT;

      else if (menuDir == 'R')
        state = VIDSTART;


      if (select)
        state = VIDEO;

      break;


    case FOTOSTART:
      if (menuDir == 'R')
        state = ANZAHL;

      else if (menuDir == 'L')
        state = STILZURUECK;

      if (select)
        stills();
      break;

    case ANZAHL:
      if (menuDir == 'L')
        state = FOTOSTART;

      else if (menuDir == 'R')
        state = FOTOPAUSE;

      if (select)
        state = CHANGEANZAHL;

      break;

    case CHANGEANZAHL:
      if (menuDir == 'L')
      {
        angles--;
        lcdI2C.selectLine(2);
        lcdI2C.print("     ");
      }
      else if (menuDir == 'R')
      {
        angles++;
        lcdI2C.selectLine(2);
        lcdI2C.print("     ");
      }

      lcdI2C.selectLine(2);
      lcdI2C.print(angles);

      if (select)
        state = FOTOSTART;
      break;

    case STILZURUECK:
      if (menuDir == 'L')
        state = FOTOPAUSE;

      else if (menuDir == 'R')
        state = FOTOSTART;

      if (select)
        state = FOTOS;
      break;

    case FOTOPAUSE:
      if (menuDir == 'L')
        state = ANZAHL;

      else if (menuDir == 'R')
        state = STILZURUECK;

      if (select)
        state = CHANGEPAUSE;
      break;

    case CHANGEPAUSE:
      if (menuDir == 'L')
      {
        picpause--;
        lcdI2C.selectLine(2);
        lcdI2C.print("     ");
      }
      else if (menuDir == 'R')
      {
        picpause++;
        lcdI2C.selectLine(2);
        lcdI2C.print("     ");
      }

      lcdI2C.selectLine(2);
      lcdI2C.print(picpause);

      if (select)
        state = FOTOSTART;
      break;
  }
}

String printState(int curstate)
{
  switch (curstate) {
    case 0:
      return "Start";
    case 1:
      return "Video";
    case 2:
      return "Fotos";
    case 3:
      return "Video starten";
    case 4:
      return "Fotos starten";
    case 5:
      return "Geschwindigkeit";
    case 6:
      return "Anzahl Fotos";
    case 7:
      return "Zur\365ck";
    case 8:
      return "Zur\365ck";
    case 9:
      return "Geschwindigkeit:";
    case 10:
      return "Anzahl Fotos:";
    case 11:
      return "Pause f\365r Foto";
    case 12:
      return "Pause in Sek:";
  }
}

void stills()
{
  lcdI2C.clear();
  lcdI2C.print(F("Start in 3"));
  //Serial.println("Start in 3");
  delay(1000);
  lcdI2C.clear();
  lcdI2C.print(F("Start in 2"));
  //Serial.println("Start in 2");
  delay(1000);
  lcdI2C.clear();
  lcdI2C.print(F("Start in 1"));
  //Serial.println("Start in 1");
  delay(1000);
  for (int i = 0 ; i < angles ; i++) {
    int imagesToTake = angles - i - 1;
    long numOfSteps = (fullRotation / angles);
    long delayTime = 0;
    long rpm = 0;
    delay(500);
    lcdI2C.clear();
    lcdI2C.print(F("Ausl\357ser, "));
    lcdI2C.print(picpause);
    lcdI2C.selectLine(2);
    if (picpause == 1)
    {
      lcdI2C.print(F("Sekunde Pause"));
    } else
    {
      lcdI2C.print(F("Sekunden Pause"));
    }
    irStillsShot();
    delay(1000 * picpause);
    lcdI2C.clear();
    if (imagesToTake == 1)
    {
      lcdI2C.print(F("Noch "));
      lcdI2C.print(imagesToTake);
      lcdI2C.print(F(" Foto"));
    }
    else
    {
      lcdI2C.print(F("Noch "));
      lcdI2C.print(imagesToTake);
      lcdI2C.print(F(" Fotos"));
    }
    delay(50);
    for (long n = 0; n < numOfSteps; n++)
    {
      if (n < accSteps)
      {
        delayTime = (accSteps - n );

        //Serial.println(delayTime);
      }
      else if (n > numOfSteps - accSteps)
      {
        delayTime = (n - numOfSteps + accSteps);

        //Serial.println(delayTime);
      }
      else
      {
        delayTime = 0;
      }

      rpm = basicSpeed + (delayTime / 20);

      digitalWrite(DIR, LOW);
      digitalWrite(EN, HIGH);
      digitalWrite(PUL, LOW);
      delayMicroseconds(rpm);
      digitalWrite(PUL, HIGH);
      delayMicroseconds(rpm);

    }
    delay(250);
  }
}

void video()
{
  lcdI2C.clear();
  lcdI2C.print(F("Start in 3"));
  //Serial.println("Start in 3");
  delay(1000);
  lcdI2C.clear();
  lcdI2C.print(F("Start in 2"));
  //Serial.println("Start in 2");
  delay(1000);
  lcdI2C.clear();
  lcdI2C.print(F("Start in 1"));
  //Serial.println("Start in 1");
  delay(1000);
  irVideoShot();
  lcdI2C.clear();
  lcdI2C.print(F("Video gestartet"));
  long numOfSteps = fullRotation;
  long delayTime = 0;
  long rpm = 0;
  for (long n = 0; n < numOfSteps; n++)
  {
    if (n < accSteps)
    {
      delayTime = (accSteps - n );

      //Serial.println(delayTime);
    }
    else if (n > numOfSteps - accSteps)
    {
      delayTime = (n - numOfSteps + accSteps);

      //Serial.println(delayTime);
    }
    else
    {
      delayTime = 0;
    }

    rpm = basicSpeed + (delayTime / 20);

    digitalWrite(DIR, LOW);
    digitalWrite(EN, HIGH);
    digitalWrite(PUL, LOW);
    delayMicroseconds(rpm);
    digitalWrite(PUL, HIGH);
    delayMicroseconds(rpm);

  }
  irVideoShot();
  lcdI2C.clear();
  delay(250);
}

//Sony IR-Codes:
//Photo     740239    B4B8F   // Shutter | take photo
//VID       76687     12B8F   // Start | stop video recording
//DISP      166799    28B8F   // Display change cycles round
//Menu      117647    1CB8F   // Enter menu | leave menu
//MenuU     379791    5CB8F   // Menu up
//menuD     904079    DCB8F   // Menu down
//menuR     1035151   FCB8F   // Menu right
//menuL     510863    7CB8F   // Menu left
//OK        641937    9CB91   // Menu OK
//Z+        338831    52B8F   // Zoom in
//Z-        863119    D2B8F   // Zoom out

void irStillsShot() {
  //Serial.println("foto...");
  for (int i = 0; i < 3; i++)
  {
    ir_led.sendSony(0xB4B8F, 20);
    delay(40);
    irrecv.enableIRIn();
    irrecv.blink13(true);
  }
}

void irVideoShot() {
  //Serial.println("video...");
  for (int i = 0; i < 3; i++)
  {
    ir_led.sendSony(0x12B8F, 20);
    delay(40);
    irrecv.enableIRIn();
    irrecv.blink13(true);
  }
}
