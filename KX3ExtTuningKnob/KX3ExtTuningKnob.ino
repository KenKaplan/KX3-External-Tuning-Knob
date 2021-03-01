/**************************************************************************
  KX3ExtTuningKnob - Ken Kaplan WB2ART September 2020 version 5.1
  Rotary encoder code de W8BH.

   2/27/21 - addedd check for serial port (on Blue Pill) to only send commands to the KX3
   1/02/21 - finished final hardware build / modified code for most recent pcb version 3.0 (version 5.0 code)
  12/26/20 - plan to add volume control - AG 000-255
  12/12/20 - added bypassAll variable. When true will only send characters to/from the KX3.
             No tuning or button pushes. This is for conflict when running WSJT-X.
             The rocker switch is the only switch checked.  version 4.8
             ***** IF YOU ARE LOADING FIRMWARE INTO THE KX3, PX3, OR KXPA100 MAKE SURE THIS SWITCH ACTIVTES bypassAll, 
             SO THE POLLING ROUTINE IS NOT ACTIVATED *****
  11/01/20 - new tuning rate routine implemented - version 4.6
  10/26/20 - fixed tuning rate for 2m and maybe other transverter bands - version 4.5
  10/17/20 - fixed tuning rate for bands 160-6m - version 4.0

  Legal: Copyright (c) 2020  Ken Kaplan
  THIS HARDWARE(PCB)/SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
  ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  HARDWARE(PCB)/SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

  Refer to the KX3 Programmers Manual for commands

  Serial2 = PA2-Txd  PA3-Rxd
  Serial3 = PB10-Txd PB11-Rxd

  TUNING RATES:
  UPn; DNn;  n: 0=1 Hz; 1=10 Hz; 2=20 Hz; 3=50 Hz; 4=1 kHz; 5=2 kHz; 6=3 kHz; 7=5 kHz; 8=100 Hz; 9=200 Hz.
  How is the tuning rate for KX3 determined?
  A DS; command is sent to the KX3, and the response is the VFO A display - see below
  ---------------------------------------------------------------------------------------------------------------------------------------------------------
  FROM THE KX3 PROGRAMMERS MANUAL: { The format of the response message is: DSttttttttaf; where tttttttt is the LCD text
  and decimal point data, a is icon data, and f is icon flash data, a and f data are ignored in this program.
  TEXT and decimal point data: This field contains 8 bytes, with values 0x30 - 0xFF (hex).
  The first byte is the left-most displayed character. Bit 7 (MSB) of each byte indicates whether the decimal point to the left of each character
  is on (1) or off (0). The other bits contain an ASCII character that corresponds to the displayed character.
  ---------------------------------------------------------------------------------------------------------------------------------------------------------
  The response on 160-40 meters looks like: DS@70095@@ (we only need the 'tttttttt' field) - the '@' signs are translated into spaces.
  We count the number of '@' signs, ignoring the first '@' sign. We actually only need the last 7 characters. The '@@' means we are in 100Hz tuning mode,
  so we need to send UP8; or DN8; to tune the KX3 100 Hz. If we are in 10Hz tuning mode, then the response is DS@700955@, and if in 1Hz tuning mode, DS@7009554.
  You can see that the number of '@' signs can be counted to determine the tuning mode.
  On 30-6 meters, there is no leading '@' sign ie: DS140065@@ (100Hz), DS1400650@ (10Hz) and DS14006505 (1Hz).

  For 2 meters, the DS response is DS1459977@, in 100Hz tuning mode. All other 100 Hz tuning modes have 2 '@' signs, however,
  if we check character 6, we see that the MSB is set (1), and there is a decimal point to the left of that position. That means
  there are 3 digits (145) in front of the decimal point. For 10 Hz tuing mode, there are still 3 digits (145) to the left of the
  decimal point, but NO '@' signs. 1 Hz tuning mode is the same as for 160-6 meters, with 2 digits to the left of the decimal point, and
  no '@'signs. There is variable called 'vhfBnd' that takes care of the tuning mode switching in the same routine as for 160-6 meters.

  Unfortunately, I cannot determine the tuning mode for any band above 2 meters, as I have no capability to test those bands.
*/

// install in prefrences: http://dan.drown.org/stm32duino/package_STM32duino_index.json
#include <ButtonEvents.h> // see: https://github.com/fasteddy516/ButtonEvents and https://github.com/thomasfredericks/Bounce2
// ignore the following error from the arduino ide:
// WARNING: library ButtonsEvents-master claims to run on avr architecture(s) and may be incompatible with your current board which runs on STM32F1 architecture(s).

// #include "EEPROM.h"

// KX3 Switch Commands
String SW1Tap = "SWT12;"; // rate
String SW1DoubleTap = "MN146;MP001;PC100;MN255;"; // pa on, set tp 100w
String SW1Hold = "SWH12;"; // khz

String SW2Tap = "SWT28;"; // spot
String SW2DoubleTap = ""; // 
String SW2Hold = "SWT44;"; // atu tune

String SW3Tap = "SWH35;SWT25;SWT25;SB1;UP4;"; // dual receive on
String SW3DoubleTap = ""; // 
String SW3Hold = "SWH25;"; // split

String SW4Tap = "SWT08;"; // band +
String SW4DoubleTap = ""; //
String SW4Hold = "SWT41;"; // band -

String SW5Tap = "SWT25;"; // A->B
String SW5DoubleTap = ""; // 
String SW5Hold = "SWT24;"; // A/B

String SW6Tap = "SWT35;"; // ofs/B
String SW6DoubleTap = "SB0;DN4;SWT25;"; // 
String SW6Hold = "SWH29;"; // vox

//**********************************

const byte SW1 = PA15;
const byte SW2 = PB3;
const byte SW3 = PB4;
const byte SW4 = PB14;
const byte SW5 = PB15;
const byte SW6 = PA8;
const byte auxSw = PA0;
const byte led1 = PB9;
const byte led2 = PB8;
const byte led3 = PB5;

ButtonEvents SW1Btn;
ButtonEvents SW2Btn;
ButtonEvents SW3Btn;
ButtonEvents SW4Btn;
ButtonEvents SW5Btn;
ButtonEvents SW6Btn;

int tuningRate = 1; // 8 = 100 Hz, 1 = 10 Hz, 0 = 1 Hz
bool bypassAll = false;
bool sentDS = false;
bool vfoB = false;
bool compSending = false;
bool vhfBnd = false;
bool audioGain = false;
int previousDir = 0;
int previousVolts = 0;
int bandNumber = 0;
int atCount = 0;
int charCount = 0;
int auxSwValue = 0;
unsigned long previousMillis = 0;
const long interval = 1500; // polling interval, set faster ie:1000 or slower ie:2000
//===================================  Hardware Connections =============================
#define ENCODER_A PA6 // Rotary Encoder output A
#define ENCODER_B PA7 // Rotary Encoder output B
//=================================== Constants =============================
#define ENCODER_TICKS 1  // Ticks required to register movement
//===================================  Rotary Encoder Variables =========================
volatile int rotaryCounter = 0;           // "position" of rotary encoder (increments CW)
//===================================  Rotary Encoder Code  =============================
/*
   Rotary Encoder Interrupt Service Routine ---------------
   This is an alternative to rotaryISR() above.
   It gives twice the resolution at a cost of using an additional interrupt line
   This function will run when either encoder pin A or B changes state.
   The states array maps each transition 0000..1111 into CW/CCW rotation (or invalid).
   The rotary "position" is held in rotary_counter, increasing for CW rotation, decreasing
   for CCW rotation. If the position changes, rotary_change will be set true.
   You should set this to false after handling the change.
   To implement, attachInterrupts to encoder pin A *and* pin B
*/

void rotaryISR()
{
  const int states[] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};
  static byte transition = 0;                     // holds current and previous encoder states
  transition <<= 2;                               // shift previous state up 2 bits
  transition |= (digitalRead(ENCODER_A));         // put encoder_A on bit 0
  transition |= (digitalRead(ENCODER_B) << 1);    // put encoder_B on bit 1
  transition &= 0x0F;                             // zero upper 4 bits
  rotaryCounter += states[transition];            // update counter +/- 1 based on rotation
}

/*
   readEncoder() returns 0 if no significant encoder movement since last call,
   +1 if clockwise rotation, and -1 for counter-clockwise rotation
*/

int readEncoder(int numTicks = ENCODER_TICKS)
{
  static int prevCounter = 0;                     // holds last encoder position
  int change = rotaryCounter - prevCounter;       // how many ticks since last call?
  if (abs(change) <= numTicks)                    // not enough ticks?
    return 0;                                     // so exit with a 0.
  prevCounter = rotaryCounter;                    // enough clicks, so save current counter values
  return (change > 0) ? 1 : -1;                   // return +1 for CW rotation, -1 for CCW
}

void setupSwitches()
{
  pinMode(SW1, INPUT_PULLUP);
  pinMode(SW2, INPUT_PULLUP);
  pinMode(SW3, INPUT_PULLUP);
  pinMode(SW4, INPUT_PULLUP);
  pinMode(SW5, INPUT_PULLUP);
  pinMode(SW6, INPUT_PULLUP);
  pinMode(auxSw, INPUT);
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);

  SW1Btn.attach(SW1);
  SW2Btn.attach(SW2);
  SW3Btn.attach(SW3);
  SW4Btn.attach(SW4);
  SW5Btn.attach(SW5);
  SW6Btn.attach(SW6);

  SW1Btn.debounceTime(10); // 10ms
  SW2Btn.debounceTime(10);
  SW3Btn.debounceTime(10);
  SW4Btn.debounceTime(10);
  SW5Btn.debounceTime(10);
  SW6Btn.debounceTime(10);

  SW1Btn.doubleTapTime(250); // 250ms
  SW2Btn.doubleTapTime(250);
  SW3Btn.doubleTapTime(250);
  SW4Btn.doubleTapTime(250);
  SW5Btn.doubleTapTime(250);
  SW6Btn.doubleTapTime(250);

  SW1Btn.holdTime(1000); // 1sec
  SW2Btn.holdTime(1000);
  SW3Btn.holdTime(1000);
  SW4Btn.holdTime(1000);
  SW5Btn.holdTime(1000);
  SW6Btn.holdTime(1000);
}

void setupEncoder()
{
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), rotaryISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), rotaryISR, CHANGE);
}

void serialCheck()
{
  if (Serial.available()) // check if computer has sent command
      {
        compSending = true;
        int inByte = Serial.read();
        Serial2.write(inByte); // send command to KX3
        //Serial.println(char(inByte));
      }
      else compSending = false;
}

void serial2Check()
{
  if (Serial2.available()) // check if KX3 has sent data
    {
      int inByte = Serial2.read(); // read a character from KX3
      Serial3.write(inByte); // send character to computer
    }
}

void serial3Check()
{
        if (Serial3.available()) // check if computer has sent command
      {
        compSending = true;
        int inByte = Serial3.read();
        Serial2.write(inByte); // send command to KX3
        //Serial.println(char(inByte));
      }
      else compSending = false;
}

void setup()
{
  // initialize serial ports
  Serial.begin(38400);  // input/output to computer
  Serial1.begin(38400);
  Serial2.begin(38400); // input/output to KX3
  Serial3.begin(38400); // second input/output = depends on jumper positions

  setupEncoder();

  setupSwitches();

  Serial2.println("AI0;"); // turn off auto-info mode
  // vfoB = false;
  // bypassAll = false;
  // audioGain = false;
}

void loop()
{
  // check AuxSw
  int auxSwValue = analogRead(auxSw);
  int testint = map(auxSwValue, 0, 4096, 0, 10);
  if (testint != previousVolts)
    //Serial.print("testint= ");
    //Serial.println(testint);
  {
    digitalWrite(led1, LOW);
    digitalWrite(led2, LOW);
    digitalWrite(led3, LOW);
    audioGain = false; 
    vfoB = false;
    bypassAll = false;
    switch (testint)
    {
      case 8:
        {
          vfoB = true;
          audioGain = true;
          digitalWrite(led3, HIGH);
          break;
        }
      case 9:
        { // just turn on led2, as other booleans are false
          digitalWrite(led2, HIGH);
          break;
        }
      case 2:
        {
          digitalWrite(led1, HIGH);
          bypassAll = true;
          break;
        }
    }
    previousVolts = testint;
  }

  if (!bypassAll)
  {

    // check the encoder
    int currentDir = readEncoder(); // check encoder
    if (currentDir != previousDir)  // did it move?
    {
      if (currentDir >= 1)
      {
        if (vfoB)
        {
          Serial2.println("UPB" + String(tuningRate) + ";"); // send tuning command to KX3
          //Serial.println("UPB" + String(tuningRate) + ";"); // send tuning command to KX3
        }
        else
        {
          Serial2.println("UP" + String(tuningRate) + ";"); // send tuning command to KX3
          // Serial.println("UP" + String(tuningRate) + ";"); // send tuning command to KX3
        }
      }

      if (currentDir < 0)
      {
        if (vfoB)
        {
          Serial2.println("DNB" + String(tuningRate) + ";");  // send tuning command to KX3
          //Serial.println("DNB" + String(tuningRate) + ";");  // send tuning command to KX3
        }
        else
        {
          Serial2.println("DN" + String(tuningRate) + ";");  // send tuning command to KX3
          // Serial.println("DN" + String(tuningRate) + ";");  // send tuning command to KX3
        }
      }
    }
    previousDir = currentDir; // store last encoder position
    // end check encoder

    // poll the KX3. Send DS; to get VFO A info. This will tell us if in 100, 10 or 1Hz tuning mode;
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval)
    {
      //Serial.print(".");
      if (!compSending) // we don't want to poll the KX3 if the computer is polling
      {
        Serial2.println("DS;");
        sentDS = true;
        atCount = 0;
        charCount = 0;
        vhfBnd = false;
        previousMillis = currentMillis;
        // Serial.println("Polling");
      }
    }

    if (Serial2.available()) // check if KX3 has sent data
    {                                                              // ***** NEED TO PARSE FOR DIFFERENT RESPONSES *****
      int inByte = Serial2.read(); // read a character from KX3
      if (!sentDS) Serial3.write(inByte); // do not send KX3 data to computer if listening for DS; response
      int vhfByte = inByte; // use vhfByte for test, as MSB is not cleared when we read character 6 MSB
      bitClear(inByte, 7);
      // Serial.write(inByte); 
      // Serial.println();
      if (sentDS)
      {
        if (inByte != ';')
        {
          ++charCount; // charCount is 0 when DS; is sent to KX3
          if (charCount >= 4 && charCount <= 10) // only need bytes at 4,5,6,7,8,9,10
          {
            if (charCount == 6 && bitRead(vhfByte, 7) == 1) vhfBnd = true;
            if (inByte == '@') ++atCount; // atCount is 0 when DS; is sent to KX3
          }
        }
        else
        {
          sentDS = false; // reset sentDS flag, because we already have complete atCount, as soon as a ; arrives.

          switch (atCount) // set tuningRate based on number of @ signs found - vhf will only have 0 or 1 '@' sign
          {
            case 1:
              { // 1 @ sign, so must be in 10Hz tuning mode
                tuningRate = 1; // Serial.println("Rate=1");
                if (vhfBnd) tuningRate = 8;
                break;
              }
            case 2:
              { // 2 @ signs, so must be in 100Hz tuning mode
                tuningRate = 8;// Serial.println("Rate=8");
                break;
              }
            default:
              { // no @ signs, so must be in 1Hz tuning mode
                tuningRate = 0; //Serial.println("Rate=0");
                if (vhfBnd) tuningRate = 1;
                break;
              }
          }
        }
      }
    }
    serialCheck(); // micro usb on Blue Pill
    serial3Check();
    
    // each of the follwing case sections has detection for switches SW1-SW6 for tap, doubleTap and hold functions.
    // poll each of the 6 switches
    if (SW1Btn.update() == true)
    {
      switch (SW1Btn.event())
      {
        case (tap):
          {
            Serial2.println(SW1Tap);
            break;
          }
        case (doubleTap):
          {
            Serial2.println(SW1DoubleTap);
            break;
          }
        case (hold):
          {
            Serial2.println(SW1Hold);
            break;
          }
      }
    }

    if (SW2Btn.update() == true)
    {
      switch (SW2Btn.event())
      {
        case (tap):
          {
            Serial2.println(SW2Tap);
            break;
          }
        case (doubleTap):
          {
            Serial2.println(SW2DoubleTap);
            break;
          }
        case (hold):
          {
            Serial2.println(SW2Hold);
            break;
          }
      }
    }

    if (SW3Btn.update() == true)
    {
      switch (SW3Btn.event())
      {
        case (tap):
          {
            Serial2.println(SW3Tap);
            break;
          }
        case (doubleTap):
          {
            Serial2.println(SW3DoubleTap);
            break;
          }
        case (hold):
          {
            Serial2.println(SW3Hold);
            break;
          }
      }
    }

    if (SW4Btn.update() == true)
    {
      switch (SW4Btn.event())
      {
        case (tap):
          {
            Serial2.println(SW4Tap);
            break;
          }
        case (doubleTap):
          {
            Serial.println(SW4DoubleTap);
            break;
          }
        case (hold):
          {
            Serial2.println(SW4Hold);
            break;
          }
      }
    }

    if (SW5Btn.update() == true)
    {
      switch (SW5Btn.event())
      {
        case (tap):
          {
            Serial2.println(SW5Tap);
            break;
          }
        case (doubleTap):
          {
            Serial.println(SW5DoubleTap);
            break;
          }
        case (hold):
          {
            Serial2.println(SW5Hold);
            break;
          }
      }
    }

    if (SW6Btn.update() == true)
    {
      switch (SW6Btn.event())
      {
        case (tap):
          {
            Serial2.println(SW6Tap);
            break;
          }
        case (doubleTap):
          {
            Serial2.println(SW6DoubleTap);
            break;
          }
        case (hold):
          {
            Serial2.println(SW6Hold);
            break;
          }
      }
    }
  } // end bypassAll = false

  if (bypassAll) 
  {
    serial2Check();
    serial3Check();
  } // end bypassALL = true

} // end void loop()
