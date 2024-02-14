/* HornoGas_V3
     See ReadMe.txt
*/

/* Mac Addresses */
// ESP32 #1: 08:3A:F2:A9:7E:DC <- Main controller (THIS ONE) (0C:B8:15:C3:80:34)
// ESP32 #2: 08:3A:F2:6C:CB:9C <- Publishes EAST DPTs, opens chimney
// ESP32 #3: 40:91:51:BF:CF:FC <- Publishes WEST DPTs, receives everything

// Libraries to include
#include <TFT_eSPI.h>           // Graphics and font library for ILI9341 driver chip
#include <Wire.h>
#include "Adafruit_MAX31856.h"  // thermocouple card library (56)
#include <PID_v1.h>             // PID temp control library
#include <SD.h>                 // SD memory card library (SPI is required)
#include <FS.h>                 // So that ESP32 recognizes SD card
#include <esp_now.h>            // For ESP-NOW communication protocol
#include <esp_wifi.h>           // To configure esp32 wireless communication
#include <WiFi.h>               // To use correct WiFi channel
#include <ShiftRegisterSPI.h>   // GPIO extender using sn74hc595 shift register
#include <Preferences.h>        // library to save variables to EPROOM

// Setup user variables (CHANGE THESE TO MATCH YOUR SETUP)
const int tempCycle = 2000;                 // Temperature reading cycle
const int espnowCycle = 2500;               // EspNow transmit cycle
const int maxTemp = 1040;                   // Maximum temperature (degrees).  If reached, will shut down.
const int pidCycle = 30000;                 // Sample time of PID (how often Output is calculated in mS).
double Kp = 0.5, Ki = 1, Kd = 0.08;         // PID constants (tunings), VERY IMPORTANT TO GET THEM RIGHT.
const int tempOffset = 0;                   // Temp offset (degrees) of thermocouplecouple, either from a cold zone or exernal factors. This is added to the input.
const int tempRange = 2;                    // This is how close the temp reading needs to be to the set point to shift to the hold phase (degrees).  Set to zero or a positive integer.
const char tempScale = 'C';                 // Temperature scale.  F = Fahrenheit.  C = Celsius
constexpr char WIFI_SSID[] = "FIX2.4";      // WiFi SSID
max31856_thermocoupletype_t TCTYPE = MAX31856_TCTYPE_K;
const int EspNowTimeOut = 10000;            // (ms) before espnow timeouts
const int topCycle = 2500;                  // Top bar refresh cycle (ms)
#define bar_color 0x53D2  // Color for top bar

// ESP NOW configuration
uint8_t ESP32_2[] = {0x08, 0x3A, 0xF2, 0x6C, 0xCB, 0x9C}; // MAC-Address of ESP32 #2
uint8_t ESP32_3[] = {0x40, 0x91, 0x51, 0xBF, 0xCF, 0xFC}; // MAC-Address of ESP32 #3
esp_now_peer_info_t peerInfo;
/* DPT pressure sensors structure */
typedef struct DPT_message {
  //char id[4]; // east or west
  float p[4]; // 2 gas pressures, 2 air pressures
} DPT_message;
/* Kiln parameters transmitter structure */
typedef struct kiln_message {
  int temp; // pidInput
  int fuegoBajo; // FB
  int fuegoAlto; // FA
  int fuegoSuperAlto; // FSA
  int SetPoint; // pidSetPoint
  int Output;  // pidOutput 0%-100%
} kiln_message;
/* Chimney actuator trigger */
typedef struct chimney_message {
  bool OPEN;
} chimney_message;

// Setup pin connections (CHANGE THESE TO MATCH YOUR SETUP)
const int upPin = 34;                 // Pin # connected to up arrow button #1
const int selectPin = 35 ;            // Pin # connected to select / start button #5
const int downPin = 36;               // Pin # connected to down arrow button #3
const int tftRstPin = 39;             // Pin # connected to tft rst button
const int thermocoupleCS = 33;        // CS pin # for thermocouplecouple. SPI is hardware such that: DO -> MISO (19), CLK -> SCLK (18)
const int shiftregisterCS = 17;       // CS pin # for shift register.
const int gasPin = 1;                 // Register pin for gas contactor. Pins mapped from (0-7) to Q0-Q7. Pin 1 is SKIPPED DUE TO SHORT
const int fbPin = 2;                  // Register pin for FB. Pins mapped from (0-7) to Q0-Q7
const int faPin = 3;                  // Register pin for FA. Pins mapped from (0-7) to Q0-Q7
const int fsaPin = 4;                 // Register pin for FSA. Pins mapped from (0-7) to Q0-Q7
const int airPin = 5;                 // Register pin for air contactor. Pins mapped from (0-7) to Q0-Q7
                 
// Setup other variables (DON'T CHANGE THESE)
unsigned long TFT_start;              // Exact time you refreshed the TFT screen (ms).  Based on millis().
unsigned long tempStart;              // Exact time you last updated the temperature (ms). Based on millis().
unsigned long espnowStart;            // Exact time you last updated the EspNow variables (ms). Based on millis().
unsigned long programStart;           // Exact time you started running the program (ms).  Based on millis().
unsigned long rampStart;              // Exact time the ramp phase of the segment started (ms).  Based on millis().
unsigned long holdStart;              // Exact time the hold phase of the segment started (ms).  Based on millis().
unsigned long topStart;              // Exact time you last refreshed the top bar 
unsigned long ESP32_2_msgTime = 0;    // Exact time you last got a message for ESP32 #2 
unsigned long ESP32_3_msgTime = 0;    // Exact time you last got a message for ESP32 #3
double pidInput;                      // Input for PID loop (actual temp reading from thermocouplecouple).  Don't change.
double pidOutput;                     // Output for PID loop (relay for heater).  Don't change.
double pidSetPoint;                   // Setpoint for PID loop (temp you are trying to reach).  Don't change.
double calcSetPoint;                  // Calculated set point (degrees)
double rampHours;                     // Time it has spent in ramp (hours)
char programDesc1[21];                // Program description #1 (first line of text file)
char* screen = "intro";               // Variable that holds screen type (start with intro)
String programMode;                   // Can be AUTOMATIC or MANUAL
int introSel = 1;                     // Intro menu selected option (start or settings)
int confirmSel;                       // Confirm selected option (back or OK)
int settingsSel = 1;                  // Settings menu selected setting
int segQuantity;                      // How many segments in firing program
int lastTemp;                         // Last setpoint temperature (degrees)
int optionNum = 1;                    // Option selected from screen #3
int programNumber;                    // Current firing program number.  This ties to the file name (ex: 1.txt, 2.txt).
int screenNum = 1;                    // Screen number displayed during firing (1 = temps / 2 = program info / 3 = tools / 4 = done
int segNum = 0;                       // Current segment number running in firing program.  0 means a program hasn't been selected yet.
int segHold[20];                      // Hold time for each segment (min).  This starts after it reaches target temp.
int segRamp[20];                      // Rate of temp change for each segment (deg/hr).
int segTemp[20];                      // Target temp for each segment (degrees).
int FAstart, FAend;                   // FA temperature range (manual event)
int FSAstart, FSAend;                 // FSA temperature range (manual event)
int FB = LOW, FA = LOW, FSA = LOW;    // Fire modes
int cooling;                          // Cooling shutoff modes
int tftwidth, tftheight;              // TFT screen dimensions (pixels)
bool upPressed = false;               // Up button press state
bool selectPressed = false;           // Select button press state
bool downPressed = false;             // Down button press state
bool programOK = false;               // Is the program you loaded OK?
bool isOnHold = false;                // Current segment phase: false = ramp, true = hold.
int modeSel = 1;                      // 1 is AUTOMATIC, 2 is MANUAL
int eventNum = 1;
int rangeOpt = 1;                     // Option selected from range screen
int coolingSel = 1;                   // Option selected from cooling screen
bool rangeScreen = false;             // is it on the range screen
bool adjustRange = false;             // boolean to adjust range
unsigned long startTime = 0;
bool ESP32_2_isOK;     
bool ESP32_3_isOK;

//******************************************************************************************************************************
//  SETUP: INITIAL SETUP (RUNS ONCE DURING START)
//******************************************************************************************************************************
/* Initialize stuff */
TFT_eSPI tft;
PID pidCont(&pidInput, &pidOutput, &pidSetPoint, Kp, Ki, Kd, DIRECT);
Preferences preferences;
Adafruit_MAX31856 thermocouple(thermocoupleCS);
ShiftRegisterSPI<1> shift_register(shiftregisterCS);

DPT_message DPT_W;
DPT_message DPT_E;
kiln_message Kiln;
chimney_message chimney;

void setup() {
  Serial.begin(115200);
  SPI.begin();

  // setup and retrieve data from EEPROM
  preferences.begin("my-app", false);
  FAstart = preferences.getInt("FAstart", 350);
  FAend = preferences.getInt("FAend", 985);
  FSAstart = preferences.getInt("FSAstart", 450);
  FSAend = preferences.getInt("FSAend", 982);
  programMode = preferences.getString("programMode", "AUTOMATIC");
  programNumber = preferences.getInt("programNumber", 1);
  cooling = preferences.getInt("cooling", 0); // 0 = shutoff with gas
  Serial.printf("FAstart: %d, FAend: %d, FSAstart: %d, FSAend: %d \n", FAstart,FAend, FSAstart, FSAend);
  
  // Setup all pin modes on board.
  pinMode(upPin, INPUT);
  pinMode(downPin, INPUT);
  pinMode(selectPin, INPUT);
  pinMode(tftRstPin, INPUT);

  // Setup thermocouple
  thermocouple.begin();
  thermocouple.setThermocoupleType(TCTYPE); // thermocouple.getThermocoupleType
 
  // Setup TFT display (320 x 240 display)
  tftwidth = 320, tftheight = 240;
  tft.init();
  tft.setRotation(3);

  // Setup SD card
  while (!SD.begin()) {
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_RED, TFT_BLACK); tft.setTextSize(4);
    tftPrintCenterWidth("ERROR", 80);
    tft.setTextColor(TFT_WHITE, TFT_BLACK); tft.setTextSize(2);
    tftPrintCenterWidth("Can't setup SD card.", 150);
    tftPrintCenterWidth("Make sure card is in.", 180);
    delay(2000);
  }
  tft.fillScreen(TFT_BLACK);

  // Set all gas SSR outputs off
  shift_register.setAllLow();
  // Set air relay output ready 
  shift_register.set(airPin, HIGH); // relay is normally closed
  
  // Initialize WiFi and ESP-NOW
  initWiFi();
  initEspNow();
}

//******************************************************************************************************************************
//  LOOP: MAIN LOOP (CONTINUOUS)
//******************************************************************************************************************************
void loop() {
  //******************************
  // shutdown gas if too hot
  if (pidInput >= maxTemp) {
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_RED, TFT_BLACK); tft.setTextSize(4);
    tftPrintCenterWidth("ERROR", 80);
    tft.setTextColor(TFT_WHITE, TFT_BLACK); tft.setTextSize(2);
    tftPrintCenterWidth("Max temp reached", 150);
    tftPrintCenterWidth("System was shut down.", 180);
    fireOff();
  }
  // Reset TFT display if it goes nuts
  if (digitalRead(tftRstPin) == LOW) {
    resetTFT();
    btnBounce(tftRstPin);
  }
  // Update temperature
  if (millis() - tempStart >= tempCycle) {
    readTemps();
    tempStart = millis();
  }
  // Update top info bar
  if (millis() - topStart >= topCycle) {
    drawTopBar();
    topStart = millis();
  }

  //*****************************
  // Intro screen
  if (segNum == 0 && screen == "intro") {
    readButtons();
    introScreen(introSel);

    if (upPressed && introSel > 1) {
      introSel -= 1;
    }
    if (downPressed && introSel < 2) {
      introSel += 1;
    }

    /* User pressed settings */
    if (selectPressed && introSel == 2) {
      screen = "settings"; // user pressed settings
      settingsSel = 1;
      tft.fillRect(0, 20, 320, 240 - 20, TFT_BLACK); // clear screen except top notch
    }

    /* User pressed START */
    if (selectPressed && introSel == 1) {
      shift_register.set(gasPin, HIGH);  // allow gas contactor to be manually energized
      
      screen = "confirm"; // go to confirm screen
      confirmSel = 2; // set selection to OK
      tft.fillRect(0, 20, 320, 240 - 20, TFT_BLACK); // clear screen except top notch
      tft.setTextSize(2); tft.setTextColor(TFT_WHITE, TFT_BLACK);
      tftPrint("  BACK  ", 10, 200);
      tftPrint("> OK <", 200, 200);

      if (programMode == "AUTOMATIC") {
        openProgram();
      }
      if (programMode == "MANUAL") { // show events;
      }
    }
  }

  /* Confirm program screen: only refreshes if buttons are pressed, great to avoid EMI from contactors*/
  if (segNum == 0 && screen == "confirm") {
    readButtons();
    tft. setTextSize(2); tft.setTextColor(TFT_WHITE, TFT_BLACK);

    if (upPressed && confirmSel == 2) {
      confirmSel -= 1;
      tftPrint("> BACK <", 10, 200);
      tftPrint("  OK  ", 200, 200);
    }
    if (downPressed && confirmSel == 1) {
      confirmSel += 1;
      tftPrint("  BACK  ", 10, 200);
      tftPrint("> OK <", 200, 200);
    }

    if (selectPressed && confirmSel == 1) { // pressed back
      shift_register.set(gasPin, LOW); // block gas contactor from being energized  
      screen = "intro";
      tft.fillRect(0, 20, 320, 240 - 20, TFT_BLACK);
    }
    
    if (selectPressed && confirmSel == 2) { // pressed OK
      segNum = 1;                                     // firing
      programStart = millis();                        // use later to know program duration
      tft.fillRect(0, 20, 320, 240 - 20, TFT_BLACK);  // clear screen except top notch

      if (programMode == "AUTOMATIC") {
        setupPIDs(HIGH); // setup PID algorithm
        rampStart = millis();
        lastTemp = pidInput;
      }
    }
  }

  //****************************
  // Settings screens
  if (strstr(screen, "settings") != NULL ) {

    int settings_options;
    if (programMode == "AUTOMATIC") {
      settings_options = 4;  // for AUTO mode (select mode, select program, cooling, done)
    }
    if (programMode == "MANUAL") {
      settings_options = 4;  // for MANUAL mode (select mode, select events, cooling, done)
    }

    // Main settings screen
    if (screen == "settings" && segNum == 0) {
      settingsScreen(settingsSel);   // Display settings screen
      readButtons();

      if (upPressed && settingsSel > 1) settingsSel -= 1;
      if (downPressed && settingsSel < settings_options) settingsSel += 1;

      /* SELECTIONS */
      if (selectPressed) tft.fillRect(0, 20, 320, 240 - 20, TFT_BLACK); // clear display except top notch
      // user pressed SELECT MODE
      if (selectPressed && settingsSel == 1) {
        screen = "settings_mode";
        modeSel = 1;
      }
      // user pressed SELECT PROGRAM
      if (selectPressed && settingsSel == 2 && programMode == "AUTOMATIC") {
        screen = "settings_program";
      }
      // user pressed SELECT EVENTS
      if (selectPressed && settingsSel == 2 && programMode == "MANUAL") {
        screen = "settings_events";
        eventNum = 1;
      }
      // user pressed COOLING
      if (selectPressed && settingsSel == 3) {
        screen = "settings_cooling";
        coolingSel = 1;
      }
      // user pressed DONE
      if (selectPressed && settingsSel == 4) {
        screen = "intro";
        introSel = 1;
      }
    }

    // Select mode screen
    if (screen == "settings_mode" && segNum == 0) {
      modeScreen(modeSel);
      readButtons();
      if (upPressed && modeSel > 1) {
        modeSel -= 1;
      }
      if (downPressed && modeSel < 2) {
        modeSel += 1;
      }
      if (selectPressed) {
        const char* Mode;
        if (modeSel == 1) {
          Mode = "AUTOMATIC";
        }
        if (modeSel == 2) {
          Mode = "MANUAL";
        }
        programMode = preferences.putString("programMode", Mode); // save it in EEPROM
        programMode = preferences.getString("programMode", "AUTOMATIC"); // update variable (idk why but yea)
        screen = "settings";
        settingsSel = 1;
        tft.fillRect(0, 20, 320, 240 - 20, TFT_BLACK); // clear screen except top notch
        tft.fillRect(240, 0, 320 - 240, 20, TFT_BLACK); // clear top notch
      }
    }

    // Select program screen
    if (screen == "settings_program" && segNum == 0) {
      openProgram();
      readButtons();
      if (upPressed && programNumber > 1) {
        programNumber -= 1;
        tft.fillRect(0, 20, 320, 240 - 20, TFT_BLACK);
      }
      if (downPressed) {
        programNumber += 1;
        tft.fillRect(0, 20, 320, 240 - 20, TFT_BLACK);
      }
      if (selectPressed) {
        programNumber = preferences.putInt("programNumber", programNumber); // save it in EEPROM
        programNumber = preferences.getInt("programNumber", 1); // retrieve from EEPROM
        screen = "settings";
        settingsSel = 4;
        tft.fillRect(0, 20, 320, 240 - 20, TFT_BLACK); // clear screen except top notch
      }
    }

    // Select manual events screen
    if (screen == "settings_events") {
      manualSetup(eventNum);
      readButtons();
      if (upPressed && eventNum > 1) {
        eventNum -= 1;
      }
      if (downPressed && eventNum < 3) {
        eventNum += 1;
      }
      if (selectPressed) {
        if (eventNum == 1 || eventNum == 2) {
          screen = "settings_editRange";
          tft.fillRect(0, 20, 320, 240 - 20, TFT_BLACK); // clear screen except top notch
        }
        if (eventNum == 3) {
          screen = "settings";
          settingsSel = 4;
          tft.fillRect(0, 20, 320, 240 - 20, TFT_BLACK); // clear screen except top notch
        }
      }
    }

    // Edit manual events screen
    if (screen == "settings_editRange") {
      displayRange();
    }

    // Cooling screen
    if (screen == "settings_cooling" && segNum == 0) {
      // cooling = 0 means shutoff with gas, cooling = 1 means manual shutoff
      coolingScreen(coolingSel);
      readButtons();

      if (upPressed && coolingSel != 1) coolingSel -= 1;
      if (downPressed && coolingSel != 3) coolingSel += 1;
      if (selectPressed) {
        if (coolingSel == 1) cooling = 0; // cooling = false
        if (coolingSel == 2) cooling = 1; // cooling = true
        if (coolingSel == 3) {
          preferences.putInt("cooling", cooling);
          tft.fillRect(0, 20, 320, 240 - 20, TFT_BLACK); // clear screen except top notch
          screen = "settings";
          settingsSel = 4;
        }
      }
    }
    /* */
  }

  //******************************
  // Running the firing program in MANUAL
  if (segNum >= 1 && programMode == "MANUAL") {
    readButtons();
    runningScreen();
    // Update heaters
    fireControl();

    // check if target temperature is reached:
    // if ( pidInput >= max(FAend, FSAend) || upPressed) {
    if (upPressed) {
      fireOff();
    } 
  }

  //******************************
  // Running the firing program in AUTOMATIC
  if (segNum >= 1 && programMode == "AUTOMATIC") {

    readButtons();
    runningScreen();

    // Up arrow button
    if (upPressed) {
      if (screenNum == 1) {
        segNum = segQuantity + 2;
      }
      if (screenNum == 2 || (screenNum == 3 && optionNum == 1)) {
        screenNum = screenNum - 1;
      }
      else if (screenNum == 3 && optionNum >= 2) {
        optionNum = optionNum - 1;
      }
      tft.fillRect(0, 20, 320, 240 - 20, TFT_BLACK); // clear screen except top notch
      //runningScreen();
      // btnBounce(upPin);
    }

    // Down arrow button
    if (downPressed) {
      if (screenNum <= 2) {
        screenNum = screenNum + 1;
      }
      else if (screenNum == 3 && optionNum <= 2) {
        optionNum = optionNum + 1;
      }
      tft.fillRect(0, 20, 320, 240 - 20, TFT_BLACK); // clear screen except top notch
      //runningScreen();
      // btnBounce(downPin);
    }

    // Select / Start button
    if (selectPressed && screenNum == 3) {
      if (optionNum == 1) {  // Add 5 min
        segHold[segNum - 1] = segHold[segNum - 1] + 5;
        optionNum = 1;
        screenNum = 2;
      }

      if (optionNum == 2) {  // Add 5 deg
        segTemp[segNum - 1] = segTemp[segNum - 1] + 5;
        optionNum = 1;
        screenNum = 1;
      }

      if (optionNum == 3) {  // Goto next segment
        segNum = segNum + 1;
        optionNum = 1;
        screenNum = 2;
      }
      tft.fillRect(0, 20, 320, 240 - 20, TFT_BLACK); // clear screen except top notch
      //runningScreen();
      // btnBounce(selectPin);
    }

    // Update PID's / turn on heaters / update segment info
    updatePIDs(); // pid cycle time is already in class
    fireControl();
    updateSeg();
  }

  // Send data through ESP-NOW
  if (millis() - espnowStart >= espnowCycle) {    
    Kiln.temp = pidInput;
    Kiln.fuegoBajo = FB;
    Kiln.fuegoAlto = FA;
    Kiln.fuegoSuperAlto = FSA;
    Kiln.SetPoint = pidSetPoint;
    Kiln.Output = pidOutput; //(int)pidOutput
    
    esp_err_t result = esp_now_send(ESP32_3, (uint8_t *) &Kiln, sizeof(Kiln));
    if (result != ESP_OK) Serial.println("Error sending the data");
    
    espnowStart = millis();
  }

  // Check if ESP #2 and #3 have sent messages in time
  if (millis() - ESP32_2_msgTime >= EspNowTimeOut)  ESP32_2_isOK = false;
  else                                              ESP32_2_isOK = true;   
  if (millis() - ESP32_3_msgTime >= EspNowTimeOut)  ESP32_3_isOK = false;
  else                                              ESP32_3_isOK = true;
  
}

//******************************************************************************************************************************
//  FIRECONTROL: TURN FB, FA & FSA VALVES ON/OFF
//******************************************************************************************************************************
void fireControl() {
  static bool soaking = false;
  FB = HIGH; // MUST ALWAYS HAVE FUEGO BAJO
  
  if (programMode == "AUTOMATIC" || soaking) {
    if (soaking) {
      // Update the PID controller based on new variables
      pidCont.Compute();
      // Only do 5 minutes
      if ( millis() - startTime >= 5*60*1000) { 
        startTime = 0;
        soaking = false;
        fireOff();
      }
    }

    if (pidOutput >= 70) {
      FSA = HIGH;   // fuego super alto ON
      FA = HIGH;    // fuego alto ON
    }
    if (pidOutput < 70 && pidOutput >= 30) {
      FSA = LOW;    // fuego super alto OFF
      FA = HIGH;    // fuego alto ON
    }
    if (pidOutput < 30) {
      FSA = LOW;    // fuego super alto OFF
      FA = LOW;     // fuego alto OFF
    }
  }

  if (programMode == "MANUAL" && !soaking) {
    // temporary: start timer when temp is reached
    if (pidInput >= max(FAend, FSAend)) {
      startTime = millis();
      // start soaking at SV = PV
      soaking = true;
      setupPIDs(HIGH);
      pidSetPoint = pidInput;
    }

    // not soaking
    else {
      // Fuego alto
      if (pidInput >= FAstart && pidInput <= FAend)  FA = HIGH;
      else FA = LOW;
      // Fuego super alto
      if (pidInput >= FSAstart && pidInput <= FSAend) FSA = HIGH;
      else FSA = LOW;
    }
  }

  //shift_register.set(gasPin, HIGH);  // gas contactor still energized
  shift_register.set(fbPin, FB);     // goes to SSR board and then to FB valve
  shift_register.set(faPin, FA);     // goes to SSR board and then to FA valve
  shift_register.set(fsaPin, FSA);   // goes to SSR board and then to FSA valve
}

//******************************************************************************************************************************
//  INTROSCREEN: UPDATE TFT WITH INTRO MENU
//******************************************************************************************************************************
void introScreen(int sel) {
  tft.setCursor(10, 70); tft.setTextColor(TFT_WHITE, TFT_BLACK); tft.setTextSize(2);
  tft.print(F("pv")); tft.setCursor(60, 60); tft.setTextSize(8);
  tft.printf("%04d%c", (int)pidInput, tempScale);
  tft.setTextSize(3);

  if (sel == 1) {
    tftPrintCenterWidth("> START <", 140);
    tftPrintCenterWidth("  SETTINGS  ", 180);
  }
  if (sel == 2) {
    tftPrintCenterWidth("  START  ", 140);
    tftPrintCenterWidth("> SETTINGS <", 180);
  }
}

//******************************************************************************************************************************
//  SETTINGSSCREEN: UPDATE TFT WITH SETTINGS MENU
//******************************************************************************************************************************
void settingsScreen(int sel) {
  char* option1 = "  SELECT MODE  ";
  char* option2;
  char* option3 = "  COOLING  ";
  char* option4 = "  DONE  ";

  if ( programMode == "AUTOMATIC" ) {
    option2 = "  SELECT PROGRAM  ";
  }
  else {
    option2 = "  SELECT EVENTS  ";
  }

  switch (sel) {
    case 1:
      option1 = "> SELECT MODE <";
      break;
    case 2:
      if ( programMode == "AUTOMATIC" ) {
        option2 = "> SELECT PROGRAM <";
      }
      else {
        option2 = "> SELECT EVENTS <";
      }
      break;
    case 3:
      option3 = "> COOLING <";
      break;
    case 4:
      option4 = "> DONE <";
      break;
  }

  tft.setTextColor(TFT_WHITE, TFT_BLACK); tft.setTextSize(3);
  tftPrintCenterWidth("SETTINGS", 40); tft.setTextSize(2);
  tftPrintCenterWidth(option1, 100);
  tftPrintCenterWidth(option2, 130);
  tftPrintCenterWidth(option3, 160);
  tft.setCursor(220, 200);
  tft.print(F(option4));
}

//******************************************************************************************************************************
//  MODESCREEN: UPDATE TFT WHEN SELECTING FIRING MODE
//******************************************************************************************************************************
void modeScreen(int sel) {
  tft.setTextColor(TFT_WHITE, TFT_BLACK); tft.setTextSize(3);
  tftPrintCenterWidth("SELECT MODE", 40);
  switch (sel) {
    case 1:
      tftPrintCenterWidth("> AUTOMATIC <", 100);
      tftPrintCenterWidth("  MANUAL  ", 140);
      break;
    case 2:
      tftPrintCenterWidth("  AUTOMATIC  ", 100);
      tftPrintCenterWidth("> MANUAL <", 140);
      break;
  }
}

//******************************************************************************************************************************
//  MANUALSETUP: SETUP A MANUAL FIRING PROGRAM
//******************************************************************************************************************************
void manualSetup(int sel) {
  char text1[30];
  char text2[30];
  char* text3 = "  DONE  ";
  switch (sel) {
    case 1:
      sprintf(text1, "> FA range:  %i-%i %c", int(FAstart), int(FAend), tempScale);
      sprintf(text2, "  FSA range: %i-%i %c", int(FSAstart), int(FSAend), tempScale);
      break;
    case 2:
      sprintf(text1, "  FA range:  %i-%i %c", int(FAstart), int(FAend), tempScale);
      sprintf(text2, "> FSA range: %i-%i %c", int(FSAstart), int(FSAend), tempScale);
      break;
    case 3:
      sprintf(text1, "  FA range:  %i-%i %c", int(FAstart), int(FAend), tempScale);
      sprintf(text2, "  FSA range: %i-%i %c", int(FSAstart), int(FSAend), tempScale);
      text3 = "> DONE <";
      break;
  }

  // Display on the screen
  tft.setTextColor(TFT_WHITE, TFT_BLACK); tft.setTextSize(3);
  tftPrintCenterWidth("MANUAL SETUP", 40); tft.setTextSize(2);
  tftPrint(text1, 20, 100);
  tftPrint(text2, 20, 130);
  tftPrint(text3, 220, 200);
}

//******************************************************************************************************************************
//  DISPLAYRANGE: DISPLAYS EVENT RANGE
//******************************************************************************************************************************
void displayRange() {
  int Tstart;
  int Tend;
  int change;
  char text1[12]; //tstart
  char text2[12]; //tend
  char* text3 = "  DONE  ";

  readButtons();

  if (selectPressed) {
    if (rangeOpt == 1 || rangeOpt == 2) {
      adjustRange = !adjustRange;  // enter/exit adjust
    }
    if (rangeOpt == 3) {
      rangeScreen = false;
      rangeOpt = 1;
      // save ranges to EEPROM
      preferences.putInt("FAstart", FAstart); preferences.putInt("FAend", FAend);
      preferences.putInt("FSAstart", FSAstart); preferences.putInt("FSAend", FSAend);
      tft.fillRect(0, 20, 320, 240 - 20, TFT_BLACK); // clear screen except top notch
      screen = "settings_events";
      return;
    }
  }

  if (adjustRange == false) {
    if (upPressed && rangeOpt > 1) {
      rangeOpt -= 1;
    }
    if (downPressed && rangeOpt < 3) {
      rangeOpt += 1;
    }
  }

  if (adjustRange == true) {
    change = 0;
    if (upPressed) {
      change = 1;
    }
    if (downPressed) {
      change = -1;
    }

    if (eventNum == 1) { // FA
      if (rangeOpt == 1) {
        FAstart += change;
      }
      if (rangeOpt == 2) {
        FAend += change;
      }
    }
    else { // FSA
      if (rangeOpt == 1) {
        FSAstart += change;
      }
      if (rangeOpt == 2) {
        FSAend += change;
      }
    }
  }

  tft.setTextColor(TFT_WHITE, TFT_BLACK); tft.setTextSize(3);
  switch (eventNum) {
    case 1:
      Tstart = FAstart;
      Tend = FAend;
      tftPrint("FA RANGE", 20, 40);
      break;
    case 2:
      Tstart = FSAstart;
      Tend = FSAend;
      tftPrint("FSA RANGE", 20, 40);
      break;
  }

  switch (rangeOpt) {
    case 1:
      sprintf(text1, "> %04d%c <", Tstart, tempScale);
      sprintf(text2, "  %04d%c  ", Tend, tempScale);
      break;
    case 2:
      sprintf(text1, "  %04d%c  ", Tstart, tempScale);
      sprintf(text2, "> %04d%c <", Tend, tempScale);
      break;
    case 3:
      sprintf(text1, "  %04d%c  ", Tstart, tempScale);
      sprintf(text2, "  %04d%c  ", Tend, tempScale);
      break;
  }

  tft.setTextSize(2);
  if (adjustRange == true && rangeOpt == 1) {
    tft.setTextColor(TFT_ORANGE, TFT_BLACK);
  }
  tftPrint(text1, 150, 100);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  if (adjustRange == true && rangeOpt == 2) {
    tft.setTextColor(TFT_ORANGE, TFT_BLACK);
  }
  tftPrint(text2, 150, 130);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tftPrint("start", 20, 100);
  tftPrint("end", 20, 130);

  if (rangeOpt == 3) {
    text3 = "> DONE <" ;
  }
  tftPrint(text3, 220, 200);
}

//******************************************************************************************************************************
//  coolingScreen: DISPLAYS COOLING MODE
//******************************************************************************************************************************
void coolingScreen(int coolingSel) {
  char* text1 = "  Shutoff with gas  ";
  char* text2 = "  Shutoff manually  ";
  char* text3 = "  DONE  ";
  
  tft.setTextColor(TFT_WHITE, TFT_BLACK); tft.setTextSize(2);
  tftPrint("Cooling shutoff method",2,40);

  switch (coolingSel) {
    case 1:
      text1 = "> Shutoff with gas <";
      break;
    case 2:
      text2 = "> Shutoff manually <";
      break;
    case 3:
      text3 = "> DONE <";
      break;
  }
  
  if (cooling == 0) {
    tft.setTextColor(TFT_ORANGE, TFT_BLACK);
    tftPrintCenterWidth(text1, 100);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tftPrintCenterWidth(text2, 130);
  }
  if (cooling == 1) {
    tftPrintCenterWidth(text1, 100);
    tft.setTextColor(TFT_ORANGE, TFT_BLACK);
    tftPrintCenterWidth(text2, 130);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
  }
  tftPrint(text3, 220, 200);  
  
  // Serial.printf("cooling sel %d\n",coolingSel);
  // Serial.printf("cooling %d\n",cooling);
  // Serial.printf("text1 %s\n",text1);
  // Serial.printf("text2 %s\n",text2);
}

//******************************************************************************************************************************
//  runningScreen: TFT SCREEN WHEN RUNNING
//******************************************************************************************************************************
void runningScreen() {
  //  tft.fillRect(0, 20, 320, 240-20, TFT_BLACK); // clear screen except top notch
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);

  // Operator screen (temperature and drying burners pressure)
  if (screenNum == 1) {
    tft.setCursor(30, 30);
    tft.print(F("pv"));   
    if (programMode == "AUTOMATIC") {
      tft.setCursor(10, 140); tft.print(F("sv"));
      tft.setCursor(60, 160); tft.setTextSize(7); 
      tft.printf("%04d%c", (int)pidSetPoint, tempScale);
    }    
    tft.setCursor(80, 40); tft.setTextSize(7);
    tft.printf("%04d%c", (int)pidInput, tempScale);

    if (programMode == "MANUAL") {
      tft.setTextSize(2);

      // Gas
      float NEG = abs(DPT_E.p[0]);
      float SWG = abs(DPT_W.p[2]);
      tftPrintF("NEG: %.2f ", NEG, 20, 120);
      tftPrintF("SWG: %.2f ", SWG, 180, 120);
      // Air
      float NEA = abs(DPT_E.p[1]);
      float NWA = abs(DPT_W.p[1]);
      float SEA = abs(DPT_E.p[3]);
      float SWA = abs(DPT_W.p[3]);
      tftPrintF("SEA: %.2f ", SEA, 20, 185);
      tftPrintF("SWA: %.2f ", SWA, 180, 185);
      tftPrintF("NEA: %.2f ", NEA, 20, 205);
      tftPrintF("NWA: %.2f ", NWA, 180, 205);
    }
  }
  
  // Info screen (or range screen)
  if (screenNum == 2) {
    //if (programMode == "MANUAL" && rangeScreen == false) {
    //  manualSetup();
    //}
    if (programMode == "AUTOMATIC") {
      tft.setCursor(0, 30);
      tft.printf("PROGRAM %i: \n\n%s", programNumber, programDesc1);
      tft.setCursor(0, 100);
      tft.printf("SEGMENT:%i / %i", segNum, segQuantity);
      if (isOnHold == 0) {
        tft.setCursor(120, 140);
        tft.printf("Ramp to %i%c", segTemp[segNum - 1], tempScale);
        tft.setCursor(120, 160);
        tft.printf("at %i%c/hr", segRamp[segNum - 1], tempScale);
      }
      else {
        tft.setCursor(120, 120);
        tft.printf("Hold at %i%c \n", segTemp[segNum - 1], tempScale);
        tft.setCursor(120, 140);
        tft.printf("for %i / %i min", (millis() - holdStart) / 60000, segHold[segNum - 1]);
      }
    }
  }

  // Tools screen for Automatic mode
  if (screenNum == 3 && programMode == "AUTOMATIC") {
    tft.setCursor(80, 30);
    tft.print(F("TOOLS:"));
    tft.setCursor(80, 80);
    tft.print(F("Add 5 min"));
    tft.setCursor(80, 100);
    tft.print(F("Increase 5 deg"));
    tft.setCursor(80, 120);
    tft.print(F("Skip to next seg"));
    tft.setCursor(0, optionNum * 20 + 60);
    tft.print(F(">"));
    tft.setCursor(300, optionNum * 20 + 60);
    tft.print(F("<"));
  }
}

//******************************************************************************************************************************
//  SETUPPIDS: INITIALIZE THE PID LOOPS
//******************************************************************************************************************************
void setupPIDs(int state) {
  pidCont.SetSampleTime(pidCycle);
  pidCont.SetOutputLimits(0, 100); // from 0%-100%
  pidOutput = 0; // output should start low because initialization means new firing
  if (state == HIGH) pidCont.SetMode(AUTOMATIC);
  if (state == LOW) pidCont.SetMode(MANUAL);
}

//******************************************************************************************************************************
//  UPDATEPIDS: UPDATE THE PID LOOPS
//******************************************************************************************************************************
void updatePIDs() {
  // Get the last target temperature
  if (segNum != 1) lastTemp = segTemp[segNum - 2];
  // Calculate the new setpoint value.  Don't set above / below target temp
  if (isOnHold == false) {
    // Ramp: measure spanned t and calculate the SP with it
    rampHours = (millis() - rampStart) / 3600000.0;
    calcSetPoint = lastTemp + (segRamp[segNum - 1] * rampHours);  
    // fix SP to target temp in case it's more than target temp
    if (segRamp[segNum - 1] >= 0 && calcSetPoint >= segTemp[segNum - 1]) {
      calcSetPoint = segTemp[segNum - 1];
    }
    if (segRamp[segNum - 1] < 0 && calcSetPoint <= segTemp[segNum - 1]) {
      calcSetPoint = segTemp[segNum - 1];
    }
  }
  else {
    calcSetPoint = segTemp[segNum - 1];  // Hold
  }
  // Set the target temp.
  pidSetPoint = calcSetPoint;
  // Update the PID controller based on new variables
  pidCont.Compute();
}

//******************************************************************************************************************************
//  UPDATESEG: UPDATE THE PHASE AND SEGMENT
//******************************************************************************************************************************
void updateSeg() {
  // Start the hold phase if temp is in range
  if ((isOnHold == false && segRamp[segNum - 1] < 0 && pidInput <= (segTemp[segNum - 1] + tempRange)) || // if ramp is negative
      (isOnHold == false && segRamp[segNum - 1] >= 0 && pidInput >= (segTemp[segNum - 1] - tempRange))) { // if ramp is positive
    isOnHold = true;
    holdStart = millis();
  }
  // Go to the next segment
  if (isOnHold == true && millis() - holdStart >= segHold[segNum - 1] * 60000) {
    segNum = segNum + 1;
    isOnHold = false;
    rampStart = millis();
  }
  // Check if complete: turn off fire, PIDs and go to intro screen
  if (segNum - 1 > segQuantity) {
    fireOff();
  }
}

//******************************************************************************************************************************
//  FIREOFF: EXTINGUISH KILN FIRE
//******************************************************************************************************************************
void fireOff() {
  // Turn off gas contactor
  shift_register.set(gasPin, LOW);
   
  // Turn off all gas valves
  FB = LOW, FA = LOW, FSA = LOW;
  shift_register.set(fbPin, FB);
  shift_register.set(faPin, FA);
  shift_register.set(fsaPin, FSA);
  
  // Turn of PID if needed
  if (programMode == "AUTOMATIC") {
    setupPIDs(LOW);
  }

  // Reset display and logic
  segNum = 0;
  screen = "intro";
  resetTFT(); // fight EMI
  tftPrintCenterWidth("Se apago", 220);

  // If not meant to cool, turn off the air
  if (!cooling) {
    shift_register.set(airPin, LOW);
    delay(1500);
    shift_register.set(airPin, HIGH); // equivalent to pressing normally closed OFF button
  }
  
  // Open chimney
  chimney.OPEN = true;
  esp_now_send(ESP32_2, (uint8_t *) &chimney, sizeof(chimney));
  delay(500);
  esp_now_send(ESP32_2, (uint8_t *) &chimney, sizeof(chimney));
  delay(500);
  esp_now_send(ESP32_2, (uint8_t *) &chimney, sizeof(chimney));
  delay(1000);
  chimney.OPEN = false;
  esp_err_t result = esp_now_send(ESP32_2, (uint8_t *) &chimney, sizeof(chimney));
  if (result != ESP_OK) Serial.println("Error sending the data");
}

//******************************************************************************************************************************
//  READTEMPS: Read the temperatures
//******************************************************************************************************************************
void readTemps() {
  float t;
  t = thermocouple.readThermocoupleTemperature();
  //t = thermocouple.readCelsius();
  if (tempScale == 'F') {
    t = 9 / 5 * t + 32;
  }

  // if there's an error
  uint8_t fault = thermocouple.readFault();
  if (isnan(t) || fault & MAX31856_FAULT_OPEN) {
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_RED, TFT_BLACK); tft.setTextSize(4);
    tftPrintCenterWidth("ERROR", 80);
    tft.setTextColor(TFT_WHITE, TFT_BLACK); tft.setTextSize(2);
    tftPrintCenterWidth("thermocouple is NaN", 150);
    tftPrintCenterWidth("System was shut down.", 180);
    fireOff();
  }

  // filter nonsense
  if (t < 5000 && t > 0) {
    pidInput = t + tempOffset;
  }
}

//******************************************************************************************************************************
//  ONDATARECV: CALLBACK FUNCTION THAT WILL BE EXECTUED WHEN DATA IS RECEIVED
//******************************************************************************************************************************
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  // Get mac address from sender
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  // memcpy data to appropiate struct
  if (memcmp(mac_addr, ESP32_2, sizeof(ESP32_2)) == 0) {
    Serial.printf("\n Packet received from: %s,   AKA ESP32 #2\n", macStr);
    memcpy(&DPT_E, incomingData, sizeof(DPT_E));
    Serial.printf("\n Message length: %u bytes\n", len);   
    Serial.printf("\n Presiones recibidas: \n NEG: %.2f in. WC \n NEA: %.2f in. WC \n SEG: %.2f in. WC \n SEA: %.2f in. WC \n",
    DPT_E.p[0], DPT_E.p[1], DPT_E.p[2], DPT_E.p[3]);
    ESP32_2_msgTime = millis();
  }
  if (memcmp(mac_addr, ESP32_3, sizeof(ESP32_3)) == 0) {
    Serial.printf("\n Packet received from: %s,   AKA ESP32 #3\n", macStr);
    memcpy(&DPT_W, incomingData, sizeof(DPT_W));
    Serial.printf("\n Message length: %u bytes\n", len);  
    Serial.printf("\n Presiones recibidas: \n NWG: %.2f in. WC \n NWA: %.2f in. WC \n SWG: %.2f in. WC \n SWA: %.2f in. WC \n",
    DPT_W.p[0], DPT_W.p[1], DPT_W.p[2], DPT_W.p[3]);
    ESP32_3_msgTime = millis();
  }
}

//******************************************************************************************************************************
//  ONDATASENT: CALLBACK FUNCTION WHEN DATA IS SENT
//******************************************************************************************************************************
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Get mac address from receiver
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);         
  Serial.printf("\n Packet sent to: %s send status:\t %s", macStr, status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail" );
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

//******************************************************************************************************************************
//  OPENPROGRAM: OPEN AND LOAD A FIRING PROGRAM FILE / DISPLAY ON SCREEN
//******************************************************************************************************************************
void openProgram() {

  // Setup all variables
  int col = 1;          // Column number (of text file).  First column is one.
  int row = 1;          // Row number (of text file).  First row is one.
  char tempChar;        // Temporary character holder (read one at a time from file)
  char tempLine[21];    // Temporary character array holder
  int tempLoc = 0;      // Current location of next character to place in tempLine array
  char programDesc2[21];  // Program description #2 (second line of text file)
  char programDesc3[21];  // Program description #3 (third line of text file)

  // Clear the arrays
  memset(programDesc1, 0, sizeof(programDesc1));
  memset(segRamp, 0, sizeof(segRamp));
  memset(segTemp, 0, sizeof(segTemp));
  memset(segHold, 0, sizeof(segHold));

  // Make sure you can open the file
  sprintf(tempLine, "/%d.txt", programNumber);
  File myFile = SD.open(tempLine, FILE_READ);

  if (myFile == false) {
    tft.setCursor(20, 40);
    tft.setTextColor(TFT_WHITE, TFT_BLACK); tft.setTextSize(2);
    tft.print(F("SELECT PROGRAM: "));
    tft.println(programNumber); tft.setCursor(20, 60);
    tft.print(F("Can't find/open file"));
    programOK = false;
    return;
  }

  // Load the data
  while (myFile.available() > 0) {

    // Read a single character
    tempChar = myFile.read();

    if (tempChar == 13) {       // Carriage return: Read another char (it is always a line feed / 10).  Add null to end.
      myFile.read();
      tempLine[tempLoc] = '\0';
    }
    else if (tempChar == 44) {  // Comma: Add null to end.
      tempLine[tempLoc] = '\0';
    }
    else if (tempLoc <= 19) {   // Add it to the temp line array
      tempLine[tempLoc] = tempChar;
      tempLoc = tempLoc + 1;
    }

    if (row == 1 && tempChar == 13) {
      memcpy(programDesc1, tempLine, 21);
    }
    else if (row == 2 && tempChar == 13) {
      memcpy(programDesc2, tempLine, 21);
    }
    else if (row == 3 && tempChar == 13) {
      memcpy(programDesc3, tempLine, 21);
    }
    else if (row >= 4 && col == 1 && tempChar == 44) {
      segRamp[row - 4] = atoi(tempLine);
    }
    else if (row >= 4 && col == 2 && tempChar == 44) {
      segTemp[row - 4] = atoi(tempLine);
    }
    else if ((row >= 4 && col == 3 && tempChar == 13) || myFile.available() == 0) {
      segHold[row - 4] = atoi(tempLine);
    }

    if (tempChar == 13) {  // End of line.  Reset everything and goto next line
      memset(tempLine, 0, 21);
      tempLoc = 0;
      row = row + 1;
      col = 1;
    }

    if (tempChar == 44) {  // Comma.  Reset everything and goto 1st column
      memset(tempLine, 0, 21);
      tempLoc = 0;
      col = col + 1;
    }

  }  // end of while(myFile.available ...

  // Close the file
  myFile.close();

  // Set some variables
  segQuantity = row - 3;
  programOK = true;

  // Fix Ramp values so it will show the correct sign (+/-).  This will help to determine when to start hold.
  for (int i = 0; i < segQuantity; i++) {
    segRamp[i] = abs(segRamp[i]);
    if (i >= 1) {
      if (segTemp[i] < segTemp[i - 1]) {
        segRamp[i] = -segRamp[i];
      }
    }
  }

  // Display on the screen
  tft.setCursor(20, 40); tft.setTextColor(TFT_WHITE, TFT_BLACK); tft.setTextSize(2);
  tft.printf("SELECT PROGRAM: %i \n\n %s \n\n %s \n\n %s", programNumber, programDesc1, programDesc2, programDesc3);
}

//******************************************************************************************************************************
//  DRAWTOPBAR: DRAW ESPNOW STATES AND FIRING MODE
//******************************************************************************************************************************
void drawTopBar() {
  int centerY = 10;
  tft.fillRect(0, 0, 320, 20, bar_color);  // clear top notch
  tft.setTextSize(1);

  // Draw program mode in top right corner
  tft.setTextColor(TFT_GREEN, bar_color);
  tft.drawString(programMode, 260, centerY, 1);

  // Draw check mark or cross
  if (ESP32_2_isOK && ESP32_3_isOK) {
    tft.drawLine(16, centerY + 1, 19, centerY + 4, TFT_GREEN);  // Draw the first diagonal line
    tft.drawLine(19, centerY + 3, 23, centerY + 0, TFT_GREEN);  // Draw the second diagonal line
    tft.drawLine(16, centerY + 2, 19, centerY + 5, TFT_GREEN);  // Draw the first inner diagonal line
    tft.drawLine(19, centerY + 4, 23, centerY + 1, TFT_GREEN);  // Draw the second inner diagonal line
    tft.drawLine(17, centerY + 1, 20, centerY + 4, TFT_GREEN);  // Draw the first outer diagonal line
    tft.drawLine(20, centerY + 3, 24, centerY + 0, TFT_GREEN);  // Draw the second outer diagonal line
  } else {
    tft.drawLine(17, centerY - 1, 23, centerY + 5, TFT_RED);  // Draw the first diagonal line
    tft.drawLine(23, centerY - 1, 17, centerY + 5, TFT_RED);  // Draw the second diagonal line
    // check who failed
    String whoFailed = "";
    if (!ESP32_2_isOK) whoFailed += "| ESP#2 |";
    if (!ESP32_3_isOK) whoFailed += "| ESP#3 |";
    tft.drawString(whoFailed, 35, centerY, 1);
  }
}

//******************************************************************************************************************************
//  INITESPNOW: INITIALIZE ESPNOW.
//******************************************************************************************************************************
void initEspNow() {
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP NOW failed to initialize");
    while (1);
  }
  peerInfo.ifidx   = WIFI_IF_STA;
  peerInfo.encrypt = false;

  // Add first peer (ESP32 #2)
  memcpy(peerInfo.peer_addr, ESP32_2, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("ESP #2 pairing failure");
    while (1);
  }
  else Serial.println("ESP #2 pairing success");
  
  // Add second peer (ESP32 #3)
  memcpy(peerInfo.peer_addr, ESP32_3, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("ESP #3 pairing failure");
    while (1);
  }
  else Serial.println("ESP #2 pairing success");
  
  // ***** Register Send CB and Recv CB *****
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
}

//******************************************************************************************************************************
//  INITWIFI: INITIALIZE WIFI.
//******************************************************************************************************************************
void initWiFi() {
  WiFi.mode(WIFI_STA);
  int32_t channel = getWiFiChannel(WIFI_SSID);
  // WiFi.printDiag(Serial); Uncomment to verify channel change before
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  // WiFi.printDiag(Serial); Uncomment to verify channel change after
}

//******************************************************************************************************************************
//  GETWIFICHANNEL: CHANNEL # FOR espNOW
//******************************************************************************************************************************
int32_t getWiFiChannel(const char *ssid) {
  if (int32_t n = WiFi.scanNetworks()) {
    for (uint8_t i = 0; i < n; i++) {
      if (!strcmp(ssid, WiFi.SSID(i).c_str())) {
        return WiFi.channel(i);
      }
    }
  }
  return 0;
}

//******************************************************************************************************************************
//  READBUTTONS: READ IF BUTTONS ARE PRESSED
//******************************************************************************************************************************
void readButtons() {
  upPressed = false;
  selectPressed = false;
  downPressed = false;

  if (digitalRead(upPin) == LOW) {
    upPressed = true;
    btnBounce(upPin);
  }
  if (digitalRead(selectPin) == LOW) {
    selectPressed = true;
    btnBounce(selectPin);
  }
  if (digitalRead(downPin) == LOW) {
    downPressed = true;
    btnBounce(downPin);
  }
}

//******************************************************************************************************************************
//  BTNBOUNCE: HOLD UNTIL BUTTON IS RELEASED.  DELAY FOR ANY BOUNCE
//******************************************************************************************************************************
void btnBounce(int btnPin) {
  //while (digitalRead(btnPin) == LOW);
  delay(150);
}

//******************************************************************************************************************************
//  RESETTFT: RESETS TFT WHEN CONTACTORS ARE OPENED (EMF) OR USER CALLS IT
//******************************************************************************************************************************
void resetTFT() {
  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);
}

//******************************************************************************************************************************
//  TFTPRINTCENTERWIDTH: CENTERS CURSOR ON WIDTH AND PRINTS
//******************************************************************************************************************************
void tftPrintCenterWidth(char* text, int y) {
  tft.setCursor((tftwidth - tft.textWidth(text)) / 2, y);
  tft.print(F(text));
}

//******************************************************************************************************************************
//  TFTPRINT: SETS CURSOR ON (X,Y) AND PRINTS
//******************************************************************************************************************************
void tftPrint(char* text, int x, int y) {
  tft.setCursor(x, y);
  tft.print(F(text));
}

// TFTPRINTF: SETS CURSOR ON (X,Y) AND PRINTS FLOAT VALUE
void tftPrintF(const char* format, float value, int x, int y) {
  char buf[64]; // Buffer to hold the formatted string. Adjust the size as needed.
  snprintf(buf, sizeof(buf), format, value); // safely format the string
  tft.setCursor(x, y);
  tft.print(F(buf));
}
