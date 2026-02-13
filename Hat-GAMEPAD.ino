/*
  --------------------------------------------------------------------------------------------------------------------------------
  ==== M5-Stack Core Basic - Bluetooth Hat-TRACKER-GAMEPAD (HAT-GAMEPAD) - with the ICM-20948 IMU Sensor with DMP support ====
  --------------------------------------------------------------------------------------------------------------------------------
  Hardware: ESP32: M5Stack Core Basic & Sensor: ICM-20948 (on sparkfun breakout board) - Quat9 (Gyro+Acc+Magn) => X,Y,Z will be sent out
  Project descripton: A simple wireless Hat-Tracker-Device (GAMEPAD version) for BluetoothLE to be attatched on your head
  to control yaw/nick/roll movements over an emulated USB-Gamecontroller-HID device for your PC. It is designed to control your
  PointOfView in a RC-Simulators for planes, helis, drones drones to get rid of the non resalitic auto-follow camera.
  There is a Hid-mouse verion too if you prefer a mouse over a joystick input but with joystick. But you are defenitely better with
  this gamepad-version because its faster, sends RX/RY/RZ absolute values where a mouse only sends X/Y deleta-move values, and
  Gamepad is and feels more accurate and "directly".

  Sends 3 axis (RX,RY,RZ) Has a display: 320*240px, 10s screen-off-delay when BT-connected, 2x yellow/blue StatusLed with a NeoPixelBar
  Menu-Buttons for: [B]=lock the current view, [C]=toggle Tilt-Lock, long-press[C]=re-center sensor
  You can turn-off the device by long pressing [A] or with the integrated M5-PowerButton or whait 6min with BT-connection off.
  The sensor technically takes up 5-30sek to warm-up and stop drifting, escpecially when its not facing a default position like vertical.
  Use (for example) OpenTrack and map the Joystick as Input and FreeTrack2.0/TrackIR as Output Protocoll for you SIM - depends on your setup.
                                                                                        
                                                                                          @by frittna - 2.Feb 2026 Arduino IDE 1.8.19
                                                        BUG: kein speichen mehr der gyro-bias werte, wieder alles rausgenommen, trotz
                                                        speichern und auslesen/setzen bringt es keine verbesserung und er nimmt 
                                                        außer actual.Gyro nix an? actual.Accel und .CPass Werte liefern immer 0!?                                                        

      ??? DEBUG SERIAL OUTPUT IS ON/OFF ???
*/

// bool debug = true; // DEBUG SERIAL OUTPUT ON
bool debug = false;  // DEBUG SERIAL OUTPUT OFF

//IMPORTANT on the old M5STACK CORE and Ardiono 1.8 (not 2.0): Boardmanager "espressif systems - eps32" -> version "2.0.17" (!)
//                                                             Menu->Tools->Board-> "ESP32 Arduino" -> "M5-Stack-Core-ESP32"

// included Libraries
#include <M5Stack.h>
#include <esp_system.h>   // Notwendig für MAC-Adressen um konfikte mit Maus-Version zu vermeiden
#include <Preferences.h>  // Alle Werte im Speicher kann man auch löschen: Bei Boot ButtonB halten!
#include "ICM_20948.h"
#include <BleGamepad.h>         //Bibliothek "ESP32 BLE Gamepad"
#include "M5StackUpdater.h"     // SD-Menu Updater (optional, SD-Card App-Wechsel-Menü) use Ver 0.5.2!!
#include "Adafruit_NeoPixel.h"  //Bibliothek LEDbar (optional, Status LED gelb/blau)

// Definition für die LEDbar (Anzahl Pixel, Pin-Nummer)
#define LED_PIN 15    // Standard-Pin für die LEDbar beim M5Stack Core
#define LED_COUNT 10  // Gesamtzahl der LEDs, du nutzt 2 (4 & 5)
Adafruit_NeoPixel LEDbar(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// Konstanten für Gamepad-Achsenbereich
const int16_t GAMEPAD_AXIS_MIN = 0;
const int16_t GAMEPAD_AXIS_MAX = 32767;
const int16_t GAMEPAD_AXIS_CENTER = 16384;

ICM_20948_I2C myICM;
BleGamepad bleGamepad("M5STACK Hat-GAMEPAD", "Frittna", 100);
BleGamepadConfiguration bleGamepadConfig;
Preferences prefs;

// Input-Konfiguration
int sensitivity = 152;          //sets the overall movement speed multiplier - 152 is perfect for me (180°view)
float precissionFactor = 0.65;  // if precisionActive, reduces movement speed, done by a multiplier
float tilt = 0.75;              // closer to 1 will tilt sooner
bool precisionActive = false;
bool tiltLockActive = false;  //nicht mehr In Verwendung, um Taste A freizubekommen (A spinnt mit kurzen ghost inputs)
bool excessiveTilt;
bool holdPositonActive;

// Invert X or Y or Z axis before output
bool invertXAxis = true;
bool invertYAxis = false;
bool invertZAxis = false;

// Offsets für das Nullsetzen
float calOffsetX = 0.0;
float calOffsetY = 0.0;
float calOffsetZ = 0.0;

// Warm-Up Management
bool isWarmingUp = true;
const unsigned long WARM_UP_TIME = 10000;     // eigentlich braucht es bis zu 30sekunden Wartezeit damit der Drift zu 100%
const unsigned long QUICK_RESET_TIME = 1000;  // verschwindet, was etwas unpraktisch ist, also hinweis auf re-calibigrierung.
bool it_is_a_QuickReset;
unsigned long calibrationTime = 0;

// ! Drift Kompensation entfernt !  bringt mir nichts, es taugt nur eine deutlich längere aufwärmzeit etwas um nicht zu driften
//float driftOffsetY = 0.0;
//int startupSamples = 0;
//const int CALIBRATION_SAMPLES = 100;

// Power Management
byte lcd_brightn = 50;     // default LCD-Brighness (50 is enought for my M5Stack Core Basic)
byte led_brightness = 10;  // NeoPixel LEDs brightness (10 is enouth for me)
bool displayOn = true;
unsigned long lastActivity = 0;
const unsigned long DISPLAY_TIMEOUT = 10000;     // Display-Timeout 10 Sek.
const unsigned long POWER_OFF_TIMEOUT = 360000;  // 6 Minuten

// Small GUI state for counter (avoid flicker by updating only when seconds change)
int prevScreenOffSec = -1;

// If true, attempt to restart advertising on disconnect (helps many reconnection failures)
const bool RESTART_BLE_ON_DISCONNECT = true;

// Bottom-line display state helpers to avoid flicker
int prevBottomState = -1;
int prevBottomX = 99999;
int prevBottomY = 99999;
int prevBottomZ = 99999;
bool prevZeroSent = false;

// Layout constants
const int BOTTOM_X = 0;
const int BOTTOM_Y = 216;
const int BOTTOM_W = 240;
const int BOTTOM_H = 32;
const int BATTERY_X = 240;
const int BATTERY_Y = 216;
const int BATTERY_W = 60;
const int BATTERY_H = 32;

// Variablen für LED Status-Blinken
unsigned long lastLedToggle = 0;
bool ledState = false;
const unsigned long LED_BLINK_INTERVAL_YELLOW = 1000;  // alle 1s blinken
const unsigned long LED_BLINK_INTERVAL_BLUE = 4000;    // alle 4s blinken

// BATTERY ---
void drawBattery() {
  if (!displayOn) return;
  int level = M5.Power.getBatteryLevel();
  uint16_t color = GREEN;
  if (level < 90) color = TFT_GREEN;
  if (level < 70) color = TFT_DARKGREEN;
  if (level < 60) color = TFT_YELLOW;
  if (level < 30) color = TFT_RED;
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.fillRect(BATTERY_X, BATTERY_Y, BATTERY_W, BATTERY_H, BLACK);
  M5.Lcd.setCursor(BATTERY_X + 8, 219);
  M5.Lcd.printf("%3d%%", level);                     // für Batterieanzeige zb "100%", rechtsbündig
  M5.Lcd.fillCircle(BATTERY_X + 71, 226, 8, color);  // grüner bis roter false der batteriespannungs
}

// REFRESH UI ---
void refreshUI() {
  if (!displayOn) return;
  M5.Lcd.clear();
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.setTextColor(TFT_ORANGE);
  M5.Lcd.print("  M5STACK BLE HAT-GAMEPAD  ");
  M5.Lcd.setCursor(0, 22);
  M5.Lcd.println("  =======================  ");
  M5.Lcd.setCursor(0, 45);
  M5.Lcd.setTextColor(BLUE);
  M5.Lcd.printf("   BLUETOOTH: ");
  M5.Lcd.setTextColor(bleGamepad.isConnected() ? GREEN : RED);
  M5.Lcd.printf("%s", bleGamepad.isConnected() ? "CONNECTED" : "WAITING...");
  M5.Lcd.setCursor(0, 75);
  M5.Lcd.setTextColor(precisionActive ? TFT_CYAN : WHITE);
  M5.Lcd.printf("Slow Mode      [B]: %s", precisionActive ? "ON" : "OFF ");
  M5.Lcd.setCursor(0, 105);
  M5.Lcd.setTextColor(holdPositonActive ? YELLOW : WHITE);  // gelb bei aktiv, sonst weiß
  M5.Lcd.printf("Hold Position  [C]: %s", holdPositonActive ? "HOLD " : "OFF");
  //  M5.Lcd.setCursor(0, 135);
  //  M5.Lcd.setTextColor(tiltLockActive ? TFT_CYAN : WHITE);
  //  M5.Lcd.printf("Tilt-Lock Mode [C]: %s", tiltLockActive ? "ON " : "OFF");
  if ((tiltLockActive && excessiveTilt) || holdPositonActive) {
    M5.Lcd.fillRect(BOTTOM_X, BOTTOM_Y, BOTTOM_W, BOTTOM_H, BLACK);
    M5.Lcd.setCursor(BOTTOM_X, BOTTOM_Y + 5);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(YELLOW, BLACK);
    M5.Lcd.print("    => LOCKED <=");
  }
  if (!isWarmingUp) {
    M5.Lcd.setCursor(0, 165);
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.printf("[C] long press = re-center");
  }
  if (isWarmingUp) {
    M5.Lcd.fillRect(0, 160, 320, 40, BLUE);
    M5.Lcd.setCursor(5, 173);
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.println(" * WAIT FOR CALIBRATION *");
  }
  drawBattery();
}

// CALIBRATION ---
void startCalibrationProcess() {
  isWarmingUp = true;
  //startupSamples = 0;
  //driftOffsetY = 0;
  calibrationTime = it_is_a_QuickReset ? (millis() + WARM_UP_TIME - QUICK_RESET_TIME) : millis();
  icm_20948_DMP_data_t data;
  myICM.readDMPdataFromFIFO(&data);
  if ((data.header & DMP_header_bitmap_Quat9) > 0) {
    calOffsetX = ((double)data.Quat9.Data.Q3) / 1073741824.0;
    calOffsetY = ((double)data.Quat9.Data.Q2) / 1073741824.0;
    calOffsetZ = ((double)data.Quat9.Data.Q1) / 1073741824.0;
  }
  prevZeroSent = false;
  prevBottomX = 99999;
  prevBottomY = 99999;
  prevBottomZ = 99999;

  myICM.resetDMP();   // Chip auf neuen Nullpunkt setzen
  myICM.resetFIFO();  // Puffer leeren
  //  if (displayOn) {
  //    M5.Lcd.fillRect(BOTTOM_X, BOTTOM_Y, BOTTOM_W, BOTTOM_H, BLACK);
  //    M5.Lcd.setCursor(BOTTOM_X + 5, BOTTOM_Y + 3);
  //    M5.Lcd.setTextSize(2);
  //    M5.Lcd.setTextColor(WHITE, BLACK);
  //    M5.Lcd.printf("X:%3d Y:%3d Z:%3d", 0, 0, 0);
  //  }
  // Gamepad sendet absolute Werte, keine Delta-Moves. Initial Mitte gesendet
  if (!prevZeroSent && bleGamepad.isConnected()) {
    bleGamepad.setRX(GAMEPAD_AXIS_CENTER);  // Sende Mittelposition (16384)
    bleGamepad.setRY(GAMEPAD_AXIS_CENTER);  // Sende Mittelposition (16384)
    bleGamepad.setRZ(GAMEPAD_AXIS_CENTER);  // Sende Mittelposition (16384)
    prevZeroSent = true;
  }

  prevBottomState = 2;
  prevBottomX = 0;
  prevBottomY = 0;
  prevBottomZ = 0;

  refreshUI();
  if (debug) Serial.printf("WAITING WARM-UP TIME: %lu SECONDS..\n", WARM_UP_TIME / 1000);
}

// SETUP ---
void setup() {
  // MAC-Adresse
  uint8_t gamepad_mac_adr[] = { 0x42, 0xFE, 0xA7, 0x02, 0x0E, 0x12 };  //nutze versch. BT-MAC adressen
  esp_base_mac_addr_set(gamepad_mac_adr);
  //uint8_t mouse_mac_adr[] = {0x32, 0xFA, 0xA7, 0x01, 0x0E, 0x10};   //nutze versch. BT-MAC adressen
  //esp_base_mac_addr_set(mouse_mac_adr);
  Serial.begin(115200);
  M5.begin();
  M5.Power.begin();
  // M5 Setting Power:   see for details: https://github.com/m5stack/m5-docs/blob/master/docs/en/api/power.md
  if (!M5.Power.canControl()) M5.Lcd.printf("IP5306 is not i2c version\n");
  M5.Power.setPowerBtnEn(true);     //allow red power Button
  M5.Power.setPowerBoostSet(true);  //one press on red turns on/off device
  M5.Power.setPowerVin(true);       //reset when usb calbe is plugged in

  M5.lcd.setBrightness(lcd_brightn);
  if (debug) Serial.println("HID-GAMEPAD STARTED..");

  LEDbar.begin();                        // LEDbar initialisieren
  LEDbar.setBrightness(led_brightness);  //Neopixel LEDs
  LEDbar.show();                         // Alle LEDs ausschalten

  // SD-MENU Loader mit Bestätigung
  if (M5.BtnA.isPressed()) {
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setCursor(0, 110);
    M5.Lcd.setTextColor(RED);
    M5.Lcd.setTextSize(2);
    M5.Lcd.println("  Boot SD-MENU-Loader ???");
    M5.Lcd.setCursor(0, 220);
    M5.Lcd.println("           [NO]    [YES]");
    while (true) {
      M5.update();
      if (M5.BtnC.isPressed()) {
        M5.Lcd.fillScreen(BLACK);
        M5.Lcd.setCursor(0, 120);
        M5.Lcd.println("      Loading App...");
        delay(250);
        M5.update();
        updateFromFS(SD);
        ESP.restart();
      }
      if (M5.BtnB.isPressed()) {
        M5.Lcd.fillScreen(BLACK);
        M5.Lcd.setCursor(0, 110);
        M5.Lcd.println(" Canceled - loading App...");
        delay(800);
        M5.update();
        M5.Lcd.fillScreen(BLACK);
        break;  // Zurück zum Hauptprogramm
      }
    }
  }

  // Start-up Logo einblenden
  if (SD.exists("/jpg/Hat-GAMEPAD_logo.jpg")) {  // Datei vorhanden -> Bild zeichnen
    M5.Lcd.drawJpgFile(SD, "/jpg/Hat-GAMEPAD_logo.jpg", 0, 0);
  } else {
    M5.Lcd.setCursor(0, 100);
    M5.Lcd.setTextColor(RED);
    M5.Lcd.setTextSize(2);
    M5.Lcd.print("no Hat-GAMEPAD_logo.jpg ??");
  }
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(20, 210);
  M5.Lcd.setTextColor(BLACK);
  M5.Lcd.print("M5STACK BLE HAT-GAMEPAD");
  delay(1500);
  M5.Lcd.fillRect(0, 0, 320, 240, BLACK);

  // --- Preferences Reset Logik ---
  if (M5.BtnB.isPressed()) {
    prefs.begin("imu_cal_G", false);
    prefs.clear();  // Alle Keys im Namespace löschen
    prefs.end();    // Preferences schließen
    if (debug) Serial.printf("***** internal Memory cleared *****");
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setCursor(0, 100);
    M5.Lcd.setTextColor(RED);
    M5.Lcd.setTextSize(2);
    M5.Lcd.println(" Gyro-bias values reset..");
    M5.Lcd.println();
    M5.Lcd.println(" Please restart device.");
    // Warten, bis der Benutzer den Button loslässt, und dann blockieren,
    // damit das Programm nicht weiterläuft.
    while (M5.BtnC.isPressed()) {
      M5.update();
    }
    // Endlosschleife, damit das Programm stoppt und der Benutzer neu starten muss
    while (true) {
      delay(100);
    }
  }

  // Sensor Bias-Werte aus Speicher laden wenn vorhanden (nur Gyro, derzeit keine Accel und CPass Daten)
  prefs.begin("imu_cal_G", true);  //schreibgeschützt öffnen
  float bX = prefs.getFloat("bX", 0);
  float bY = prefs.getFloat("bY", 0);
  float bZ = prefs.getFloat("bZ", 0);
  prefs.end();  // schließen
  if (debug) {
    Serial.println(">>  BiasGyro Data loaded from Flash:");
    Serial.printf("     X:%f, Y:%f, Z:%f\n\n", bX, bY, bZ);
  }

  Wire.begin(21, 22);
  Wire.setClock(400000);
  myICM.begin(Wire, 1);
  myICM.initializeDMP();
  myICM.enableDMPSensor(INV_ICM20948_SENSOR_ROTATION_VECTOR);  //DOF9
  myICM.setDMPODRrate(DMP_ODR_Reg_Quat9, 0);                   //DOF9
  //myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR); //DOF6
  //myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0);                       //DOF6

  myICM.setBiasGyroX(bX);  //nicht vergessen, laden von speicher offset-werten sind trotzdem deaktiviert !!
  myICM.setBiasGyroY(bY);
  myICM.setBiasGyroZ(bZ);

  myICM.enableFIFO();
  myICM.enableDMP();
  myICM.resetDMP();
  myICM.resetFIFO();

  // BLE-GAMEPAD optionale Config-Daten setzen, weil ja setAutoReport(false)
  // 1. Zuerst alle Standard-Achsen deaktivieren
  bleGamepadConfig.setWhichAxes(false, false, false, false, false, false, false, false);
  // 2. Nur die benötigten 3 Achsen aktivieren
  // Die Parameter sind: x, y, z, rx, ry, rz, slider1, slider2
  bleGamepadConfig.setWhichAxes(false, false, false, true, true, true, false, false);
  // 3. Optional: Buttons einschränken (z.B. auf 0, wenn du keine hast)
  bleGamepadConfig.setButtonCount(0);
  // 4. Den Auto-Report nach Bedarf setzen
  bleGamepadConfig.setAutoReport(false);
  // 5. WICHTIG: Den Pointer (&) an begin übergeben
  bleGamepad.begin(&bleGamepadConfig);
  refreshUI();
  startCalibrationProcess();
}

// LOOP ---
void loop() {
  M5.update();
  unsigned long now = millis();

  // Power Off Logik nach lange Button press, oder bt-nicht-connect-timeout
  if (M5.BtnA.pressedFor(2000) || (!bleGamepad.isConnected() && now - lastActivity > POWER_OFF_TIMEOUT)) {
    LEDbar.setPixelColor(4, LEDbar.Color(255, 0, 0));  // Rot
    LEDbar.setPixelColor(5, LEDbar.Color(255, 0, 0));  // Rot
    LEDbar.setPixelColor(0, LEDbar.Color(255, 0, 0));  // Rot
    LEDbar.setPixelColor(1, LEDbar.Color(255, 0, 0));  // Rot
    LEDbar.setPixelColor(2, LEDbar.Color(255, 0, 0));  // Rot
    LEDbar.setPixelColor(3, LEDbar.Color(255, 0, 0));  // Rot
    LEDbar.setPixelColor(6, LEDbar.Color(255, 0, 0));  // Rot
    LEDbar.setPixelColor(7, LEDbar.Color(255, 0, 0));  // Rot
    LEDbar.setPixelColor(8, LEDbar.Color(255, 0, 0));  // Rot
    LEDbar.setPixelColor(9, LEDbar.Color(255, 0, 0));  // Rot
    LEDbar.show();                                     //faded
    for (int i = lcd_brightn; i > 0; i--) {
      lcd_brightn -= 1;
      M5.lcd.setBrightness(lcd_brightn);
      delay(20);
    }
    for (int i = led_brightness; i > 0; i--) {
      led_brightness -= 1;
      LEDbar.setBrightness(led_brightness);
      delay(30);
      LEDbar.show();
    }
    LEDbar.setPixelColor(4, LEDbar.Color(0, 0, 0));  // Aus
    LEDbar.setPixelColor(5, LEDbar.Color(0, 0, 0));  // Aus
    LEDbar.setPixelColor(0, LEDbar.Color(0, 0, 0));  // Aus
    LEDbar.setPixelColor(1, LEDbar.Color(0, 0, 0));  // Aus
    LEDbar.setPixelColor(2, LEDbar.Color(0, 0, 0));  // Aus
    LEDbar.setPixelColor(3, LEDbar.Color(0, 0, 0));  // Aus
    LEDbar.setPixelColor(6, LEDbar.Color(0, 0, 0));  // Aus
    LEDbar.setPixelColor(7, LEDbar.Color(0, 0, 0));  // Aus
    LEDbar.setPixelColor(8, LEDbar.Color(0, 0, 0));  // Aus
    LEDbar.setPixelColor(9, LEDbar.Color(0, 0, 0));  // Aus
    LEDbar.show();
    delay(100);
    M5.update();
    M5.powerOFF();  //FAKE POWER-OFF, cunsumes still power until completely empty! but M5.Power.powerOFF uses 108% of ram in total when compiled :(
  }

  // Bluetooth connection LED Logik
  static bool prevBTconnected = false;
  bool btConnected = bleGamepad.isConnected();

  // LED Steuerung: 75ms Blau bei Verbindung, 75ms Gelb ohne Verbindung
  if (now - lastLedToggle >= (ledState ? 75 : (btConnected ? LED_BLINK_INTERVAL_BLUE : LED_BLINK_INTERVAL_YELLOW))) {
    lastLedToggle = now;
    ledState = !ledState;

    if (ledState) {
      // LED AN
      uint32_t color = btConnected ? LEDbar.Color(0, 0, 255) : LEDbar.Color(255, 255, 0);
      LEDbar.setPixelColor(4, color);
      LEDbar.setPixelColor(5, color);
    } else {
      // LED AUS
      LEDbar.clear();
    }
    LEDbar.show();
  }

  if (btConnected != prevBTconnected) {
    prevBTconnected = btConnected;
    lastActivity = now;  // reset timer on status change
    if (btConnected) {
      if (!displayOn) {
        M5.Lcd.wakeup();
        M5.Lcd.setBrightness(lcd_brightn);
        displayOn = true;
      }
      refreshUI();
    } else {
      if (displayOn) {
        M5.Lcd.setTextSize(2);
        M5.Lcd.setCursor(0, 45);
        M5.Lcd.setTextColor(BLUE);
        M5.Lcd.printf("  BLUETOOTH: ");
        M5.Lcd.setTextColor(RED, BLACK);
        M5.Lcd.printf("DISCONNECTED ");
      }
      if (RESTART_BLE_ON_DISCONNECT) {
        bleGamepad.end();
        delay(50);
        bleGamepad.begin();
      }
    }
  }

  // 1. Button-wake & Quick Reset logic
  if (!displayOn) {
    if (M5.BtnC.pressedFor(700) && !isWarmingUp) {
      it_is_a_QuickReset = true;  //wird danach nie mehr zurückgesetzt! btw
      holdPositonActive = !holdPositonActive;
      M5.Lcd.wakeup();
      M5.Lcd.setBrightness(lcd_brightn);
      displayOn = true;
      lastActivity = millis();
      prevScreenOffSec = -1;
      refreshUI();
      startCalibrationProcess();
    } else if (M5.BtnB.wasPressed() || M5.BtnC.wasPressed()) {  //M5.BtnA.wasPressed() ||
      M5.Lcd.wakeup();
      M5.Lcd.setBrightness(lcd_brightn);
      displayOn = true;
      lastActivity = millis();
      prevScreenOffSec = -1;
      refreshUI();
    }
  } else {
    // 2. Button A/B/C + long & short press handling
    if (M5.BtnC.pressedFor(700) && !isWarmingUp) {
      it_is_a_QuickReset = true;
      holdPositonActive = !holdPositonActive;  // toggelt holdPositonActive wenn lange re-calib gedrückt wurde weil selber button benutzt wird
      refreshUI();
      startCalibrationProcess();
      while (M5.BtnC.isPressed()) {
        M5.update();
      }
    } else if (M5.BtnC.wasPressed()) {
      holdPositonActive = !holdPositonActive;
      lastActivity = now;
      refreshUI();
    } else if (M5.BtnB.wasPressed()) {
      precisionActive = !precisionActive;
      lastActivity = now;
      refreshUI();
    }
    //else if (M5.BtnA.wasPressed()) {
    //      tiltLockActive = !tiltLockActive;
    //      lastActivity = now;
    //      refreshUI();
    //    }
  }

  // 3. Display Auto-Off
  if (!isWarmingUp && displayOn && btConnected && (now - lastActivity > DISPLAY_TIMEOUT)) {
    M5.Lcd.setBrightness(0);
    M5.Lcd.sleep();
    displayOn = false;
    prevScreenOffSec = -1;
  }
  // lcd-off countdown
  if (displayOn && btConnected && !isWarmingUp) {
    long timeLeft = (long)DISPLAY_TIMEOUT - (long)(now - lastActivity);
    int secLeft = (timeLeft > 0) ? (int)((timeLeft + 999) / 1000) : 0;
    if (secLeft > 0 && secLeft <= 5) {
      if (secLeft != prevScreenOffSec) {
        M5.Lcd.setTextSize(2);
        M5.Lcd.fillRect(0, 192, 320, 15, BLACK);
        M5.Lcd.setCursor(5, 192);
        M5.Lcd.setTextColor(TFT_CYAN, BLACK);
        M5.Lcd.printf("*** LCD-OFF IN %d SEC. ***", secLeft);
        prevScreenOffSec = secLeft;
      }
    } else {
      if (prevScreenOffSec != -1) {
        M5.Lcd.fillRect(0, 192, 320, 15, BLACK);
        prevScreenOffSec = -1;
      }
    }
  }

  // 4. Gamepad logic
  icm_20948_DMP_data_t data;
  while (myICM.readDMPdataFromFIFO(&data) == ICM_20948_Stat_Ok) {
    //Zuweisungen der Achsen vom DMP auf den Gamecontroller-Treiber
    if ((data.header & DMP_header_bitmap_Quat9) > 0) {

      //in der Sparkfun Library entspricht folgende physische Bewegung des Boards:
      //data.Quat9.Data.Q1 = qY (q1) -> Sensor Kippen vor/zurück (Pitch/nicken)
      //data.Quat9.Data.Q2 = qX (q2) -> Sensor Kippen seitlich (Roll/rollen)
      //data.Quat9.Data.Q3 = qZ (q3) -> Sensor Drehung flach auf dem Tisch (Yaw/gieren)

      // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      //  Für die Gamepad Ausgabe muss:
      //  qY(q1) zur Z Ausgabe werden (roll Achse, seitlich rollen)
      //  qX(q2) zur Y Ausgabe werden (nick, hoch/runter)
      //  qZ(q3) zur X Ausgabe werden (yaw Achse, gieren links rechts)
      //  je nach Einbaulage des IMU-Sensors kann q1 mit q2 vertauscht sein
      // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

      // Achsen Zuweisung
      double q1 = ((double)data.Quat9.Data.Q1) / 1073741824.0;  // meine .setRZ Achse Gamepad
      double q2 = ((double)data.Quat9.Data.Q2) / 1073741824.0;  // meine .setRY Achse Gamepad
      double q3 = ((double)data.Quat9.Data.Q3) / 1073741824.0;  // meine .setRX Achse Gamepad
      double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
      if (isWarmingUp) {
        if (now - calibrationTime >= WARM_UP_TIME) {  // "Wenn Daten kommen und Warm-Up abläuft dann .."
          //if (startupSamples < CALIBRATION_SAMPLES) {
          //  driftOffsetY += q2;
          //  startupSamples++;
          //  if (startupSamples == CALIBRATION_SAMPLES) {
          //    driftOffsetY /= (float)CALIBRATION_SAMPLES;
          isWarmingUp = false;
          lastActivity = millis();
          //          if (displayOn) {
          //            M5.Lcd.fillRect(BOTTOM_X, BOTTOM_Y, BOTTOM_W, BOTTOM_H, BLACK);
          //            M5.Lcd.setCursor(BOTTOM_X + 5, BOTTOM_Y + 3);
          //            M5.Lcd.setTextSize(2);
          //            M5.Lcd.setTextColor(WHITE, BLACK);
          //            M5.Lcd.printf(" X:%3d Y:%3d Z:%3d", 0, 0, 0);
          //          }
          // Sende Mittelposition
          if (!prevZeroSent && bleGamepad.isConnected()) {
            bleGamepad.setRX(GAMEPAD_AXIS_CENTER);
            bleGamepad.setRY(GAMEPAD_AXIS_CENTER);
            bleGamepad.setRZ(GAMEPAD_AXIS_CENTER);
            prevZeroSent = true;
          }
          prevBottomState = 2;
          prevBottomX = 0;
          prevBottomY = 0;
          prevBottomZ = 0;
          refreshUI();
          calOffsetX = q3;
          calOffsetY = q2;
          calOffsetZ = q1;
        }
        //}
        //}
      } else {  //"wenn nicht kalibriert wird und die daten kommen"
        // Gamepad Achsen-Logik für Werte zwischen 0-32767 / min/max
        excessiveTilt = (q0 < tilt);

        int16_t sentX = GAMEPAD_AXIS_CENTER;  // Standard ist Mitte
        int16_t sentY = GAMEPAD_AXIS_CENTER;  // Standard ist Mitte
        int16_t sentZ = GAMEPAD_AXIS_CENTER;  // Standard ist Mitte

        if (!tiltLockActive || !excessiveTilt) {
          float currentMultiplier = precisionActive ? (sensitivity * precissionFactor) : (float)sensitivity;

          float moveX_raw = (q3 - calOffsetX) * currentMultiplier;
          float moveY_raw = (q2 - calOffsetY) * currentMultiplier;  //nach q2 das entfernt: - driftOffsetY
          float moveZ_raw = (q1 - calOffsetZ) * currentMultiplier;

          int16_t rawX_offset = (int16_t)(moveX_raw * sensitivity);
          int16_t rawY_offset = (int16_t)(moveY_raw * sensitivity);
          int16_t rawZ_offset = (int16_t)(moveZ_raw * sensitivity);


          // debug ausgabe
          if (debug) {
            unsigned int debug_now = millis();
            if (debug_now % 250 < 20) {  // Nur alle 250ms ausgeben, (läuft mit display-on aber schneller)
              Serial.println("--- [LIVE] DMP-Werte ---");
              Serial.printf("Live Quat9 | Q1:%1.5f | Q2:%1.5f | Q3:%1.5f\n", q1, q2, q3);
              Serial.printf("calculated |  X:%3.3f |  Y:%3.3f |  Z:%3.3f (incl. offsets)\n\n", moveX_raw, moveY_raw, moveZ_raw);
            }
          }

          // Begrenzen der Werte auf den erlaubten Bereich und Verschiebung in die Mitte
          sentX = constrain(GAMEPAD_AXIS_CENTER + rawX_offset, GAMEPAD_AXIS_MIN, GAMEPAD_AXIS_MAX);
          sentY = constrain(GAMEPAD_AXIS_CENTER + rawY_offset, GAMEPAD_AXIS_MIN, GAMEPAD_AXIS_MAX);
          sentZ = constrain(GAMEPAD_AXIS_CENTER + rawZ_offset, GAMEPAD_AXIS_MIN, GAMEPAD_AXIS_MAX);
        }

        // X,Y und Z-Achse invertieren falls invert-Flag gesetzt ist
        int16_t finalX = invertXAxis ? (GAMEPAD_AXIS_MAX - sentX) : sentX;
        int16_t finalY = invertYAxis ? (GAMEPAD_AXIS_MAX - sentY) : sentY;
        int16_t finalZ = invertZAxis ? (GAMEPAD_AXIS_MAX - sentZ) : sentZ;

        // Wenn TiltLockActive und ExcessiveTilt oder holdPositonActive, nimm die letzen Werte
        if ((tiltLockActive && excessiveTilt) || holdPositonActive) {
          finalX = prevBottomX;
          finalY = prevBottomY;
          finalZ = prevBottomZ;
        }

        // ***Hauptelement*** - hier werden die Bewegungsdaten gesetzt die später an den PC gesendet werden
        if (finalX != prevBottomX || finalY != prevBottomY || finalZ != prevBottomZ || prevZeroSent) {
          bleGamepad.setRX(finalX);
          bleGamepad.setRY(finalY);
          bleGamepad.setRZ(finalZ);
          prevZeroSent = false;
        }

        if (displayOn) {
          if ((excessiveTilt && tiltLockActive) || holdPositonActive) {
            if (prevBottomState != 1) {
              M5.Lcd.fillRect(BOTTOM_X, BOTTOM_Y, BOTTOM_W, BOTTOM_H, BLACK);
              M5.Lcd.setCursor(BOTTOM_X, BOTTOM_Y + 5);
              M5.Lcd.setTextSize(2);
              M5.Lcd.setTextColor(YELLOW, BLACK);
              M5.Lcd.print("    => LOCKED <=");
              prevBottomState = 1;
              prevBottomX = 0;
              prevBottomY = 0;
              prevBottomZ = 0;
            }
          } else {
            if (prevBottomState != 0 || prevBottomX != sentX || prevBottomY != sentY || prevBottomZ != sentZ) {
              // Anzeige X: Y: Z: auf dem Display
              M5.Lcd.setCursor(BOTTOM_X + 5, BOTTOM_Y + 3);
              M5.Lcd.setTextSize(2);
              M5.Lcd.setTextColor(btConnected ? GREEN : WHITE, BLACK);
              // Skaliert die Werte auf -100 bis +100 für die LCD-Anzeige
              int displayX = (int)round(((float)finalX / (float)GAMEPAD_AXIS_CENTER) * 100.0);
              int displayY = (int)round(((float)finalY / (float)GAMEPAD_AXIS_CENTER) * 100.0);
              int displayZ = (int)round(((float)finalZ / (float)GAMEPAD_AXIS_CENTER) * 100.0);
              M5.Lcd.printf("X:%3d Y:%3d Z:%3d", displayX - 100, displayY - 100, displayZ - 100);
              prevBottomState = 0;
              prevBottomX = finalX;
              prevBottomY = finalY;
              prevBottomY = finalZ;
            }
          }
          if (now % 10000 < 20) drawBattery();
        }
      }
    }
  }
  if (bleGamepad.isConnected()) {
    bleGamepad.sendReport();
    delay(10);
  }
}
//END LOOP
