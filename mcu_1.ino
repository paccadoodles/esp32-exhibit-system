// ==========================================
// MCU 1 - WIND TURBINE STATION FIRMWARE
// ==========================================
#include 
#include 
#include 
#include 
#include 
#include "espnow_protocol.h"

// ==========================================
// HARDWARE PIN DEFINITIONS
// ==========================================
const int SLP_PIN = 32;       // DRV8833 Sleep Control Pin

// Fan 1 (Motor A)
const int AIN1 = 27;
const int AIN2 = 26;

// Fan 2 (Motor B)
const int BIN1 = 25;
const int BIN2 = 33;

// HC-SR04 Ultrasonic Pins
const int TRIG_PIN = 18;
const int ECHO_PIN = 19;

// KY-040 Rotary Encoder Pins
const int ENCODER_CLK = 16;
const int ENCODER_DT  = 4;
const int ENCODER_SW  = 17;

// ESP32 Core v3.x PWM Configuration
const int freq = 20000;       // 20kHz switching frequency
const int resolution = 8;     // 8-bit resolution (0 - 255)

// Timings & Cutoffs
const unsigned long TRIGGER_DURATION_MS = 15000; // 15s wind burst duration
const float PROXIMITY_THRESHOLD_CM = 25.0;        // Distance cutoff for car detection
const int REQUIRED_CONSECUTIVE_READINGS = 3;      // Filter false triggers

LiquidCrystal_I2C lcd(0x27, 20, 4);

// ==========================================
// GLOBAL STATE & VOLATILE COMMAND QUEUE
// ==========================================
StationTelemetryPayload myStationData;
bool isSleeping = false;
bool emergencyStop = false;
bool masterShutdown = false;
uint8_t fleetSpeedCap = 100;   // 0 - 100% executive global speed limit

char currentMode = 'e';        // 'e' = Manual/Encoder, 's' = Proximity, 'x' = Off, '1'..'3' = Presets
bool isTriggeredActive = false;
unsigned long burstStartTime = 0;
int consecutiveDetectionCount = 0;

int manualEncoderSpeed = 150;  // Initial PWM speed
int currentFanSpeed = 0;       // Tracked live fan PWM for telemetry calculation
int lastStateCLK;
bool lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

unsigned long buttonPressStartTime = 0;
bool buttonWasPressed = false;
bool longPressTriggered = false;

// Display Banner & Mode Change States
unsigned long modeSwitchMessageTimer = 0;
bool showingModeChangeMessage = false;

char bannerMessage[21] = "";
unsigned long bannerTimer = 0;
bool showingBanner = false;

unsigned long lastLcdUpdate = 0;
const unsigned long lcdUpdateInterval = 250;

unsigned long lastTelemetryTx = 0;
unsigned long telemetryInterval = 2000; // 2s active, 10s power-save

// Thread-Safe Volatile Command Queue Flags
volatile bool hasPendingCmd = false;
volatile uint8_t pendingCmd = 0;
volatile uint16_t pendingParam = 0;

volatile bool hasPendingBanner = false;
char pendingBannerText[21] = "";

// Forward Declarations
void runFunctionalityTest();

// ==========================================
// DISPLAY ENGINE (PADDED BUFFER)
// ==========================================

void writeLcdLine(uint8_t line, const char* formattedText) {
  char lineBuffer[21];
  snprintf(lineBuffer, sizeof(lineBuffer), "%-20s", formattedText);
  lcd.setCursor(0, line);
  lcd.print(lineBuffer);
}

void updateLCD(float distance) {
  if (isSleeping) return;

  if (millis() - lastLcdUpdate < lcdUpdateInterval) return;
  lastLcdUpdate = millis();

  // 1. Master Shutdown State
  if (masterShutdown) {
    writeLcdLine(0, "=== STN 1: WIND ====");
    writeLcdLine(1, " MODE: SHUTDOWN     ");
    writeLcdLine(2, " FLEET HALTED       ");
    writeLcdLine(3, "====================");
    return;
  }

  // 2. Emergency Stop State
  if (emergencyStop) {
    writeLcdLine(0, "=== STN 1: WIND ====");
    writeLcdLine(1, " MODE: EMERGENCY    ");
    writeLcdLine(2, " ALL MOTORS HALTED  ");
    writeLcdLine(3, "====================");
    return;
  }

  // 3. Custom Broadcast Banner Overlay
  if (showingBanner) {
    if (millis() - bannerTimer < 3500) {
      writeLcdLine(0, "=== MASTER ALERT ===");
      writeLcdLine(1, bannerMessage);
      writeLcdLine(2, "====================");
      writeLcdLine(3, "                    ");
      return;
    } else {
      showingBanner = false;
      lcd.clear();
    }
  }

  // 4. Mode Switch Message
  if (showingModeChangeMessage) {
    if (millis() - modeSwitchMessageTimer < 1500) {
      writeLcdLine(0, "====================");
      writeLcdLine(1, "  MODE SWITCHED!    ");
      char buf[21];
      snprintf(buf, sizeof(buf), "    Active: [%c]     ", currentMode);
      writeLcdLine(2, buf);
      writeLcdLine(3, "====================");
      return;
    } else {
      showingModeChangeMessage = false;
      lcd.clear();
    }
  }

  // 5. Standard Operating Displays
  char lineBuf[21];
  if (currentMode == 'e') {
    writeLcdLine(0, "=== WIND TURBINE ===");
    snprintf(lineBuf, sizeof(lineBuf), "Mode: Enc | Cap:%3d%%", fleetSpeedCap);
    writeLcdLine(1, lineBuf);
    snprintf(lineBuf, sizeof(lineBuf), "Spd: %3d | %4.0f RPM", currentFanSpeed, myStationData.sensor_metric);
    writeLcdLine(2, lineBuf);
    writeLcdLine(3, "Turn:Spd | Click:Off");
  } else if (currentMode == 's') {
    writeLcdLine(0, "=== PROXIMITY MODE ==");
    writeLcdLine(1, "  Come closer! :)   ");
    if (distance >= 0) {
      snprintf(lineBuf, sizeof(lineBuf), "Dist: %d cm | CAR:%s", (int)distance, myStationData.car_status ? "YES" : "NO ");
    } else {
      snprintf(lineBuf, sizeof(lineBuf), "Dist: --- cm | CAR:NO ");
    }
    writeLcdLine(2, lineBuf);
    snprintf(lineBuf, sizeof(lineBuf), "Trig Cutoff: <%dcm  ", (int)PROXIMITY_THRESHOLD_CM);
    writeLcdLine(3, lineBuf);
  } else {
    writeLcdLine(0, "=== SPEED CONTROL ==");
    snprintf(lineBuf, sizeof(lineBuf), "Active Mode: [%c]    ", currentMode);
    writeLcdLine(1, lineBuf);
    if (currentMode == 'x') writeLcdLine(2, "Status: OFF (Paused)");
    else                    writeLcdLine(2, "Status: Preset Spd  ");
    writeLcdLine(3, "Hold btn to change ");
  }
}

// ==========================================
// HARDWARE CONTROLS & SENSORS
// ==========================================

void setFanSpeed(int fanID, int rawSpeed) {
  int scaledSpeed = rawSpeed;

  if (emergencyStop || isSleeping || masterShutdown) {
    scaledSpeed = 0;
  } else {
    // Apply Executive Fleet Speed Cap (0 - 100%)
    scaledSpeed = (rawSpeed * fleetSpeedCap) / 100;
    if (scaledSpeed > 0 && scaledSpeed < 60) scaledSpeed = 60; // Deadzone threshold
  }
  scaledSpeed = constrain(scaledSpeed, 0, 255);

  if (fanID == 1) {
    if (scaledSpeed == 0) { ledcWrite(AIN1, 0); ledcWrite(AIN2, 0); }
    else                  { ledcWrite(AIN1, scaledSpeed); ledcWrite(AIN2, 0); }
  } else if (fanID == 2) {
    if (scaledSpeed == 0) { ledcWrite(BIN1, 0); ledcWrite(BIN2, 0); }
    else                  { ledcWrite(BIN1, scaledSpeed); ledcWrite(BIN2, 0); }
  }

  currentFanSpeed = scaledSpeed; // Track live scaled speed
}

float readUltrasonicDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 25000); // 25ms timeout (~4m max)
  if (duration == 0) return -1.0; 

  float calculatedDistance = duration * 0.0343 / 2.0;
  if (calculatedDistance < 2.0 || calculatedDistance > 400.0) return -1.0;

  return calculatedDistance;
}

void setPowerMode(bool enableSleep) {
  isSleeping = enableSleep;
  if (isSleeping) {
    myStationData.status = CMD_POWER_SAVE;
    telemetryInterval = 10000;
    setFanSpeed(1, 0);
    setFanSpeed(2, 0);
    
    lcd.noBacklight();
    writeLcdLine(0, "--------------------");
    writeLcdLine(1, " STN 1: POWER SAVE  ");
    writeLcdLine(2, "   STANDBY MODE     ");
    writeLcdLine(3, "--------------------");
    Serial.println(F("[POWER] Entered Soft-Sleep Mode."));
  } else {
    myStationData.status = 1; // Active Run
    telemetryInterval = 2000;
    emergencyStop = false;
    masterShutdown = false;
    
    lcd.backlight();
    lcd.clear();
    Serial.println(F("[POWER] Restored Active Operational Mode."));
  }
}

void cycleToNextMode() {
  if (currentMode == 'e')      currentMode = 's';
  else if (currentMode == 's') currentMode = '1';
  else if (currentMode == '1') currentMode = '2';
  else if (currentMode == '2') currentMode = '3';
  else                         currentMode = 'e';

  isTriggeredActive = false;
  showingModeChangeMessage = true;
  modeSwitchMessageTimer = millis();
}

void togglePowerKillSwitch() {
  if (currentMode == 'x') {
    currentMode = 'e'; 
  } else {
    currentMode = 'x'; 
    setFanSpeed(1, 0); 
    setFanSpeed(2, 0);
  }
}

void handleButtonPress() {
  int currentStateCLK = digitalRead(ENCODER_CLK);
  if (currentStateCLK != lastStateCLK && currentStateCLK == LOW) {
    if (digitalRead(ENCODER_DT) == HIGH) manualEncoderSpeed = min(255, manualEncoderSpeed + 4); 
    else                                 manualEncoderSpeed = max(0, manualEncoderSpeed - 4);   
    
    if (currentMode != 'x') currentMode = 'e'; 
    isTriggeredActive = false;
  }
  lastStateCLK = currentStateCLK;

  bool currentButtonState = digitalRead(ENCODER_SW);

  if (currentButtonState == LOW && lastButtonState == HIGH) {
    if (millis() - lastDebounceTime > debounceDelay) {
      buttonPressStartTime = millis();
      buttonWasPressed = true;
      longPressTriggered = false;
      lastDebounceTime = millis();
    }
  }

  if (currentButtonState == LOW && buttonWasPressed && !longPressTriggered) {
    if (millis() - buttonPressStartTime >= 800) {
      longPressTriggered = true;
      cycleToNextMode();
    }
  }

  if (currentButtonState == HIGH && lastButtonState == LOW) {
    if (buttonWasPressed && !longPressTriggered) {
      togglePowerKillSwitch();
    }
    buttonWasPressed = false;
  }

  lastButtonState = currentButtonState;
}

// ==========================================
// ESP-NOW COMMUNICATIONS & ACK HANDSHAKE
// ==========================================

void sendTelemetry() {
  myStationData.packet_type = PKT_TELEMETRY;
  myStationData.station_id = 1;
  myStationData.actuator_state = (currentFanSpeed > 0);
  myStationData.voltage = isSleeping ? 11.2f : 12.5f;
  myStationData.sensor_metric = (isSleeping || emergencyStop || masterShutdown) ? 0.0f : (currentFanSpeed * 12.0f); // Simulated Wind RPM

  if (masterShutdown) {
    myStationData.status = CMD_MASTER_SHUTDOWN;
  } else if (emergencyStop) {
    myStationData.status = CMD_EMERGENCY_STOP;
  } else if (isSleeping) {
    myStationData.status = CMD_POWER_SAVE;
  } else {
    myStationData.status = 1; // Active
  }

  esp_now_send(MCU0_MAC, (uint8_t *)&myStationData, sizeof(myStationData));
}

void sendAck(uint8_t cmdReceived, bool success) {
  CommandAckPayload ack;
  ack.packet_type = PKT_ACK;
  ack.station_id = 1;
  ack.command_received = cmdReceived;
  ack.success = success ? 1 : 0;

  esp_now_send(MCU0_MAC, (uint8_t *)&ack, sizeof(ack));
}

// Thread-safe interrupt-free receive handler
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  if (len == sizeof(ControllerCommandPayload)) {
    ControllerCommandPayload cmdPayload;
    memcpy(&cmdPayload, incomingData, sizeof(cmdPayload));

    if (cmdPayload.target_station_id != 0 && cmdPayload.target_station_id != 1) return;

    pendingCmd = cmdPayload.command;
    pendingParam = cmdPayload.parameter;
    hasPendingCmd = true;
  } 
  else if (len == sizeof(CustomBannerPayload)) {
    CustomBannerPayload bannerPayload;
    memcpy(&bannerPayload, incomingData, sizeof(bannerPayload));

    if (bannerPayload.target_station_id != 0 && bannerPayload.target_station_id != 1) return;

    snprintf(pendingBannerText, sizeof(pendingBannerText), "%-20s", bannerPayload.banner_text);
    hasPendingBanner = true;
  }
}

void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {}

// ==========================================
// MAIN-THREAD COMMAND PROCESSOR
// ==========================================

void processPendingCommand() {
  // Handle Banner Message
  if (hasPendingBanner) {
    hasPendingBanner = false;
    snprintf(bannerMessage, sizeof(bannerMessage), "%s", pendingBannerText);
    showingBanner = true;
    bannerTimer = millis();
  }

  // Handle Command Packets
  if (!hasPendingCmd) return;

  uint8_t cmd = pendingCmd;
  uint16_t param = pendingParam;
  hasPendingCmd = false;

  Serial.printf("[CMD RX] Executing Command ID: 0x%02X (Param: %d)\n", cmd, param);

  switch (cmd) {
    case CMD_RUN_TEST:
      sendAck(cmd, true);
      runFunctionalityTest();
      break;

    case CMD_POWER_SAVE:
      setPowerMode(true);
      sendAck(cmd, true);
      break;

    case CMD_WAKEUP:
      setPowerMode(false);
      sendAck(cmd, true);
      break;

    case CMD_EMERGENCY_STOP:
      emergencyStop = true;
      setFanSpeed(1, 0);
      setFanSpeed(2, 0);
      sendAck(cmd, true);
      Serial.println(F("[ALERT] EMERGENCY STOP ACTIVATED! Wind Turbine Halted."));
      break;

    case CMD_MASTER_SHUTDOWN:
      masterShutdown = true;
      setFanSpeed(1, 0);
      setFanSpeed(2, 0);
      sendAck(cmd, true);
      Serial.println(F("[ALERT] MASTER SHUTDOWN DISPATCHED! All local systems locked."));
      break;

    case CMD_SET_FLEET_SPEED:
      fleetSpeedCap = constrain((uint8_t)param, 0, 100);
      sendAck(cmd, true);
      Serial.printf("[SPEED] Fleet speed scaling cap set to %d%%\n", fleetSpeedCap);
      break;

    case CMD_ACTUATOR_CTRL:
      if (param > 0) {
        currentMode = 'e';
        manualEncoderSpeed = (param <= 255) ? param : 255;
      } else {
        currentMode = 'x';
        setFanSpeed(1, 0);
        setFanSpeed(2, 0);
      }
      sendAck(cmd, true);
      Serial.printf("[ACTUATOR] Wind speed target set via remote: %d\n", manualEncoderSpeed);
      break;

    case CMD_SET_CONFIG:
      if (param >= 500 && param <= 30000) {
        telemetryInterval = param;
        sendAck(cmd, true);
        Serial.printf("[CFG] Telemetry report interval set to %d ms\n", param);
      } else {
        sendAck(cmd, false);
      }
      break;

    case CMD_REBOOT:
      sendAck(cmd, true);
      Serial.println(F("[SYSTEM] Rebooting MCU 1 in 500ms..."));
      delay(500);
      ESP.restart();
      break;

    default:
      sendAck(cmd, false);
      break;
  }
}

// ==========================================
// DIAGNOSTICS ROUTINE
// ==========================================

void runFunctionalityTest() {
  Serial.println(F("[TEST] Running Station 1 Diagnostics..."));
  myStationData.status = CMD_RUN_TEST;
  sendTelemetry();

  if (!isSleeping) lcd.backlight();
  
  writeLcdLine(0, "====================");
  writeLcdLine(1, " STN 1 DIAGNOSTICS  ");
  writeLcdLine(2, " Testing Motors...  ");
  writeLcdLine(3, "====================");

  // Pulse Fans
  setFanSpeed(1, 128); setFanSpeed(2, 128);
  delay(1000);
  setFanSpeed(1, 0);   setFanSpeed(2, 0);

  // Read Sensor
  float dist = readUltrasonicDistance();
  if (dist >= 0) {
    char distBuf[21];
    snprintf(distBuf, sizeof(distBuf), " Sensor OK: %d cm   ", (int)dist);
    writeLcdLine(2, distBuf);
  } else {
    writeLcdLine(2, " Sensor ERROR!      ");
  }
  delay(1500);

  myStationData.status = 1; // Return to Active
  sendTelemetry();
  lcd.clear();
}

// ==========================================
// SETUP & MAIN LOOP
// ==========================================

void setup() {
  Serial.begin(115200);

  strcpy(myStationData.station_name, "Wind Turbine");
  myStationData.station_id = 1;
  myStationData.status = 1;
  myStationData.battery_pct = 100;
  myStationData.car_status = 0;

  // 1. Hardware Pins & Motor LEDC
  pinMode(SLP_PIN, OUTPUT);
  digitalWrite(SLP_PIN, HIGH); 

  ledcAttach(AIN1, freq, resolution);
  ledcAttach(AIN2, freq, resolution);
  ledcAttach(BIN1, freq, resolution);
  ledcAttach(BIN2, freq, resolution);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  pinMode(ENCODER_CLK, INPUT_PULLUP);
  pinMode(ENCODER_DT, INPUT_PULLUP);
  pinMode(ENCODER_SW, INPUT_PULLUP); 
  lastStateCLK = digitalRead(ENCODER_CLK);

  // 2. LCD Setup
  Wire.begin(21, 23); 
  lcd.init();          
  lcd.backlight();
  writeLcdLine(0, "System Booting...");
  delay(500);
  lcd.clear();

  // 3. Wi-Fi & ESP-NOW
  WiFi.mode(WIFI_STA);
  Serial.print(F("[INFO] Station 1 MAC: "));
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println(F("[ERROR] ESP-NOW Init Failed!"));
    myStationData.status = 0;
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, MCU0_MAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) == ESP_OK) {
    Serial.println(F("[SUCCESS] Controller Peer Registered."));
  }

  Serial.println(F("[READY] Station 1 Active."));
}

void loop() {
  // 1. Execute queued commands on main thread
  processPendingCommand();

  // 2. Hardware UI Controls
  handleButtonPress();

  // 3. Sensor & Presence Logic
  float distance = readUltrasonicDistance();
  
  if (distance >= 0 && distance <= PROXIMITY_THRESHOLD_CM) {
    consecutiveDetectionCount++;
    if (consecutiveDetectionCount >= REQUIRED_CONSECUTIVE_READINGS) {
      myStationData.car_status = 1; 
    }
  } else {
    consecutiveDetectionCount = 0;
    myStationData.car_status = 0;   
  }

  // 4. Drive Motors
  if (currentMode == 's') {
    if (isTriggeredActive) {
      if (millis() - burstStartTime < TRIGGER_DURATION_MS) {
        setFanSpeed(1, 255); setFanSpeed(2, 255);
      } else {
        isTriggeredActive = false;
        setFanSpeed(1, 0); setFanSpeed(2, 0);
      }
    } else {
      if (myStationData.car_status == 1) {
        isTriggeredActive = true;
        burstStartTime = millis();
        setFanSpeed(1, 255); setFanSpeed(2, 255);
      } else {
        setFanSpeed(1, 0); setFanSpeed(2, 0);
      }
    }
  } 
  else if (currentMode == 'e') {
    setFanSpeed(1, manualEncoderSpeed);
    setFanSpeed(2, manualEncoderSpeed);
  }
  else if (currentMode == 'x') {
    setFanSpeed(1, 0); setFanSpeed(2, 0);
  }
  else if (currentMode == '1') { setFanSpeed(1, 80);  setFanSpeed(2, 80);  }
  else if (currentMode == '2') { setFanSpeed(1, 160); setFanSpeed(2, 160); }
  else if (currentMode == '3') { setFanSpeed(1, 255); setFanSpeed(2, 255); }

  // 5. Refresh Display
  updateLCD(distance);

  // 6. Send Telemetry Heartbeat
  if (millis() - lastTelemetryTx >= telemetryInterval) {
    lastTelemetryTx = millis();
    sendTelemetry();
  }

  delay(10); 
}
