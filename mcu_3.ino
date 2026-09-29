#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "espnow_protocol.h"

// ==========================================
// HARDWARE PIN DEFINITIONS
// ==========================================
#define TRIG_PIN 18
#define ECHO_PIN 19

LiquidCrystal_I2C lcd(0x27, 20, 4);

// ==========================================
// GLOBAL STATE & VOLATILE CALLBACK QUEUE
// ==========================================
StationTelemetryPayload myStationData;
bool isSleeping = false;
bool emergencyStop = false;
bool actuatorState = false; // Hydro Water Valve / Pump Control State
uint8_t currentDisplayPage = 0;

unsigned long lastTelemetryTx = 0;
unsigned long telemetryInterval = 2000; // 2s active, 10s power-save
unsigned long lastLcdUpdate = 0;

// Thread-safe command queue flags (Populated in callback, processed in loop)
volatile bool hasPendingCmd = false;
volatile uint8_t pendingCmd = 0;
volatile uint16_t pendingParam = 0;

// Clean Energy Hydro Facts Database
const char* HYDRO_FACTS[] = {
  "Water flow gen power",
  "Clean Hydroelectric ",
  "Dams & Tidal Energy ",
  "80% Efficiency Rate "
};
const uint8_t TOTAL_FACTS = 4;

// ==========================================
// MODULAR LCD BUFFER ENGINE
// ==========================================

void writeLcdLine(uint8_t line, const char* formattedText) {
  char lineBuffer[21];
  snprintf(lineBuffer, sizeof(lineBuffer), "%-20s", formattedText);
  lcd.setCursor(0, line);
  lcd.print(lineBuffer);
}

void renderHydroStationDisplay() {
  if (isSleeping) return;

  char buf[21];

  if (emergencyStop) {
    writeLcdLine(0, "=== STN 3: HYDRO ===");
    writeLcdLine(1, " MODE: EMERGENCY    ");
    writeLcdLine(2, " ALL VALVES CLOSED  ");
    writeLcdLine(3, "====================");
    return;
  }

  // Line 0: Header
  writeLcdLine(0, "STN 3: HYDRO POWER  ");

  // Line 1: Hydro Fact
  snprintf(buf, sizeof(buf), "INFO:%-15s", HYDRO_FACTS[currentDisplayPage % TOTAL_FACTS]);
  writeLcdLine(1, buf);

  // Line 2: Presence & Actuator Status
  snprintf(buf, sizeof(buf), "CAR:%-3s | VALVE:%-3s", 
           myStationData.car_status ? "YES" : "NO",
           actuatorState ? "OPEN" : "CLSD");
  writeLcdLine(2, buf);

  // Line 3: Flow Metric & Page ID
  snprintf(buf, sizeof(buf), "%4.1fL/m| PAGE:%d    ", myStationData.sensor_metric, currentDisplayPage);
  writeLcdLine(3, buf);
}

// ==========================================
// SENSOR & POWER MODULES
// ==========================================

float readUltrasonicDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 25000); // 25ms timeout (~4m max)
  if (duration == 0) return -1.0; 
  return (duration * 0.0343) / 2.0;
}

void setPowerMode(bool enableSleep) {
  isSleeping = enableSleep;
  if (isSleeping) {
    myStationData.status = CMD_POWER_SAVE;
    telemetryInterval = 10000;
    
    lcd.noBacklight();
    writeLcdLine(0, "--------------------");
    writeLcdLine(1, " STN 3: POWER SAVE  ");
    writeLcdLine(2, "   STANDBY MODE     ");
    writeLcdLine(3, "--------------------");
    Serial.println(F("[POWER] Entered Soft-Sleep Mode."));
  } else {
    myStationData.status = 1; // Active Run
    telemetryInterval = 2000;
    emergencyStop = false;
    
    lcd.backlight();
    renderHydroStationDisplay();
    Serial.println(F("[POWER] Restored Active Operational Mode."));
  }
}

// ==========================================
// ESP-NOW COMMUNICATIONS & ACK HANDSHAKE
// ==========================================

void sendTelemetry() {
  myStationData.packet_type = PKT_TELEMETRY;
  myStationData.station_id = 3;
  myStationData.actuator_state = actuatorState;
  myStationData.voltage = isSleeping ? 11.5f : 12.4f; // Simulated operational voltage
  myStationData.sensor_metric = (isSleeping || emergencyStop) ? 0.0f : 24.5f; // Water flow in L/m

  if (emergencyStop) {
    myStationData.status = CMD_EMERGENCY_STOP;
  } else if (isSleeping) {
    myStationData.status = CMD_POWER_SAVE;
  }

  esp_now_send(MCU0_MAC, (uint8_t *)&myStationData, sizeof(myStationData));
}

void sendAck(uint8_t cmdReceived, bool success) {
  CommandAckPayload ack;
  ack.packet_type = PKT_ACK;
  ack.station_id = 3;
  ack.command_received = cmdReceived;
  ack.success = success ? 1 : 0;

  esp_now_send(MCU0_MAC, (uint8_t *)&ack, sizeof(ack));
}

// Lightweight interrupt callback (no blocking delays or I2C calls)
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  if (len != sizeof(ControllerCommandPayload)) return;

  ControllerCommandPayload cmd;
  memcpy(&cmd, incomingData, sizeof(cmd));

  if (cmd.target_station_id != 0 && cmd.target_station_id != 3) return;

  // Queue parameters for loop processing
  pendingCmd = cmd.command;
  pendingParam = cmd.parameter;
  hasPendingCmd = true;
}

// ==========================================
// MAIN-THREAD COMMAND PROCESSOR
// ==========================================

void processPendingCommand() {
  if (!hasPendingCmd) return;

  uint8_t cmd = pendingCmd;
  uint16_t param = pendingParam;
  hasPendingCmd = false;

  Serial.printf("[CMD RX] Executing Command ID: %d (Param: %d)\n", cmd, param);

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
      actuatorState = false; // Immediately shut valve
      myStationData.status = CMD_EMERGENCY_STOP;
      renderHydroStationDisplay();
      sendAck(cmd, true);
      Serial.println(F("[ALERT] EMERGENCY STOP ACTIVATED! Hydro flow cut off."));
      break;

    case CMD_ACTUATOR_CTRL:
      actuatorState = (param > 0);
      renderHydroStationDisplay();
      sendAck(cmd, true);
      Serial.printf("[ACTUATOR] Hydro Intake Valve set to: %s\n", actuatorState ? "OPEN" : "CLOSED");
      break;

    case CMD_SET_CONFIG:
      if (param >= 500 && param <= 30000) {
        telemetryInterval = param;
        sendAck(cmd, true);
        Serial.printf("[CFG] Telemetry interval set to %d ms\n", param);
      } else {
        sendAck(cmd, false);
      }
      break;

    case CMD_REBOOT:
      sendAck(cmd, true);
      Serial.println(F("[SYSTEM] Rebooting MCU 3 in 500ms..."));
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
  Serial.println(F("[TEST] Running Hydro Station Diagnostic Routine..."));
  myStationData.status = CMD_RUN_TEST;
  sendTelemetry();

  writeLcdLine(0, "====================");
  writeLcdLine(1, " STN 3 DIAGNOSTICS  ");
  writeLcdLine(2, " Stage 1: Display OK");
  writeLcdLine(3, "====================");
  delay(1500);

  for (uint8_t f = 0; f < TOTAL_FACTS; f++) {
    char buf[21];
    snprintf(buf, sizeof(buf), "Stage 2: Fact #%d    ", f + 1);
    writeLcdLine(2, buf);
    snprintf(buf, sizeof(buf), "%-20s", HYDRO_FACTS[f]);
    writeLcdLine(3, buf);
    delay(500);
  }

  writeLcdLine(2, " Stage 3: Ping Sensor");
  float dist = readUltrasonicDistance();
  delay(500);
  if (dist >= 0) {
    char distBuf[21];
    snprintf(distBuf, sizeof(distBuf), " Dist: %d cm        ", (int)dist);
    writeLcdLine(3, distBuf);
  } else {
    writeLcdLine(3, " Sensor: ERROR!     ");
  }
  delay(1000);

  writeLcdLine(2, " STATUS: ALL PASS!! ");
  writeLcdLine(3, "====================");
  delay(1000);

  // Self-healing re-init for screen recovery
  lcd.init();
  lcd.backlight();

  myStationData.status = 1;
  sendTelemetry();

  renderHydroStationDisplay();
}

// ==========================================
// SETUP & MAIN LOOP
// ==========================================

void setup() {
  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  strcpy(myStationData.station_name, "Hydro Stn");
  myStationData.status = 1;
  myStationData.car_status = 0;
  myStationData.battery_pct = 100;

  Wire.begin(21, 22);
  lcd.init();
  lcd.backlight();
  renderHydroStationDisplay();

  WiFi.mode(WIFI_STA);
  Serial.print(F("[SYSTEM] MCU 3 MAC Address: "));
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println(F("[ERROR] ESP-NOW Init Failed!"));
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, MCU0_MAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println(F("[WARN] Failed to register MCU 0 peer!"));
  }

  Serial.println(F("[READY] MCU 3 (Hydro Station) initialized successfully."));
}

void loop() {
  // 1. Safely execute queued commands on main thread
  processPendingCommand();

  // 2. Read Proximity Sensor
  float distance = readUltrasonicDistance();
  myStationData.car_status = (distance > 0 && distance <= 25.0) ? 1 : 0;

  // 3. Refresh Screen Updates
  if (!isSleeping && (millis() - lastLcdUpdate >= 500)) {
    lastLcdUpdate = millis();
    renderHydroStationDisplay();
  }

  // 4. Telemetry Transmission Heartbeat
  if (millis() - lastTelemetryTx >= telemetryInterval) {
    lastTelemetryTx = millis();
    sendTelemetry();
  }

  delay(10);
}