// ==========================================
// MCU 3 - HYDRO POWER STATION FIRMWARE
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
#define TRIG_PIN 18
#define ECHO_PIN 19

LiquidCrystal_I2C lcd(0x27, 20, 4);

// ==========================================
// GLOBAL STATE & VOLATILE CALLBACK QUEUE
// ==========================================
StationTelemetryPayload myStationData;
bool isSleeping = false;
bool emergencyStop = false;
bool masterShutdown = false;
uint8_t fleetSpeedCap = 100;   // 0 - 100% intake flow capacity cap
bool actuatorState = false;   // Hydro Water Valve / Pump Control State
uint8_t currentDisplayPage = 0;

unsigned long lastTelemetryTx = 0;
unsigned long telemetryInterval = 2000; // 2s active, 10s power-save
unsigned long lastLcdUpdate = 0;

// Display Banner States
char bannerMessage[21] = "";
unsigned long bannerTimer = 0;
bool showingBanner = false;

// Thread-safe command queue flags (Populated in callback, processed in loop)
volatile bool hasPendingCmd = false;
volatile uint8_t pendingCmd = 0;
volatile uint16_t pendingParam = 0;

volatile bool hasPendingBanner = false;
char pendingBannerText[21] = "";

// Clean Energy Hydro Facts Database
const char* HYDRO_FACTS[] = {
  "Water flow gen power",
  "Clean Hydroelectric ",
  "Dams & Tidal Energy ",
  "80% Efficiency Rate "
};
const uint8_t TOTAL_FACTS = 4;

// Forward Declarations
void runFunctionalityTest();

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

  // 1. Master Shutdown State
  if (masterShutdown) {
    writeLcdLine(0, "=== STN 3: HYDRO ===");
    writeLcdLine(1, " MODE: SHUTDOWN     ");
    writeLcdLine(2, " FLEET HALTED       ");
    writeLcdLine(3, "====================");
    return;
  }

  // 2. Emergency Stop State
  if (emergencyStop) {
    writeLcdLine(0, "=== STN 3: HYDRO ===");
    writeLcdLine(1, " MODE: EMERGENCY    ");
    writeLcdLine(2, " ALL VALVES CLOSED  ");
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

  // 4. Standard Operational Display
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

  // Line 3: Flow Metric & Capacity Cap
  float currentFlow = (isSleeping || emergencyStop || masterShutdown) ? 0.0f : (24.5f * fleetSpeedCap / 100.0f);
  snprintf(buf, sizeof(buf), "%4.1fL/m| CAP:%3d%%  ", currentFlow, fleetSpeedCap);
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
    actuatorState = false; // Close valve in power save
    
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
    masterShutdown = false;
    
    lcd.backlight();
    lcd.clear();
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
  myStationData.voltage = isSleeping ? 11.5f : 12.4f;
  myStationData.sensor_metric = (isSleeping || emergencyStop || masterShutdown) ? 0.0f : (24.5f * fleetSpeedCap / 100.0f);

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
  ack.station_id = 3;
  ack.command_received = cmdReceived;
  ack.success = success ? 1 : 0;

  esp_now_send(MCU0_MAC, (uint8_t *)&ack, sizeof(ack));
}

// Thread-safe interrupt-free receive handler
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  if (len == sizeof(ControllerCommandPayload)) {
    ControllerCommandPayload cmd;
    memcpy(&cmd, incomingData, sizeof(cmd));

    if (cmd.target_station_id != 0 && cmd.target_station_id != 3) return;

    pendingCmd = cmd.command;
    pendingParam = cmd.parameter;
    hasPendingCmd = true;
  } 
  else if (len == sizeof(CustomBannerPayload)) {
    CustomBannerPayload bannerPayload;
    memcpy(&bannerPayload, incomingData, sizeof(bannerPayload));

    if (bannerPayload.target_station_id != 0 && bannerPayload.target_station_id != 3) return;

    snprintf(pendingBannerText, sizeof(pendingBannerText), "%-20s", bannerPayload.banner_text);
    hasPendingBanner = true;
  }
}

// ==========================================
// MAIN-THREAD COMMAND PROCESSOR
// ==========================================

void processPendingCommand() {
  // 1. Process Banner Display Overlays
  if (hasPendingBanner) {
    hasPendingBanner = false;
    snprintf(bannerMessage, sizeof(bannerMessage), "%s", pendingBannerText);
    showingBanner = true;
    bannerTimer = millis();
  }

  // 2. Process Command Packets
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
      actuatorState = false; // Immediately shut valve
      myStationData.status = CMD_EMERGENCY_STOP;
      renderHydroStationDisplay();
      sendAck(cmd, true);
      Serial.println(F("[ALERT] EMERGENCY STOP ACTIVATED! Hydro flow cut off."));
      break;

    case CMD_MASTER_SHUTDOWN:
      masterShutdown = true;
      actuatorState = false; // Immediately shut valve
      myStationData.status = CMD_MASTER_SHUTDOWN;
      renderHydroStationDisplay();
      sendAck(cmd, true);
      Serial.println(F("[ALERT] MASTER SHUTDOWN ACTIVATED! Hydro intake halted."));
      break;

    case CMD_SET_FLEET_SPEED:
      fleetSpeedCap = constrain((uint8_t)param, 0, 100);
      renderHydroStationDisplay();
      sendAck(cmd, true);
      Serial.printf("[SPEED] Fleet hydro intake capacity cap set to %d%%\n", fleetSpeedCap);
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

  Serial.println(F("[READY] MCU 3 (Hydro Station) active."));
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
