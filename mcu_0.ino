// ==========================================
// MCU 0 - MASTER HUB & FLEET ORCHESTRATOR
// ==========================================
#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "espnow_protocol.h"

// ==========================================
// 1. HARDWARE & PEER CONFIGURATION
// ==========================================
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Explicit MAC Address Mapping for Worker Nodes (Index 0 = Unused/Broadcast)
static const uint8_t STATION_MACS[4][6] = {
  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 
  {0x24, 0x6F, 0x28, 0xAA, 0xBB, 0x01}, // MCU 1 (Wind)  - Replace with actual MAC
  {0x24, 0x6F, 0x28, 0xAA, 0xBB, 0x02}, // MCU 2 (Solar) - Replace with actual MAC
  {0x24, 0x6F, 0x28, 0xAA, 0xBB, 0x03}  // MCU 3 (Hydro) - Replace with actual MAC
};

struct StationRecord {
  StationTelemetryPayload telemetry;
  unsigned long lastSeen;
  bool isOnline;
  uint8_t lastAckCmd;
  bool lastAckSuccess;
};

StationRecord stationRegistry[4];
const unsigned long HEARTBEAT_TIMEOUT = 6000;

// LCD & UI State
uint8_t currentViewMode = 0; // 0 = Overview, 1..3 = Station Detail Views
unsigned long lastLcdRefresh = 0;
const unsigned long LCD_REFRESH_INTERVAL = 300;
bool lcdNeedsClear = false;

// Non-Blocking Automated Sequence State Machine
enum SystemRunMode { MODE_NORMAL, MODE_TEST_SEQUENCE, MODE_DEMO_SEQUENCE };
SystemRunMode currentRunMode = MODE_NORMAL;
uint8_t sequenceStep = 0;
unsigned long sequenceTimer = 0;

// Serial CLI Input Buffer
char cliInputBuffer[128];
uint8_t cliBufferIndex = 0;

// ==========================================
// 2. DISPLAY ENGINE
// ==========================================

void printHelpMenu() {
  Serial.println(F("\n=================================================================="));
  Serial.println(F("              MCU 0 (BOSS CONTROLLER) MASTER CLI                  "));
  Serial.println(F("=================================================================="));
  Serial.println(F(" NAVIGATION & STATUS:"));
  Serial.println(F("   n / next    : Cycle LCD Views (Overview -> Stn 1 -> 2 -> 3)"));
  Serial.println(F("   v0..v3      : Jump directly to Overview (v0) or Station detail (v1..v3)"));
  Serial.println(F("   status / s  : Print detailed telemetry matrix to Serial"));
  Serial.println(F("   help / ?    : Display command menu\n"));
  Serial.println(F(" EMERGENCY & POWER:"));
  Serial.println(F("   estop / e   : Emergency stop target (e.g. 'e', 'e0', 'e1')"));
  Serial.println(F("   shutdown    : MASTER SHUTDOWN (Immediate network-wide halt)"));
  Serial.println(F("   pall / p0   : Put ALL stations to Sleep (p1, p2, p3 for single)"));
  Serial.println(F("   w0..w3      : Wake up ALL (w0) or target station (w1, w2, w3)\n"));
  Serial.println(F(" EXECUTIVE & ADVANCED CONTROL:"));
  Serial.println(F("   speed <0-100>   : Set global fleet production output speed %"));
  Serial.println(F("   banner    : Broadcast custom 20-char message to LCDs"));
  Serial.println(F("   demo            : Launch non-blocking 30-second automated demo"));
  Serial.println(F("   tall / t0       : Launch non-blocking diagnostic sweep"));
  Serial.println(F("   act  <1|0>  : Toggle station actuator directly"));
  Serial.println(F("   cfg     : Set remote telemetry report interval"));
  Serial.println(F("   boot        : Force remote station reboot"));
  Serial.println(F("==================================================================\n"));
}

void writeLcdLine(uint8_t line, const char* formattedText) {
  char lineBuffer[21];
  snprintf(lineBuffer, sizeof(lineBuffer), "%-20s", formattedText);
  lcd.setCursor(0, line);
  lcd.print(lineBuffer);
}

void renderOverviewLCD() {
  writeLcdLine(0, "=== BOSS CONTROL ===");
  char buf[21];
  snprintf(buf, sizeof(buf), "STN 1 (Wind) :  [%s]", stationRegistry[1].isOnline ? "OK" : " X");
  writeLcdLine(1, buf);
  snprintf(buf, sizeof(buf), "STN 2 (Solar):  [%s]", stationRegistry[2].isOnline ? "OK" : " X");
  writeLcdLine(2, buf);
  snprintf(buf, sizeof(buf), "STN 3 (Hydro):  [%s]", stationRegistry[3].isOnline ? "OK" : " X");
  writeLcdLine(3, buf);
}

void renderStationDetailLCD(uint8_t stationId) {
  if (stationId < 1 || stationId > 3) return;
  StationRecord &stn = stationRegistry[stationId];
  char lineBuf[21];

  // Line 0: ID, Name, Link state
  snprintf(lineBuf, sizeof(lineBuf), "STN %d:%-9s [%s]", 
           stationId, stn.telemetry.station_name, stn.isOnline ? "OK" : " X");
  writeLcdLine(0, lineBuf);

  // Line 1: Mode state
  if (!stn.isOnline) {
    writeLcdLine(1, "MODE: UNREACHABLE   ");
  } else if (stn.telemetry.status == CMD_EMERGENCY_STOP || stn.telemetry.status == CMD_MASTER_SHUTDOWN) {
    writeLcdLine(1, "MODE: EMERGENCY STOP");
  } else if (stn.telemetry.status == CMD_POWER_SAVE) {
    writeLcdLine(1, "MODE: POWER SAVE    ");
  } else if (stn.telemetry.status == CMD_RUN_TEST) {
    writeLcdLine(1, "MODE: DIAGNOSTIC    ");
  } else {
    writeLcdLine(1, "MODE: ACTIVE RUN    ");
  }

  // Line 2: Sensor Metric + Car Presence
  if (stn.isOnline) {
    const char* unit = (stationId == 1) ? "RPM" : (stationId == 2) ? "W" : "L/m";
    snprintf(lineBuf, sizeof(lineBuf), "CAR:%-3s|%5.1f%-3s ON:%s", 
             stn.telemetry.car_status ? "YES" : "NO", 
             stn.telemetry.sensor_metric, unit,
             stn.telemetry.actuator_state ? "Y" : "N");
  } else {
    snprintf(lineBuf, sizeof(lineBuf), "CAR:N/A | MTRC: N/A ");
  }
  writeLcdLine(2, lineBuf);

  // Line 3: Battery & System Voltage
  if (stn.isOnline) {
    snprintf(lineBuf, sizeof(lineBuf), "BAT:%3d%% | V:%4.1fV", stn.telemetry.battery_pct, stn.telemetry.voltage);
  } else {
    snprintf(lineBuf, sizeof(lineBuf), "BAT: ---%% | V: --.-V");
  }
  writeLcdLine(3, lineBuf);
}

void updateLcdDisplay() {
  if (lcdNeedsClear) {
    lcd.clear();
    lcdNeedsClear = false;
  }
  if (currentViewMode == 0) {
    renderOverviewLCD();
  } else {
    renderStationDetailLCD(currentViewMode);
  }
}

void selectDisplayView(uint8_t viewMode) {
  if (viewMode <= 3 && viewMode != currentViewMode) {
    currentViewMode = viewMode;
    lcdNeedsClear = true;
    updateLcdDisplay();
    Serial.printf("[LCD] View mode set to: %d\n", viewMode);
  }
}

void cycleDisplayView() {
  uint8_t nextView = currentViewMode + 1;
  if (nextView > 3) nextView = 0;
  selectDisplayView(nextView);
}

// ==========================================
// 3. ESP-NOW TRANSMISSION & PIPELINE
// ==========================================

void sendCommand(uint8_t target_id, CommandType cmd, uint16_t param = 0, uint16_t extra = 0) {
  ControllerCommandPayload cmdPacket;
  cmdPacket.packet_type = PKT_CMD;
  cmdPacket.target_station_id = target_id;
  cmdPacket.command = (uint8_t)cmd;
  cmdPacket.parameter = param;
  cmdPacket.extra_param = extra;

  if (target_id == 0) {
    for (int i = 1; i <= 3; i++) {
      esp_now_send(STATION_MACS[i], (uint8_t *)&cmdPacket, sizeof(cmdPacket));
    }
    Serial.printf("[NOW TX] Broadcast Command 0x%02X sent to ALL stations.\n", cmd);
  } else if (target_id >= 1 && target_id <= 3) {
    esp_now_send(STATION_MACS[target_id], (uint8_t *)&cmdPacket, sizeof(cmdPacket));
    Serial.printf("[NOW TX] Command 0x%02X sent to Station %d (Param: %d)\n", cmd, target_id, param);
  }
}

void sendBroadcastBanner(const char* message) {
  CustomBannerPayload bannerPacket;
  bannerPacket.packet_type = PKT_BANNER;
  bannerPacket.target_station_id = 0; // Broadcast
  bannerPacket.command = (uint8_t)CMD_BROADCAST_BANNER;
  snprintf(bannerPacket.banner_text, sizeof(bannerPacket.banner_text), "%-20s", message);

  for (int i = 1; i <= 3; i++) {
    esp_now_send(STATION_MACS[i], (uint8_t *)&bannerPacket, sizeof(bannerPacket));
  }
  Serial.printf("[NOW TX] Banner Broadcast Sent: '%s'\n", bannerPacket.banner_text);
}

void processIncomingTelemetry(const StationTelemetryPayload &incoming) {
  uint8_t id = incoming.station_id;
  if (id < 1 || id > 3) return;

  stationRegistry[id].telemetry = incoming;
  stationRegistry[id].lastSeen = millis();
  stationRegistry[id].isOnline = true;
}

void updateStationHealth() {
  unsigned long now = millis();
  for (uint8_t i = 1; i <= 3; i++) {
    if (stationRegistry[i].isOnline && (now - stationRegistry[i].lastSeen > HEARTBEAT_TIMEOUT)) {
      stationRegistry[i].isOnline = false;
      stationRegistry[i].telemetry.status = 0;
      Serial.printf("[WARN] Connection lost to Station %d\n", i);
    }
  }
}

void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  if (len == sizeof(StationTelemetryPayload)) {
    StationTelemetryPayload tempPayload;
    memcpy(&tempPayload, incomingData, sizeof(tempPayload));
    if (tempPayload.packet_type == PKT_TELEMETRY) {
      processIncomingTelemetry(tempPayload);
    }
  } else if (len == sizeof(CommandAckPayload)) {
    CommandAckPayload ackPayload;
    memcpy(&ackPayload, incomingData, sizeof(ackPayload));
    if (ackPayload.packet_type == PKT_ACK && ackPayload.station_id >= 1 && ackPayload.station_id <= 3) {
      stationRegistry[ackPayload.station_id].lastAckCmd = ackPayload.command_received;
      stationRegistry[ackPayload.station_id].lastAckSuccess = (ackPayload.success == 1);
      Serial.printf("[ACK RX] Station %d ACK Command 0x%02X -> Status: %s\n",
                    ackPayload.station_id, ackPayload.command_received, 
                    ackPayload.success ? "SUCCESS" : "FAILED");
    }
  }
}

void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {}

// ==========================================
// 4. NON-BLOCKING AUTOMATED SEQUENCES
// ==========================================

void startTestSequence(uint8_t targetId) {
  if (targetId >= 1 && targetId <= 3) {
    selectDisplayView(targetId);
    sendCommand(targetId, CMD_RUN_TEST);
  } else {
    Serial.println(F("[SEQ] Starting Non-Blocking System Diagnostic Sweep..."));
    currentRunMode = MODE_TEST_SEQUENCE;
    sequenceStep = 1;
    sequenceTimer = millis();
    selectDisplayView(1);
    sendCommand(1, CMD_RUN_TEST);
  }
}

void startDemoShowcase() {
  Serial.println(F("[DEMO] Launching Automated Microgrid Demo Showcase..."));
  currentRunMode = MODE_DEMO_SEQUENCE;
  sequenceStep = 1;
  sequenceTimer = millis();
  sendBroadcastBanner("DEMO MODE ACTIVE");
  sendCommand(0, CMD_SET_FLEET_SPEED, 80); // 80% Output
}

void processSequenceEngine() {
  if (currentRunMode == MODE_NORMAL) return;

  unsigned long now = millis();

  // Mode 1: Non-Blocking Diagnostic Sweep
  if (currentRunMode == MODE_TEST_SEQUENCE) {
    if (now - sequenceTimer >= 5000) { // 5 seconds per node
      sequenceTimer = now;
      sequenceStep++;
      if (sequenceStep <= 3) {
        selectDisplayView(sequenceStep);
        sendCommand(sequenceStep, CMD_RUN_TEST);
      } else {
        Serial.println(F("[SEQ] Diagnostic Sweep Complete. Returning to Overview."));
        currentRunMode = MODE_NORMAL;
        selectDisplayView(0);
      }
    }
  }
  // Mode 2: Automated Demo Showcase
  else if (currentRunMode == MODE_DEMO_SEQUENCE) {
    if (now - sequenceTimer >= 8000) { // 8 seconds per step
      sequenceTimer = now;
      sequenceStep++;
      if (sequenceStep == 2) {
        selectDisplayView(1); // Focus Wind
        sendCommand(1, CMD_ACTUATOR_CTRL, 1);
      } else if (sequenceStep == 3) {
        selectDisplayView(2); // Focus Solar
        sendCommand(2, CMD_ACTUATOR_CTRL, 1);
      } else if (sequenceStep == 4) {
        selectDisplayView(3); // Focus Hydro
        sendCommand(3, CMD_ACTUATOR_CTRL, 1);
      } else {
        Serial.println(F("[DEMO] Showcase Completed. Resetting to Normal Mode."));
        currentRunMode = MODE_NORMAL;
        sendCommand(0, CMD_ACTUATOR_CTRL, 0);
        selectDisplayView(0);
      }
    }
  }
}

// ==========================================
// 5. SERIAL COMMAND PARSER (C-STRING CLI)
// ==========================================

void printSystemStatusTable() {
  Serial.println(F("\n========================================================================================"));
  Serial.println(F("                                MCU 0 TELEMETRY MATRIX                                  "));
  Serial.println(F("========================================================================================"));
  Serial.println(F("ID | STN NAME   | LINK | STATUS     | CAR | ACTUATOR | METRIC     | VOLT  | BAT  | ACK  "));
  Serial.println(F("---+------------+------+------------+-----+----------+------------+-------+------+------"));

  for (uint8_t i = 1; i <= 3; i++) {
    StationRecord &stn = stationRegistry[i];
    char nameBuf[11];
    snprintf(nameBuf, sizeof(nameBuf), "%-10s", stn.telemetry.station_name);

    const char* statusStr = "UNREACH   ";
    if (stn.isOnline) {
      if (stn.telemetry.status == CMD_EMERGENCY_STOP || stn.telemetry.status == CMD_MASTER_SHUTDOWN) statusStr = "E-STOP    ";
      else if (stn.telemetry.status == CMD_RUN_TEST)   statusStr = "DIAGNOSTIC";
      else if (stn.telemetry.status == CMD_POWER_SAVE) statusStr = "POWER SAVE";
      else                                              statusStr = "ACTIVE RUN";
    }

    char metricBuf[12];
    if (stn.isOnline) {
      const char* unit = (i == 1) ? "RPM" : (i == 2) ? "W  " : "L/m";
      snprintf(metricBuf, sizeof(metricBuf), "%6.1f %s", stn.telemetry.sensor_metric, unit);
    } else {
      strcpy(metricBuf, "   N/A    ");
    }

    char ackBuf[7];
    if (stn.lastAckCmd != 0) {
      snprintf(ackBuf, sizeof(ackBuf), "0x%02X:%s", stn.lastAckCmd, stn.lastAckSuccess ? "OK" : "ER");
    } else {
      strcpy(ackBuf, "NONE ");
    }

    Serial.printf("%-2d | %s | %s  | %s | %s | %-8s | %s | %4.1fV | %3d%% | %s\n",
                  i, nameBuf, stn.isOnline ? "OK" : " X", statusStr, 
                  stn.isOnline ? (stn.telemetry.car_status ? "YES" : "NO ") : "N/A",
                  stn.isOnline ? (stn.telemetry.actuator_state ? "ACTIVE" : "OFF") : "N/A",
                  metricBuf, stn.isOnline ? stn.telemetry.voltage : 0.0f,
                  stn.isOnline ? stn.telemetry.battery_pct : 0, ackBuf);
  }
  Serial.println(F("========================================================================================\n"));
}

void parseAndExecuteCLI(char* line) {
  // Navigation & Shortcuts
  if (strcmp(line, "n") == 0 || strcmp(line, "next") == 0) { cycleDisplayView(); return; }
  if (strcmp(line, "v0") == 0) { selectDisplayView(0); return; }
  if (strcmp(line, "v1") == 0) { selectDisplayView(1); return; }
  if (strcmp(line, "v2") == 0) { selectDisplayView(2); return; }
  if (strcmp(line, "v3") == 0) { selectDisplayView(3); return; }
  if (strcmp(line, "help") == 0 || strcmp(line, "?") == 0) { printHelpMenu(); return; }
  if (strcmp(line, "status") == 0 || strcmp(line, "s") == 0) { printSystemStatusTable(); return; }

  // Executive Emergency Shutdown
  if (strcmp(line, "shutdown") == 0) {
    currentRunMode = MODE_NORMAL; // Cancel ongoing test/demo
    sendCommand(0, CMD_MASTER_SHUTDOWN);
    Serial.println(F("[ALERT] MASTER SHUTDOWN DISPATCHED NETWORK-WIDE!"));
    return;
  }

  // E-STOP Command Shortcuts
  if (strcmp(line, "estop") == 0 || strcmp(line, "e") == 0 || strcmp(line, "e0") == 0) {
    currentRunMode = MODE_NORMAL;
    sendCommand(0, CMD_EMERGENCY_STOP);
    return;
  }
  if (strcmp(line, "e1") == 0) { sendCommand(1, CMD_EMERGENCY_STOP); return; }
  if (strcmp(line, "e2") == 0) { sendCommand(2, CMD_EMERGENCY_STOP); return; }
  if (strcmp(line, "e3") == 0) { sendCommand(3, CMD_EMERGENCY_STOP); return; }

  // Power & Test Shortcuts
  if (strcmp(line, "demo") == 0) { startDemoShowcase(); return; }
  if (strcmp(line, "t1") == 0)   { startTestSequence(1); return; }
  if (strcmp(line, "t2") == 0)   { startTestSequence(2); return; }
  if (strcmp(line, "t3") == 0)   { startTestSequence(3); return; }
  if (strcmp(line, "tall") == 0 || strcmp(line, "t0") == 0) { startTestSequence(0); return; }
  if (strcmp(line, "p1") == 0)   { sendCommand(1, CMD_POWER_SAVE); return; }
  if (strcmp(line, "p2") == 0)   { sendCommand(2, CMD_POWER_SAVE); return; }
  if (strcmp(line, "p3") == 0)   { sendCommand(3, CMD_POWER_SAVE); return; }
  if (strcmp(line, "pall") == 0 || strcmp(line, "p0") == 0) { sendCommand(0, CMD_POWER_SAVE); return; }
  if (strcmp(line, "w0") == 0)   { sendCommand(0, CMD_WAKEUP); return; }
  if (strcmp(line, "w1") == 0)   { sendCommand(1, CMD_WAKEUP); return; }
  if (strcmp(line, "w2") == 0)   { sendCommand(2, CMD_WAKEUP); return; }
  if (strcmp(line, "w3") == 0)   { sendCommand(3, CMD_WAKEUP); return; }

  // Parameterized Tokenizer ("command arg1 arg2")
  char* command = strtok(line, " ");
  char* arg1    = strtok(NULL, " ");
  char* arg2    = strtok(NULL, " ");

  if (!command) return;

  if (strcmp(command, "speed") == 0 && arg1) {
    uint16_t speedVal = constrain(atoi(arg1), 0, 100);
    sendCommand(0, CMD_SET_FLEET_SPEED, speedVal);
  }
  else if (strcmp(command, "banner") == 0) {
    // Reassemble full text string after "banner "
    char* fullText = line + strlen("banner") + 1;
    sendBroadcastBanner(fullText);
  }
  else if (strcmp(command, "act") == 0 && arg1 && arg2) {
    sendCommand(atoi(arg1), CMD_ACTUATOR_CTRL, atoi(arg2));
  }
  else if (strcmp(command, "cfg") == 0 && arg1 && arg2) {
    sendCommand(atoi(arg1), CMD_SET_CONFIG, atoi(arg2));
  }
  else if (strcmp(command, "boot") == 0 && arg1) {
    sendCommand(atoi(arg1), CMD_REBOOT);
  }
  else {
    Serial.println(F("[CLI] Command unrecognized. Type 'help' or '?' for menu."));
  }
}

void processSerialInput() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\r') continue; // Ignore carriage returns
    if (c == '\n') {
      cliInputBuffer[cliBufferIndex] = '\0'; // Null terminate
      if (cliBufferIndex > 0) {
        parseAndExecuteCLI(cliInputBuffer);
      }
      cliBufferIndex = 0; // Reset buffer
    } else if (cliBufferIndex < sizeof(cliInputBuffer) - 1) {
      cliInputBuffer[cliBufferIndex++] = c;
    }
  }
}

// ==========================================
// 6. SETUP & MAIN LOOP
// ==========================================

void setup() {
  Serial.begin(115200);
  delay(500);

  strcpy(stationRegistry[1].telemetry.station_name, "Wind Turb");
  strcpy(stationRegistry[2].telemetry.station_name, "Solar Farm");
  strcpy(stationRegistry[3].telemetry.station_name, "Hydro Stn");

  for (int i = 1; i <= 3; i++) {
    stationRegistry[i].isOnline = false;
    stationRegistry[i].telemetry.station_id = i;
    stationRegistry[i].lastAckCmd = 0;
  }

  Wire.begin(21, 22);
  lcd.init();
  lcd.backlight();

  writeLcdLine(0, "====================");
  writeLcdLine(1, " MCU 0 MASTER CTRL  ");
  writeLcdLine(2, " INITIALIZING...    ");
  writeLcdLine(3, "====================");
  delay(1200);

  WiFi.mode(WIFI_STA);
  Serial.print(F("[SYSTEM] MCU 0 STA MAC Address: "));
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println(F("[ERROR] ESP-NOW Init Failed!"));
    writeLcdLine(2, " ESP-NOW INIT FAIL  ");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);

  esp_now_peer_info_t peerInfo = {};
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  for (int i = 1; i <= 3; i++) {
    memcpy(peerInfo.peer_addr, STATION_MACS[i], 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.printf("[WARN] Peer registration failed for Station %d\n", i);
    }
  }

  lcd.clear();
  updateLcdDisplay();
  printHelpMenu();
  Serial.println(F("[READY] MCU 0 Master Orchestrator running."));
}

void loop() {
  updateStationHealth();
  processSequenceEngine(); // Non-blocking state machine for tests/demos

  if (millis() - lastLcdRefresh >= LCD_REFRESH_INTERVAL) {
    lastLcdRefresh = millis();
    updateLcdDisplay();
  }

  processSerialInput(); // Non-blocking C-string CLI parser
  delay(5);
}
