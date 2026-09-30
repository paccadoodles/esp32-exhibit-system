// ==========================================
// MCU 0 - MAIN CONTROLLER SKETCH
// ==========================================
#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "espnow_protocol.h"

LiquidCrystal_I2C lcd(0x27, 20, 4);

struct StationRecord {
  StationTelemetryPayload telemetry;
  unsigned long lastSeen;
  bool isOnline;
  uint8_t lastAckCmd;
  bool lastAckSuccess;
};

StationRecord stationRegistry[4];
const unsigned long HEARTBEAT_TIMEOUT = 6000;

uint8_t currentViewMode = 0; // 0 = Overview, 1..3 = Station Detail Views
unsigned long lastLcdRefresh = 0;
const unsigned long LCD_REFRESH_INTERVAL = 300;

// ==========================================
// DISPLAY ENGINE
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
  Serial.println(F("   estop / e   : EMERGENCY STOP ALL STATIONS (e1, e2, e3 for single)"));
  Serial.println(F("   pall / p0   : Put ALL stations to Sleep (p1, p2, p3 for single)"));
  Serial.println(F("   w0..w3      : Wake up ALL (w0) or target station (w1, w2, w3)\n"));
  Serial.println(F(" DIAGNOSTICS & HARDWARE CONTROL:"));
  Serial.println(F("   tall / t0   : Trigger sequential diagnostic test (t1, t2, t3 for single)"));
  Serial.println(F("   act <id> <1|0>   : Remote toggle station actuator (e.g. 'act 1 1')"));
  Serial.println(F("   cfg <id> <ms>    : Set remote telemetry interval (e.g. 'cfg 2 1000')"));
  Serial.println(F("   boot <id>        : Force remote station reboot (e.g. 'boot 3')"));
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
  } else if (stn.telemetry.status == CMD_EMERGENCY_STOP) {
    writeLcdLine(1, "MODE: EMERGENCY STOP");
  } else if (stn.telemetry.status == CMD_POWER_SAVE) {
    writeLcdLine(1, "MODE: POWER SAVE    ");
  } else if (stn.telemetry.status == CMD_RUN_TEST) {
    writeLcdLine(1, "MODE: DIAGNOSTIC    ");
  } else {
    writeLcdLine(1, "MODE: ACTIVE RUN    ");
  }

  // Line 2: Sensor Metric + Car Presence (BUG FIXED: Outputted to Line 2)
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
  if (currentViewMode == 0) {
    renderOverviewLCD();
  } else {
    renderStationDetailLCD(currentViewMode);
  }
}

void selectDisplayView(uint8_t viewMode) {
  if (viewMode <= 3 && viewMode != currentViewMode) {
    currentViewMode = viewMode;
    lcd.clear(); // Prevents layout ghosting across view changes
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
// ESP-NOW PIPELINE & ACK HANDSHAKE
// ==========================================

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

void sendCommand(uint8_t target_id, CommandType cmd, uint16_t param = 0, float extra = 0.0f) {
  ControllerCommandPayload cmdPacket;
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

void handleTestCommand(uint8_t targetId) {
  if (targetId >= 1 && targetId <= 3) {
    selectDisplayView(targetId);
    sendCommand(targetId, CMD_RUN_TEST);
  } 
  else if (targetId == 0) {
    Serial.println(F("\n[TEST MASTER] Initiating Global Sequential System Test (Stations 1 -> 3)..."));

    for (uint8_t id = 1; id <= 3; id++) {
      selectDisplayView(id);
      sendCommand(id, CMD_RUN_TEST);

      unsigned long testStart = millis();
      while (millis() - testStart < 6500) {
        updateStationHealth();
        if (millis() - lastLcdRefresh >= LCD_REFRESH_INTERVAL) {
          lastLcdRefresh = millis();
          updateLcdDisplay();
        }
        delay(10);
      }
    }

    Serial.println(F("[TEST MASTER] Global Sequential Test Complete.\n"));
    selectDisplayView(0);
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
// SERIAL COMMAND INTERACTION (CLI)
// ==========================================

void printSystemStatusTable() {
  unsigned long now = millis();
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
      if (stn.telemetry.status == CMD_EMERGENCY_STOP) statusStr = "E-STOP    ";
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

void handleSerialCommands() {
  if (!Serial.available()) return;

  String input = Serial.readStringUntil('\n');
  input.trim();
  if (input.length() == 0) return;

  // Shortcuts & Direct Actions
  if (input == "n" || input == "next") { cycleDisplayView(); return; }
  if (input == "v0") { selectDisplayView(0); return; }
  if (input == "v1") { selectDisplayView(1); return; }
  if (input == "v2") { selectDisplayView(2); return; }
  if (input == "v3") { selectDisplayView(3); return; }
  if (input == "help" || input == "?") { printHelpMenu(); return; }
  if (input == "status" || input == "s") { printSystemStatusTable(); return; }

  // E-STOP Command Shortcuts
  if (input == "estop" || input == "e" || input == "e0") { sendCommand(0, CMD_EMERGENCY_STOP); return; }
  if (input == "e1") { sendCommand(1, CMD_EMERGENCY_STOP); return; }
  if (input == "e2") { sendCommand(2, CMD_EMERGENCY_STOP); return; }
  if (input == "e3") { sendCommand(3, CMD_EMERGENCY_STOP); return; }

  // Test & Power Shortcuts
  if (input == "t1")   { handleTestCommand(1); return; }
  if (input == "t2")   { handleTestCommand(2); return; }
  if (input == "t3")   { handleTestCommand(3); return; }
  if (input == "tall" || input == "t0") { handleTestCommand(0); return; }
  if (input == "p1")   { sendCommand(1, CMD_POWER_SAVE); return; }
  if (input == "p2")   { sendCommand(2, CMD_POWER_SAVE); return; }
  if (input == "p3")   { sendCommand(3, CMD_POWER_SAVE); return; }
  if (input == "pall" || input == "p0") { sendCommand(0, CMD_POWER_SAVE); return; }
  if (input == "w0")   { sendCommand(0, CMD_WAKEUP); return; }
  if (input == "w1")   { sendCommand(1, CMD_WAKEUP); return; }
  if (input == "w2")   { sendCommand(2, CMD_WAKEUP); return; }
  if (input == "w3")   { sendCommand(3, CMD_WAKEUP); return; }

  // Advanced Command Parser ("action target_id param extra")
  int firstSpace = input.indexOf(' ');
  String action = (firstSpace == -1) ? input : input.substring(0, firstSpace);
  action.toLowerCase();
  
  String rest = (firstSpace != -1) ? input.substring(firstSpace + 1) : "";
  int secondSpace = rest.indexOf(' ');
  uint8_t targetId = (secondSpace == -1) ? rest.toInt() : rest.substring(0, secondSpace).toInt();
  uint16_t param = (secondSpace != -1) ? rest.substring(secondSpace + 1).toInt() : 0;

  if (action == "act")         sendCommand(targetId, CMD_ACTUATOR_CTRL, param);
  else if (action == "cfg")    sendCommand(targetId, CMD_SET_CONFIG, param);
  else if (action == "boot")   sendCommand(targetId, CMD_REBOOT);
  else {
    Serial.println(F("[CLI] Command unrecognized. Type 'help' or '?' for syntax."));
  }
}

// ==========================================
// SETUP & MAIN LOOP
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
  Serial.print(F("[SYSTEM] MCU 0 MAC Address: "));
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
  Serial.println(F("[READY] MCU 0 Master Controller active."));
}

void loop() {
  updateStationHealth();

  if (millis() - lastLcdRefresh >= LCD_REFRESH_INTERVAL) {
    lastLcdRefresh = millis();
    updateLcdDisplay();
  }

  handleSerialCommands();
  delay(10);
}
