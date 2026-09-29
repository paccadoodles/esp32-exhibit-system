#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "espnow_protocol.h"

// ==========================================
// HARDWARE & DISPLAY CONFIGURATION
// Note: Change 0x27 to 0x3F if display remains completely blank
// ==========================================
LiquidCrystal_I2C lcd(0x27, 20, 4);

struct StationRecord {
  StationTelemetryPayload telemetry;
  unsigned long lastSeen;
  bool isOnline;
};

StationRecord stationRegistry[4];             // Indices 1, 2, 3 map to stations
const unsigned long HEARTBEAT_TIMEOUT = 6000; // 6 seconds before marked offline

uint8_t activeViewStation = 1;                // Currently displayed station on LCD
bool autoCarousel = true;                     // Auto-cycle screen between stations
unsigned long lastCarouselSwap = 0;
const unsigned long CAROUSEL_INTERVAL = 4000; // Cycle station view every 4s

unsigned long lastLcdRefresh = 0;
const unsigned long LCD_REFRESH_INTERVAL = 250;

// ==========================================
// MODULAR SYSTEM FUNCTIONS
// ==========================================

// Write padded 20-character line to eliminate screen flicker
void writeLcdLine(uint8_t line, const char* formattedText) {
  char lineBuffer[21];
  snprintf(lineBuffer, sizeof(lineBuffer), "%-20s", formattedText);
  lcd.setCursor(0, line);
  lcd.print(lineBuffer);
}

void renderStationViewLCD(uint8_t stationId) {
  if (stationId < 1 || stationId > 3) return;
  StationRecord &stn = stationRegistry[stationId];
  char lineBuf[21];

  // Line 0: Header & Link
  if (!stn.isOnline) {
    snprintf(lineBuf, sizeof(lineBuf), "STN %d:%-10s [OFF]", stationId, stn.telemetry.station_name);
  } else {
    snprintf(lineBuf, sizeof(lineBuf), "STN %d:%-10s  [OK]", stationId, stn.telemetry.station_name);
  }
  writeLcdLine(0, lineBuf);

  // Line 1: Mode
  if (!stn.isOnline) {
    writeLcdLine(1, "MODE: UNREACHABLE   ");
  } else if (stn.telemetry.status == 3) {
    writeLcdLine(1, "MODE: POWER SAVE    ");
  } else if (stn.telemetry.status == 2) {
    writeLcdLine(1, "MODE: DIAGNOSTIC    ");
  } else {
    writeLcdLine(1, "MODE: ACTIVE RUN    ");
  }

  // Line 2: Presence Sensor
  if (stn.isOnline) {
    snprintf(lineBuf, sizeof(lineBuf), "CAR PRESENCE: %-6s", stn.telemetry.car_status ? "DETECT" : "NONE");
  } else {
    writeLcdLine(2, "CAR PRESENCE: N/A   ");
  }

  // Line 3: Battery & View Indicator
  if (stn.isOnline) {
    snprintf(lineBuf, sizeof(lineBuf), "BAT:%3d%% | VIEW:[STN%d]", stn.telemetry.battery_pct, stationId);
  } else {
    snprintf(lineBuf, sizeof(lineBuf), "BAT: ---%% | VIEW:[STN%d]", stationId);
  }
  writeLcdLine(3, lineBuf);
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
      stationRegistry[i].telemetry.status = 0; // Marked as Offline
      
      Serial.print(F("[WARN] Connection lost to Station "));
      Serial.println(i);
    }
  }
}

// ==========================================
// ESP-NOW TRANSMISSION PIPELINE
// ==========================================

void sendCommand(uint8_t target_id, CommandType cmd, uint16_t param = 0) {
  ControllerCommandPayload cmdPacket;
  cmdPacket.target_station_id = target_id;
  cmdPacket.command = (uint8_t)cmd;
  cmdPacket.parameter = param;

  if (target_id == 0) {
    for (int i = 1; i <= 3; i++) {
      esp_now_send(STATION_MACS[i], (uint8_t *)&cmdPacket, sizeof(cmdPacket));
    }
    Serial.println(F("[NOW TX] Dispatched command to ALL stations."));
  } else if (target_id >= 1 && target_id <= 3) {
    esp_now_send(STATION_MACS[target_id], (uint8_t *)&cmdPacket, sizeof(cmdPacket));
    Serial.printf("[NOW TX] Dispatched command to Station %d\n", target_id);
  } else {
    Serial.println(F("[ERROR] Invalid Station ID specified."));
  }
}

void handleTestCommand(uint8_t targetId) {
  if (targetId >= 1 && targetId <= 3) {
    activeViewStation = targetId;
    autoCarousel = false;
    sendCommand(targetId, CMD_RUN_TEST);
  } 
  else if (targetId == 0) {
    Serial.println(F("\n[TEST MASTER] Initiating Sequential Global System Test (Stations 1 -> 3)..."));
    autoCarousel = false;

    for (uint8_t id = 1; id <= 3; id++) {
      activeViewStation = id;
      sendCommand(id, CMD_RUN_TEST);

      // Wait 6.5 seconds while actively updating MCU 0 LCD
      unsigned long testStart = millis();
      while (millis() - testStart < 6500) {
        updateStationHealth();

        if (millis() - lastLcdRefresh >= LCD_REFRESH_INTERVAL) {
          lastLcdRefresh = millis();
          renderStationViewLCD(activeViewStation);
        }
        delay(10);
      }
    }

    Serial.println(F("[TEST MASTER] Sequential Global System Test Completed.\n"));
    autoCarousel = true;
  }
}

void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  if (len != sizeof(StationTelemetryPayload)) return;

  StationTelemetryPayload tempPayload;
  memcpy(&tempPayload, incomingData, sizeof(tempPayload));
  processIncomingTelemetry(tempPayload);
}

void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {}

// ==========================================
// DIAGNOSTIC CLI MODULES
// ==========================================

void printSystemStatusTable() {
  unsigned long now = millis();

  Serial.println(F("\n=================================================================="));
  Serial.println(F("                       EXHIBIT SYSTEM STATUS                      "));
  Serial.println(F("=================================================================="));
  Serial.println(F("ID | STN NAME   | LINK | STATUS     | CAR DETECT | BAT  | LAST SEEN"));
  Serial.println(F("---+------------+------+------------+------------+------+---------"));

  for (uint8_t i = 1; i <= 3; i++) {
    StationRecord &stn = stationRegistry[i];
    
    char nameBuf[11];
    snprintf(nameBuf, sizeof(nameBuf), "%-10s", stn.telemetry.station_name);

    const char* linkStr = stn.isOnline ? "OK  " : "OFF ";
    
    const char* statusStr = "UNREACH   ";
    if (stn.isOnline) {
      if (stn.telemetry.status == 1)      statusStr = "ACTIVE RUN";
      else if (stn.telemetry.status == 2) statusStr = "DIAGNOSTIC";
      else if (stn.telemetry.status == 3) statusStr = "POWER SAVE";
    }

    const char* carStr = stn.isOnline ? (stn.telemetry.car_status ? "YES       " : "NO        ") : "N/A       ";

    char batBuf[6];
    if (stn.isOnline) snprintf(batBuf, sizeof(batBuf), "%3d%%", stn.telemetry.battery_pct);
    else              strcpy(batBuf, " ---%");

    char ageBuf[12];
    if (stn.isOnline) snprintf(ageBuf, sizeof(ageBuf), "%lums ago", now - stn.lastSeen);
    else              strcpy(ageBuf, "TIMEOUT");

    Serial.printf("%-2d | %s | %s | %s | %s | %s | %s\n",
                  i, nameBuf, linkStr, statusStr, carStr, batBuf, ageBuf);
  }
  Serial.println(F("==================================================================\n"));
}

void handleSerialCommands() {
  if (!Serial.available()) return;

  String input = Serial.readStringUntil('\n');
  input.trim();
  if (input.length() == 0) return;

  // Quick Shortcuts
  if (input == "t1")   { handleTestCommand(1); return; }
  if (input == "t2")   { handleTestCommand(2); return; }
  if (input == "t3")   { handleTestCommand(3); return; }
  if (input == "tall" || input == "t0") { handleTestCommand(0); return; }
  if (input == "p1")   { sendCommand(1, CMD_POWER_SAVE); return; }
  if (input == "p2")   { sendCommand(2, CMD_POWER_SAVE); return; }
  if (input == "p3")   { sendCommand(3, CMD_POWER_SAVE); return; }
  if (input == "pall" || input == "p0") { sendCommand(0, CMD_POWER_SAVE); return; }
  if (input == "v1")   { activeViewStation = 1; autoCarousel = false; return; }
  if (input == "v2")   { activeViewStation = 2; autoCarousel = false; return; }
  if (input == "v3")   { activeViewStation = 3; autoCarousel = false; return; }
  if (input == "c")    { autoCarousel = true; return; }

  // Tokenize Input: "action target_id parameter"
  int firstSpace  = input.indexOf(' ');
  int secondSpace = input.indexOf(' ', firstSpace + 1);

  String action = (firstSpace == -1) ? input : input.substring(0, firstSpace);
  action.toLowerCase();

  uint8_t targetId = 0;
  uint16_t param   = 0;

  if (firstSpace != -1) {
    targetId = (secondSpace == -1) ? input.substring(firstSpace + 1).toInt() 
                                   : input.substring(firstSpace + 1, secondSpace).toInt();
  }
  if (secondSpace != -1) {
    param = input.substring(secondSpace + 1).toInt();
  }

  if (action == "test" || action == "t") {
    handleTestCommand(targetId);
  } 
  else if (action == "sleep" || action == "p") {
    sendCommand(targetId, CMD_POWER_SAVE);
  } 
  else if (action == "wake" || action == "w") {
    sendCommand(targetId, CMD_WAKEUP);
  } 
  else if (action == "set") {
    sendCommand(targetId, CMD_SET_PWM_VAL, param);
  } 
  else if (action == "status" || action == "stat") {
    printSystemStatusTable();
  } 
  else if (action == "view" || action == "v") {
    if (targetId >= 1 && targetId <= 3) {
      activeViewStation = targetId;
      autoCarousel = false;
      Serial.printf("[LCD] Locked view to Station %d\n", targetId);
    }
  } 
  else {
    Serial.println(F("[CLI] Unknown command. Type 'status', 'test 0', 'sleep 1', or 'view 1'."));
  }
}

// ==========================================
// SETUP & MAIN LOOP
// ==========================================

void setup() {
  Serial.begin(115200);
  delay(500);

  // Default labels so screen renders immediately before telemetry arrives
  strcpy(stationRegistry[1].telemetry.station_name, "Wind Turb");
  strcpy(stationRegistry[2].telemetry.station_name, "Solar Farm");
  strcpy(stationRegistry[3].telemetry.station_name, "Hydro Stn");

  for (int i = 1; i <= 3; i++) {
    stationRegistry[i].isOnline = false;
    stationRegistry[i].telemetry.station_id = i;
  }

  // 1. Initialize LCD Hardware
  Wire.begin(21, 22); // SDA = GPIO 21, SCL = GPIO 22
  lcd.init();
  lcd.backlight();

  writeLcdLine(0, "====================");
  writeLcdLine(1, " MCU 0 CONTROLLER   ");
  writeLcdLine(2, " INITIALIZING...    ");
  writeLcdLine(3, "====================");
  delay(1500);

  // 2. Initialize Wi-Fi & ESP-NOW
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

  // 3. Register Station Peers
  esp_now_peer_info_t peerInfo = {};
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  for (int i = 1; i <= 3; i++) {
    memcpy(peerInfo.peer_addr, STATION_MACS[i], 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.printf("[WARN] Peer reg failed for Station %d\n", i);
    }
  }

  lcd.clear();
  renderStationViewLCD(activeViewStation);
  Serial.println(F("[READY] MCU 0 initialized successfully."));
}

void loop() {
  // 1. Watchdog evaluation
  updateStationHealth();

  // 2. Screen Carousel Timer
  if (autoCarousel && (millis() - lastCarouselSwap >= CAROUSEL_INTERVAL)) {
    lastCarouselSwap = millis();
    activeViewStation++;
    if (activeViewStation > 3) activeViewStation = 1;
  }

  // 3. Refresh LCD
  if (millis() - lastLcdRefresh >= LCD_REFRESH_INTERVAL) {
    lastLcdRefresh = millis();
    renderStationViewLCD(activeViewStation);
  }

  // 4. CLI Input Processing
  handleSerialCommands();

  delay(10);
}