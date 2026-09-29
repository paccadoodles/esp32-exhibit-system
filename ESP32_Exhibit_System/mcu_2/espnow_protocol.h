#ifndef ESPNOW_PROTOCOL_H
#define ESPNOW_PROTOCOL_H

#include 

// ==========================================
// 1. MAC ADDRESS DEFINITIONS & ALIASES
// ==========================================
// Central Controller MAC Address (Replace with your MCU 0's real MAC)
static const uint8_t MAC_CONTROLLER[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; 

// Direct Alias to fix 'MCU0_MAC' undeclared errors across station files
#define MCU0_MAC MAC_CONTROLLER

// Broadcast MAC (sends to all ESP-NOW devices on channel)
static const uint8_t MAC_BROADCAST[6]  = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; 

// ==========================================
// 2. PACKET IDENTIFIERS
// ==========================================
typedef enum {
    PKT_TELEMETRY = 1,
    PKT_CMD       = 2,
    PKT_ACK       = 3,
    PKT_BANNER    = 4
} PacketType;

// ==========================================
// 3. FULL NETWORK COMMAND ENUM
// ==========================================
typedef enum {
    CMD_NONE            = 0,
    
    // Core Station Commands
    CMD_RUN_TEST        = 1,
    CMD_POWER_SAVE      = 2,
    CMD_WAKEUP          = 3,
    CMD_EMERGENCY_STOP  = 4,
    CMD_ACTUATOR_CTRL   = 5,
    CMD_SET_CONFIG      = 6,
    CMD_REBOOT          = 7,
    CMD_SET_PWM_VAL     = 8,
    CMD_PING            = 9,
    
    // Executive Boss Commands
    CMD_MASTER_SHUTDOWN = 10,
    CMD_SEQUENTIAL_BOOT = 11,
    CMD_SET_FLEET_SPEED = 12,
    CMD_SYSTEM_AUDIT    = 13,
    CMD_BROADCAST_BANNER= 14,
    CMD_DEMO_SHOWCASE   = 15
} CommandType;

// ==========================================
// 4. PACKED ESP-NOW PAYLOAD STRUCTURES
// ==========================================

// Telemetry Payload (Sent from MCU 1, 2, 3 -> MCU 0)
typedef struct __attribute__((packed)) {
    uint8_t  packet_type;      // PKT_TELEMETRY
    uint8_t  station_id;       // 1 = Wind, 2 = Solar, 3 = Hydro
    char     station_name[16]; // Human-readable station tag
    uint8_t  status;           // 0 = Boot/Error, 1 = Active Run, 2 = Testing, 3 = Soft-Sleep, 4 = Emergency
    uint8_t  battery_pct;      // System battery level (0-100%)
    uint8_t  car_status;       // 0 = Clear, 1 = Proximity Triggered (<25cm)
    uint8_t  actuator_state;   // 0 = OFF/Closed, 1 = ON/Open
    float    voltage;          // Live bus voltage
    float    sensor_metric;    // Dynamic metric (Wind RPM, Solar Watts, Hydro L/min)
} StationTelemetryPayload;

// Controller Command Payload (Sent from MCU 0 -> Stations)
typedef struct __attribute__((packed)) {
    uint8_t  packet_type;        // PKT_CMD
    uint8_t  target_station_id;  // 0 = Broadcast to All, 1 = Wind, 2 = Solar, 3 = Hydro
    uint8_t  command;            // Enum from CommandType
    uint16_t parameter;          // Primary command parameter (e.g., speed, interval)
    uint16_t extra_param;        // Secondary option flag or timing parameter
} ControllerCommandPayload;

// Command Acknowledgement Payload (Sent from Stations -> MCU 0)
typedef struct __attribute__((packed)) {
    uint8_t  packet_type;        // PKT_ACK
    uint8_t  station_id;         // Responding Station ID (1-3)
    uint8_t  command_received;   // The Command ID being acknowledged
    uint8_t  success;            // 1 = Command Executed OK, 0 = Failed/Rejected
} CommandAckPayload;

// Custom LCD Text Broadcast Payload (Sent from MCU 0 -> Stations)
typedef struct __attribute__((packed)) {
    uint8_t  packet_type;        // PKT_BANNER
    uint8_t  target_station_id;  // 0 = All Stations
    uint8_t  command;            // CMD_BROADCAST_BANNER
    char     banner_text[20];    // 20-character string override for LCD line 0
} CustomBannerPayload;

#endif // ESPNOW_PROTOCOL_H