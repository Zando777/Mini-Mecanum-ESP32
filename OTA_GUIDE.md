# ESP32 OTA Setup Guide

## Overview
Your ESP32 Mecanum Robot project now has complete OTA (Over-The-Air) firmware update capabilities. The setup includes both direct USB serial upload and wireless OTA updates.

## Upload Methods

### 1. **USB Serial Upload** (Initial Setup/Recovery)
- **Protocol**: Auto-detected by PlatformIO for ESP32 boards
- **Connection**: USB cable via CH340/CP2102 chip
- **Use Case**: First upload, recovery from corrupted firmware
- **Environment**: `upesy_wroom_upload`

### 2. **OTA Wireless Upload** (Regular Updates)
- **Protocol**: `espota` over WiFi
- **Connection**: Network connection to ESP32's IP
- **Use Case**: Regular firmware updates after initial setup
- **Environment**: `upesy_wroom` (default)

## Configuration Files

### 1. `platformio.ini` - Main Configuration
- **Default Environment**: `upesy_wroom` (OTA wireless updates)
- **USB Environment**: `upesy_wroom_upload` (USB serial upload)
- **OTA Upload Port**: Set to `192.168.2.126` (update with your ESP32's actual IP)
- **USB Port**: Set to `/dev/cu.usbserial-14420` (your actual USB port)

### 2. `huge_app.csv` - Flash Memory Layout
- **app0/ota_0**: Primary firmware partition (244KB)
- **app1/ota_1**: Secondary firmware partition (244KB)
- **otadata**: OTA metadata storage (8KB)
- **nvs**: Non-volatile storage (20KB)
- **spiffs**: File system storage (464KB)

## Setup Process

### Step 1: Initial USB Upload
1. Connect ESP32 via USB cable
2. Identify your USB port:
   ```bash
   pio device list
   ```
3. The `platformio.ini` has been automatically updated with your correct USB port: `/dev/cu.usbserial-14420`
4. Upload via PlatformIO IDE or command line:
   - **PlatformIO IDE**: Select `upesy_wroom_upload` environment → Upload
   - **Command Line**: `pio device upload --environment upesy_wroom_upload`

### Step 2: Find ESP32 IP Address
After successful USB upload, check the serial monitor for IP:
```
Connected! IP: 192.168.2.126
```

### Step 3: Configure OTA Upload
Update `platformio.ini` line 22 with your ESP32's IP:
```ini
upload_port = 192.168.2.126  ; Your ESP32's actual IP
```

### Step 4: Subsequent OTA Updates
Now use OTA for wireless updates:
- **PlatformIO IDE**: Select `upesy_wroom` environment → Upload
- **Command Line**: `pio device upload`

## Arduino IDE Usage

### Network Port Method
1. Install ESP32 board package in Arduino IDE
2. Go to **Tools > Ports**
3. Select **Network Ports > MecanumBot at 192.168.x.x**
4. Upload normally - OTA will be used automatically

### Manual OTA Upload
```bash
pio device upload --upload-protocol espota --upload-port 192.168.x.x
```

## Code Integration

### In `src/main.cpp`
- **OTA Hostname**: "MecanumBot"
- **OTA Setup**: `ArduinoOTA.begin()` in setup()
- **OTA Handler**: `ArduinoOTA.handle()` in loop()
- **Status Logging**: "OTA Ready. Use 'MecanumBot.local' in Arduino IDE."

### Web Interface Compatibility
- Web server continues running during OTA updates
- OTA updates don't interfere with motor control
- Both services operate simultaneously

## Environment Usage

### `upesy_wroom` (Default)
```bash
pio device upload              # OTA wireless update
pio device monitor             # Serial monitoring
pio device upload --environment upesy_wroom
```

### `upesy_wroom_upload` (USB)
```bash
pio device upload --environment upesy_wroom_upload  # USB serial upload
pio device monitor --environment upesy_wroom_upload # USB serial monitor
```

## Troubleshooting

### USB Upload Issues
1. **Check USB port**: Use `pio device list` to find correct port
2. **Update port in config**: Change `upload_port` in `[env:upesy_wroom_upload]`
3. **Check drivers**: Install CH340/CP2102 drivers if needed
4. **Reduce upload speed**: If upload fails, try 115200 baud (already configured)
5. **Power supply**: Ensure ESP32 has stable power supply
6. **USB cable**: Use quality USB cable, avoid charge-only cables
7. **Reset ESP32**: Press reset button while uploading
8. **Check connections**: Verify all ESP32 pins are properly connected

### OTA Upload Issues
1. **Check IP address**: Ensure ESP32 is connected and IP is correct
2. **Network connectivity**: Verify ESP32 and computer are on same network
3. **Firewall**: Ensure OTA port 3232 is not blocked

### Recovery Steps
- If OTA fails, use USB upload environment to recover
- Monitor serial output for detailed error messages
- Verify WiFi credentials in `include/wifi_config.h`

## Security Considerations
- OTA password is disabled for easier development
- Network should be trusted (WPA2/WPA3)
- Consider enabling OTA password for production:
  ```cpp
  ArduinoOTA.setPassword("your_password");
  ```

## Quick Commands Reference
```bash
# List available devices
pio device list

# Upload via USB (initial/recovery)
pio device upload --environment upesy_wroom_upload

# Upload via OTA (regular updates)
pio device upload

# Monitor serial output
pio device monitor

# Upload with custom IP
pio device upload --upload-port 192.168.x.x
```

Your ESP32 is now configured for both USB and OTA firmware updates!