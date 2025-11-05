# OTA (Over-The-Air) Firmware Update Guide

## Overview

The Weather Station now supports OTA (Over-The-Air) firmware updates via Zigbee network. This allows you to update the device firmware remotely without physical access to the device.

## Features

- **Automatic firmware validation**: New firmware runs in validation mode on first boot
- **Rollback protection**: If new firmware crashes, device automatically rolls back to previous version
- **Progress tracking**: OTA progress logged during update
- **Zero-copy header skipping**: Zigbee OTA header is automatically detected and skipped

## Firmware Version

The current firmware version is defined in the project's `CMakeLists.txt`:

```cmake
project(WeatherStation VERSION 1.0.0)
```

The version format is `MAJOR.MINOR.PATCH` (semantic versioning).

## OTA Configuration

### Device Configuration
- **Manufacturer Code**: `0xFABC` (DIY manufacturer code)
- **Image Type**: `0x1000`
- **OTA Endpoint**: Endpoint 1 (first light endpoint)

### Partition Table

Ensure your `partitions.csv` includes OTA partitions:

```csv
# Name,   Type, SubType, Offset,  Size, Flags
nvs,      data, nvs,     0x9000,  0x6000,
phy_init, data, phy,     0xf000,  0x1000,
factory,  app,  factory, 0x10000, 0x100000,
ota_0,    app,  ota_0,   ,        0x100000,
ota_1,    app,  ota_1,   ,        0x100000,
```

## Creating OTA Images

### Step 1: Build the Firmware

```bash
idf.py build
```

This creates the binary at: `build/WeatherStation.bin`

### Step 2: Convert to Zigbee OTA Format

Use the `esp_ota_image_create` tool from ESP-Zigbee-SDK:

```bash
python esp_ota_image_create.py \
    --manufacturer-code 0xFABC \
    --image-type 0x1000 \
    --file-version 0x01000100 \
    --min-hw-version 0 \
    --max-hw-version 0 \
    build/WeatherStation.bin \
    WeatherStation_v1.0.1.ota
```

**Version Format**: `0xMMmmpppp`
- `MM` = Major version (1 byte)
- `mm` = Minor version (1 byte)  
- `pppp` = Patch version (2 bytes)

Example: Version 1.0.1 = `0x01000100`

### Step 3: Upload to Zigbee2MQTT

1. Place the `.ota` file in your Zigbee2MQTT `ota` directory:
   ```bash
   cp WeatherStation_v1.0.1.ota /path/to/zigbee2mqtt/data/ota/
   ```

2. Configure Zigbee2MQTT to use custom OTA directory in `configuration.yaml`:
   ```yaml
   ota:
     zigbee_ota_override_index_location: data/ota/index.json
   ```

3. Create `data/ota/index.json`:
   ```json
   [
     {
       "fileVersion": 16777472,
       "fileSize": 1048576,
       "manufacturerCode": 64188,
       "imageType": 4096,
       "url": "WeatherStation_v1.0.1.ota"
     }
   ]
   ```

   **Note**: 
   - `fileVersion`: Decimal of 0x01000100 (version 1.0.1) = 16777472
   - `manufacturerCode`: Decimal of 0xFABC = 64188
   - `imageType`: Decimal of 0x1000 = 4096
   - `fileSize`: Size of the .ota file in bytes

## Performing OTA Update

### Via Zigbee2MQTT Web UI

1. Go to the device page in Zigbee2MQTT
2. Click on "OTA" tab
3. Click "Check for updates"
4. If update is available, click "Update"

### Via MQTT

```bash
# Check for update
mosquitto_pub -t 'zigbee2mqtt/WeatherStation/ota_update/check' -m ''

# Start update
mosquitto_pub -t 'zigbee2mqtt/WeatherStation/ota_update/update' -m ''
```

## OTA Update Process

1. **Query**: Device queries coordinator for available OTA images
2. **Download**: Device downloads firmware in chunks
3. **Verify**: Device verifies the downloaded image
4. **Apply**: Device sets new partition as boot partition
5. **Reboot**: Device reboots into new firmware
6. **Validate**: New firmware runs in validation mode
7. **Confirm**: If stable, firmware marks itself as valid

## Monitoring OTA Progress

View device logs during OTA update:

```
I (12345) ESP_ZB_OTA: === OTA UPGRADE STARTED ===
I (12346) ESP_ZB_OTA: OTA write session started
I (12400) ESP_ZB_OTA: First chunk received: 64 bytes
I (12401) ESP_ZB_OTA: Found ESP32 magic byte (0xE9) at offset 56
I (62500) ESP_ZB_OTA: OTA progress: 50000 bytes written
I (112600) ESP_ZB_OTA: OTA progress: 100000 bytes written
I (200000) ESP_ZB_OTA: === OTA UPGRADE APPLY ===
I (200100) ESP_ZB_OTA: Verifying OTA image...
I (200200) ESP_ZB_OTA: New firmware version: 1.0.1
I (200300) ESP_ZB_OTA: ✓ OTA upgrade successful!
I (200400) ESP_ZB_OTA: Rebooting in 3 seconds...
```

After reboot:

```
I (5000) ESP_ZB_OTA: *** OTA UPDATE VALIDATION MODE ***
I (5001) ESP_ZB_OTA: New firmware is running but not yet validated
I (5002) ESP_ZB_OTA: ✓ OTA update validated successfully!
I (5003) ESP_ZB_OTA: New firmware is now permanent
```

## Troubleshooting

### Update Fails to Start

- **Check manufacturer code and image type match** in both device and OTA file
- **Verify partition table** has ota_0 and ota_1 partitions
- **Ensure OTA file is accessible** in Zigbee2MQTT ota directory

### Device Reboots to Old Firmware

- **Validation failed**: New firmware didn't call `esp_ota_mark_app_valid_cancel_rollback()`
- **Check logs** for crash or error before validation
- This is **normal behavior** if new firmware is unstable

### OTA Never Completes

- **Check Zigbee network stability**
- **Reduce distance** between device and coordinator
- **Avoid interference** from WiFi or other devices
- **Battery devices**: Ensure device stays awake during OTA

## Best Practices

1. **Test OTA images locally first** before deploying
2. **Increment version numbers** correctly for each release
3. **Keep previous working firmware** available for rollback
4. **Document changes** in each firmware version
5. **Monitor first boot** after OTA to ensure validation succeeds
6. **For battery devices**: Plan OTA updates when device is awake or trigger wake-up first

## Version History

- **v1.0.0**: Initial release with OTA support
- **v1.0.1**: (Example) Bug fixes and improvements

## References

- [ESP-IDF OTA Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/ota.html)
- [ESP-Zigbee OTA Documentation](https://docs.espressif.com/projects/esp-zigbee-sdk/en/latest/)
- [Zigbee2MQTT OTA Updates](https://www.zigbee2mqtt.io/guide/usage/ota_updates.html)
