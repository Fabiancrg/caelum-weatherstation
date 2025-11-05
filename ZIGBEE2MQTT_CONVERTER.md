# Caelum Weather Station - Zigbee2MQTT External Converter

This external converter enables full integration of the Caelum Weather Station with Zigbee2MQTT and Home Assistant.

## Features

The converter provides the following entities:

### Environmental Sensors (Endpoint 1)
- **Temperature** - BME280 sensor readings (-40°C to +85°C)
- **Humidity** - Relative humidity (0-100% RH)  
- **Pressure** - Atmospheric pressure (300-1100 hPa)

### Rain Gauge (Endpoint 2)
- **Total Rainfall** - Cumulative rainfall in millimeters
- **Real-time updates** - Reports immediately on rain detection
- **Persistent storage** - Values persist through device reboots

### Sleep Configuration (Endpoint 3)  
- **Sleep Duration** - Configurable wake-up interval (60-7200 seconds)
- **Remote control** - Adjust sleep timing from Home Assistant
- **Battery optimization** - Longer intervals = better battery life

## Installation

1. **Copy the converter file** to your Zigbee2MQTT data directory:
   ```bash
   cp caelum-weather-station.js /opt/zigbee2mqtt/data/
   ```
   
   > **Note**: For Zigbee2MQTT v1.33.0+, converters placed in the `data` directory are automatically loaded.
   > For older versions, you may need to create an `external_converters` subdirectory and update `configuration.yaml`:
   > ```yaml
   > external_converters:
   >   - caelum-weather-station.js
   > ```

2. **Restart Zigbee2MQTT** to load the converter

3. **Pair the device** using the built-in button (long press for factory reset)

## Home Assistant Integration

Once paired, the device will automatically appear in Home Assistant with the following entities:

### Sensors
- `sensor.caelum_weather_station_temperature`
- `sensor.caelum_weather_station_humidity` 
- `sensor.caelum_weather_station_pressure`
- `sensor.caelum_weather_station_rainfall`
- `sensor.caelum_weather_station_battery`

### Configuration
- `number.caelum_weather_station_sleep_duration` (60-7200 seconds)

## Usage Examples

### Basic Weather Monitoring
```yaml
# automation.yaml
- alias: "Rain Alert"
  trigger:
    platform: numeric_state
    entity_id: sensor.caelum_weather_station_rainfall
    above: 5.0  # 5mm of rain
  action:
    service: notify.mobile_app
    data:
      message: "Rain detected: {{ states('sensor.caelum_weather_station_rainfall') }}mm"
```

### Dynamic Sleep Configuration
```yaml
# Shorter intervals during rain season
- alias: "Rain Season - Frequent Updates"
  trigger:
    platform: time
    at: "06:00:00"
  condition:
    condition: template
    value_template: "{{ now().month in [6,7,8,9] }}"  # Rain season
  action:
    service: number.set_value
    target:
      entity_id: number.caelum_weather_station_sleep_duration
    data:
      value: 300  # 5 minutes

# Longer intervals during dry season  
- alias: "Dry Season - Battery Conservation"
  trigger:
    platform: time
    at: "06:00:00"
  condition:
    condition: template
    value_template: "{{ now().month in [12,1,2,3] }}"  # Dry season
  action:
    service: number.set_value
    target:
      entity_id: number.caelum_weather_station_sleep_duration
    data:
      value: 1800  # 30 minutes
```

### Battery Monitoring
```yaml
- alias: "Low Battery Alert"
  trigger:
    platform: numeric_state
    entity_id: sensor.caelum_weather_station_battery
    below: 20
  action:
    service: notify.persistent_notification
    data:
      title: "Weather Station Battery Low"
      message: "Battery: {{ states('sensor.caelum_weather_station_battery') }}%"
```

## Reporting Intervals

- **Temperature**: 30s-15min (or 0.5°C change)
- **Humidity**: 30s-15min (or 2% change) 
- **Pressure**: 30s-15min (or 1 hPa change)
- **Rainfall**: Immediate (or hourly maximum)
- **Sleep Duration**: When changed (or hourly maximum)

## Troubleshooting

### Device Not Pairing
1. Ensure device is in factory reset mode (long press button)
2. Check Zigbee2MQTT logs for pairing attempts
3. Verify converter is loaded in external_converters

### Missing Entities
1. Check that all 3 endpoints are discovered (1, 2, 3)
2. Verify converter configuration in Zigbee2MQTT
3. Restart Home Assistant to refresh entity registry

### Configuration Not Working
1. Ensure sleep_duration is within valid range (60-7200)
2. Check device logs for configuration acceptance
3. Verify endpoint 3 is properly bound

### Battery Issues
1. Monitor voltage levels in device logs
2. Adjust sleep duration for better battery life
3. Check for stuck in wake state (short sleep intervals)

## Technical Details

- **Zigbee Device Type**: End Device (sleepy)
- **Clusters Used**: Basic, Temperature, Humidity, Pressure, Analog Input
- **Endpoints**: 3 (Environmental, Rain, Config)
- **OTA Support**: Yes (automatic updates)
- **Power**: Battery optimized with configurable sleep

For more information, see the main project documentation.