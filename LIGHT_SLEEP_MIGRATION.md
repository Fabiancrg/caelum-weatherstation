# Light Sleep Migration Guide

## Overview
This document describes the migration from deep sleep to light sleep mode for the ESP32-H2 Zigbee Weather Station.

## Key Changes

### 1. Sleep Mode
- **Before:** Deep sleep mode - full device reset, RAM lost, Zigbee stack restarted
- **After:** Light sleep mode - RAM retained, Zigbee stack stays active, instant wake-up

### 2. Power Consumption
- **Deep Sleep:** ~5 µA → ~1.7 years battery life (2500 mAh)
- **Light Sleep:** ~150-300 µA → ~1-2 years battery life (2500 mAh)

### 3. Wake-Up Behavior
- **Deep Sleep:** 
  - Device restarts from scratch
  - 2-5 seconds to rejoin Zigbee network
  - "Device announce" messages in Zigbee2MQTT logs
  
- **Light Sleep:**
  - Device continues from where it left off
  - Instant message delivery (~100ms)
  - No "device announce" messages
  - Zigbee network connection maintained

### 4. Code Changes

#### Files Modified:
1. `main/sleep_manager.h` - Updated function declarations and comments
2. `main/sleep_manager.c` - Replaced `enter_deep_sleep()` with `enter_light_sleep()`
3. `main/esp_zb_weather.c` - Updated sleep preparation and wake-up logic

#### Key Function Changes:
- `enter_deep_sleep()` → `enter_light_sleep()`
- `prepare_for_deep_sleep()` → `prepare_for_light_sleep()`
- `deep_sleep_scheduled` → `sleep_scheduled`

#### Important Behavioral Changes:
1. **Zigbee Stack:**
   - No longer calls `esp_zb_sleep_enable(true)` before sleep
   - Stack automatically enters low-power mode during light sleep
   - No stack restart or network rejoin needed

2. **Wake-Up Flow:**
   - After `enter_light_sleep()` returns, execution continues
   - Schedules sensor reporting immediately after wake
   - Schedules next sleep cycle
   - LED turns back on if network is connected

3. **GPIO Wake-Up:**
   - Light sleep supports any GPIO (not just RTC-capable pins)
   - Uses `gpio_wakeup_enable()` instead of RTC GPIO functions

## Benefits of Light Sleep

### ✅ Advantages:
1. **Instant Responsiveness** - Messages sent immediately on wake (100ms vs 2-5s)
2. **No Network Rejoin** - Stays connected to Zigbee network
3. **Faster Rain Detection** - Immediate reporting on rain pulses
4. **Cleaner Logs** - No "device announce" spam in Zigbee2MQTT
5. **More Reliable** - No risk of network rejoin failures

### ⚠️ Trade-offs:
1. **Higher Power** - ~30-60x higher sleep current (150-300µA vs 5µA)
2. **Shorter Battery Life** - ~1-2 years vs ~1.7 years (still excellent for most use cases)
3. **Requires Periodic Wake** - Cannot sleep indefinitely

## Testing Recommendations

1. **Monitor Power Consumption:**
   - Measure actual sleep current with multimeter
   - Should see 150-300µA average during sleep
   - Compare to previous 5µA deep sleep baseline

2. **Test Wake-Up Speed:**
   - Trigger rain pulse manually
   - Check time until message appears in Zigbee2MQTT/Home Assistant
   - Should be < 500ms (vs 2-5s before)

3. **Battery Life Testing:**
   - Monitor battery voltage over several days
   - Estimate discharge rate
   - Verify 1-2 year projection

4. **Network Stability:**
   - Check for any "device offline" messages
   - Verify no "device announce" messages after wake
   - Confirm instant reporting after each wake cycle

## Reverting to Deep Sleep

If you need to revert to deep sleep:
1. Checkout the previous commit before this migration
2. Or manually replace:
   - `enter_light_sleep()` → `enter_deep_sleep()`
   - `prepare_for_light_sleep()` → `prepare_for_deep_sleep()`
   - `sleep_scheduled` → `deep_sleep_scheduled`
   - Add back `esp_zb_sleep_enable(true)` before sleep

## Next Steps

1. Build and flash the updated firmware:
   ```powershell
   idf.py build
   idf.py -p COM<X> flash monitor
   ```

2. Observe the logs for:
   - Successful light sleep entry
   - Wake-up messages with wake reason (TIMER or GPIO)
   - Immediate sensor reporting after wake
   - Next sleep cycle scheduling

3. Test in Zigbee2MQTT/Home Assistant:
   - Verify instant message delivery
   - Check battery reporting
   - Monitor for any connection issues

## Notes

- Light sleep is ideal for applications requiring fast response time
- Battery life is still excellent for most deployments (1-2 years)
- Consider deep sleep only if multi-year battery life is critical
- The Zigbee stack handles power management automatically in light sleep
