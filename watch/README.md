# Autopilot WearOS Watch App

BLE remote control for the ESP32 autopilot. Connects to the "Pilot" BLE GATT service for glanceable status and basic steering adjustments from a Pixel Watch.

## Prerequisites

- Android Studio (for SDK, emulator, and Gradle toolchain)
- WearOS emulator image or physical Wear OS device

## SDK Setup

In Android Studio:

1. **Tools → SDK Manager → SDK Platforms** — check **Wear OS 4** (API 34) or **Wear OS 3** (API 30+)
2. **SDK Tools** tab — ensure **Android Emulator** is installed
3. Apply/download

## Create a Wear OS AVD

1. **Tools → Device Manager → Create Device**
2. Choose **Wear OS** category → pick **Wear OS Small Round** or **Wear OS Large Round**
3. Select the API 34 (or 30+) Wear OS system image
4. Finish

## Build & Run

### Option A: Android Studio

1. **File → Open** → select this `watch/` directory
2. Select the Wear OS emulator as run target
3. Click **Run**

### Option B: Command line

```bash
cd watch

# Generate Gradle wrapper if not present
gradle wrapper --gradle-version 8.5

# Build
./gradlew assembleDebug

# Start the emulator
emulator -list-avds
emulator -avd <wear-avd-name> &

# Install once booted
adb install app/build/outputs/apk/debug/app-debug.apk
```

## Fake BLE Mode (Emulator)

Debug builds use a fake BLE client by default (`FAKE_BLE = true` in `app/build.gradle.kts`). This simulates a connected autopilot so you can test the full UI without hardware:

- Scan screen shows a "Pilot (fake)" device after ~1 second
- Tapping it "connects" and navigates to the main screen
- State updates at 1 Hz with a slowly drifting compass heading
- All commands (adjust, mode change, standby, pilot type) work locally
- A transient fault triggers every 30 seconds to test the fault alert

To switch to real BLE in debug builds, change `FAKE_BLE` to `false` in the debug buildType. Release builds always use real BLE.

## BLE Testing (Physical Device)

For end-to-end BLE testing, use a physical Pixel Watch (or other WearOS device). Set `FAKE_BLE = false` for debug builds, or use a release build. Connect via `adb` over WiFi or USB debug cable, then:

1. Flash the ESP32 firmware — verify "Pilot" appears in nRF Connect on a phone
2. Install the watch app: `adb -s <watch-serial> install app/build/outputs/apk/debug/app-debug.apk`
3. Launch the app, grant BLE permissions, tap "Pilot" in the scan list
4. Verify state updates at 1 Hz and commands (adjust, mode change, standby) propagate to the ESP32
