# GP13 claw serial calibration

This opt-in µT-Kernel SMP image tests the car's GP13 claw. It does not link the
mission, wheel motor driver, camera, or IMU application. It does not change
the saved mission angles, pickup threshold, or reverse settings.

## Build (Windows PowerShell)

From the repository root:

```powershell
$env:Path = 'C:\Users\Yc\.pico-sdk\toolchain\15_2_Rel1\bin;C:\msys64\ucrt64\bin;C:\msys64\usr\bin;' + $env:Path
$clawCmake = 'C:\Users\Yc\.pico-sdk\cmake\v4.3.4\bin\cmake.exe'
& $clawCmake -S integration -B build/claw_serial_test -G Ninja '-DCMAKE_MAKE_PROGRAM=C:/Users/Yc/.pico-sdk/ninja/v1.13.2/ninja.exe'
& $clawCmake --build build/claw_serial_test --target claw_serial_test
```

Flash `third_party/mtk3smp-rp2040/build_make/mtk3pico_smp1_usb_cdc_claw_test.uf2`
to the Pico W using BOOTSEL and the RPI-RP2 drive. This temporarily replaces the
Pico's normal firmware. Reflash your mission UF2 to return to normal operation.
The ESP32 firmware does not need changing.

## Test

Keep the car stationary with its wheels clear and the claw empty. Keep fingers
out of the jaws; switching the PWM off removes holding torque.

Open the Pico COM port in a USB serial terminal with DTR enabled. Select a
newline ending (LF, CR, or CRLF). Wait for the `GP13 CLAW TEST` banner.

```text
status
claw 35
claw 40
claw 45
off
```

Send one command at a time. PWM starts off. Each valid angle applies PWM for
up to two seconds; `off` stops it immediately. Closing the terminal or
disconnecting USB also stops PWM when the console detects the session ending.
`status` reports the last active command, not measured jaw position.

The test accepts whole degrees from 0 to 180, independent of the mission
setpoints. This is the servo command range, not a verified travel range for
the assembled jaws. Start at a known angle and move in small steps. Send `off`
immediately if the servo binds or buzzes. Test the extremes with the linkage
disconnected. Invalid input does not move the claw. To adopt a proven angle in
the mission, separately update the relevant constant in `components/drive/claw.h`.
