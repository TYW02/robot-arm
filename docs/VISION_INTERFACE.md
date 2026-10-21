# Stage 1: Vision -> System Coordinator (no motion)

Target branch: your existing `Yc`, based on `37e1b78`.
Prepared separately in the existing `Yc-vision-interface` worktree; no new
branch or commit was created for this folder reorganisation.
Vision baseline: `origin/feature/vision_uart` at `9951e56`.
The original `Yc` checkout and its uncommitted Motion changes are untouched.

The reviewed branch's `third_party/mtk3smp-rp2040` tree, integration build
adapter and checksum-enabled `xiao_vision/main/main.c` were imported, without
merging the remote branch. The kernel implementation is unchanged. Tiny
`app_program/vision_*_app.c` and `coordinator_app.c` adapters compile project-owned
sources as separate translation units, following `robot_arm_app.c`.
The Windows CMake command selects `E2U=tools/elf2uf2.exe` to fix UF2 packaging.

## Wire protocol and coordinate meaning

ESP32 UART0, TX GPIO43 / RX GPIO44, 115200 baud, 8 data bits, no parity,
1 stop bit, no flow control. Pico UART0 is BSP device `sera`, TX GP0 / RX GP1.
Connect ESP32 TX to Pico RX, ESP32 RX to Pico TX, and common ground.
The Pico T-Monitor console is USB-only so logging cannot share this UART.

Packets end in LF (CRLF is also accepted). `HH` is exactly two hexadecimal
digits: XOR of all payload bytes before `*`, excluding `*`, checksum and LF.

| Payload before `*HH\n` | Meaning |
|---|---|
| `C,R,x,y`, `C,G,x,y`, `C,B,x,y` | Colour centroid |
| `C,N` | No colour object; no coordinate fields. Complete packet: `C,N*21\n` |
| `A,id,x,y,width,distance` | ArUco marker detection |

Pico-to-camera commands are the existing four-byte `M,S\n` (ArUco scan),
`M,C\n` (colour), or `M,I\n` (idle), without checksum or acknowledgement.
There is no timestamp, confidence, scan/session ID, depth for colour, or arm
joint information on the wire. Do not assume a command queue success means
the camera has already switched; `VisionResult.kind` identifies each packet.

X/Y are **image pixels, not millimetres or robot/workspace XYZ**:

- Image resolution: 320 x 240 (`FRAMESIZE_QVGA` and `ArucoLite<320,240,...>`).
- Origin: top-left of the image buffer. X increases right, Y increases down.
  This does not establish physical camera/robot axes, mirroring or mounting.
- Colour: centroid of matching sampled pixels (stride 4, still in original
  image coordinates), then averaged for the winning colour in a five-frame
  history requiring at least three matching votes. Integer X/Y on the wire.
- ArUco: average of the four marker corner positions, cast to integer pixels
  for transmission. Width is the mean top/bottom edge length in pixels.
- ArUco distance is the sender's **estimate in millimetres**:
  `80.0 mm * 277.0 px / width_px`. These are the existing marker-size/focal
  constants, not evidence of validated camera-to-arm calibration. Width and
  distance are transmitted with one decimal place. Neither supplies robot XYZ.
- ArUco scan mode sends a packet only when a marker is detected. Silence is
  therefore a timeout, NOT an explicit `NO_OBJECT` observation. Colour mode
  sends changes/movement and approximately one-second refreshes, including `N`.

Source references: `xiao_vision/main/main.c` (`detection_t`,
`detect_dominant_color`, `history_consensus`, `send_with_checksum`, `app_main`)
and `xiao_vision/components/aruco_detect/aruco_detect.cpp` / `include/aruco_detect.h`.

Still required in a future stage:

```text
image coordinates -> validated robot workspace coordinates -> ArmPose
```

There is no validated transformation in the existing vision code. This stage
does not add one, inverse kinematics, or any Vision-to-MovJ call.

## Result contract and message flow

`components/vision_pico/vision.h` is the hardware-independent result definition:

| Status / kind | Valid fields |
|---|---|
| `VISION_OK` / `VISION_COLOUR` | `colour`, `x_px`, `y_px` |
| `VISION_OK` / `VISION_ARUCO` | `marker_id`, `x_px`, `y_px`, `marker_width_px`, `distance_mm` |
| `VISION_NO_OBJECT` / `VISION_COLOUR` | No coordinates; `colour=NONE`, X/Y = -1 |
| `VISION_INVALID_PACKET` | No detection fields valid; X/Y and marker ID = -1 |
| `VISION_TIMEOUT` | No result arrived within the requested wait; not absence of an object |

Unused distance/width fields are zero. Never use inactive fields as measurements.
Packets must pass checksum, exact field count, numeric syntax/overflow and
320x240 pixel bounds checks. Missing/trailing fields, unsupported colours,
negative pixels, nonpositive/nonfinite ArUco measurements, and overlong lines
are rejected. Unrelated boot/log lines are ignored. Invalid fields are cleared;
the checksum is never part of the payload cursor.

```text
Coordinator --VisionSetMode--> command MBF (4) --> Vision task --> camera UART
Camera UART --> Vision task/parser --> result MBF (8) --> VisionReceive --> Coordinator
```

The old parser-owned shared shelf map is replaced by individual typed ArUco
results. Any future shelf-map aggregation belongs with its consuming application,
not the UART parser. The old move/pick state placeholders are absent from this
receive-only example so they cannot accidentally actuate the arm.

`components/vision_pico/vision_task.h` exposes:

```c
ER VisionStart(void);                         /* once, from usermain */
ER VisionSetMode(VisionMode mode);            /* nonblocking command enqueue */
ER VisionReceive(VisionResult *result, TMO timeout);
```

`integration/main.c` is startup only: it calls `VisionStart` and then
`CoordinatorStart` from `components/state_machine/coordinator.h`. The latter
creates the receiving task defined in `components/state_machine/coordinator.c`.
`VisionStart` creates both message buffers, exclusively opens `sera`, then
starts the UART-owning task. Call it before starting the Coordinator. The
single Coordinator consumer calls `VisionReceive(&result, 2000)` in a loop:
`E_OK` means inspect `result.status`; `E_TMOUT` also sets `VISION_TIMEOUT`;
other errors are IPC/setup faults, not detections. Receive timeout values use
the port's normal millisecond `TMO` convention. `TMO_POL` and `TMO_FEVR` work too.

The result FIFO is bounded. Publication uses `TMO_POL` so UART draining cannot
block behind a slow consumer; if full, a NEW result is dropped with an error
log. The Coordinator must drain continuously. This is a first-stage observation
interface, not a lossless scan database or a freshness guarantee for actuation.

Core assignment preserves the reviewed vision arrangement: Vision priority 5
on processor 1 (physical core 0, owner of UART interrupts); Coordinator priority
10 on processor 2 (physical core 1). All queues and task waits use micro T-Kernel.
The ESP32 sender still uses its existing ESP-IDF/FreeRTOS runtime; this change
ports neither that processor nor the Motion architecture.

## Build and hardware acceptance test

Build **this worktree's `integration` target**, not `components/servo`.
No Motion sources, motion task, PCA9685 commands or `MovJ` symbols are linked.
`components/servo` remains byte-for-byte unchanged from committed `Yc`.

On this Windows installation, from the worktree root in PowerShell:

```powershell
$env:PATH = 'C:\Users\Yc\.pico-sdk\toolchain\15_2_Rel1\bin;C:\msys64\ucrt64\bin;C:\msys64\usr\bin;' + $env:PATH
& 'C:\Users\Yc\.pico-sdk\cmake\v4.3.4\bin\cmake.exe' -S integration -B build_vision_interface -G Ninja '-DCMAKE_MAKE_PROGRAM=C:/Users/Yc/.pico-sdk/ninja/v1.13.2/ninja.exe'
& 'C:\Users\Yc\.pico-sdk\cmake\v4.3.4\bin\cmake.exe' --build build_vision_interface
```

Output: `third_party/mtk3smp-rp2040/build_make/mtk3pico_smp1_usb_cdc.uf2`.
MSYS2 make, host g++, Arm compiler and the local Pico SDK/TinyUSB are required;
`PICO_SDK_PATH` can be overridden at CMake configure time.

Hardware test (not performed automatically):

1. Keep servo V+ disconnected for this first interface test.
2. Flash the ESP32 with this worktree's checksum-enabled `xiao_vision` firmware
   (unless it already runs that same protocol); boot it before starting Pico.
3. Put the Pico into BOOTSEL and copy the integration UF2 to its USB drive.
4. Open Pico USB serial. Expected banner: `Vision interface test: MOTION DISABLED`.
   The Coordinator requests colour mode twice, 50 ms apart, as in the previous
   retry scheme. If the camera booted later and missed both commands, reset Pico.
5. Present a red object under suitable lighting and allow the five-frame history
   to settle. Expect e.g.:

   ```text
   [coordinator] VISION_OK colour=RED x_px=160 y_px=120
   ```

6. Remove the object; after the colour history settles, expect:

   ```text
   [coordinator] VISION_NO_OBJECT
   ```

7. Disconnect camera UART TX. After 2000 ms without a queued result, expect
   `VISION_TIMEOUT (not NO_OBJECT)`. No arm movement is requested in any case.

To observe ArUco instead, change only the Coordinator's two initial
`VISION_MODE_COLOUR` arguments in `components/state_machine/coordinator.c` to
`VISION_MODE_SCAN`; marker width/distance
remain available in the result even though this port's `tm_printf` cannot
format floats. Return to colour mode for the explicit no-object test.

## Repeatable host tests

From the worktree root with host gcc available:

```text
gcc -std=c11 -Wall -Wextra -Werror components/vision_pico/tests/vision_parser_test.c components/vision_pico/vision_parser.c -o components/vision_pico/tests/vision_parser_test.exe
gcc -std=c11 -Wall -Wextra -Werror -I components/vision_pico/tests/stubs components/vision_pico/tests/vision_flow_test.c components/vision_pico/vision_parser.c -o components/vision_pico/tests/vision_flow_test.exe
```

Run both executables. The parser test covers R/G/B, `C,N*21`, ArUco fields,
bad/missing checksums, malformed/trailing/overflowing fields, pixel bounds and
10,000 bounded arbitrary inputs. The flow test runs the actual Vision task and
actual Coordinator code with simulated device reads/message buffers, checking
split packets, NO_OBJECT, invalid packets, timeout, line-overflow recovery and
nonblocking full-queue behaviour. It is not a hardware or SMP scheduling test.
