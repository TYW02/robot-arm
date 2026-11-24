# Mission Logic, Design Decisions and Evidence

[Return to README](../README.md)

This document explains the mobile robot implementation at source commit **`731bea5857c92889c5e1f7e7ed64f12f70f0c08a`**. It was written after assessment to make the code and available evidence easier to follow. It does not change firmware, rewrite the existing test register, or claim that the assessed demonstration succeeded consistently.

**How to read the claims:** implementation descriptions below come from the referenced source. Test results are attributed to the existing register. The four-run assessment outcome is a retrospective team account. Design explanations describe the purpose and trade-offs of the present implementation; they do not establish that every choice was planned or experimentally validated before implementation.

## Mission walkthrough

The mission state machine is in [`integration/main.c`](../integration/main.c), principally `mission_task()` and the helpers beginning at `mission_map_directions()`. The reviewed version can be read through this [fixed-revision source link](https://github.com/TYW02/robot-arm/blob/731bea5857c92889c5e1f7e7ed64f12f70f0c08a/integration/main.c#L2441).

| Stage | Implemented action | Reason for the check or separation |
|---|---|---|
| Initialize | `usermain()` initializes motors and claw, starts the CLI and IMU, and waits for acceptable IMU data | Heading-dependent motion should not start using uncalibrated or invalid gyro data |
| Wait for start | `cli_bridge_wait_for_start()` sets IDLE and waits on a kernel event flag | Operator authorization is separate from boot and hardware initialization |
| Map directions | `mission_map_directions()` performs the direction-marker sweep | Save bearings to known delivery-marker IDs before pickup cycles begin |
| Search/approach object | `mission_pick_up_object()` calls `approach_marker()` | Acquire and reobserve the target while moving toward it |
| Grip | Command claw close, wait 400 ms, then check the interlock | Allow commanded servo movement to settle; this is not grip-force feedback |
| Select delivery marker | Try the saved bearing map, then live search if needed | Use a previously observed bearing as an initial direction, then visually reacquire |
| Align/approach delivery | `mission_approach_direction()` aligns and approaches the selected marker | Separate coarse turning from forward approach and repeated observation |
| Release | Open, wait 400 ms, disable claw PWM, record the object as released | Execute a release sequence and avoid selecting that object ID again |
| Retreat/repeat | Reverse, confirm the stop, delay one second and start another cycle | Clear the release area before searching for another object |

The drive task handles wheel commands independently of the mission task. Motion completion is software-controlled using the gyro for turns and timing for drive pulses. The robot has no implemented wheel-odometry position estimate or claw feedback confirming that a block was acquired or deposited.

### Startup gate and IDLE

`usermain()` waits five seconds before its application initialization. `claw_init()` initially sets the PWM compare value to zero, so automatic pickup does not begin by moving the claw.

The IMU accepts `WHO_AM_I` values `0x70` and `0x71`. It waits three seconds before collecting 200 calibration samples. Calibration rejects excessive variance on any gyro axis. The application then requires healthy, fresh readings, `G_Hz10` between 900 and 1100, and Z rate between −0.5 and +0.5 degrees/s continuously for 2,000 ms. The readiness wait has a 60-second limit.

`G_Hz10` is sampling frequency multiplied by ten: **1000 means approximately 100 Hz**. The 25-second stillness instruction in the test register is an operating instruction, not a fixed 25-second firmware gate.

After startup succeeds, the mission waits for `start`. The earlier startup message saying it is "starting direction mapping" does not bypass the subsequent operator-start wait. Startup failures park the initial application task and can require a power cycle rather than a mission reset.

Sources: [`usermain()`](https://github.com/TYW02/robot-arm/blob/731bea5857c92889c5e1f7e7ed64f12f70f0c08a/integration/main.c#L2695), [`wait_for_imu_startup()`](../integration/main.c), [`imu_heading.c`](../components/imu/imu_heading.c), [`cli_bridge_wait_for_start()`](../integration/cli_bridge.c).

### Mapping delivery markers

The application assigns IDs 1, 2 and 3 to the log labels North, West and East. These are station names; they do not come from a compass. Object IDs are 4 through 999.

The mapping sweep requests headings at 0, 15, 30, …, 345 degrees relative to the startup heading, then completes the revolution. Each direction collection window is 800 ms, with braking and settling around the scan. Actual gyro headings are used when recording observations. Scan turns avoid reversing to trim an overshoot, and a heading already passed can be skipped; the sequence of requested headings is not a promise of exact physical angular spacing.

For each direction marker, the map retains the observation closest to the image centre. Its bearing is calculated from the chassis heading plus the marker's horizontal image offset:

```text
offset_px       = marker_x − 160
image_angle_deg = atan(offset_px / 277) × 180 / pi
camera_bearing  = (current_gyro_yaw − startup_gyro_yaw) × TURN_GYRO_RIGHT_SIGN
marker_bearing  = wrap_to_0_360(camera_bearing + image_angle_deg)
```

`TURN_GYRO_RIGHT_SIGN` is −1.0, matching the configured gyro/motor convention. The published map is clockwise/right positive relative to startup. It is not a global XY map. Each of the three entries stores a mapped flag, bearing, observed distance and centring error.

`DIRECTION_MAP_REQUIRE_ALL` is **0** in this revision, so **at least one** direction marker is sufficient. The code does not require all three stations to be mapped before proceeding.

Source: [`update_startup_bearings()` and mapping sweep](https://github.com/TYW02/robot-arm/blob/731bea5857c92889c5e1f7e7ed64f12f70f0c08a/integration/main.c#L1858).

### Pickup and confirmation

The object search selects the nearest positive-distance object in the current scan when no specific object has yet been tracked. Once selected, the approach tries to retain that object's ID. IDs previously marked as released are excluded.

Before opening the claw, the code requires the same candidate ID on a subsequent observation. It opens to **30 commanded degrees**, waits 400 ms, disables claw PWM and takes a fresh scan. Disabling PWM removes the active holding command; it does not measure the resulting jaw position.

During approach, the horizontal error is `x − 160`. Outside the near-pickup zone it produces differential wheel corrections. Within **200 mm** estimated distance (140 mm threshold plus 60 mm margin), the code uses 40 ms forward pulses and 3-degree centring turns if the horizontal error exceeds ±16 pixels.

At or below the **140 mm** threshold, the robot stops and scans the **same object ID again**. It proceeds to close only if the new estimate is still within the threshold and the horizontal error is within ±16 pixels. The claw closes to **50 commanded degrees** and waits 400 ms. There is no force, contact or object-presence sensor confirming a successful grip.

If a tracked object is lost near pickup, stationary scans are retried up to three misses before recovery or a stop, depending on its last distance. Losing the object within closing range can latch a stop. An approach is limited to 30 iterations, but outer search/retry loops can continue afterward.

Sources: [`approach_marker()`](https://github.com/TYW02/robot-arm/blob/731bea5857c92889c5e1f7e7ed64f12f70f0c08a/integration/main.c#L2140), [`claw.h`](../components/drive/claw.h), [`claw.c`](../components/drive/claw.c).

### Delivery selection, release and reversing

The saved-map path chooses the mapped station requiring the **smallest absolute heading change** from the current heading. Its saved distance does not determine this ranking. The robot turns toward that bearing and tries to see the same station again. If mapped candidates cannot be confirmed, it falls back to the general direction-marker search. The live-scan helper chooses the nearest observed direction marker in its current scan, not the nearest station throughout the arena.

The saved bearings are based on the earlier mapping location. After translating toward a pickup block, the robot has not recomputed station positions using odometry. A saved bearing is therefore an approximate initial search direction, which is why visual reacquisition is necessary.

The delivery alignment loop allows up to 12 attempts, using 5-degree turns outside a ±74-pixel horizontal tolerance. Forward approach then uses distance and horizontal correction. The delivery threshold is **300 mm of camera-estimated marker distance**. This is not a measured distance from the claw tip to a container. The threshold check does not verify bin occupancy or successful placement.

After the delivery approach returns success, the mission opens the claw, waits 400 ms and releases PWM. It marks the object ID as released and increments `delivered`, then requests reverse wheel speeds of −25% for a 500 ms pulse. Wheel ramping occurs around drive commands, so 500 ms is not an exact total manoeuvre time or a calibrated reverse distance. After a confirmed stop, the next-cycle delay is 1,000 ms.

**The delivery counter records execution of this release sequence. It does not independently confirm a physical delivery.** It is incremented before the retreat completes.

Sources: [`find_delivery_marker()`](https://github.com/TYW02/robot-arm/blob/731bea5857c92889c5e1f7e7ed64f12f70f0c08a/integration/main.c#L2022), [`align_direction_marker()`](../integration/main.c), [`mission_release_object()`](https://github.com/TYW02/robot-arm/blob/731bea5857c92889c5e1f7e7ed64f12f70f0c08a/integration/main.c#L2549).

## Search strategy and its limitation

The robot does not know a pickup block's bearing before detecting it. `find_marker()` therefore performs a stop–scan–turn search. With no usable target in the scan results, it turns **15 degrees right**, stops and scans again. This increment provides intermediate viewing directions instead of making one large turn and sampling only its endpoint. The source does not establish that 15 degrees was selected through a validated camera field-of-view or detection-coverage study.

**The discovery loop has no maximum number of revolutions or overall search timeout.** While scans return no target and the drive/IMU checks permit movement, it keeps turning. Individual turn timeouts and the 30-iteration approach limit do not bound the discovery loop. Repeated circling during assessment is consistent with this implemented no-target behaviour. It does not, by itself, identify why a usable target was unavailable.

For a previously seen marker, local recovery is different: save the current heading, scan left at 5-degree offsets up to 30 degrees, return to origin, scan right similarly, then return again. Each recovery collection window is 1,200 ms. If a direction marker is still absent, recovery returns failure; the mission transitions back to live direction search rather than assuming it reached the delivery point. That outer search may also continue turning indefinitely without a target.

Before collecting, `collect_marker_scan()` requests a confirmed wheel brake and gyro stillness within 1 degree/s over 250 ms of sample timestamps, with a 3-second settling timeout. It sends the selected camera mode twice, 50 ms apart, waits another 150 ms, clears the old results and collects the new window. The mutex is not held across these waits. Regular tracking windows are 700 ms for objects and 800 ms for directions; discovery/recovery windows are 1,200 ms.

These measures aim to obtain observations after motion has settled and avoid using pre-turn results. They do not make a low, obscured or out-of-frame marker visible, nor do they prove that every heading produces a valid detection. A finite search budget, a distinction between "no marker" and "camera unavailable", and image visibility during failures would have needed consideration earlier. They are future design lessons, not features added by this documentation.

Sources: [`collect_marker_scan()` and `find_marker()`](https://github.com/TYW02/robot-arm/blob/731bea5857c92889c5e1f7e7ed64f12f70f0c08a/integration/main.c#L1738), [`rescan_marker()`](https://github.com/TYW02/robot-arm/blob/731bea5857c92889c5e1f7e7ed64f12f70f0c08a/integration/main.c#L2083).

## Vision measurements

The XIAO captures **320 × 240 QVGA** frames. ArUco detection uses the vendored **ArucoLite** code with the `ARUCO_DB_4X4_1000` dictionary and `ArucoLite<320, 240, 16, false>`. An upstream release/version number is not recorded in the inspected wrapper. This is not an OpenCV-based detector in the embedded application.

For a detected marker, the application averages its four corner coordinates to obtain the image centre. It averages the top and bottom edge lengths to estimate width, then calculates:

```text
distance_mm = configured_marker_size_mm × 277.0 / measured_width_px
```

This is a size-based camera distance estimate. It is not a separate ranging sensor, a full 3D marker-pose solution or a calibrated floor-plane position. An angled marker can change its apparent width and bias the estimate. The exact error during assessment is unknown; this is a limitation of the calculation, not a confirmed cause of the demonstration failure.

| Field | Meaning |
|---|---|
| `id` | ArUco identifier; 1–3 are stations, 4–999 are eligible objects |
| `x`, `y` | Image centre in pixels in the original 320 × 240 frame; horizontal column and vertical row |
| `width` | Average top/bottom edge length in pixels |
| `distance` | Estimate in millimetres using the selected physical marker size |

The image coordinates are **not robot workspace coordinates**. The 160-pixel horizontal centre and 277-pixel focal-length constant are used for steering and bearing estimates. No camera-to-robot XY transformation or inverse kinematics is implemented in this mobile mission.

The Pico sends `M,3\n` for 30 mm object markers and `M,8\n` for 80 mm station markers. These commands select a physical-size assumption and scan mode on the XIAO. All markers processed under a selected mode use that size; a different-sized marker seen at the same time can therefore receive an incorrect distance estimate. The current commands have no explicit camera-mode acknowledgement/sequence ID, so sending them twice is not proof that a mode transition was confirmed.

Detection reports have this format, at **115200 baud on both ends**:

```text
A,<id>,<x_px>,<y_px>,<width_px>,<distance_mm>*<two_hex_digits>\n
```

The checksum XORs every payload byte before `*`, including commas. It excludes `*`, the two hexadecimal characters, newline and string terminator. The Pico requires a valid suffix and matching checksum before dispatch. A checksum detects some transmission corruption; it does not establish measurement accuracy or complete semantic validation of every field.

The current application is ArUco-only. Commit `731bea5` removed the unused colour-mode implementation from both camera and Pico integration code. In scan mode, no visible markers means no `A` reports for that frame, rather than a typed no-marker acknowledgement. A lack of usable scan results alone cannot distinguish an empty view from all communication/detection failures.

Sources: [`aruco_detect.cpp`](../xiao_vision/components/aruco_detect/aruco_detect.cpp), [`XIAO mode and report code`](https://github.com/TYW02/robot-arm/blob/731bea5857c92889c5e1f7e7ed64f12f70f0c08a/xiao_vision/main/main.c#L189), [`Pico checksum and parsing`](https://github.com/TYW02/robot-arm/blob/731bea5857c92889c5e1f7e7ed64f12f70f0c08a/integration/main.c#L903).

## RTOS tasks and communication

These are the application and supporting service tasks relevant to the **Pico SMP mission**. Core 0 corresponds to `TP_PRC1`; core 1 corresponds to `TP_PRC2`. Smaller priority numbers have higher priority. Stack sizes are requested bytes, not measured high-water marks. Kernel initialization/idle tasks are outside this application table.

| Task entry / service | Physical core | Priority | Stack bytes | Responsibility |
|---|---:|---:|---:|---|
| `camera_rx_task` | 0 | 5 | 4,096 | Reassemble UART reports, check checksum and update scan records |
| `camera_tx_task` | 0 | 6 | 4,096 | Send queued camera mode commands |
| `mission_task` | 1 | 10 | 4,096 | Own the mission sequence and decisions |
| `task_drive` | 1 | 5 | 4,096 | Own runtime wheel PWM and execute movement commands |
| `task_imu_monitor` | 1 | 8 | 4,096 | Check the IMU interlock and report health |
| `imu_heading_task` | 0 | 4 | 4,096 | Sample, calibrate and publish gyro heading |
| CLI task (`task_entry`) | 0 | 9 | 4,096 | Receive USB operator commands |
| USB console service | 0 | 3 | 4,096 | Service TinyUSB and console queues |
| CYW43 radio service, Wi-Fi builds only | 0 | 4 | 6,144 | Own radio/lwIP work, logs and remote commands |

Core placement keeps the port's USB/radio services on their required core and puts mission sequencing with drive execution on core 1. Giving the drive task higher priority than mission planning lets it service control work without waiting for the lower-priority mission to finish its next decision. IMU and communication work share core 0; this is not complete timing isolation. Affinity and priority alone do not prove schedulability or a measured worst-case response time.

Sources: [`application task creation`](https://github.com/TYW02/robot-arm/blob/731bea5857c92889c5e1f7e7ed64f12f70f0c08a/integration/main.c#L193), [`IMU task`](../components/imu/imu_heading.c), [`CLI configuration`](../integration/cli_bridge.c), [`USB service`](../third_party/mtk3smp-rp2040/lib/libtm/sysdepend/pico_rp2040/usb/usb_console.c), [`radio task`](../third_party/mtk3smp-rp2040/lib/libwifi/sysdepend/pico_rp2040/cyw43_utk_kernel.c).

### IPC, locks and waiting

| Mechanism | What it protects or transfers |
|---|---|
| Camera command message buffer | Mission sends a four-byte `ModeCommand` to the UART TX task |
| Drive message buffer | Mission/manual commands send `DriveCommand` records to the drive task |
| Turn/brake result message buffer | Drive task reports completion/error for operations that require acknowledgement |
| Scan-results mutex | Camera RX and mission access detection tables without simultaneous updates |
| Start event flag | Operator `start` releases the waiting mission task |
| IMU sample event flag | A 10 ms cyclic handler wakes the heading task |
| µT-Kernel spinlocks | Brief shared-state copies, stop latch and Wi-Fi log-queue operations |

The scan mutex is created with **`TA_TFIFO | TA_INHERIT`**: FIFO waiting and priority inheritance, not priority ceiling. Scan delays occur after releasing table locks. The IMU's cyclic callback only signals the sample flag; I2C reads occur in task context. Task waits use `tk_dly_tsk()`, kernel object waits or sleeping tasks rather than Pico SDK sleep calls in the active mission.

The three message-buffer backing sizes are 80 bytes (camera commands), 192 bytes (drive commands) and 16 bytes (turn results) for the RP2040 data layout. Their maximum payloads are 4, 36 and 8 bytes respectively. Backing size includes kernel message-storage overhead and is not a count of payload-only slots.

Camera queue send/receive and normal mission drive sends use `TMO_FEVR`. Manual-drive enqueue and drive-result sends use `TMO_POL`. The drive task waits up to 10 ms for a command. Brake acknowledgement uses 1,000 ms send/receive waits; turn submission uses 1,000 ms and polls the result in 100 ms waits with an outer guard. These policies expose some failures as stop conditions, but indefinitely waiting operations are not proof of bounded end-to-end timing.

Sources: [`kernel object definitions`](https://github.com/TYW02/robot-arm/blob/731bea5857c92889c5e1f7e7ed64f12f70f0c08a/integration/main.c#L253), [`drive submission/acknowledgement`](https://github.com/TYW02/robot-arm/blob/731bea5857c92889c5e1f7e7ed64f12f70f0c08a/integration/main.c#L1469), [`IMU cyclic handler`](https://github.com/TYW02/robot-arm/blob/731bea5857c92889c5e1f7e7ed64f12f70f0c08a/components/imu/imu_heading.c#L365).

### Turn control, heading hold and stopping

`drive_turn_run()` uses relative gyro yaw and angular rate in a proportional/damping controller. Normal turn completion uses a 2-degree tolerance and requires angular rate within 5 degrees/s for 200 ms. Scan turns deliberately brake on reaching/passing their stopping threshold and do not reverse to remove overshoot. A turn can fail for wrong direction, invalid gyro data, no angular progress for 1.5 seconds, or a 10-second timeout.

Straight heading hold is requested during approach when vision steering correction is zero. The drive command carries a separate hold-heading flag and target yaw; equal left/right speeds by themselves do not enable it. Gyro drift beyond a ±0.75-degree deadband produces a trim capped at ±8 percentage points. The reverse command does not request heading hold.

Freshness is checked by `imu_heading_read()`: samples older than **50 ms** become invalid. A sample interval above 15,000 µs increments the late counter; a gap above 50,000 µs can fault sampling. The stop latch is checked throughout drive work. `mission_checked_delay()` splits waits into at most 10 ms chunks, while turn and heading-hold loops also check the interlock. These are code check intervals, not a measured USB/Wi-Fi-to-mechanical-stop guarantee.

`stop` records a latched reason and prevents further permitted drive execution. The mission parks at a checkpoint. `reset` can restart a parked/ended mission, clears the pending start event, and returns to IDLE. If the task has not parked, it replies `busy: mission still stopping, try again`. An unhealthy IMU can cause the interlock to latch again after reset. Stopping during startup may require a power cycle. Neither leaving the laptop CLI nor losing Wi-Fi is an implemented heartbeat stop.

Sources: [`drive_turn.c`](../components/drive/drive_turn.c), [`drive_turn.h`](../components/drive/drive_turn.h), [`drive/interlock logic`](https://github.com/TYW02/robot-arm/blob/731bea5857c92889c5e1f7e7ed64f12f70f0c08a/integration/main.c#L517), [`reset handler`](https://github.com/TYW02/robot-arm/blob/731bea5857c92889c5e1f7e7ed64f12f70f0c08a/integration/cli_bridge.c#L168).

## Logs and evidence

### Existing recorded excerpts

The following short excerpts/numbers are copied from evidence fields in the [existing test register](../test%20cases/test_cases.json). They are recorded summaries, not newly captured logs, independent reproductions or a complete chronological run transcript. The hardware records below are dated **17 November 2026**. An exact flashed commit and complete raw transcript are not supplied for each entry.

| Test | Recorded excerpt or measurement | What it supports and what remains unverified |
|---|---|---|
| `MIS-001` | `status: state: IDLE` | An IDLE status was observed; this alone is not a measurement of wheel output |
| `MIS-002` | `Turn angle about -358 degrees recorded; per-step log lines were not captured` | Approximately one revolution was reported; each 15-degree observation is not independently evidenced |
| `MIS-003` | `status: stop: Not enough direction markers mapped` | The no-usable-station stop reason was observed |
| `SAF-001` | `G_Hz10=1000 on the merged build; status: imu: OK` | Approximately 100 Hz and healthy status were recorded; not a full-run timing bound |
| `SAF-002` | `Log: Gyro calibration too noisy on axis 1 / Fault -57 / STOP LATCHED: IMU startup checks failed` | Calibration failure and stop were recorded; separators are part of the evidence summary |
| `SAF-003` | `G_Hz10=1948, then: [startup] IMU readiness timed out.` | Historical pre-clock-fix failure, not the expected nominal result of the current source |
| `WIFI-002` | `Join times: 10 s, 15 s, 10 s. status answered 3/3` | Three recorded cold-start outcomes; not a guarantee for all networks or boots |
| `CMD-001` | `Reply: state: IDLE  holding: -  delivered: 0 / stop: none / imu: OK  turned 94 deg since start-up` | A remote status reply was recorded; not a physical mission-success result |

`MIS-005` (successful mapping) and `MIS-006` (pickup and delivery) remain **Operator reported — no log captured** in the register. `SAF-009` remains **Inconclusive** about camera-connected startup. Tests marked Not run remain unexecuted records; this document does not upgrade them from code inspection.

The current source explicitly initializes the watchdog microsecond tick in [`cpu_clock.c`](../third_party/mtk3smp-rp2040/kernel/sysdepend/pico_rp2040/cpu_clock.c). That explains the relevance of the historical cold-start comparison, but does not independently reproduce the recorded measurements.

### Reading the current application's messages

The entries below are **source-derived templates**, not a captured successful mission. Angle-bracket fields stand for runtime values. Lines from different phases are not evidence that one run reached every phase.

| Message/template | Interpretation |
|---|---|
| `[MPU] WHO_AM_I=<hex>` | Device identity read; accepted IDs are 0x70/0x71 |
| `[IMU] Keep still; gyro calibration starts in 3 seconds` | Calibration preparation, not readiness |
| `[startup] Healthy hold=<elapsed>/2000 ms` | Startup is accumulating acceptable readings |
| `[mission] READY: type 'start'` | Mission is waiting for operator start |
| `[map] scanning at <angle> degrees` | Requested sweep location; read the gyro result separately for achieved angle |
| `[map] sweep complete; mapped <count>/3; require <required>` | Number of station IDs retained; required count is currently 1 |
| `[map] bearings relative to startup; right positive` | Bearing reference and sign, not magnetic compass direction |
| `[map] ID:<id> <name> distance=<mm> mm bearing=<degrees> deg` | Stored station observation and approximate startup-relative bearing |
| `[object search] no marker; turning right 15 degrees` | No usable target selected from the scan window; this does not identify the cause |
| `[gripper] confirming object ID:<id> before opening` | First candidate seen; another observation is required |
| `[mission] target ID:<id> at <mm> mm (stop limit <mm> mm); x:<x> y:<y>` | Image coordinates in pixels and camera-estimated distance |
| `[gripper] confirming object ID:<id> within 140 mm before closing` | Close-range identity/position confirmation requested, not proof of grip |
| `[map aim] ID:<id> bearing_cd=<bearing> turn_cd=<turn>` | Saved-map candidate; `_cd` values are hundredths of a degree |
| `[direction rescan] no marker in sweep; resuming direction search` | Local sweep failed; caller returns to broader search |
| `[mission] reached <name> marker; releasing object ID:<id>` | Software delivery approach succeeded and release is being commanded |
| `[mission] reversing away from delivery point` | Retreat command follows release; confirm completion separately |
| `[mission] STOP LATCHED: <reason>; reset required to restart` | First latched stop reason retained |

`REL_cd` is relative yaw in hundredths of a degree. `GZ_cdps` is angular rate in hundredths of degrees/s. `G_Hz10` is sampling rate times ten. `LATE` counts intervals above the late-sample threshold; no full-run late-count transcript is available in the reviewed tracked evidence.

### What appears on USB and Wi-Fi

`robot_logf()` emits to the USB console and enqueues a copy for Wi-Fi. The queue contains 16 records of at most 240 bytes each and drops the oldest record when full. Plain `tm_printf()` output, including some IMU, turn and CLI messages, does not automatically enter that application log queue. Therefore the Wi-Fi stream is not necessarily a complete copy of USB output. XIAO `ESP_LOG*` messages belong to the camera firmware and should be read from its own configured console.

UDP 5005 carries robot logs; UDP 5006 carries remote command datagrams and replies. UDP transport or a full queue can lose messages. A missing remote log line does not, by itself, prove that a state was never executed. Neither channel carries camera images in the current application.

The `stats` command reports the worst locally recorded USB CLI acknowledgement-processing interval and USB RX drops. Its timing begins after a full input line is collected and is sampled before the reply is printed. It excludes typing time, full transport delay and mechanical stopping. The remote-command path can read the counter but does not update it with end-to-end network timing. The host parser's separate benchmark is a PC measurement, not Pico task timing.

Sources: [`robot_logf()` and queue](https://github.com/TYW02/robot-arm/blob/731bea5857c92889c5e1f7e7ed64f12f70f0c08a/integration/main.c#L292), [`CLI instrumentation`](../components/cli/cli_utkernel.c), [`Wi-Fi service`](../third_party/mtk3smp-rp2040/lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c), [`laptop tools`](../tools).

### Test interpretation

The register contains 44 selected cases, not a measured statement of full code/path coverage. Its ten CLI entries are a subset of an executable host suite containing 34 parser checks and six line-editor checks. The suite asserts parser return codes/command fields and editor outcomes; reply strings are printed, rather than independently asserted as expected strings.

Some procedural qualifications matter when reading it: `CLI-010` names the "too long" editor case, whose actual input is the oversized `drive 111...` string in the host test, not the literal words `too long`. `MIS-007` requires waiting/retrying reset until the mission is parked. `BRG-004` describes software stop-check timing, not a measured mechanical stopping bound. These explanations do not modify the existing records or claim that those tests have now passed.

## Limitations and lessons

### Assessed outcome and unresolved cause

The team's account is that **one of four assessed runs achieved two successful pickups**, while the other three repeatedly searched by turning. Delivery completion was not specified in that account. An earlier pre-assessment whole-flow run succeeded and is the recording stored in the repository. These observations demonstrate that previous operation did not establish repeatability under the assessed conditions.

The team does not know why detections were unreliable at assessment. Possible categories to investigate include image framing/visibility, marker orientation/occlusion, detector behaviour, mode/size selection and UART delivery. These are hypotheses, not diagnosed faults. There is no synchronized assessed-run camera capture and Pico trace in the evidence used for this document to select among them.

### Visual observability should have been planned earlier

Detection packets describe markers that were successfully detected. They do not show the image areas where detection failed. When the robot kept turning, the team could not inspect the camera's actual view to determine whether the pickup marker was visible, clipped, obstructed or poorly framed.

The professor's suggestion to provide a live feed or captured image addresses that missing observation. Capturing images during the same failing manoeuvre, together with mode, timestamps and detector results, would let the team verify camera positioning before moving on to other components. A representative feed could support ruling out a specific positioning hypothesis; one earlier successful image would not rule out intermittent positioning or visibility problems throughout a run.

This is a lesson about **fault isolation**, not a claim that live video alone would fix detection. Streaming also consumes processing and bandwidth, so a future diagnostic design would need to preserve or measure its effect on the detector and control timing. No live-feed functionality was added for this explanation.

### Search completion and spatial assumptions

The 15-degree search was implemented without a mission-level "searched a full revolution without a target" outcome. It assumes repeated scanning will eventually produce a usable observation. If that assumption fails, the current behaviour is continued searching, which explains the visible circling but does not diagnose the missing observations. Search exhaustion, camera health and no-object outcomes should have been distinguished during initial planning.

The saved bearing map also lacks translation-aware station positions, and size-based ranging lacks a validated 3D/ground-plane model. The selected station is not guaranteed to be the closest in travel distance. Pickup and delivery setpoints depend on camera mounting, marker size and the relationship between camera and claw; camera distance is not claw clearance. These assumptions limit where a previous successful run can be expected to repeat.

### What the present evidence can support

The source supports explanations of task separation, typed commands, checksum rejection, scan synchronization, startup checks, gyro-assisted turns and the intended release/retreat sequence. The test register records component observations and measurements with varying evidence strength. The earlier video is a record of a different test, and the assessment account records inconsistent pickup performance.

Together, they support an account of **implemented logic, partial operation and known limitations**. They do not establish reliable autonomous delivery across the arena, a proven mechanical stop deadline, or resolution of the assessment failure. No further hardware testing was possible during documentation because hardware access had ended. Future improvements described here remain proposals, rather than changes to the assessed firmware.
