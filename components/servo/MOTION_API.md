# Motion Control API

Teammates include `motion.h`. Pulse widths are microseconds; there is no angle
conversion or inverse kinematics. All functions block until the existing Motion
task finishes the commanded trajectory and settling delays, or reports a fault.
They can be called from ordinary tasks on either core. Do not call them from an
ISR, from the Motion task itself, or while holding the shared I2C mutex.

```c
#include "motion.h"

MotionResult scan_shelf(void)
{
    MotionResult result = GripperOpen();
    if (result != MOTION_OK) return result;
    return MoveShelfScan();
}

MotionResult custom_move(void)
{
    ArmPose target = ITEMS_SCAN;
    target.base = 2500;
    target.gripper = 1450;
    return MovJ(&target);
}
```

`main.c` retains responsibility for startup, the task definitions, and sequencing.
Its existing initialization command establishes HOME before normal API calls.
Startup is open-loop and assumes the arm is positioned suitably for that existing
startup procedure; it does not measure the initial physical pose.

## Pose semantics

| Call | Behavior |
| --- | --- |
| `MovJ(&target)` | Validate and command all six requested pulse widths; gripper moves last only if changed. |
| `MoveHome()` | Preserve gripper; use 1600 us base from shelf, 1550 us directly from items; repeated HOME retains that correction. |
| `MoveShelfScan()` / `MoveItemsScan()` | Move to the named arm posture while preserving the current gripper pulse. |
| `GripperOpen()` / `GripperClose()` | Move only the gripper to 1450 / 2700 us. |

`MovJ(&HOME)` is a literal joint target, so its base is always 1600 us and its
gripper target is 1450 us. Use `MoveHome()` for the tested approach correction.
The implementation tracks `commanded_pose`, not measured position. A partial
I2C failure updates only successfully written commands, reports `MOTION_IO_ERROR`,
and prevents subsequent normal moves until explicit startup initialization
succeeds. Initialization failure leaves the arm not ready. An IPC failure latches
the API unavailable for the remainder of the run, preventing a stale completion
from being used for a later request.

## Preserved calibration

| Joint | HOME | SHELF_SCAN | ITEMS_SCAN | Accepted range (us) |
| --- | ---: | ---: | ---: | ---: |
| Base | 1600 | 400 | 2900 | 400-2900 |
| Shoulder | 2000 | 1850 | 1700 | 1300-2450 |
| Elbow | 1800 | 1900 | 1500 | 1100-2500 |
| Wrist pitch | 700 | 1600 | 800 | 500-1850 |
| Wrist rotation | 400 | 400 | 400 | 400 only |
| Gripper | 1450 | 1450 | 1450 | 1450-2700 |

Ranges are inclusive. NULL and invalid targets return `MOTION_INVALID_POSE` before
queueing or writing a servo; the Motion task also validates received targets.
Validation checks individual calibrated bounds, not collision clearance for
every possible combination of otherwise valid joints.

## Movement and timing

The existing staged interpolation is retained:

1. Wrist pitch to transit (700 us).
2. Shoulder and elbow together to transit (2000 / 1800 us).
3. Base to target.
4. Shoulder and elbow together to target.
5. Wrist pitch to target.
6. Wrist rotation to its validated target (currently fixed at 400 us).
7. Gripper separately, if a generic target requires a change.

The working settings remain 30 interpolation steps with a 15 ms task delay per
step, and 300 ms pauses between the first five stages. Completion now includes a
final 300 ms arm settling delay; a changed gripper uses the same interpolation
and its own 300 ms settling delay. Scheduling and I2C transactions add time to
these nominal durations. Completion is inferred from commands and timing;
orientation verification remains the IMU's responsibility.

The two existing tasks, core assignments, command/report buffers and per-I2C
transaction mutex remain. A separate API mutex serializes each request/reply
exchange; it is distinct from the I2C mutex, which is never held across a delay.
The System Coordinator never directly executes servo writes.

## Build and verification

Build `robot_main` with the existing CMake target, then flash `build/robot_main.uf2`.
`calibration.c` and its separate Pico SDK calibration target are unchanged.

Host regression checks (MSYS2/UCRT GCC and pthreads on this Windows setup):

```powershell
& C:\msys64\ucrt64\bin\gcc.exe -std=c11 -Wall -Wextra -Werror -pthread -Itests/stubs tests/motion_api_test.c -o build/motion_api_test.exe
& .\build\motion_api_test.exe
```

The host checks exercise the actual motion implementation with a mocked bus and
kernel calls, including a recorded pre-refactor PWM trace for the existing scan
sequence (844 servo writes and 600 interpolation delays). They do not verify
physical servo position or load behavior.
