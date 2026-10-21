# Copy the prepared interface into your existing Yc checkout

This is a source-only transfer, not a new repository. Your destination is:

`C:\Users\Yc\Desktop\AAI_Y2\T1\embedded_systems\robot-arm`

The prepared copy folder is:

`C:\Users\Yc\Desktop\AAI_Y2\T1\embedded_systems\project\transfer_to_Yc`

## Folder ownership

```text
robot-arm/
  components/
    imu/                    existing; untouched
    servo/                  existing Motion API; untouched
    vision_pico/            Pico UART, parser, VisionResult, IPC, unit tests
    state_machine/          Coordinator implementation and header
  integration/              usermain startup and combined Pico build
  xiao_vision/              ESP32 camera project; only main/main.c is supplied
  third_party/
    mtk3smp-rp2040/          shared kernel source and small app build adapters
  docs/                     interface contract, units, tests, this guide
  tools/                    existing; untouched
```

`third_party` is the one additional root folder required by the reviewed
vision build. It contains the shared kernel dependency, not a second copy of
application code. Do not copy the local `components/servo/third_party` or
`components/servo/tmp` directories as part of this transfer. The existing servo
standalone build remains untouched; only `integration` is the stage-1 test build.

## Copy procedure

1. In the cloned destination repository, confirm `git branch --show-current`
   prints `Yc`. Do not check out or push `main`.
2. Copy the **contents** of `transfer_to_Yc` into the repository root. Merge
   folders; do not delete/replace the entire existing `components`, `docs`,
   `integration`, or `xiao_vision` directories. In particular, keep your servo,
   IMU and other teammates' files.
3. The intended existing-file replacements are `integration/main.c`,
   `integration/CMakeLists.txt`, and the checksum-enabled `xiao_vision/main/main.c`.
   These were compared with the current source checkout. If you have edited
   those files since preparing this package, compare them before overwriting.
4. The supplied `.gitignore` is based on `Yc`; merge rather than discard any
   additional local ignore rules. New rules are `/build_vision_interface/` and
   `*.exe`. No `.git`, temporary files, compiler outputs, or local SDK is supplied.
5. Read `docs/VISION_INTERFACE.md`, build the integration target, then do the
   no-motion camera acceptance test. Do not flash the old servo demo by mistake.

Check only the intended changes before committing, for example:

```text
git status --short --branch
git add .gitignore components/vision_pico components/state_machine integration/main.c integration/CMakeLists.txt docs/COPY_TO_Yc.md docs/VISION_INTERFACE.md third_party/mtk3smp-rp2040 xiao_vision/main/main.c
git diff --cached --stat
git diff --cached --name-only
```

Do not use an indiscriminate `git add .`: your original checkout already has
unrelated local servo dependency/temporary folders. Your pending Motion source
edit is also separate and is not included or reverted by this package.

Commit and push to `Yc` only after your own review. Nothing has been committed,
pushed, merged into `Yc`, or changed on `main` automatically.

## Safety boundary

The data path remains camera -> UART -> Vision task -> typed VisionResult ->
Coordinator. X/Y are image pixels. This firmware does not link Motion, perform
camera-to-arm conversion, or command servos. Keep servo V+ disconnected for the
first interface test. Hardware testing is still required after copying/flashing.
