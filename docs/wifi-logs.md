# Wi-Fi quick start (logs + remote commands)

Do this once per person. Your Wi-Fi name and password stay on your own machine:
`wifi_credentials.h` and `wifi_log_config.h` are gitignored, so they are never
committed and a fresh clone does not have them.

1. **Create your local Wi-Fi config.** From the project root:

   ```
   python3 tools/setup_wifi.py        # Windows: py tools\setup_wifi.py
   ```

   It asks for your Wi-Fi name and password and writes both config files in
   `third_party/mtk3smp-rp2040/config/`. Use a plain 2.4 GHz network or phone
   hotspot (no emoji in the name). iPhone hotspot: turn on *Maximize
   Compatibility* and keep the Personal Hotspot screen open while the Pico joins.

2. **Build with Wi-Fi on** (Wi-Fi is off in the default CMake build):

   ```
   cd third_party/mtk3smp-rp2040/build_make
   make SMP=1 CONSOLE=usb_cdc WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1 WIFI_LOG=1 PICO_SDK_PATH=<path to your pico-sdk> -j8
   ```

   On macOS, if make stops with a "dependency file" error, run the same command
   again. Rebuild and re-flash whenever you change the Wi-Fi name or password.
   Never share the `.uf2`: it contains your password.

3. **Flash.** Hold BOOTSEL, plug in the Pico W, and copy
   `build_make/mtk3pico_smp1_usb_cdc_wifi_log.uf2` onto it.

4. **Receive logs.** Start `python3 tools/wifi_log_receiver.py` before powering
   the Pico, and open the USB serial console (115200). Wait for `link=3` and
   `[wifi-log] own <ip> -> ...`. That `<ip>` is the Pico's address.

5. **Send operator commands.** They are the same commands as the USB console
   (`help` lists them). One command:

   ```
   python3 tools/wifi_cmd_send.py <pico-ip> status
   ```

   It prints the Pico's answer (state, held object, stop reason, IMU). Or open an
   interactive prompt:

   ```
   python3 tools/wifi_cli.py <pico-ip>
   ```

   Wheels off the ground for the first test: wait for `[mission] READY` in the
   log, then `f` (drive forward, 30% for 500 ms), `l`, `r`, `b`, `stop`, `reset`.

The sections below explain each part in more detail.

---

# Viewing robot logs over Wi-Fi

The Pico W sends each robot log message by UDP unicast on port **5005**.
The laptop and Pico W must be on the same Wi-Fi network. A phone hotspot can
be used if both devices are connected to it.

## Set Wi-Fi credentials locally

From the `integration` folder, copy either sample only if its local config file
does not already exist. `-NoClobber` preserves any credentials you configured
earlier:

```powershell
Copy-Item -NoClobber ..\third_party\mtk3smp-rp2040\config\wifi_credentials.example.h `
  ..\third_party\mtk3smp-rp2040\config\wifi_credentials.h
Copy-Item -NoClobber ..\third_party\mtk3smp-rp2040\config\wifi_log_config.example.h `
  ..\third_party\mtk3smp-rp2040\config\wifi_log_config.h
```

Edit `wifi_credentials.h`: set `UTK_WIFI_CREDENTIALS_SET` to `1`, then enter
the Wi-Fi name and password in `UTK_WIFI_SSID` and `UTK_WIFI_PASSWORD`. The
real credentials file is ignored by Git; do not commit it. Use a 2.4 GHz
network supported by the Pico W.

Edit `wifi_log_config.h` to set `UTK_WIFI_LOG_TARGET_A` through `_D` to the
laptop's Wi-Fi IPv4 address from `ipconfig`. The address currently set for
this project is `10.34.167.128`. This file is ignored by Git as well. Update
it and rebuild if the laptop's Wi-Fi address changes.

## Build and receive logs

From the `integration` folder, build with:

```powershell
cmake --build build --target robotic_arm
```

The resulting UF2 is
`..\third_party\mtk3smp-rp2040\build_make\mtk3pico_smp1_usb_cdc_wifi_log.uf2`.

On the laptop, start the receiver before powering the car:

```powershell
python ..\tools\wifi_log_receiver.py
```

The first run may cause Windows Firewall to ask whether Python can receive
traffic. Allow it on the active private network. Then flash the new UF2 to the
Pico W and power the car from the power bank. The laptop can stay connected to
the same Wi-Fi without a USB cable to the Pico.

The transport is UDP, so it does not need a broker and a log line may be lost
if Wi-Fi is disconnected or the small queue fills. USB CDC output remains
available when the Pico is connected to the laptop.

## macOS / Linux steps

```bash
cd third_party/mtk3smp-rp2040/config
[ -f wifi_credentials.h ] || cp wifi_credentials.example.h wifi_credentials.h
[ -f wifi_log_config.h ]  || cp wifi_log_config.example.h  wifi_log_config.h
# edit wifi_credentials.h (SSID, password, UTK_WIFI_CREDENTIALS_SET 1)

cd ../build_make
make SMP=1 CONSOLE=usb_cdc WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1 \
     WIFI_LOG=1 PICO_SDK_PATH=$HOME/.pico-sdk/sdk/2.3.1 -j8

python3 ../../../tools/wifi_log_receiver.py     # on the laptop, before powering the Pico
```

Find the laptop address with `ipconfig getifaddr en0` (macOS) or `ipconfig` (Windows).

## No log lines arriving? Read the USB status line

With the Pico on USB, the console prints a line like:

```
[wifi-log] own 192.168.1.42 -> 192.168.1.255:5005 sent=37 failed=0 last_err=0
```

- No `[wifi-log]` line at all: the Pico has not joined Wi-Fi or has no IP. Check SSID and password, and use a 2.4 GHz network.
- `sent` rises but nothing arrives: the target is wrong or the laptop firewall or network is blocking UDP 5005. Allow Python through the firewall; some hotspots and campus networks block device-to-device traffic.
- `failed` rises: `last_err` is the lwIP error code (-1 = out of memory).

## Broadcast mode (no fixed laptop IP)

Setting `UTK_WIFI_LOG_BROADCAST 1` in `wifi_log_config.h` makes the Pico send to the
subnet broadcast address (shown as `x.y.z.255` in the status line), so the logs reach the
laptop whatever address DHCP gave it. The example config enables it. Set it to 0 to send
to the fixed `UTK_WIFI_LOG_TARGET_*` address instead.

## Sending commands to the Pico over Wi-Fi (UDP port 5006)

The Pico listens on UDP port **5006** once it has an IP address. Each datagram is **one operator
command line**, the same text you would type on the USB console. It goes through the same parser
(`components/cli`) and the same handlers (`integration/cli_bridge.c`) as the USB CLI, so the same
rules apply: `start` only from IDLE, `stop` works at any time and stays latched until `reset`, the
debug commands only work in IDLE, and the IMU safety interlock still guards every move. The Pico
sends the answer back to the sender, and the log receiver shows a `[cmd] "<command>" -> <answer>` line.

| Command | What it does | When it works |
|---|---|---|
| `start` | start the mission (360° wall-mapping sweep, then pick up and deliver blocks) | IDLE only |
| `stop` (or `x`) | emergency stop; brakes the motors; the stop stays latched | any time |
| `reset` | clear a stop and return to IDLE | only after a stop |
| `status` | state, held block, blocks delivered, stop reason, gyro health | any time |
| `help` (or `?`) | list the commands | any time |
| `f` / `b` / `l` / `r` `[speed] [ms]` | forward / backward / spin left / spin right (default 30%, 500 ms) | IDLE only |
| `drive <L> <R> [ms]` | each wheel -100 to 100 % | IDLE only |
| `open` / `close` | claw to its open / close angle | IDLE only |
| `claw <0-180>` / `claw off` | claw to an angle / stop the claw signal | IDLE only |
| `stats` | worst command reply time so far, in µs (NFR1 evidence) | test only |

Speed is 1-100 % and time is 1-5000 ms. Commands are not case-sensitive.

```bash
# The Pico IP is the "own x.x.x.x" in the [wifi-log] line
python3 tools/wifi_cmd_send.py 172.20.10.5 status      # one command
python3 tools/wifi_cli.py 172.20.10.5                   # interactive prompt
```

If a command comes back with `rejected: not ready yet (IMU start-up)`, the car is still doing its
IMU start-up checks: keep it still and flat, and wait for `[mission] READY: type 'start'` in the log.
UDP is unreliable, so a missing reply does not always mean the command was lost. Do not send the
same command from the USB console and over Wi-Fi at the same moment.
