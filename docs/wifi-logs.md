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
