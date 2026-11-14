# fw-nrf52840-ble-sensor-monitor

Reads the nRF52840 die temperature and the supply voltage (VDD) and sends them to a phone as BLE notifications. Zephyr RTOS, nRF Connect SDK.

## What it does

- Periodic die temperature reading and VDD reading through the SAADC.
- Custom GATT service with automatic notifications every 2 seconds.
- Minimum and maximum temperature tracking, and an LED alert at 40 C or above.
- Readings logged over UART.
- The device advertises as `nRF52840_Sensor`.

GATT service:

| Item | UUID | Properties | Data |
|---|---|---|---|
| Service | `12345678-1234-5678-1234-56789abcdef0` | | |
| Temperature | `12345678-1234-5678-1234-56789abcdef1` | read, notify | float, 4 bytes, degrees C |
| Battery | `12345678-1234-5678-1234-56789abcdef2` | read, notify | uint8, 1 byte, percent |

LEDs and buttons:

| Control | Pin | Function |
|---|---|---|
| LED1 | P0.13 | heartbeat every second |
| LED2 | P0.14 | BLE connected |
| LED3 | P0.15 | flashes on each notification |
| LED4 | P0.16 | temperature alert |
| BTN1 | P0.11 | immediate reading |
| BTN2 | P0.12 | notifications on or off |
| BTN3 | P0.24 | reset min and max |
| BTN4 | P0.25 | print full status |

Threads: main (priority 0) runs init and the LED1 heartbeat. A notify thread (1024 B stack, priority 7) reads the sensors and sends the notifications. Buttons use GPIO interrupts.

## Hardware

| Item | Detail |
|---|---|
| Board | nRF52840 DK (PCA10056) |
| MCU | nRF52840, Cortex-M4F, 64 MHz, 1 MB flash, 256 KB RAM |
| Sensors | on-die temperature, SAADC 12-bit for VDD |
| UART | J-Link VCOM, 115200 8N1 |
| Phone | nRF Connect app |

## Build

Requires nRF Connect SDK v3.x. Use a project path without spaces.

```bash
west build -b nrf52840dk/nrf52840 --no-sysbuild --pristine
```

If the temperature stays at 0, rebuild with `--pristine` so the overlay is applied. The overlay defines the TEMP node and the SAADC VDD channel, and `prj.conf` needs `CONFIG_TEMP_NRF5=y` and `CONFIG_ADC=y`.

## Flash

```bash
west flash --runner jlink
```

## Test

Serial: open the DK's COM port at 115200 8N1. After the banner, a line like this appears every 2 seconds:

```
[00:00:02.001] #1 | Temp: 26.5 C | VDD: 3287 mV (99%)
```

BLE:

1. In nRF Connect, scan and connect to `nRF52840_Sensor`.
2. Open the service `...def0` and enable notifications on the temperature and battery characteristics.
3. The values update every 2 seconds.

Buttons: BTN1 forces a reading, BTN2 pauses and resumes notifications, BTN3 resets min and max, BTN4 prints the status.

## License

MIT, see [LICENSE](LICENSE).
