# AwesomeImu

Firmware for an STM32F103 (Blue Pill) that reads a Murata **SCH16T** 6-axis
IMU (gyro + accelerometer) and streams every sample, timestamped against GPS
time, over USB and UART. Python tools for a Linux host (Raspberry Pi) plot the
data and show the board's orientation in 3D.

- Sample rates of **743 Hz or 1486 Hz** (more with the Release build)
- **GPS-disciplined timestamps:** NMEA time plus PPS, corrected for the
  crystal's frequency error, accurate to a few µs between pulses
- **USB virtual COM port** (CDC) with every sample, plus USART1 at 460800 baud
- A per-packet **GPS sync status** and **CRC-8**

```
SCH16T ──SPI──┐                         ┌── USB CDC ──► host (/dev/ttyACM*)
   DRY ──EXTI─┤                         │
              ├── STM32F103 (72 MHz) ───┤
u-blox ─NMEA──┤                         └── USART1 ───► 460800 baud (≤ ~1200 pkt/s)
   PPS ──EXTI─┘
```

## Hardware

- STM32F103C8 Blue Pill with an **8 MHz crystal**. The firmware runs from the
  crystal (HSE ×9 = 72 MHz) because USB needs an accurate 48 MHz clock; it
  won't start on a board without one.
- Murata SCH16T on SPI
- Optional GPS receiver with NMEA output and PPS, e.g. u-blox
- ST-Link V2 for flashing and debugging

### Wiring

| Function       | Blue Pill pin | Connect to                    | Notes                               |
|----------------|---------------|-------------------------------|-------------------------------------|
| SPI1 SCK       | PA5           | SCH16T SCK                    | 9 MHz, mode 0, 16-bit frames        |
| SPI1 MISO      | PA6           | SCH16T MISO                   |                                     |
| SPI1 MOSI      | PA7           | SCH16T MOSI                   |                                     |
| SPI CS         | PB0           | SCH16T CS                     |                                     |
| DRY            | PA3           | SCH16T DRY/SYNC               | Data ready, rising edge             |
| EXTRESN        | PA4           | SCH16T EXTRESN                | Sensor reset                        |
| NMEA in        | PB11          | GPS TX                        | USART3 RX, 9600 8N1, needs RMC      |
| PPS in         | PB6           | GPS PPS                       | Rising edge, internal pull-down     |
| USB D− / D+    | PA11 / PA12   | Blue Pill USB connector       | CDC virtual COM port                |
| USART1 TX / RX | PA9 / PA10    | USB-serial adapter (optional) | 460800 8N1                          |
| SWDIO / SWCLK  | PA13 / PA14   | ST-Link                       |                                     |
| LED1           | PC13          | on board                      | Blinks fast on a fatal error        |
| LED2           | PC14          | (optional LED)                | Toggles every 100 samples           |

All grounds must be connected. **Power the board from one source only.** The
Blue Pill connects USB 5 V directly to its 5 V pin, so also feeding it from an
ST-Link or another supply can push current back into the host's USB port. With
USB plugged in, connect only GND, SWDIO and SWCLK from the ST-Link, or use a
USB isolator.

## Timestamps and GPS sync

Each packet's timestamp is taken in the interrupt for the sensor's DRY pulse,
from the CPU cycle counter (DWT CYCCNT, extended to 64 bits).

- **Without GPS** the timestamp is µs since boot.
- **With GPS** it is Unix time in µs. At each PPS pulse the firmware takes the
  time from the last RMC sentence, rounds it down to the whole second and adds
  1 s. This works at any NMEA rate; the u-blox default of 5 Hz is fine. A pulse
  is accepted only if a valid RMC arrived in the last second and at least
  900 ms have passed since the previous pulse.
- **Crystal drift correction:** at each PPS exactly 1 s after the previous
  one, the firmware measures how many CPU cycles one GPS second took, and
  converts timestamps with that rate instead of the nominal 72 MHz.
  Measurements more than 200 ppm off are rejected, and the rest are smoothed
  by 1/8. On the test board the crystal is +49 ppm off; without the correction
  timestamps drifted up to 49 µs ahead between pulses.
- **Holdover:** if PPS stops, the clock keeps running at the last measured
  rate, and `gps_status` reports it.

Interrupt priorities are set so the timestamps aren't delayed: DRY and PPS
(EXTI) are at 0, USB at 2, NMEA (USART3) at 3 and SysTick at 15. USART1's DMA
and TIM2 are also at 0, so they can delay a timestamp by a few µs.

## Sample rate

The SCH16T samples internally at about 23.77 kHz and outputs every Nth sample,
where N is the decimation. Only these rates are possible:

| Decimation | Output rate | Sample period |
|-----------:|------------:|--------------:|
|         32 |    ~743 Hz  |       1346 µs |
|         16 |   ~1486 Hz  |        673 µs |
|          8 |   ~2971 Hz  |        337 µs |
|          4 |   ~5942 Hz  |        168 µs |
|          2 |  ~11884 Hz  |         84 µs |

The two supported setups are **0.7 kHz** (decimation 32) and **1.4 kHz**
(decimation 16, the default). Both are verified on hardware with the Debug
build: every sample arrives over USB, with no gaps or duplicate timestamps.

To change the rate, set both decimation values in `Core/Inc/config.h` (they
must be equal), then rebuild and flash.

```c
// 0.7 kHz (743 Hz)
#define DECIMATION_RATE     32
#define DECIMATION_ACC      32

// 1.4 kHz (1486 Hz)
#define DECIMATION_RATE     16
#define DECIMATION_ACC      16
```

Nothing else needs to change: the packet format stays the same, and the rate
can be read from the timestamps.

**Limits:**

- USART1 (460800 baud) carries at most about 1200 packets/s. Above that it
  skips packets, while USB still gets every sample.
- Higher rates depend on the build. At decimation 4 (5942 Hz) the Release
  build (`-Os`) delivered every sample and the Debug build (`-O0`) about 82%.
  At decimation 2 the Release build delivered about 83%. Decimation 8
  (2971 Hz) hasn't been tested.

## Building and flashing

Requirements: CMake 3.22+, Ninja, the `arm-none-eabi-gcc` toolchain and
OpenOCD. On macOS: `brew install cmake ninja openocd` and the
`arm-none-eabi-gcc` toolchain. On a Raspberry Pi: `sudo apt install openocd`.

```sh
cmake --preset Debug          # or Release
cmake --build build/Debug
openocd -f stm32f103c8_blue_pill.cfg -c "program build/Debug/AwesomeImu.elf verify reset exit"
```

The Debug build uses about 93% of the 64 KB flash; the Release build about 50%.

### CLion

Open the folder as a CMake project and use one of the run configurations in
`.idea/runConfigurations/` with the **Debug** button (not Run):

- **Flash and Debug (Mac)**: ST-Link plugged into this machine. Runs OpenOCD
  locally.
- **Debug on Pi (OpenOCD)**: ST-Link plugged into the Raspberry Pi. Runs
  OpenOCD on the Pi over SSH and tunnels the GDB port. It needs key-based SSH
  to `pi@192.168.2.100` and `~/kamod/stm32f103c8_blue_pill.cfg` on the Pi.

Both build, flash if the firmware changed, reset and stop at breakpoints.

### Regenerating with STM32CubeMX

`AwesomeImu.ioc` is the CubeMX project (CubeMX 6.17 or later, STM32Cube
FW_F1 V1.8.7, CMake toolchain). Code inside `/* USER CODE BEGIN */ … END */` blocks
survives regeneration; anything else in generated files is overwritten.
CubeMX writes CRLF line endings, so convert them back to LF before
committing. The PPS interrupt (EXTI9_5) is enabled by hand in `main.c`, not in
the `.ioc`.

## Host tools

The tools need Python 3 with `pyserial`, `numpy` and `matplotlib`. On a
Raspberry Pi:

```sh
sudo apt install python3-serial python3-matplotlib python3-tk
sudo usermod -aG dialout $USER      # serial port access; log in again
```

The board appears as `/dev/ttyACM*`. If another USB serial device such as a
u-blox is plugged in, the number can change, so use the stable name:

```sh
PORT=$(ls /dev/serial/by-id/*STM32_Virtual_ComPort*)
```

### `test_viz.py`: live plot

```sh
python3 test_viz.py --port $PORT            # plot of rate, acceleration, temperature
python3 test_viz.py --port $PORT --console  # one line of text per 0.1 s
```

The plot shows the last 500 samples (`HISTORY`): about 0.67 s at 0.7 kHz and
0.34 s at 1.4 kHz. Both modes show the GPS status.

### `imu_3d.py`: 3D orientation

```sh
python3 imu_3d.py --port $PORT
```

A box that follows the board's orientation, with roll, pitch, yaw, sample rate
and GPS status. **Keep the board still for the first 2 s** while it measures
the gyro's zero offset. Roll and pitch are held against gravity (Mahony
filter); yaw has no magnetometer to correct it and drifts slowly. Press **r**
to reset the orientation and **q** to quit.

## Packet format

Each packet is 39 bytes, packed little-endian (`<BQ3i3iiBB` in Python):

| Field        | Type       | Notes                                                       |
|--------------|------------|-------------------------------------------------------------|
| sync         | `uint8`    | `0xAA`                                                      |
| timestamp_us | `uint64`   | Unix time in µs once GPS-locked, otherwise µs since boot    |
| rate[3]      | `int32` ×3 | X, Y, Z; 1600 LSB per °/s                                   |
| acc[3]       | `int32` ×3 | X, Y, Z; 3200 LSB per m/s²                                  |
| temp         | `int32`    | Raw sensor value                                            |
| gps_status   | `uint8`    | GPS sync flags, see below                                   |
| crc          | `uint8`    | CRC-8 (polynomial 0x07, init 0) over the first 38 bytes     |

`0xAA` can also appear inside a packet, so a reader should check the CRC and,
if it fails, resync from the next `0xAA`.

`gps_status` bits:

| Bit  | Name           | Meaning                                                          |
|------|----------------|------------------------------------------------------------------|
| 0x01 | GPS_TIME_VALID | Locked to PPS at least once since boot: timestamps are Unix time |
| 0x02 | GPS_PPS_OK     | Last PPS accepted less than 1.1 s ago. If clear while TIME_VALID is set, the clock is holding over on the last measured rate |
| 0x04 | GPS_NMEA_OK    | A valid RMC time arrived less than 1 s ago                       |
| 0x08 | GPS_RATE_CAL   | The CPU clock rate has been measured against PPS                 |

Fully synced is `0x0F`.

## Project layout

| Path                          | Contents                                                        |
|-------------------------------|-----------------------------------------------------------------|
| `Core/Src/main.c`             | Main loop, packet assembly, timestamps, GPS/PPS handling        |
| `Core/Src/SCH1.c`, `SCH1.h`   | Murata SCH16T driver                                            |
| `Core/Src/hw.c`               | SPI transfer and other hardware glue for the driver             |
| `Core/Inc/config.h`           | Sensor filter, sensitivity and decimation (sample rate)         |
| `Core/Src/minmea.c`           | NMEA parser                                                     |
| `USB_DEVICE/`, `Middlewares/` | USB CDC stack (generated by CubeMX)                             |
| `Drivers/`                    | STM32 HAL and CMSIS                                             |
| `AwesomeImu.ioc`              | STM32CubeMX project                                             |
| `stm32f103c8_blue_pill.cfg`   | OpenOCD config for ST-Link + STM32F103                          |
| `test_viz.py`, `imu_3d.py`    | Host tools                                                      |

## Known limitations

- **Stale data when the USB port is opened.** If nothing has read the port for
  a while, the first transfer can contain up to 52 old packets. Drop packets
  until the timestamps are continuous.
- **No USART3 error recovery.** A UART error on the NMEA input, such as an
  overrun from plugging in a powered GPS, stops NMEA reception until reset.
- **The SCH16T start-up check can report OK with no sensor attached**, because
  it doesn't confirm the sensor answers. "SCH1 OK" on USART1 doesn't prove the
  sensor is connected.
- **NMEA age counter wraps** after about 49.7 days without NMEA, which could
  accept one PPS with a stale time.
