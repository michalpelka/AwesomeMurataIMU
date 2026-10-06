# KAmod prototype

STM32F103 (Blue Pill) firmware that reads a Murata SCH16T IMU and streams
timestamped samples over USB (virtual COM port) and USART1.

## Sample rate

The sensor samples internally at about 23.77 kHz and outputs every Nth sample,
where N is the decimation. Only these rates are possible:

| Decimation | Output rate | Sample period |
|-----------:|------------:|--------------:|
|         32 |    ~743 Hz  |       1346 µs |
|         16 |   ~1486 Hz  |        673 µs |
|          8 |   ~2971 Hz  |        337 µs |
|          4 |   ~5942 Hz  |        168 µs |
|          2 |  ~11884 Hz  |         84 µs |

The two supported setups are **0.7 kHz** (decimation 32) and **1.4 kHz**
(decimation 16). Both are verified on hardware with the Debug build: every
sample arrives over USB, with no gaps or duplicate timestamps.

### Setting the rate

Edit both decimation values in `Core/Inc/config.h`; they must be equal.

0.7 kHz (743 Hz):

```c
#define DECIMATION_RATE     32
#define DECIMATION_ACC      32
```

1.4 kHz (1486 Hz):

```c
#define DECIMATION_RATE     16
#define DECIMATION_ACC      16
```

Then rebuild and flash (see below). Nothing else needs to change: the packet
format is the same and each timestamp is taken from the sensor's DRY signal,
so the rate can be read from the timestamps.

### Limits

- **USART1 (460800 baud)** carries at most about 1200 packets/s. At 1.4 kHz it
  skips packets; USB still gets every sample. Use USB at 1.4 kHz.
- **Higher rates** depend on the build. At decimation 4 (5942 Hz) the Release
  build (`-Os`) delivered every sample and the Debug build (`-O0`) about 82%.
  At decimation 2 the Release build delivered about 83%. Decimation 8
  (2971 Hz) has not been tested since these optimizations.

## Build and flash

Build the Debug preset:

```sh
cmake --preset Debug
cmake --build build/Debug
```

Flash with an ST-Link connected to this machine:

```sh
openocd -f stm32f103c8_blue_pill.cfg -c "program build/Debug/KAmod_prototype.elf verify reset exit"
```

In CLion, use the **Flash and Debug (Mac)** run configuration with the Debug
button. It builds, flashes and starts debugging.

## Viewing the data

The board shows up as `/dev/ttyACM0` on Linux, including the Raspberry Pi.
It needs `pyserial` and `matplotlib`:

```sh
python3 test_viz.py --port /dev/ttyACM0            # live plot
python3 test_viz.py --port /dev/ttyACM0 --console  # text output
```

The plot shows the last `HISTORY` samples (500 by default): about 0.67 s at
0.7 kHz and 0.34 s at 1.4 kHz.

Each packet is 38 bytes, packed little-endian (`<BQ3i3iiB`):

| Field          | Type        | Notes                                             |
|----------------|-------------|---------------------------------------------------|
| sync           | `uint8`     | `0xAA`                                            |
| timestamp_us   | `uint64`    | GPS/Unix time once NMEA + PPS are locked, otherwise time since boot |
| rate[3]        | `int32` ×3  | X, Y, Z; 1600 LSB per °/s                         |
| acc[3]         | `int32` ×3  | X, Y, Z; 3200 LSB per m/s²                        |
| temp           | `int32`     | raw                                               |
| crc            | `uint8`     | CRC-8 (poly 0x07, init 0) over the first 37 bytes |
