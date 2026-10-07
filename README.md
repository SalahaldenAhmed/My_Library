# PS4-Controlled 3-Axis Stepper Robot Arm (ESP32 + STM32)

A 3-axis robot arm driven by TB6600 stepper drivers and controlled wirelessly with a PS4 (DualShock 4) controller.

- **ESP32** connects to the PS4 controller over Bluetooth, reads the sticks and sends compact UART frames.
- **STM32F103C8T6 (Blue Pill)** receives the frames and generates the step/direction signals for three TB6600 drivers.

```
PS4 controller --Bluetooth--> ESP32 --UART 115200--> STM32F103 --PUL/DIR--> 3x TB6600 --> 3x stepper
```

## Features

- Proportional speed control: further stick push = faster stepping
- Configurable deadzone to ignore stick drift
- Framed UART protocol with header and XOR checksum (noise-resistant)
- Non-blocking step generation for 3 axes at once (free-running 1 us timer)
- Failsafe: motors stop about 200 ms after the last valid frame (controller or ESP32 disconnects)
- Onboard LED (PC13) lights while any motor is commanded to move

## Repository structure

```
.
├── ESP32/
│   └── esp32_robot/esp32_robot.ino      # Arduino sketch (PS4 -> UART frames)
├── STM32/
│   └── Stepper_Motor/                   # STM32CubeIDE project (.ioc, Src/main.c, ...)
├── docs/                                # Screenshots (CubeMX, wiring)
└── README.md
```

## Hardware

| Part | Notes |
|---|---|
| ESP32 dev board | Any ESP32 with classic Bluetooth (not S3/C3) |
| STM32F103C8T6 | "Blue Pill" |
| 3x TB6600 stepper drivers | Set microstep and current DIP switches for your motors |
| 3x stepper motors | |
| DualShock 4 controller | Must be paired with the ESP32 Bluetooth MAC |
| Motor power supply | Match your TB6600/motor ratings (typically 12-24 V) |

## Wiring

### ESP32 <-> STM32

| ESP32 | STM32 | Note |
|---|---|---|
| GPIO17 (TX2) | PB11 (USART3 RX) | Data to STM32 |
| GPIO16 (RX2) | PB10 (USART3 TX) | Optional (not used yet) |
| GND | GND | **Common ground is required** |

### STM32 <-> TB6600

| Axis | Function | STM32 pin |
|---|---|---|
| 0 - Base | PUL (step) | PB15 |
| 0 - Base | DIR | PB14 |
| 1 - Shoulder | PUL | PA5 |
| 1 - Shoulder | DIR | PA6 |
| 2 - Elbow | PUL | PB0 |
| 2 - Elbow | DIR | PA0 |

Connect the driver inputs (PUL+/PUL-, DIR+/DIR-) according to your driver wiring (signal pin to the - terminals with + to 3.3 V, or the reverse). Share GND between the STM32 and the driver logic side. The firmware idles all signal pins LOW.

### Stick mapping

| Stick | Axis |
|---|---|
| Left stick X | Axis 0 (Base) |
| Left stick Y | Axis 1 (Shoulder) |
| Right stick Y | Axis 2 (Elbow) |

## STM32CubeMX (.ioc) configuration

The firmware configures the clock, UART and TIM1 itself, so the .ioc only needs a clean base project.

> Add your screenshots to `docs/` and they will show below.

![Pinout](docs/cubemx_pinout.png)
![Clock](docs/cubemx_clock.png)
![USART3](docs/cubemx_usart3.png)

**Project:** MCU `STM32F103C8Tx`, toolchain STM32CubeIDE.

**System Core**

| Setting | Value |
|---|---|
| SYS -> Debug | Serial Wire |
| SYS -> Timebase Source | SysTick |
| RCC -> HSE | Disable (internal HSI is used, no crystal needed) |

**Clock Configuration**

| Item | Value |
|---|---|
| Source | HSI /2 -> PLL x16 |
| SYSCLK / HCLK | 64 MHz |
| APB1 prescaler | /2 (32 MHz) |
| APB2 prescaler | /1 (64 MHz, TIM1 clock) |

**GPIO (Output Push-Pull, speed High, initial level Low)**

| Pin | Use |
|---|---|
| PC13 | Onboard LED (initial level High, active low) |
| PB15 / PB14 | Base PUL / DIR |
| PA5 / PA6 | Shoulder PUL / DIR |
| PB0 / PA0 | Elbow PUL / DIR |

**Connectivity -> USART3**

| Setting | Value |
|---|---|
| Mode | Asynchronous (PB10 TX, PB11 RX) |
| Baud rate | 115200 |
| Word length / Parity / Stop bits | 8 bits / None / 1 |
| DMA | None |
| NVIC | USART3 global interrupt unchecked |

**Timers -> TIM1:** can be left disabled (registers are set in code). If it is enabled in the .ioc, `main.c` must define `htim1` (already included in this repo).

## Build and flash (STM32)

1. Open `STM32/Stepper_Motor` in STM32CubeIDE (File -> Open Projects from File System).
2. Build with the hammer icon (Debug configuration).
3. Connect an ST-Link and press Run/Debug to flash.
4. On power-up, the PC13 LED blinks 3 times to confirm the firmware is running.

## Build and flash (ESP32)

1. Install the Arduino IDE with ESP32 board support.
2. Install the **PS4Controller** library.
3. Open `ESP32/esp32_robot/esp32_robot.ino`, select your ESP32 board and upload.
4. Open the Serial Monitor at 115200 baud. It prints your stick values and axis commands.

**Pairing:** the DualShock 4 must be paired to the ESP32's Bluetooth MAC address (for example with SixaxisPairTool, or by writing the ESP32 MAC into the controller). Once paired, hold PS + Share to put the controller in pairing mode, or just press PS if it was already paired.

## UART protocol

12-byte frame at about 50 Hz, 115200 baud, 8N1:

```
[0xAA][0x55][dir0][int0_H][int0_L][dir1][int1_H][int1_L][dir2][int2_H][int2_L][checksum]
```

- `dir`: 1 = forward, 0 = reverse
- `int`: 16-bit step interval in microseconds (0 = stop)
- `checksum`: XOR of the 9 payload bytes (`dir`/`int` bytes)

The STM32 parses with a polled state machine and ignores any frame with a bad header or checksum.

## Tuning

**ESP32 (`esp32_robot.ino`)**

| Parameter | Default | Effect |
|---|---|---|
| `MIN_INTERVAL_US` | 600 | Interval at full stick. Lower = faster top speed. Raise it if a motor stalls or hums. |
| `MAX_INTERVAL_US` | 2000 | Interval just past the deadzone. Higher = slower creep speed. |
| `DEADZONE` | 15 | Stick range that is ignored. Raise it if a motor creeps at center. |

To reverse an axis, negate its stick value, for example `mapStick(-PS4.RStickX(), dir[2], iv[2]);`, or swap one motor coil pair.

**STM32 (`main.c`)**

| Parameter | Default | Effect |
|---|---|---|
| `PULSE_WIDTH_US` | 10 | Step pulse width (TB6600 needs at least about 2.5 us) |
| `MIN_SAFE_US` | 100 | Hard lower limit on the step interval |
| `FAILSAFE_WRAPS` | 3 | Failsafe timeout, about 65 ms per unit (3 = about 200 ms) |

**Hardware:** set the TB6600 microstep and current DIP switches to match your motors.

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| PC13 does not blink 3 times at start | Firmware not flashed, or wrong clock/board setup |
| ESP32 prints values but nothing moves | Missing common GND, TX/RX wiring, or wrong pin on PB11 |
| Motor hums but does not turn | Interval too small: raise `MIN_INTERVAL_US`; check TB6600 current setting |
| Motor creeps at stick center | Raise `DEADZONE` |
| Axis turns the wrong way | Negate the stick value or swap one coil pair |
| Link error `undefined reference to htim1` | Add `TIM_HandleTypeDef htim1;` to `main.c` or disable TIM1 in the .ioc |

## Possible improvements

- Acceleration/deceleration ramps to avoid skipped steps on heavy joints
- Inverse kinematics for end-effector control
- Limit switches and homing
- Two-way UART feedback (position, faults) to the ESP32

## License

Choose a license (for example MIT) and add a `LICENSE` file.