# AGENTS.md — AthenaDrive firmware

> Preprompt and onboarding for AI coding agents working in this repository.
> Last verified against the repository and the `AthenaDrive/PCB` v3 BOM: **2026-11-10**.
> If this file and the code disagree, trust the code, then update this file.

---

## 0. Read this first

This is real motor-control firmware. A mistake can spin a motor, dump current into a
power stage, or brick a board. The agent cannot validate any of that from the repo.

**Hard rules**

- **Never flash, monitor, erase, or reset the board without explicit user approval.**
  Do not run `idf.py flash`, `idf.py monitor`, `idf.py erase-flash`, or anything else
  that touches the serial port or energizes hardware unless the user asked for it.
- **Do not change these without explicit approval and an explanation of the risk:**
  - FOC sign conventions (`_betaSign`, current sign, encoder direction)
  - phase-order detection / `PhaseOrderMode`
  - alignment voltage, timing, or startup thresholds
  - current, torque, velocity, or position limits
  - PWM frequency, duty mapping, or dead-time assumptions
  - pin assignments (`main/Pinout.hpp`) or the DRV8320S register configuration
  - the daisy-chain IP scheme or the UDP/TCP wire format
- **Build verification is the agent's job. Hardware verification is the user's.**
  Never claim a motor, sensor, or network change works just because it compiles.
- **Do not commit unless the user asks.** Do not rewrite or revert uncommitted work.
- Ask before large refactors, cross-component renames, or changes that alter the
  public behavior of a task/component. Small, local diffs are strongly preferred.

---

## 1. What this project is

**AthenaDrive** is ESP-IDF C++ firmware for the **AthenaDrive v3** ESP32-S3
BLDC/PMSM motor driver board. The PCB design lives in the separate
[`AthenaDrive/PCB`](https://github.com/AthenaDrive/PCB) repository; its `BOM.xlsx`
is the authoritative parts list.

Main capabilities:

- Field-oriented control (FOC) of a 3-phase motor via a TI **DRV8320S** gate driver
  (SPI variant, no internal current-sense amps; phase current is measured externally)
- **AS5047P** magnetic encoder for rotor position
- Phase-current sensing with **3x TI INA241A3** current-sense amplifiers and
  **1 mΩ** phase shunts, read through **ADC1 continuous/DMA**
- **INA238** bus voltage/current/power monitor, **LM75A** temperature sensor,
  **LSM6DSO** IMU
- Two **W5500** SPI Ethernet controllers forming a **daisy chain**
- UDP telemetry and a request/response TCP command protocol
- A generated global-variable layer shared by all tasks

The project is called `AthenaDrive` in `CMakeLists.txt`. The build output is
`build/AthenaDrive.bin` / `.elf`.

The code is actively evolving and contains deliberate workarounds and experiments
(the user writes candid first-person comments). Do not "clean up" behavior that
looks odd without asking; it may compensate for real hardware.

---

## 2. Repository map

Everything important is in `main/` and `components/`. `build/` and
`managed_components/` are generated/ignored.

| Path | Purpose |
| --- | --- |
| `main/main.cpp` | `app_main`, sensor polling loop, task startup |
| `main/Pinout.hpp` | Canonical GPIO map |
| `main/Config.hpp` | I2C addresses, INA238 calibration, IP/port/startup constants |
| `main/CMakeLists.txt` | `main` component requirements |
| `TODO.md` | Actionable next-iteration checklist (PCB erratum, firmware TODOs) |
| `components/GlobalVariableManager/` | Generated shared state (`Variables.txt` is the source of truth) |
| `components/FOCTask/` | FOC task, startup/alignment state machine, current sensing |
| `components/Controller/` | Clarke/Park transforms and d/q PI current loop |
| `components/StateEstimation/` | PLL-based angle unwrap, velocity, acceleration |
| `components/CascadingPID/` | Position/velocity cascade used by modes 3/4 |
| `components/DRV8320S/` | Gate-driver SPI registers and PWM mode |
| `components/AS5047P/` | Encoder SPI protocol and pipelined reads |
| `components/MCPWMDriver/` | 3-phase center-aligned PWM |
| `components/ADCContinuous/` | Phase-current ADC, ADC1 continuous/DMA (active path) |
| `components/ADCOneshot/` | Older blocking phase-current driver, no longer used by FOC |
| `components/EthernetTask/` | W5500 Ethernet, daisy-chain routing, UDP/TCP protocol |
| `components/EthernetTask/Discovery.*` | Auto-IP position discovery |
| `components/W5500/` | Local dual-W5500 wrapper around `espressif/w5500` |
| `components/I2CBase/`, `SPIBase/` | Shared bus wrappers with ref counting |
| `components/INA238/`, `LM75A/`, `LSM6DSO/` | Sensor drivers |
| `managed_components/` | Fetched by the component manager; never edit |
| `build/` | Generated; never edit or commit |

`MCP23017`, `SdCard`, and `RealTimeTask` were removed in November 2026. Do not
resurrect references to them; the remaining component list is the current firmware.

### Source of truth

- GPIOs: `main/Pinout.hpp` — but see the duplication warning in §8.
- Board parts / schematic: `AthenaDrive/PCB` repo (`BOM.xlsx`, board v3).
- Network/startup constants: `main/Config.hpp`.
- Shared state: `components/GlobalVariableManager/Variables.txt`.
- Wire protocol: `components/EthernetTask/EthernetTask.cpp` (not the structs).
- Control math: `components/Controller/` and `components/FOCTask/`.

---

## 3. Build and verify

Environment used by this project:

- **ESP-IDF 6.0.1** at `C:\esp\v6.0.1\esp-idf` (see `.vscode/settings.json`)
- Target: **esp32s3**
- Serial port: **COM13** (may change; confirm with the user)
- Build system: CMake + Ninja, standard ESP-IDF component layout

Typical commands (from the repo root, inside the IDF environment):

```powershell
idf.py build
idf.py reconfigure      # after adding/removing components or manifests
idf.py -p COM13 flash monitor   # ONLY with explicit user approval
```

Notes:

- `sdkconfig` is machine-local and gitignored. Persistent Kconfig changes belong in
  `sdkconfig.defaults`; ask before changing either.
- The target is pinned to **esp32s3** in the top-level `CMakeLists.txt`, so a fresh
  clone builds for the right chip without running `idf.py set-target`.
- `managed_components/` is gitignored but present locally. Do not edit fetched code.
- After adding a component or a new dependency, update the consuming component's
  `CMakeLists.txt` (`REQUIRES`/`PRIV_REQUIRES`) and run `idf.py reconfigure`.
- `.clangd` strips `-f*`/`-m*` flags so clangd can consume
  `build/compile_commands.json`. Do not "fix" that file casually.
- There is **no test suite and no CI**. The minimum bar is a clean `idf.py build`.
  For behavior, give the user explicit hardware test steps (what to look for in the
  serial log, telemetry, or motor response).

---

## 4. Runtime architecture

### Contexts and rates

| Context | Entry | Rate / priority | Notes |
| --- | --- | --- | --- |
| `app_main` | `main/main.cpp` | ~1 ms polling loop | Core 0; reads I2C sensors, publishes to GVM |
| FOC | `FOCTask::taskLoop` via esp_timer notification | **100 µs (10 kHz)**, priority 22, pinned core 1 | Treat as hard real-time: no blocking, no allocation, no heavy logging |
| Ethernet | `EthernetTask::begin` | 4 tasks, priority 12, pinned core 0 | 2 UDP + 2 TCP, 10 ms loops |
| Discovery responder | `Discovery.cpp` | FreeRTOS task, priority 8, pinned core 0 | ETH_1, only when auto-IP is enabled |

Core split: **core 1 runs only the FOC task**; core 0 owns the esp_timer task,
Ethernet socket tasks, lwIP, W5500 RX tasks, discovery, and `app_main`. The esp_timer
callback itself only calls `xTaskNotify()`; it does not run the control loop. Keep
new work off core 1 and avoid unpinned tasks (they can migrate there).

The ESP32-S3 has a **shared flash instruction cache**. This project runs with
`CONFIG_ESP32S3_INSTRUCTION_CACHE_32KB=y`; at 16 KB the FOC and Ethernet code
thrashed each other and the loop was ~2.6× slower. Do not reduce it without
re-measuring. At the 100 µs period the loop is ~32 µs average and ~47 µs max
with `FOC_PROFILE 1`; the profiler adds ~6-7 µs of `esp_timer_get_time()` calls,
so a normal build is roughly 25-26 µs average. Phase currents come from the ADC
continuous/DMA ring buffer, which the FOC loop drains once per iteration.

### Boot sequence

1. `app_main` creates the I2C bus and initializes LM75A, INA238, LSM6DSO.
2. INA238 ADC config and shunt calibration are applied (`2 mΩ`, `20 A` full scale
   in `Config.hpp`); this is the **bus** monitor, separate from phase shunts.
3. GPIO ISR service is installed; **SPI3_HOST** is initialized manually because the
   `esp_eth` W5500 MAC driver expects a ready SPI bus.
4. `FOCTask::begin()` configures DRV control pins, starts 30 kHz center-aligned PWM
   with outputs low, starts SPI2, initializes the encoder and ADC, creates the
   dedicated `foc` task (core 1, priority 22, 8 KB stack), then starts the 100 µs
   esp_timer. The timer callback only notifies that task.
5. `EthernetTask` constructor initializes both W5500s and assigns temporary IPs;
   `begin()` runs discovery, assigns final IPs, starts the responder, and spawns the
   four socket tasks.
6. `app_main` enters the sensor loop. It never exits.

### FOC startup state machine

`FOCTask::update()` returns early until the bus is valid, then walks through:

1. `NO_VOLTAGE` — wait for `busVoltage >= 9.0 V`.
2. `VOLTAGE_SETTLE` — require 500 ms above the threshold.
3. `INIT_DRV` — DRV8320S init, 3x PWM mode, register dump, 50% neutral, INL high.
4. `START_ONE_PHASE` — apply a 2% voltage vector at 0 electrical degrees.
5. `SETTLE` / `MEASURE` — 5000 settle loops + 1000 measure loops; circular mean
   gives `_elPosOffset = wrap(-polePairs * lockedAngle)`.
6. Phase-order check:
   - `AUTO`: ramp the field to +90 electrical degrees, settle, measure direction;
     `_reversePhaseSequence = delta < 0`.
   - `NORMAL` / `REVERSED`: force `_betaSign` and skip the test.
7. `CURRENT` — all low-side FETs on (duty 0), 5000 settle + 5000 samples to measure
   the zero-current ADC offsets.
8. `DONE` — restore 50% neutral and run closed-loop FOC. `drivingMode` selects
   the torque reference: 3 = position cascade, 4 = velocity cascade, 5 = direct
   `torqueSetpoint`, and any other value falls back to the direct setpoint.

At the current 100 µs period the full startup takes roughly **2.4 seconds** after the
bus becomes valid (18,500 loop-counted steps = 1.85 s, plus the 500 ms voltage
settle). The alignment/direction/current windows are loop-counted, so changing the
period again changes them; convert them to time if the loop rate changes
substantially. The active PI gains have `Ki = 0`; if integral gain is enabled, its
per-iteration scale also depends on the period.

While the bus is below 9 V, the FOC timer still runs so the encoder keeps updating
the GVM, but the gate driver is never initialized or enabled.

### Task data flow

All cross-task state goes through `globalVariableManager`. Do not add ad-hoc globals
or share driver objects across tasks.

```
app_main sensors ─┐
FOCTask ──────────┼─> GlobalVariableManager <─> EthernetTask (UDP/TCP)
Discovery ────────┘
```

- Scalar floats are stored as `std::atomic_uint32_t` bit patterns (`memcpy`), so
  reads/writes are atomic but not synchronized as a group.
- The four byte buffers (`udp/tcpFromPeripheral/ControllerBuffer`) use mutexes and
  have `get(..., capacity, empty)` / `set(..., size)` semantics.
- Do not assume multiple GVM fields form a coherent snapshot; they do not.

---

## 5. GlobalVariableManager is generated

**Never hand-edit** `components/GlobalVariableManager/include/GlobalVariableManager.hpp`
or `GlobalVariableManager.cpp`. Both start with a "DO NOT MODIFY" banner.

Edit `components/GlobalVariableManager/Variables.txt` instead. Format:

```
# Variables
type, name, default
Buffer, someBuffer, 1024
# Consts
float, PI, 3.141592653589793f
```

Then regenerate **from the repository root** (the script uses relative paths):

```powershell
python components/GlobalVariableManager/GenerateGVM.py
```

What the generator produces:

- `get<Name>()` / `set<Name>(...)` accessors for every variable.
- `Buffer` entries become `get<Name>(uint8_t*, uint32_t capacity, bool empty)` /
  `set<Name>(const uint8_t*, uint32_t size)` with an internal mutex.
- `# Consts` entries become `static constexpr`.
- Floats are stored as atomic 32-bit patterns.
- Scalar getters/setters and the float load/store helpers are emitted with
  `IRAM_ATTR` so the FOC loop does not take a flash miss. `Buffer` accessors stay
  in flash because they take a mutex and `memcpy`. Floats use `std::bit_cast`
  instead of `memcpy`.

Gotchas:

- `numPolePairs` currently defaults to **7** in both `Variables.txt` and the
  generated header. Changing it changes commutation; confirm with the user first.
- The script only constructs float defaults; integer/bool defaults come from the
  generated in-class initializers.
- If a new variable must be exposed over Ethernet, also add it to the protocol
  switches in `EthernetTask.cpp` and (for documentation/static checks) to
  `CommunicationConfig.hpp`.

---

## 6. Networking and protocol

### Daisy chain physical layout

Each board has two W5500s:

- **ETH_0 = upstream** (faces the PC or the previous board)
- **ETH_1 = downstream** (faces the next board)

**Cabling:** the W5500 has **no auto-MDIX**, and both RJ45 ports on board v3 are
wired as MDI. A board-to-board link therefore requires a **crossover cable** or a
switch between the boards. A straight-through cable only works toward the PC or a
switch because those peers auto-correct the MDI/MDI-X pairing. See also the port-2
RX termination erratum in §8.

IP scheme with auto-IP enabled (`Config.hpp`):

| Node | ETH_0 (upstream) | ETH_1 (downstream) |
| --- | --- | --- |
| PC | `192.168.0.1` | — |
| Board 0 | `192.168.0.2` | `192.168.0.3` |
| Board 1 | `192.168.0.4` | `192.168.0.5` |
| Board N | `192.168.0.(2N+2)` | `192.168.0.(2N+3)` |

Discovery runs on **UDP port 5003**:

- Magic `0x41544C41` ("ATLA"), version 1, packet types request/response.
- A board with no upstream link becomes position 0.
- A board that receives a response becomes `upstream_position + 1`.
- After discovery, a responder task on ETH_1 answers downstream boards.
- With `W5500_USE_AUTO_IP 0`, position is forced to 0 and the static IPs are used.

### UDP telemetry and commands (port 5000)

Wire format is hand-packed in `EthernetTask.cpp`:
`uint32_t header` (bitmask) followed by **4 bytes per set bit**, in bit order.
Bits are little-endian native ESP32 values. If a bit is set but has no `case`, the
code still advances 4 bytes and sends whatever was in the buffer — **never set
undefined bits**.

Board → upstream telemetry:

| Bit | Field |
| --- | --- |
| 0 | timestamp (ms, uint32) |
| 1 | position (rad, float) |
| 2 | velocity (rad/s, float) |
| 3 | acceleration (rad/s², float) |
| 4 | torque (`targetTorque`, float) |
| 5 | phase current A |
| 6 | phase current B |
| 7 | phase current C |
| 8 | bus current |
| 9 | bus voltage |
| 10 | error register (currently hard-coded 0) |
| 11 | FOC loop time |
| 12 | secondary loop time |
| 13–15 | accel X/Y/Z |
| 16–18 | gyro X/Y/Z |
| 24–27 | `udpDataDebugFloat0..3` |
| 28–29 | `udpDataDebugInt0..1` |
| 30–31 | `udpDataDebugUint0..1` |

Upstream → board command:

| Bit | Field |
| --- | --- |
| 0 | `torqueSetpoint` |
| 1 | `velocitySetpoint` |
| 2 | `positionSetpoint` |
| 3 | `drivingMode` |
| 24–27 | `udpCommandDebugFloat0..3` |
| 28–29 | `udpCommandDebugInt0..1` |
| 30–31 | `udpCommandDebugUint0..1` |

### TCP framing (port 5001)

The PC connects to the **ETH_0 TCP server**; each board also runs a TCP client on
ETH_1 that connects to the next board's server, so the chain relays frames.

Frame:

```
uint32_t prefix   // high 16 bits = 0xF879 (63609); low 16 bits = payload length
uint8_t  payload[payload_length]
uint32_t header   // bitmask inside the payload
...               // 4 bytes per set bit
```

- The length prefix counts bytes **after** the prefix itself.
- `header` bit 0 selects direction in the server:
  - **bit 0 set = command** (PC → board)
  - **bit 0 clear = data request** (board → PC)
- Command bits (TCP server): 1 torque setpoint, 2 torque Kp, 3 torque Ki,
  5 torque limit, 6 velocity setpoint, 7–9 velocity Kp/Ki/Kd, 10 velocity limit,
  11 position setpoint, 12–14 position Kp/Ki/Kd, 15 driving mode,
  16 bus current limit, 17 phase current limit, 18 pole pairs,
  19 UDP telemetry header, 20 clear error flags (still consumes 4 bytes),
  21 LED status, 24–31 TCP command debug values.
- Data-request bits: 1 bus voltage, 2 bus current, 4 button status,
  5 bus current limit, 6 phase current limit, 7 board state, 8 driving mode,
  9 pole pairs, 10 phase RMS voltage, 11 error flags, 12 temperature,
  24–31 TCP data debug values.
- Bits with no `case` still consume 4 bytes. Keep both ends of the protocol in sync.
- `CommunicationConfig.hpp` defines structs and `offsetof` arrays for documentation
  and static checks. **It is not the serializer.** The actual wire format is the
  `switch` code in `EthernetTask.cpp`; update both when the protocol changes.

### Socket tasks

- `udp_as_peripheral_task` (ETH_0) sends telemetry upstream and receives commands.
- `udp_as_controller_task` (ETH_1) relays downstream.
- `tcp_as_peripheral_task` (ETH_0) is the server the PC connects to.
- `tcp_as_controller_task` (ETH_1) is the client that relays to the next board.
- All four are 8 KB stacks, priority 12, pinned to core 0, with 10 ms polling.
- Sockets are bound to a netif with `SO_BINDTODEVICE`.

---

## 7. Control units and conventions

- Mechanical angle: **radians**, cumulative (unwrapped).
- Velocity: **rad/s**. Acceleration: **rad/s²**.
- Electrical angle: `wrap(mechanicalAngle * polePairs + _elPosOffset)`.
- Pole pairs come from the GVM (`numPolePairs`), not a compile-time constant.
- PWM duty is **percent**: 50.0 is neutral, 0.0 is low-side on, 100.0 is high-side on.
- FOC output is constrained to ±30 percentage points, then biased by +50, so the
  physical duty range is 20–80%.
- Phase-current conversion in the FOC loop is `(millivolts - offset) / 50.0f`.
  That is correct for board v3: 1 mΩ shunt × INA241A3 gain of 50 V/V = 50 mV/A.
- `main.cpp` sets `cUsePhaseBForFoc = true`, so the controller uses the real phase
  A and B currents. The field still supports `false`, which uses phase A and
  reconstructs `Ib = -(Ia + Ic)`; phase C is always measured and published via the
  GVM. Do not assume a phase-current sensor is unused or broken.
- Bus current is **negated** in `main.cpp` because the shunt wiring is reversed.
  Do not remove that sign without checking the hardware.
- `drivingMode` values in `CascadingPID`: 3 = position, 4 = velocity, 5 = torque.
  The cascade runs every FOC iteration. Mode 3 chains position -> velocity ->
  torque, mode 4 runs velocity -> torque, and mode 5 uses `torqueSetpoint`
  directly. Modes outside 3-5 also fall back to the direct torque setpoint.
  Position/velocity gains come from `positionKp/Ki/Kd` and
  `velocityKp/Ki/Kd` in the GVM and are only written into the PID objects when
  they change, so TCP tuning preserves the integrator state. GVM defaults are
  all zero, so modes 3/4 output zero torque until gains are set. Position mode
  publishes its generated velocity setpoint but leaves the user's
  `torqueSetpoint` untouched.

---

## 8. Hardware map and driver gotchas

### Buses and pins

| Bus | Host | Devices / pins |
| --- | --- | --- |
| I2C | new I2C master | SDA GPIO11, SCL GPIO12, 400 kHz |
| SPI2 | `SPI2_HOST` | DRV8320S (CS GPIO48), AS5047P (CS GPIO13) |
| SPI3 | `SPI3_HOST` | W5500 #0 (CS17, INT16), W5500 #1 (CS15, INT7) |
| ADC1 | continuous (DMA) | phase A GPIO6, B GPIO5, C GPIO4 |
| MCPWM | group 0 | INHA 39, INHB 40, INHC 41, 30 kHz center-aligned |
| GPIO | — | DRV ENABLE 38, nFAULT 3, INL 42, brake chopper 1 |

I2C addresses (`Config.hpp`): INA238 `0x40`, LM75A `0x48`, LSM6DSO `0x6A`.
`CS_EXTERNAL` (GPIO10) remains in `Pinout.hpp` but is unused by the current firmware.

**Pin duplication warning:** `MCPWMDriver.cpp` hard-codes GPIO 39/40/41 and
`ADCContinuous.cpp` hard-codes the phase-current channel map (GPIO 4/5/6),
duplicating `Pinout.hpp`. If pinout changes, update all three places (and any
`Config.hpp` assumptions).

### Board v3 key parts (from `AthenaDrive/PCB` BOM)

| Function | Part |
| --- | --- |
| MCU | ESP32-S3-WROOM-1U-N16R8 |
| Gate driver | TI **DRV8320SRTVR** (`U18`) |
| Power stage | 3x TI **CSD88599Q5DC** dual N-channel MOSFET power blocks (`MOSFET_A1/B1/C1`) |
| Phase current | 3x TI **INA241A3IDDFR** (`U21`–`U23`) + **1 mΩ** shunts (`R135`–`R137`) |
| Bus current | TI **INA238AIDGSR** + **2 mΩ** shunt (`SHUNTPWR`) |
| Rotor encoder | ams **AS5047P-ATSM** (`U13`) |
| Ethernet | 2x **W5500** (`U4`, `U15`) with 25 MHz crystals (`X4`, `X5`) |
| IMU / temperature | **LSM6DSOWTR** (`U5`), **LM75AD,118** (`U11`) |
| Power input / motor | XT30PW-M, 3-pin motor terminal, brake-resistor header |

### Board v3 errata (check before bringing up a new PCB)

- **Downstream RJ45 / W5500 #1 RX termination is wrong.** The two W5500s are
  `U15` (port 1 / ETH_0) and `U4` (port 2 / ETH_1). Port 1's RX network is the
  reference: `RXP`/`RXN` each have a 33 Ω series resistor, then a 49.9 Ω resistor
  to the transformer center tap, with 10 nF from the center tap to GND and 6.8 nF
  from each RX node to the RJ45. On port 2, `R117` (49.9 Ω) is wired **across RXP
  and RXN** instead of RXP→center tap, the RJ45 center tap (`RJ5` pin `2(5)`) is
  tied directly to RXN, and the RXP→center-tap 49.9 Ω resistor is missing. The
  BOM reflects the error: 7× 49.9 Ω instead of 8×.
  - Symptom: ETH_1 never links and its LEDs stay dark even with a known-good PC
    cable or crossover cable. `PHYCFGR` reports `0xfa` (`rst=1`, `opmd=7`,
    `link=0`) — the PHY is configured correctly but has no link.
  - Next PCB revision: copy port 1's RX network to port 2, isolate the center tap
    from RXN, add the missing 49.9 Ω, and update the BOM to 8× 49.9 Ω.
  - Prototype bodge: cut the RXN/center-tap net, repurpose `R117` as RXP→center
    tap, add a 49.9 Ω from RXN→center tap, and verify the center tap is no longer
    shorted to RXN.
- **No W5500 hardware-reset GPIO.** Both `RSTn` nets are pull-up only; the ESP32
  does not control them. In `W5500::init()` the PHY config must keep
  `phy_cfg.reset_gpio_num = -1`. ESP-IDF 6.0.1's `ETH_PHY_DEFAULT_CONFIG()`
  defaults to GPIO5, which is `ADC_CUR_B` on this board; leaving the default would
  make Ethernet init drive the phase-B current-sense input high after
  `ADCContinuous::init()` and corrupt phase-B readings. This was fixed; do not
  revert it.

### SPI details

- `SPIBase` ref-counts buses by host + pins; multiple drivers can share a bus.
- **SPI2 is initialized with `SPI_DMA_DISABLED`.** The encoder/DRV transfers are
  16-bit; with DMA enabled the IDF SPI driver allocates two bounce buffers, memcpys,
  and cache-syncs for every transaction, which cost more than the transfer. Do not
  re-enable DMA for SPI2 without measuring.
- `DRV8320S` is constructed in `FOCTask` with **SPI mode 1 at 500 kHz**, even though
  the constructor default is mode 0.
- AS5047P uses **SPI mode 1 at 10 MHz** (the constructor default is 5 MHz); it
  supports a pipelined angle read and applies even parity.
- W5500 runs at **30 MHz**; both instances use `SPI3_HOST`, which `main.cpp`
  initializes manually with a 4096-byte max transfer.
- W5500 #0 and #1 get different MAC addresses (base MAC and base+1).
- `W5500::log_phy_status()` reads and decodes PHYCFGR (`rst`, `opsel`, `opmd`,
  `link`, speed, duplex); `EthernetTask` also logs link changes from a monitor
  task. Use these before assuming an IP/socket bug when a port will not link.

### Gate driver notes (DRV8320S)

- Board v3 populates a **DRV8320SRTVR** (`U18`, WQFN-32). It is the SPI variant
  with **no** internal current-sense amplifiers; phase current is measured by the
  external INA241A3s. The firmware component and class are named `DRV8320S` to
  match. The DRV832x family shares the register map used here, so do not change
  the SPI protocol without checking the datasheet and asking the user. The
  component was renamed from `DRV8323` to `DRV8320S`; older commits/branches use
  the old name.
- Configured for **3x PWM mode** via `REG_DRIVER_CONTROL`.
- `PWM_MODE` is **bits 6:5**, not bits 10:8. This was recently fixed; do not revert.
- The driver requires all INHx/INLx low while changing `PWM_MODE`. `FOCTask`
  deliberately leaves the MCPWM compares at 0 until `initializeDriver()` finishes.
- `DRV8320S_ENABLE` is driven high early; `DRV8320S_INL` goes high only after the
  mode is set and the bridge is at 50% neutral.
- Fault/status reads exist but are currently commented out in the FOC loop, and the
  reported error register is hard-coded to 0.

### Phase current sensing (board v3)

Confirmed from `AthenaDrive/PCB` `BOM.xlsx`:

| Item | Part | Notes |
| --- | --- | --- |
| Phase current amps | **3x TI INA241A3IDDFR** (`U21`, `U22`, `U23`) | SOT-23-8, 50 V/V, bidirectional, 1.1 MHz, ±10 µV max offset |
| Phase shunts | **1 mΩ ±1%**, `R135`, `R136`, `R137` | R2512, `ECSR2512FR001K9`, one per phase |
| Bus shunt | **2 mΩ**, `SHUNTPWR` | R2512, `LR2512-23R002F4`, feeds INA238 |

The phase-current transfer function is therefore:

```
Vout = Iphase × 1 mΩ × 50 V/V = Iphase × 50 mV/A
Iphase = (Vout_mV - zeroOffset_mV) / 50
```

This exactly matches `FOCTask`'s `/ 50.0f` conversion. All three phases are
populated and measured. `main.cpp` currently sets `cUsePhaseBForFoc = true`, so the
controller uses the real A and B currents; C is still published. Setting it to
`false` switches the controller to A plus a reconstructed B. The INA241 outputs are
biased around mid-supply (the firmware assumes 1.65 V); it measures the
zero-current offset during the startup `CURRENT` phase and subtracts it.

`ADCContinuous::init()` in `FOCTask` is configured as `0.001f, 50.0f, 1.65f` to
match the board-v3 shunt, amplifier gain, and mid-supply reference. It runs ADC1
in continuous/DMA mode at 60 kHz total (20 kHz per phase), with 48-byte DMA
frames and a 1 KB pool flushed from the oldest end if the FOC loop ever falls
behind. `read_voltage_all()` drains the pool once per FOC iteration and keeps the
newest raw value of each channel.

Do not call `adc_cali_raw_to_voltage()` from the FOC loop. The IDF curve-fitting
path is a 5-term 64-bit polynomial with runtime 64-bit divisions; three calls
were ~27 µs average with >100 µs peaks. `ADCContinuous::init()` builds a 65-entry
raw→mV table once, and the hot path interpolates it (a few mV worst-case error,
well under 0.1 A at 50 mV/A).

The ADC continuous DMA ISR is allocated when `ADCContinuous::init()` creates the
driver from `app_main`, so it runs on **core 0**. The FOC task drains the ring
buffer on **core 1**. Keep it that way: moving the ISR to core 1 would make it
preempt the control loop. `refresh()`, `parse_latest()`, `read_voltage_all()` and
the LUT interpolation are placed in IRAM so core 0's Ethernet traffic cannot
evict them from the shared flash I-cache. The IDF `adc_continuous_read()` and
`xRingbufferReceiveUpTo()` are still flash-resident; `CONFIG_RINGBUF_IN_IRAM=y`
would move the latter into IRAM if the remaining max spikes matter.

---

## 9. Known TODOs and weak spots

Treat these as current reality, not as instructions to fix everything at once.
The actionable checklist for the next firmware/PCB iteration lives in `TODO.md`.

- **Downstream Ethernet hardware erratum:** board v3 port 2 has the W5500 #1 RX
  termination error described in §8. ETH_1 cannot link until that is reworked or
  fixed in the next PCB revision. Do not chase this as a software bug.
- **Discovery race:** if two boards boot simultaneously, the downstream board can
  time out before the upstream responder starts (responder is up at ~7.5 s; the
  current discovery window ends at ~7 s) and both boards default to position 0.
- **Task-config lifetime:** the `_TaskConfigUDP` / `_TaskConfigTCP` structs are
  stack locals in `EthernetTask::begin()` passed by pointer to tasks. The 1 s
  delay hides it, but make the storage static or heap-owned.
- **TCP server VLA:** `uint8_t message[lengthPrefix & 0xFFFF]` in
  `tcp_as_peripheral_task` can overflow the 8 KB task stack.
- **FOC loop timing:** runs at a 100 µs period (10 kHz). With the 32 KB I-cache
  and the IRAM pass, the profiled loop is ~32 µs average / ~47 µs max. The
  profiler itself costs ~6-7 µs (7 intermediate `esp_timer_get_time()` calls at
  ~0.85 µs), so `FOC_PROFILE 0` gives roughly 25-26 µs average. The 100 µs period
  leaves ~50 µs of headroom at the observed max; watch for overruns when
  Ethernet is active. Startup delays are loop-counted and now total ~2.4 s; the
  active PI has `Ki = 0`, but re-check the integrator scale if you enable it.
- **State estimation:** `StateEstimation` is a type-2 tracking PLL on the wrapped
  encoder angle (`cBandwidthHz` default 100 Hz, damping 1.0), not a filtered
  derivative. Velocity is the PLL state, so it has no steady-state lag at
  constant speed; acceleration is a 50 Hz filtered derivative of that state and
  is intended for telemetry/feedforward. The published cumulative angle is raw
  (no low-pass), because it feeds the position loop and a filter would add
  `v * tau` error. The estimator receives a measured `dt` from the FOC loop and
  rejects samples that imply more than `cMaxSpeedRadPerSec` (currently
  2000 rad/s), carrying the elapsed time forward. Tune the PLL in Hz, not with
  magic alphas. Encoder frame parity is still not validated.
- **Motor constants:** `Controller.cpp` currently hard-codes
  `motor_r = 39 mΩ`, `motor_l = 16 µH`, `motor_flux = 2.92 mWb`; verify against
  the motor before relying on them.
- **CascadingPID is active:** `FOCTask` calls `compute()` every FOC iteration
  with `drivingMode` and a `CascadePidValues` struct. It still has no GVM access;
  the caller supplies setpoints/feedback and publishes the generated velocity
  setpoint. The PID output is limited by `PID_Reg`'s `_maxOut` (currently 15.0),
  and the cascade PIDs reset when `drivingMode` changes. Gains are synced from
  the GVM every iteration but written into `PID_Reg` only when a value changes.
- **Limits/gains over TCP are not all applied:** position/velocity gains are now
  applied, but `torqueKp/Ki/Kd` and `currentLimit*` are still only stored.
  Setpoints and `drivingMode` are applied.
- **Error reporting is incomplete:** gate-driver fault/VGS reads are commented out
  and `errorRegister` is always 0. The TCP command only clears `errorFlags`.
- **Socket error paths:** some `sock < 0` branches call `vTaskDelete(nullptr)` and
  then continue; do not copy that pattern into new code.
- **Duplication:** `bind_socket_to_netif()` exists twice
  (`EthernetTask.cpp` and `Discovery.cpp`); `constrain()` in `FOCTask.cpp` is a
  file-scope helper. Keep edits local unless the user wants a refactor.
- **No tests/CI.** Changes are validated by building and by the user's hardware run.

---

## 10. Coding conventions

Match the surrounding code rather than imposing a new style.

- C++ with ESP-IDF; headers use `#pragma once`.
- Drivers are classes with `init()` / `deinit()` returning `esp_err_t`, an
  `is_initialized()` accessor, and RAII cleanup in the destructor.
- Use `ESP_ERROR_CHECK` for fatal setup failures in `main`/task startup;
  use `ESP_RETURN_ON_ERROR` / explicit error handling inside reusable drivers.
- Logging is mixed: `ESP_LOGI/E/W` in driver components, `printf` in task and
  application code. Follow the file you are editing.
- Config structs are passed by reference to task constructors and stored by value;
  fields use a `c` prefix (`cDRV8320S_CS`). Tasks expose `begin()`.
- Naming: device-driver methods use `snake_case`; control classes use `camelCase`
  private methods; generated GVM accessors use `getPascalCase` / `setPascalCase`.
- Shared state goes through the GVM. Add the variable to `Variables.txt` and
  regenerate instead of introducing a new global.
- Units in names/comments are essential: `_mv`, `_ohm`, `_v`, `_a`, `rad`, `ms`.
- Comments in this repo explain hardware "why", often with measurements and
  workarounds. Preserve that style; do not strip comments as cleanup.
- Keep the hot path allocation-free. No `new`, `malloc`, `std::string`, blocking
  waits, or per-iteration logging in `FOCTask::update()`.
- **Hot-path math gotchas:** avoid `std::fmod` (`fmodf` pulls in fdlibm and cost
  ~35 µs), avoid double constants such as `M_PI` (soft-double calls on the S3),
  and avoid full-range `sinf`/`cosf` (they run `__kernel_rem_pio2f`). Use the
  bounded `wrapAngle` helpers in `FOCTask.cpp` and `fastSinCos` in `Controller.cpp`.
- **Measure before/after hot-path changes.** `FOCTask.cpp` has a compile-time
  `FOC_PROFILE` macro that prints per-stage avg/max timings plus core ID and
  effective MHz over UART. It must be `0` for normal builds. At the 100 µs period
  the profiled loop is ~32 µs average / ~47 µs max: encoder ~15 µs, state
  ~3-4 µs, ADC ~3-5 µs, ctrl/pwm ~2 µs. The profiler itself adds ~6-7 µs, and
  maxima are typically 2-3x the averages.
- **The core-1 hot path is in IRAM.** `FOCTask::update`, `Controller::update` and
  its helpers, `PI_Reg::update`, `CascadingPID::compute`, `PID_Reg::update`/`reset`,
  `LowpassFilter::update`, `StateEstimation::estimate`, `MCPWMDriver::set_duty`,
  `AS5047P::pipeline_read_angle`/`_transfer`, and `SPIBase::polling_transmit` use
  `IRAM_ATTR`. On the IDF side, `CONFIG_MCPWM_CTRL_FUNC_IN_IRAM=y`,
  `CONFIG_FREERTOS_IN_IRAM=y` (required by `CONFIG_SPI_MASTER_IN_IRAM=y`),
  `CONFIG_SPI_MASTER_IN_IRAM=y`, `CONFIG_RINGBUF_IN_IRAM=y`, and
  `CONFIG_ADC_CONTINUOUS_ISR_IRAM_SAFE=y` keep the SPI/ringbuf/MCPWM paths out
  of flash. The generated GVM scalar accessors are in IRAM too; GVM `Buffer`
  accessors and `adc_continuous_read()` remain flash-resident. Keep new
  per-iteration functions in IRAM.
- A new component needs its own `CMakeLists.txt` and an entry in the consumer's
  `REQUIRES` list.

---

## 11. Agent workflow

1. **Orient:** read this file, then `git status`, `git diff`, and `git log -5`.
   The worktree may contain deliberate uncommitted experiments; never revert them.
2. **Locate the real source of truth** for the change (see §2) before editing.
3. **Plan:** for a cross-cutting change or anything touching motor behavior, state
   the plan and ask before implementing.
4. **Edit narrowly:** keep the diff small, follow local conventions, and avoid
   unrelated formatting. Do not edit `build/`, `managed_components/`, generated GVM
   files, or machine-local `sdkconfig`.
5. **Build:** run `idf.py build` and report the actual result. If the build fails,
   fix it before claiming completion.
6. **Hand off hardware testing:** give the user concrete steps and expected signs
   (serial messages, telemetry bits, motor behavior). Do not imply you tested it.
7. **Document:** if architecture, pins, protocol, or build steps change, update
   this file and any relevant comment in the same change.
8. **Commit only when asked.** Use terse, imperative messages consistent with the
   existing history (e.g. "Fix current reading", "Add phase order checking").
   Do not add AI attribution unless the user asks for it.

### Useful first commands

```powershell
git status --short
git log --oneline -10
idf.py build
```

For targeted reading, prefer `rg`/grep over opening `build/` or
`managed_components/`. The project source is small: `main/` plus `components/`.
