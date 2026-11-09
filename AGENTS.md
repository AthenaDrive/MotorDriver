# AGENTS.md — AthenaDrive firmware

> Preprompt and onboarding for AI coding agents working in this repository.
> Last verified against the working tree and the `AthenaDrive/PCB` v3 BOM: **2026-11-09**.
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
  **1 mΩ** phase shunts, read through **ADC1 oneshot**
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
| `components/GlobalVariableManager/` | Generated shared state (`Variables.txt` is the source of truth) |
| `components/FOCTask/` | FOC task, startup/alignment state machine, current sensing |
| `components/Controller/` | Clarke/Park transforms and d/q PI current loop |
| `components/StateEstimation/` | Angle unwrap, velocity, acceleration |
| `components/CascadingPID/` | Position/velocity cascade (currently **not called**) |
| `components/DRV8320S/` | Gate-driver SPI registers and PWM mode |
| `components/AS5047P/` | Encoder SPI protocol and pipelined reads |
| `components/MCPWMDriver/` | 3-phase center-aligned PWM |
| `components/ADCOneshot/` | Phase-current ADC |
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
| `app_main` | `main/main.cpp` | ~1 ms polling loop | Reads I2C sensors, publishes to GVM |
| FOC | `FOCTask::update` via `esp_timer` | **1000 µs (1 kHz)**, `ESP_TIMER_TASK` | Treat as hard real-time: no blocking, no allocation, no heavy logging |
| Ethernet | `EthernetTask::begin` | 4 tasks, priority 12, pinned core 0 | 2 UDP + 2 TCP, 10 ms loops |
| Discovery responder | `Discovery.cpp` | FreeRTOS task, priority 8 | ETH_1, only when auto-IP is enabled |

The FOC callback is not an ISR, but it runs in the esp_timer task and must finish
well inside its period. The code comments say the loop currently overruns at times
(peaks of 800+ µs), so do not add work there without measuring.

### Boot sequence

1. `app_main` creates the I2C bus and initializes LM75A, INA238, LSM6DSO.
2. INA238 ADC config and shunt calibration are applied (`2 mΩ`, `20 A` full scale
   in `Config.hpp`); this is the **bus** monitor, separate from phase shunts.
3. GPIO ISR service is installed; **SPI3_HOST** is initialized manually because the
   `esp_eth` W5500 MAC driver expects a ready SPI bus.
4. `FOCTask::begin()` configures DRV control pins, starts 30 kHz center-aligned PWM
   with outputs low, starts SPI2, initializes the encoder and ADC, then starts the
   1 kHz esp_timer.
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
8. `DONE` — restore 50% neutral and run closed-loop FOC using
   `torqueSetpoint` directly as `iqRef`.

At 1 kHz the full startup takes roughly **19 seconds** after the bus becomes valid.
That is expected; do not shorten the settle/measure counts without approval.

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
- `cUsePhaseBForFoc = false` by default: the controller receives phase A and a
  reconstructed `Ib = -(Ia + Ic)`. The real phase B current is still measured and
  published via the GVM. Do not assume the B sensor is unused or broken.
- Bus current is **negated** in `main.cpp` because the shunt wiring is reversed.
  Do not remove that sign without checking the hardware.
- `drivingMode` values in `CascadingPID`: 3 = position, 4 = velocity, 5 = torque.
  This cascade is currently **not invoked**; the active FOC loop consumes
  `torqueSetpoint` directly regardless of mode. Do not claim position/velocity
  modes work.

---

## 8. Hardware map and driver gotchas

### Buses and pins

| Bus | Host | Devices / pins |
| --- | --- | --- |
| I2C | new I2C master | SDA GPIO11, SCL GPIO12, 400 kHz |
| SPI2 | `SPI2_HOST` | DRV8320S (CS GPIO48), AS5047P (CS GPIO13) |
| SPI3 | `SPI3_HOST` | W5500 #0 (CS17, INT16), W5500 #1 (CS15, INT7) |
| ADC1 | oneshot | phase A GPIO6, B GPIO5, C GPIO4 |
| MCPWM | group 0 | INHA 39, INHB 40, INHC 41, 30 kHz center-aligned |
| GPIO | — | DRV ENABLE 38, nFAULT 3, INL 42, brake chopper 1 |

I2C addresses (`Config.hpp`): INA238 `0x40`, LM75A `0x48`, LSM6DSO `0x6A`.
`CS_EXTERNAL` (GPIO10) remains in `Pinout.hpp` but is unused by the current firmware.

**Pin duplication warning:** `MCPWMDriver.cpp` hard-codes GPIO 39/40/41 and
`ADCOneshot.cpp` hard-codes GPIO 4/5/6, duplicating `Pinout.hpp`. If pinout changes,
update all three places (and any `Config.hpp` assumptions).

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

### SPI details

- `SPIBase` ref-counts buses by host + pins; multiple drivers can share a bus.
- `DRV8320S` is constructed in `FOCTask` with **SPI mode 1 at 500 kHz**, even though
  the constructor default is mode 0.
- AS5047P uses **SPI mode 1** and the default **5 MHz**; it supports a pipelined
  angle read and applies even parity.
- W5500 runs at **30 MHz**; both instances use `SPI3_HOST`, which `main.cpp`
  initializes manually with a 4096-byte max transfer.
- W5500 #0 and #1 get different MAC addresses (base MAC and base+1).

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
populated and measured; the FOC controller currently uses A and C and reconstructs
B (`cUsePhaseBForFoc = false`), but B is still published. The INA241 outputs are
biased around mid-supply (the firmware assumes 1.65 V); it measures the
zero-current offset during the startup `CURRENT` phase and subtracts it.

`ADCOneshot::init()` in `FOCTask` is configured as `0.001f, 50.0f, 1.65f` to match
the board-v3 shunt, amplifier gain, and mid-supply reference.

---

## 9. Known TODOs and weak spots

Treat these as current reality, not as instructions to fix everything at once:

- **FOC loop timing:** 1000 µs period, but the loop can take 800+ µs and the comment
  says it needs optimization. Raising the rate without profiling is unsafe.
- **State estimation:** velocity is a low-passed derivative of the encoder angle and
  is noisy; acceleration is described in-code as not working. A Kalman/PLL is a
  known TODO.
- **Motor constants:** `Controller.cpp` hard-codes `motor_l = 1 mH` and
  `motor_flux = 0.67 mWb` with `TODO! Find motor L/flux`.
- **CascadingPID is dead code in the active path:** it is constructed but
  `compute()` is never called. `drivingMode` 3/4/5 currently do not engage the
  position/velocity cascade.
- **Limits/gains over TCP are not all applied:** torque/velocity/position gains and
  `currentLimit*` are accepted and stored, but the active FOC path only reads
  `torqueSetpoint` (plus pole pairs and a few GVM values). Do not assume setting a
  gain changes the running loop.
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
