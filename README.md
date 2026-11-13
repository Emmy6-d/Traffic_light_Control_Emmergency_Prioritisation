# 🚦 Intelligent Multi-Controller Traffic Signal System with Emergency Vehicle Prioritization

> A final year project in Electronics and Telecommunication Engineering — University of Rwanda, College of Science and Technology (June 2026)

---

## 📋 Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Hardware Requirements](#hardware-requirements)
- [Pin Configuration](#pin-configuration)
- [Wiring Guide](#wiring-guide)
- [Software Architecture](#software-architecture)
- [Normal Traffic Cycle](#normal-traffic-cycle)
- [Emergency Priority System](#emergency-priority-system)
- [Project Structure](#project-structure)
- [How to Build & Upload](#how-to-build--upload)
- [How It Works — Step by Step](#how-it-works--step-by-step)
- [State Machine Diagram](#state-machine-diagram)
- [Authors](#authors)
- [License](#license)

---

## Overview

This project implements an intelligent 4-way traffic intersection controller with real-time emergency vehicle prioritization using two Arduino microcontrollers communicating over hardware UART.

- An **Arduino Mega 2560** sits at the intersection and manages the full traffic signal cycle, pedestrian signals, and emergency responses.
- An **Arduino Nano/UNO** is mounted inside an emergency vehicle and transmits direction-coded priority requests to the intersection controller.

When an emergency request is received, the system immediately suspends the normal traffic cycle, clears all lanes except the emergency approach, and resumes the interrupted cycle seamlessly once the vehicle has passed — without losing track of where the normal phase was.

---

## System Architecture

```
┌─────────────────────────────────┐          UART 9600 baud          ┌──────────────────────────────┐
│   Emergency Vehicle             │  ──── TX ──────────────► RX1 ──► │   Intersection Controller    │
│   Arduino Nano / UNO            │  ◄─── GND ─────────────────────  │   Arduino Mega 2560          │
│                                 │                                   │                              │
│  • 4 push buttons (S/E/N/W)     │   Packet: $EMG,S#                │  • 4-phase traffic cycle     │
│  • Sends packet on press        │           $EMG,E#                │  • Timer3 ISR (1-second tick)│
│  • Retransmits every 200 ms     │           $EMG,N#                │  • FIFO emergency queue      │
│    while button held            │           $EMG,W#                │  • Pedestrian signals        │
│  • One lane active at a time    │                                   │  • Buzzer alert              │
└─────────────────────────────────┘                                   └──────────────────────────────┘
```

---

## Hardware Requirements

| Component | Quantity | Notes |
|---|---|---|
| Arduino Mega 2560 | 1 | Intersection controller |
| Arduino Nano or UNO | 1 | Emergency vehicle transmitter |
| Traffic LEDs (Red, Yellow, Green) | 4 sets × 3 | One set per approach |
| Right-Turn Yellow LEDs | 4 | Caution signal, always ON |
| Pedestrian LEDs (Red, Green) | 4 sets × 2 | One set per approach |
| Push Buttons | 4 | South, East, North, West requests |
| Buzzer | 1 | Emergency alert |
| LED (Debug) | 1 | Emergency status indicator |
| Resistors (220 Ω) | As needed | For LEDs |
| Jumper wires | As needed | — |
| Breadboard | As needed | — |

---

## Pin Configuration

### Arduino Mega 2560 — Traffic Light Outputs

| Approach | Green | Yellow | Red | Right-Turn Yellow |
|---|---|---|---|---|
| South | 22 | 23 | 24 | 25 |
| East | 26 | 27 | 28 | 29 |
| North | 30 | 31 | 32 | 33 |
| West | 34 | 35 | 36 | 37 |

### Arduino Mega 2560 — Pedestrian Signal Outputs

| Approach | Green | Red |
|---|---|---|
| East | 40 | 41 |
| North | 42 | 43 |
| West | 44 | 45 |
| South | 46 | 47 |

### Arduino Mega 2560 — Auxiliary

| Function | Pin |
|---|---|
| Buzzer | 48 |
| Debug LED | 49 |
| UART RX1 (from vehicle) | 19 |

### Arduino Nano/UNO — Button Inputs (INPUT_PULLUP)

| Direction | Pin |
|---|---|
| South | 2 |
| East | 3 |
| North | 4 |
| West | 5 |

---

## Wiring Guide

```
Arduino Nano/UNO                    Arduino Mega 2560
─────────────────                   ─────────────────
        TX (pin 1)  ──────────────►  RX1 (pin 19)
               GND  ──────────────►  GND

Push Buttons (INPUT_PULLUP — no external resistor needed):
  Button one side → pin 2 / 3 / 4 / 5
  Button other side → GND
```

> **Important:** Do **not** connect the Nano/UNO 5 V to the Mega. Only TX and GND are shared.

---

## Software Architecture

### Transmitter (Nano/UNO)

- Polls four buttons every loop iteration with **50 ms software debounce**.
- First confirmed button press locks that lane as active; other buttons are ignored until it is released.
- On lock-in, sends the packet **immediately**, then **every 200 ms** while held.
- Uses no `delay()` — all timing via `millis()`.

### Receiver (Mega 2560)

- **Timer3 CTC interrupt** fires every 1 second — drives the normal traffic phase timer.
- **Three-state top-level state machine:**

| Mode | Description |
|---|---|
| `MODE_NORMAL` | Running the 4-phase traffic cycle |
| `MODE_EMERGENCY` | Serving an active emergency vehicle |
| `MODE_POST_EMG_RED` | 1-second all-red clearance after emergency ends |

- **Byte-by-byte serial parser** on Serial1 — packet delimited by `$` (start) and `#` (end); all malformed packets discarded silently.
- **FIFO emergency queue** (depth 4) with overflow protection and duplicate suppression.
- `lastEmergencyRx` timestamp updated on every valid packet — 3-second keepalive window.
- Normal phase (`currentPhase`) and elapsed seconds (`phaseTimer`) are **never modified** during emergency mode, so the cycle resumes exactly where it was interrupted.

---

## Normal Traffic Cycle

The cycle runs continuously across four phases. Right-turn yellow lights (pins 25, 29, 33, 37) remain **ON at all times** as permanent caution signals.

```
Phase 1 │ South GREEN  (15 s)  + East Pedestrian GREEN
        │ South YELLOW  (3 s)
        │ All RED       (1 s)
        │
Phase 2 │ East GREEN   (10 s)  + North Pedestrian GREEN
        │ East YELLOW   (3 s)
        │ All RED       (1 s)
        │
Phase 3 │ North GREEN  (15 s)  + West Pedestrian GREEN
        │ North YELLOW  (3 s)
        │ All RED       (1 s)
        │
Phase 4 │ West GREEN   (10 s)  + South Pedestrian GREEN
        │ West YELLOW   (3 s)
        │ All RED       (1 s)
        └──────────────────────── repeat ──────────────────────────►
```

---

## Emergency Priority System

### Activation
1. Receiver gets a valid `$EMG,X#` packet on Serial1.
2. If `MODE_NORMAL` or `MODE_POST_EMG_RED`: direction is enqueued (if not a duplicate).
3. Queue is checked each loop; first entry is dequeued and becomes the active emergency.
4. Normal cycle is suspended — `currentPhase` and `phaseTimer` are frozen.

### During Emergency
- All pedestrian signals → **RED**.
- All right-turn yellows stay **ON**.
- **Buzzer ON**, **Debug LED ON**.
- Active emergency approach → **GREEN**; all others → **RED**.
- `lastEmergencyRx` refreshed on every valid incoming packet.

### Clearance
- If no valid packet arrives for **3000 ms**, emergency times out.
- Buzzer and debug LED turn **OFF**.
- All approaches go **RED** for **1 second** (clearance interval).
- If another request is queued, it is serviced next.
- Otherwise, normal cycle resumes from the exact phase and second it was interrupted.

### Queue Behaviour
- Maximum 4 pending requests (FIFO).
- Duplicate direction for the **active emergency** → refreshes keepalive only (not re-queued).
- Duplicate direction **in the queue** → silently dropped.
- Queue full → new request dropped with a serial debug message.

---

## Project Structure

```
/
├── EmergencyVehicle_Transmitter_Nano.ino   # Arduino Nano/UNO — vehicle transmitter
├── TrafficController_Receiver_Mega.ino     # Arduino Mega 2560 — intersection controller
└── README.md                               # This file
```

---

## How to Build & Upload

### Prerequisites
- [Arduino IDE](https://www.arduino.cc/en/software) 1.8.x or 2.x
- Board package: **Arduino AVR Boards** (included by default)
- No external libraries required — standard Arduino core only.

### Steps

1. **Clone the repository**
   ```bash
   git clone https://github.com/<your-username>/<repo-name>.git
   cd <repo-name>
   ```

2. **Upload the Transmitter**
   - Open `EmergencyVehicle_Transmitter_Nano.ino` in Arduino IDE.
   - Select board: `Arduino Nano` or `Arduino Uno`.
   - Select the correct COM port.
   - Click **Upload**.

3. **Upload the Receiver**
   - Open `TrafficController_Receiver_Mega.ino` in Arduino IDE.
   - Select board: `Arduino Mega or Mega 2560`.
   - Select the correct COM port.
   - Click **Upload**.

4. **Wire the two boards** as described in the [Wiring Guide](#wiring-guide).

5. **Power both boards** and observe the traffic cycle. Press any button on the Nano/UNO to trigger an emergency request.

---

## How It Works — Step by Step

1. On power-up the Mega starts **Phase 1** (South Green) and the Timer3 ISR begins counting seconds.
2. The Nano/UNO polls its buttons every loop iteration. No button → no transmission.
3. A user presses the **East button** (pin 3). After 50 ms debounce, `$EMG,E#` is sent immediately, then every 200 ms while held.
4. The Mega's parser assembles the packet, validates it, and enqueues `DIR_E`.
5. The next `loop()` iteration detects the queue, dequeues `DIR_E`, saves the current `currentPhase` and `phaseTimer`, and enters `MODE_EMERGENCY`.
6. East approach goes **Green**; all others go **Red**; all pedestrians go **Red**; buzzer sounds.
7. Every 200 ms a new `$EMG,E#` arrives, refreshing `lastEmergencyRx`.
8. The user releases the button. No more packets arrive.
9. After **3000 ms** of silence, the Mega times out, kills the buzzer, holds all-red for 1 second, then resumes the normal cycle from exactly the phase and second it was at when interrupted.

---

## State Machine Diagram

```
                    ┌──────────────────┐
         ┌─────────►│   MODE_NORMAL    │◄────────────────────┐
         │          │  (Timer3 driven) │                     │
         │          └────────┬─────────┘                     │
         │                   │ EMG queue non-empty            │
         │                   ▼                               │
         │          ┌──────────────────┐                     │
         │          │  MODE_EMERGENCY  │                     │
         │          │  (buzzer ON,     │                     │
         │          │   green active   │                     │
         │          │   approach only) │                     │
         │          └────────┬─────────┘                     │
         │                   │ 3000 ms timeout               │
         │                   ▼                               │
         │          ┌──────────────────┐                     │
         │          │ MODE_POST_EMG_   │                     │
         └──────────│      RED         │─────────────────────┘
    queue empty &   │  (all red 1 s)   │  next EMG in queue
    1 s elapsed     └──────────────────┘  → back to EMERGENCY
```

---

## Authors

**Emmanuel** — Electronics and Telecommunication Engineering, University of Rwanda  
**Damien Iradukunda** — Final Year Project Team  
**Jean Baptiste Hakizimana** — Final Year Project Team

---

## License

This project is released for academic and educational purposes.  
Feel free to use, modify, and build upon it with appropriate attribution.

---

> *"Engineering solutions for safer roads."*
