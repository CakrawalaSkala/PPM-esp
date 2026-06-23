# Main Transmitter-Side Controller (Voice/Pedal → PPM)

ESP32 firmware running on the **transmitter side**, acting as the central controller for this system. It receives pilot commands from two selectable input sources — a voice-recognition computer (via UART) or a dual foot-pedal switch (via tap-combination gestures) — and converts the resulting command into PPM channel values that are transmitted to the RC transmitter. It also drives a status OLED display and a buzzer for audible feedback.

---

## 1. Input Source Selection

A physical toggle switch on **GPIO27** (`SWpin`) selects which input source is active, read continuously inside `uart_task`:

- **Switch OFF → `ManualMode = 0` → Voice mode**: commands are accepted from UART2 (RX on GPIO16, 115200 baud) — i.e. from the connected voice-recognition computer.
- **Switch ON → `ManualMode = 1` → Manual mode**: commands are derived from two foot-pedal/touch switches (GPIO12 = right pedal, GPIO14 = left pedal, pull-down) via `manual_command()`, and UART input is ignored for command execution (`checksum == ... && ManualMode == 0` gates UART-driven execution).

Every mode transition is logged (`"MANUAL ON"` / `"VOICE ON"`).

---

## 2. Voice Command Reception (UART2)

In voice mode, `uart_task` reads 3-byte packets from UART2:

```
[ 0xAA (header) ] [ cmd ] [ checksum ]
```

A packet is accepted only if `checksum == (cmd & 0xFF)` and the device is currently in voice mode. This matches the packet format sent by the voice-recognition script described previously, confirming this firmware is the receiving end of that link.

One additional interlock exists specifically for command **5** (switch drone): it is only executed if GPIO12 or GPIO14 reads high at that moment, in addition to the checksum/mode check — every other command executes from a valid UART packet alone. On any successfully executed UART command, GPIO2 is also driven high as a status/acknowledgement signal.

If a malformed packet (wrong length) is received, the UART input buffer is flushed.

---

## 3. Manual Pedal Command Decoding (`manual_command`)

In manual mode, the two foot pedals are decoded into command IDs using **tap counting and simultaneous-press detection**:

- **Release-then-press edge detection**: a tap is only counted on a clean press following a release (`readyR`/`readyL` arming logic), avoiding double-counting from a single held press.
- **Simultaneous press (`syncWaiting`)**: if both pedals are pressed within a **120 ms** window (`SYNC_WINDOW`) of each other, it's treated as a synchronized tap (`syncCount`) instead of two separate single-pedal taps, and any in-progress single-pedal tap counts are cleared.
- **Sync tap resolution** (once both pedals are released and `MULTI_CLICK_TIME` = 500 ms have passed):
  - `syncCount == 1` → command **4** (switch camera)
  - `syncCount == 2` → command **5** (switch drone)
- **Right-pedal-only tap resolution** (resolved 500 ms after release, if no sync occurred):
  - `countR == 1` → command **3**
  - `countR == 2` → command **7**
  - `countR == 3` → command **8**
- **Left-pedal-only tap resolution**:
  - `countL == 1` → command **2**
  - `countL == 2` → command **9**
  - `countL == 3` → command **6**
- **Tap timeout**: if a pedal is held down for more than `TAP_CONFIRM_TIMEOUT` (3000 ms) after a tap was registered without the gesture resolving, the tap count for that pedal is discarded.

Note: a variable `cmd5Pending` is checked as a guard condition in this logic but is never assigned anywhere in the code — it currently always evaluates as not-pending and has no observable effect on behavior as written.

---

## 4. Command IDs (`enum data_map`)

| ID | Name |
|---|---|
| 1 | `ERROR_CMD` |
| 2 | `PAYLOAD_LEFT_CMD` |
| 3 | `PAYLOAD_RIGHT_CMD` |
| 4 | `SWITCH_CAMERA_CMD` |
| 5 | `SWITCH_DRONE_CMD` |
| 6 | `PAYLOAD_RESET_CMD` |
| 7 | `PAYLOAD_LOADER_CMD` |
| 8 | `RESET_DROPPER` |
| 9 | `DRONE_ROTATE` |

These IDs line up with the serial IDs produced by the connected voice-recognition system, confirming the two firmwares/scripts are designed to interoperate directly over the UART link.

---

## 5. Command Execution (`execute_command`)

Each command updates one or more entries in the 8-channel PPM array (`channel_val[]`, see channel map below), and sets a buzzer pattern (`buzz`). Execution of most commands is additionally gated by `enVoiceA` / `enVoiceB` flags — though as written, both flags are unconditionally set to `1` in every mode, so they don't currently restrict execution in practice.

| Command | Behavior |
|---|---|
| `PAYLOAD_RIGHT_CMD` (3) | Pulses `PAYLOAD_CH` to 2000 then back to 1500 after 100 ms; advances `DropperPosA` (mod 3); buzzer pattern 4 |
| `PAYLOAD_LEFT_CMD` (2) | Pulses `PAYLOAD_CH` to 1000 then back to 1500; advances `DropperPosB` (mod 3); buzzer pattern 4 |
| `SWITCH_CAMERA_CMD` (4) | Toggles `CAM_CH` between 1000 and 2000 in 1000 µs steps (`CAM_STEP`); buzzer pattern 1 |
| `SWITCH_DRONE_CMD` (5) | Cycles `DRONE_CH` through 1000 → 1500 → 2000 → 1000 in 500 µs steps (`DRONE_STEP`); resets `DropperPosA`/`DropperPosB` to 0; pulses `RESET_DROPPER_CH`; buzzer pattern 2 |
| `PAYLOAD_LOADER_CMD` (7) | Advances `PaySeq` (mod 4); cycles `PAYLOAD_LOADER_CH` upward in 300 µs steps (`LOADER_STEP`) up to 1800 then wraps to 1000; resets `DropperPosA`/`DropperPosB` and pulses `RESET_DROPPER_CH`; buzzer pattern 1 |
| `PAYLOAD_RESET_CMD` (6) (or `resetMan`) | Sets `ROTATE_CH` and `PAYLOAD_LOADER_CH` to 1000; resets `PaySeq` and `DronePos` to 0; buzzer pattern 3 |
| `DRONE_ROTATE` (9) | Advances `DronePos` (mod 4); maps to `ROTATE_CH` = 1500 / 1000 / 2000 / 1000 for positions 1/2/3/0; buzzer pattern 1 |
| `RESET_DROPPER` (8) (or `resetMan`) | Pulses `RESET_DROPPER_CH` from 1000 to 1500; resets `DropperPosA`/`DropperPosB`; buzzer pattern 3 |
| (unmatched) | Logs a warning: unknown command or button not yet actuated |

**Cross-mechanism interlock**: after any command executes, if `DropperPosA` or `DropperPosB` has just reached position **2** (detected via one-shot `dropperTriggeredA`/`dropperTriggeredB` flags), `ROTATE_CH` is forced back to 1000, and if `PaySeq > 1`, `DronePos` is clamped into position 2 (or wrapped to 0 if it had gone past 2). This ties dropper-arm position to drone rotation position once a reload sequence is underway. The trigger flags reset once the corresponding dropper position returns to 0.

---

## 6. PPM Output (`enum channel_map` + RMT)

The system outputs a standard 8-channel PPM frame using the ESP32's RMT peripheral on **GPIO5**, framed every **20,000 µs** (`FRAME_DURATION_US`):

| Index | Channel |
|---|---|
| 0 | `ROLL` |
| 1 | `PITCH` |
| 2 | `RESET_DROPPER_CH` |
| 3 | `PAYLOAD_LOADER_CH` |
| 4 | `ROTATE_CH` |
| 5 | `CAM_CH` |
| 6 | `DRONE_CH` |
| 7 | `PAYLOAD_CH` |

`ppm_encoder_callback` builds the RMT symbol stream from `channel_val[]`: each channel pulse is clamped to the 1000–2000 µs range, separated by a fixed 100 µs sync pulse (`PPM_PULSE_WIDTH`), with a final filler pulse sized to pad the total frame out to exactly 20,000 µs. `rmt_task` continuously re-transmits this frame in a tight loop. A `DEBUG` build flag (currently disabled, `#define DEBUG 0`) exists to inject a fixed test sequence on `PAYLOAD_CH` instead of relying on live commands.

---

## 7. OLED Status Display (`oled_task`)

The OLED continuously displays the current system state — the same information surfaced by the companion monitoring app described previously:

- **Drone label** (top-left): "DRONE 1" / "DRONE 2" / "DRONE 3" based on `DRONE_CH` value (1000 / 1500 / 2000)
- **Camera label** (top-right): "CAM 1" if `CAM_CH == 1000`, otherwise "CAM 2"
- **Mode**: "Manual" or "Voice " depending on `ManualMode`
- **Payload Seq**: current `PaySeq` value (0–3)
- **Drone Pos**: "Front" / "Right" / "Front" / "Left " for `DronePos` 0/1/2/3 respectively (positions 0 and 2 both display "Front" as currently written)
- **Dropper Pos – Front** (`DropperPosA`): "Right" / "Mid  " / "Left " for values 0/1/2
- **Dropper Pos – Rear** (`DropperPosB`): "Right" / "Mid  " / "Left " for values 0/1/2

---

## 8. Buzzer Feedback (`buzzer_task`)

A single buzzer on **GPIO26** plays one of four patterns depending on the `buzz` value set by `execute_command`:

| `buzz` value | Pattern |
|---|---|
| 1 | Two short beeps (200 ms on, 20 ms off, ×2) |
| 2 | Three short beeps (200 ms on, 20 ms off, ×3) |
| 3 | One long beep (700 ms) followed by one short beep (200 ms) |
| 4 | One short beep (300 ms) |

After playing, `buzz` is reset to 0.

---

## 9. Task Structure

| Task | Core | Priority | Role |
|---|---|---|---|
| `uart_task` | 0 | 4 | Reads UART2 voice commands, reads pedal/mode GPIOs, runs `manual_command()` in manual mode |
| `rmt_task` | 1 | 7 | Continuously transmits the PPM frame via RMT |
| `oled_task` | 1 | 5 | Updates the OLED status display |
| `buzzer_task` | 1 | 4 | Plays buzzer feedback patterns |

---

## 10. Pinout

| Function | GPIO |
|---|---|
| PPM output (RMT) | 5 |
| UART2 RX (from voice computer) | 16 |
| Mode select toggle (Voice/Manual) | 27 |
| Right pedal input | 12 |
| Left pedal input | 14 |
| Buzzer output | 26 |
| Command-received status output | 2 |

---