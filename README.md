# MudraSynth

MudraSynth is a small, portable gesture-controlled synthesizer built around an Arduino Nano. A light-dependent resistor (LDR) turns hand position and shadow into pitch, letting the player “cast” notes without touching a conventional keyboard. A gate button articulates the sound, while a second button records and repeats a phrase through an independent playback voice so the live instrument remains playable over the loop.

The instrument uses the [Mozzi](https://sensorium.github.io/Mozzi/) audio library and a custom phase-distortion resonant voice. The LDR selects notes from C major across C1–C7. Pitch changes are smoothed and stabilized, with 50 ms portamento between notes. The live and loop voices receive the same phase-distortion movement and are mixed equally when they overlap.

## Controls and pinout

| Arduino pin | Connect to | Function |
| --- | --- | --- |
| `A0` | LDR/resistor divider midpoint | Light-controlled pitch in C major, C1–C7 |
| `D3` | Momentary gate button to GND | Plays the live voice; active low using the Nano's internal pull-up |
| `D5` | LED anode through a 220 Ω–1 kΩ resistor | Envelope display; about 10% brightness at rest, rising with the louder voice |
| `D6` | Momentary looper button to GND | Cycles through record → playback → normal; active low using the internal pull-up |
| `D9` | Mozzi standard audio-output circuit | Mixed PWM audio from the live and loop voices |
| `5V` | LDR supply | Control supply |
| `VIN` | Optional 9 V battery positive | Portable power input through the Nano's onboard regulator |
| `GND` | Resistor divider, buttons, LED, audio, and battery negative | Common circuit ground |

## Wiring

### LDR pitch control

Build a voltage divider with the LDR above the sensing node and a 10 kΩ resistor below it:

```text
5V ---- LDR ----+---- A0
                |
               10 kΩ
                |
               GND
```

Brighter light lowers the LDR's resistance and moves the reading toward `5V`; shading it moves the reading toward `GND`. Swapping the LDR and 10 kΩ resistor reverses the response.

### Buttons and LED

Both buttons are normally-open momentary switches. Connect the gate button between `D3` and `GND`, and the looper button between `D6` and `GND`. The sketch enables the Nano's internal pull-ups, so neither button needs an external pull-up resistor.

Connect `D5` through a 220 Ω–1 kΩ current-limiting resistor to the LED anode (long leg), then connect the cathode (short leg) to `GND`.

### Audio and power

`D9` carries Mozzi's standard single-pin PWM audio. Route it through a suitable Mozzi standard output/low-pass circuit and into a powered speaker, amplifier, mixer, or other high-impedance audio input. Do not connect a passive speaker directly to `D9`, and keep the amplifier ground connected to Arduino `GND`.

Power the Nano over USB while programming. For portable operation, a 9 V battery can connect positive to `VIN` and negative to `GND`.

## Playing and looping

- Move a hand over the LDR to select notes; press and hold the `D3` gate button to sound the live voice.
- Press `D6` once to begin recording pitch and gate changes. The looper stores control events rather than audio samples.
- Press `D6` again to start playback through the second voice. The live voice stays available, and overlapping voices are mixed 50/50.
- Press `D6` a third time to stop playback and return to normal mode.

The first playback pass preserves the performance's original timing while a background process finds a 60–160 BPM, 4/4 interpretation. Later passes move note timing toward a sixteenth-note grid. The loop stores up to 64 pitch and gate events.

## Current sound mode

This snapshot preserves the final state used in the **New Theremin project** task. `BYPASS_VOLUME_ENVELOPES` is set to `true`, a temporary listening mode that keeps the phase-distortion voices at full volume while their PD envelopes run. To restore the adaptive internal and external volume shaping developed earlier in the task, change this line near the top of the sketch:

```cpp
const bool BYPASS_VOLUME_ENVELOPES = false;
```

With volume shaping restored, rapid note-ons shorten the attack from 50 ms toward 10 ms in 5 ms steps; after 500 ms without a note-on, it recovers toward 50 ms. Release time is 1500 ms. Live and loop voices adapt independently.

## Build and upload

### Arduino IDE

1. Install the **Arduino AVR Boards** platform in Boards Manager.
2. Install **Mozzi 2.0.4** and **FixMath 1.0.9** in Library Manager.
3. Copy `libraries/PDResonantCustom` from this repository into your Arduino sketchbook's `libraries` folder, then restart the Arduino IDE.
4. Open `simple_beepit_v1/simple_beepit_v1.ino`.
5. Select **Arduino Nano** as the board and **ATmega328P (Old Bootloader)** as the processor.
6. Select the Nano's serial port and click **Upload**.

### Arduino CLI

From the repository root, with Arduino AVR Boards, Mozzi, and FixMath installed:

```sh
arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328old --libraries libraries simple_beepit_v1
arduino-cli upload --port COM3 --fqbn arduino:avr:nano:cpu=atmega328old simple_beepit_v1
```

Replace `COM3` with the port used by your Nano.

## Repository contents

- `simple_beepit_v1/simple_beepit_v1.ino` — main gesture synth and two-voice looper sketch
- `libraries/PDResonantCustom` — project-specific resonant phase-distortion voice built on Mozzi

The sketch retains its Mozzi copyright and LGPL notice. Mozzi and FixMath remain external dependencies and are not bundled in this repository.
