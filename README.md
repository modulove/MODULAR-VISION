# MODULAR VISION by Modulove

Alternative firmware for the **SPCTRL × WGD Modular VISION / FORK** — a 1u (Intellijel) Eurorack oscilloscope module.

This firmware extends the original VISION oscilloscope with additional display modes, a tuner, adjustable parameters, and screensaver.

The module is a collaboration between [SPCTRL Modular](https://github.com/BastianSPCTRL/VISION) and [WGD Modular](https://wgdmodular.de) from Germany.

---

## Features

### Scope Mode
- Waveform display across the screen
- **time base settings** from 10µs to .5s
- **voltage range settings** for zooming into smaller signals
- **Adjustable trigger threshold** with rising-edge detection and dotted trigger line indicator
- **Display offset** to shift the waveform vertically (±16 pixels)
- full-screen waveform when menu times out

### Tuner Mode
- **Note display** showing note name, octave, and cents deviation with a visual bar graph
- **Hz display** showing measured frequency 
- **Configurable reference pitch** (A4 = 400–460 Hz in 1 Hz steps)
- **Adjustable averaging speed** (Fast / Medium / Slow) for trading responsiveness vs. stability
- Enlarged readout fills the screen when menu is hidden

### General
- **Hold mode** freezes the current display (long-press encoder button)
- **Starfield screensaver** activates after 30 seconds of idle — only when no signal is present
- **Auto-hiding menu** — parameter overlay disappears after 3 seconds
- Settings persist in EEPROM across power cycles

---

## Controls

| Action | Function |
|---|---|
| **Rotate encoder** | Adjust the selected parameter |
| **Short press** | Cycle to next parameter, then switch mode (Scope → Tune → Scope) |
| **Long press** | Toggle hold mode |

### Scope Parameters (cycle with short press)
1. **Time base** — capture window from 10µs to 10s
2. **Voltage range** — display scale from 0.5V to 5V
3. **Trigger threshold** — voltage level for waveform sync
4. **Display offset** — vertical shift of the waveform

### Tuner Parameters (cycle with short press)
1. **Display format** — Hz or Note name
2. **Reference pitch** — A4 tuning reference (400.0–460.0 Hz)
3. **Averaging speed** — Fast, Medium, or Slow

---

## Hardware

- **Module**: SPCTRL × WGD Modular VISION (1u Intellijel Eurorack)
- **MCU**: Arduino Nano (ATmega328P)
- **Display**: SSD1306 128×32 OLED via I2C
- **Input**: 0–5V analog on A0 (via voltage divider to internal 1.1V reference)
- **Encoder**: Rotary encoder with push button (pins 2, 4, 5)
- **Attenuation control**: Pin 12 (set HIGH-Z for 5V direct input)

---

## Dependencies

Install the following libraries via Arduino Library Manager:

- [Adafruit SSD1306](https://github.com/adafruit/Adafruit_SSD1306)
- [Adafruit GFX](https://github.com/adafruit/Adafruit-GFX-Library)
- [EncoderButton](https://github.com/Stutchbury/EncoderButton)

---

## Flashing

1. Open `VISION_firmware.ino` in Arduino IDE
2. Select **Board**: Arduino Nano, **Processor**: ATmega328P (Old Bootloader)
3. Connect via USB and upload

---

## Credits

- **Hardware design**: [SPCTRL Modular](https://github.com/BastianSPCTRL/VISION) & [WGD Modular](https://wgdmodular.de)
- **Alternative firmware**: [Modulove](https://modulove.io)
- **Original firmware**: SPCTRL × WGD Modular
