# ESP32 Clean Hearing Booster

This sketch is based on the ESP32 I2S mic-to-DAC loop for four INMP441/ICS-43434
I2S MEMS mics and a UDA1334A I2S DAC.

Wiring:

- Front-left mic L/R pin goes to GND, SD/DOUT goes to `MIC_FRONT_SD_PIN`.
- Front-right mic L/R pin goes to 3.3 V, SD/DOUT goes to `MIC_FRONT_SD_PIN`.
- Back-left mic L/R pin goes to GND, SD/DOUT goes to `MIC_BACK_SD_PIN`.
- Back-right mic L/R pin goes to 3.3 V, SD/DOUT goes to `MIC_BACK_SD_PIN`.
- All four mics share `I2S_BCLK_PIN` and `I2S_WS_PIN`.
- The DAC receives processed headphone audio on `DAC_DIN_PIN`.

The sketch uses two I2S receivers:

- I2S0 master full-duplex reads the front mic pair and writes the DAC output.
- I2S1 slave RX reads the back mic pair using the same BCLK/WS clocks.

The cleanup chain:

1. Voice high-pass filter to remove rumble and handling noise.
2. Voice low-pass filter to reduce hiss and distracting sharp noise.
3. Presence boost around speech consonants for clarity.
4. Four-mic direction scoring to favor the strongest voice-like direction.
5. Adaptive direction weighting for front-left, front-right, back-left, and
   back-right.
6. Adaptive noise expander that lowers steady background sound without fully
   muting quiet speech.
7. AGC to lift quiet speech.
8. Compressor and soft limiter to protect against painful loud sounds and DAC
   clipping.

## First things to tune

- If the front pair is left/right swapped, set `SWAP_FRONT_LR` to `1`.
- If the back pair is left/right swapped, set `SWAP_BACK_LR` to `1`.
- If the sound is too mono, raise `OUTPUT_STEREO_WIDTH`.
- If direction focus is too weak, raise `DIRECTION_FOCUS_POWER`.
- If direction focus is too jumpy or unnatural, lower `DIRECTION_FOCUS_POWER`
  or raise `DIRECTION_MIN_WEIGHT`.
- If quiet speech is not loud enough, raise `TARGET_SPEECH_LEVEL` slightly.
- If room noise pumps or gets too loud, lower `MAX_AGC_GAIN`.
- If output still rattles, lower `LIMIT_START`, for example `24000.0f`.

## Hardware noise checks

Software helps, but constant buzzing or rattling is often electrical:

- Power the mic and DAC from a clean 3.3 V supply, not a noisy USB rail if you
  can avoid it.
- Keep the analog headphone/jack wires short and away from BCLK/WS/DIN lines.
- Make sure ESP32, mic, DAC, and jack ground all share one solid ground.
- Add local decoupling near the mic and DAC modules, for example 0.1 uF plus
  10 uF across power and ground.
- Do not drive passive headphones directly from an I2S DAC line; use a headphone
  amp or a DAC module intended for headphones.
