# ESP32 Clean Hearing Booster

This sketch is based on the ESP32 I2S mic-to-DAC loop for two INMP441/ICS-43434
I2S MEMS mics and a UDA1334A I2S DAC.

Wiring:

- Both mic SD/DOUT pins share `MIC_SD_PIN`.
- Both mic BCLK/SCK pins share `I2S_BCLK_PIN`.
- Both mic WS/LRCLK pins share `I2S_WS_PIN`.
- Left mic L/R pin goes to GND.
- Right mic L/R pin goes to 3.3 V.

The cleanup chain:

1. Voice high-pass filter to remove rumble and handling noise.
2. Voice low-pass filter to reduce hiss and distracting sharp noise.
3. Presence boost around speech consonants for clarity.
4. Mid/side focus to reduce side-heavy distractions.
5. Adaptive noise expander that lowers steady background sound without fully
   muting quiet speech.
6. AGC to lift quiet speech.
7. Compressor and soft limiter to protect against painful loud sounds and DAC
   clipping.

## First things to tune

- If left/right sound swapped, set `SWAP_MIC_CHANNELS` to `1`.
- If voice sounds too narrow/mono, raise `SIDE_KEEP`.
- If surrounding distractions are still too strong, lower `SIDE_KEEP`.
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
