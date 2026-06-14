# ESP32 Clean Hearing Booster

This sketch is based on the provided ESP32 I2S mic-to-DAC loop, with a small
real-time voice cleanup chain:

1. Select one real I2S mic channel instead of amplifying both stereo slots.
2. Copy the cleaned mono signal to both headphone channels.
3. Remove DC and low-frequency rumble with a high-pass filter.
4. Smooth high-frequency hiss with a gentle low-pass filter.
5. Reduce idle hum with a soft noise gate.
6. Prevent loud rattling with a soft limiter.

## First things to tune

- If the mic is silent or weak, change `MIC_CHANNEL_INDEX` in the `.ino` file
  from `0` to `1`.
- If room noise is still heard during silence, raise `GATE_OPEN_THRESHOLD` and
  `GATE_CLOSE_THRESHOLD` a little.
- If quiet syllables are being cut off, lower the gate thresholds.
- If the DAC does not like `16000`, set `SAMPLE_RATE` back to `44100`.

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
