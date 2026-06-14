# ESP32 Clean Hearing Booster

This sketch is based on the provided ESP32 I2S mic-to-DAC loop for an
INMP441/ICS-43434 I2S MEMS mic and UDA1334A I2S DAC. It keeps the working
44.1 kHz setup, but uses 32-bit I2S slots because these MEMS mics output
24-bit audio inside a 32-bit word.

The cleanup chain:

1. Select the real I2S mic slot with `MIC_CHANNEL_INDEX`.
2. Convert the 24-bit mic data to a signed 16-bit working sample.
3. Copy the cleaned mono signal to both DAC/headphone channels.
4. Remove DC and low-frequency rumble with a high-pass filter.
5. Smooth high-frequency hiss with a gentle low-pass filter.
6. Prevent loud rattling with a soft limiter.

## First things to tune

- The sketch defaults to `MIC_CHANNEL_INDEX 1` for a right-slot mic. If there
  is rattling but no voice, change it to `0`.
- If the output is too dull, raise `LOWPASS_ALPHA` a little, for example `0.55`.
- If the output still rattles, lower `LIMIT_START`, for example `24000.0f`.
- Keep `SAMPLE_RATE` at `44100` while debugging because this matches the
  original working code.

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
