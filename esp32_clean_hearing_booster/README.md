# ESP32 Clean Hearing Booster

This sketch is based on the provided ESP32 I2S mic-to-DAC loop for an
INMP441/ICS-43434 I2S MEMS mic and UDA1334A I2S DAC. It intentionally keeps
the same 44.1 kHz / 16-bit shared I2S bus format as the original working
sketch, then adds light cleanup in the sample loop.

The cleanup chain:

1. Process both interleaved stereo slots so it does not matter whether the
   INMP441 L/R pin is tied high or low.
2. Remove DC and low-frequency rumble with a high-pass filter.
3. Smooth high-frequency hiss with a gentle low-pass filter.
4. Prevent loud rattling with a soft limiter instead of harsh clipping.

## First things to tune

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
