#include <Arduino.h>
#include <driver/i2s_std.h>
#include <math.h>

// --- PIN DEFINITIONS FOR YOUR V1 BOARD ---
#define MIC_SD_PIN     15  // Labeled D15 (Shared Mic Data Input)
#define DAC_DIN_PIN    18  // Labeled D18 (Dedicated DAC Data Output)
#define I2S_BCLK_PIN   16  // Labeled RX2 (Shared Clock)
#define I2S_WS_PIN     17  // Labeled TX2 (Shared Clock)

#define VOL_UP_PIN      4  // Labeled D4
#define VOL_DOWN_PIN    5  // Labeled D5

// --- AUDIO CONFIGURATION ---
// Voice does not need 44.1 kHz. A lower rate removes high-frequency hiss and
// leaves the ESP32 more time for filtering. Use 44100 if your DAC requires it.
#define SAMPLE_RATE    16000
#define BUFFER_FRAMES  64

// Most I2S MEMS mics drive only one stereo slot. If the audio sounds silent or
// very weak, change this from 0 to 1.
#define MIC_CHANNEL_INDEX 0  // 0 = left slot, 1 = right slot

// --- SIMPLE VOICE CLEANUP TUNING ---
// High-pass cutoff is about 80 Hz at 16 kHz. This removes DC, handling rumble,
// and much of the low-frequency hum without hurting speech intelligibility.
const float HPF_COEFF = 0.969f;

// One-pole low-pass smoothing. Lower values remove more hiss but make speech
// duller. 0.65 is a conservative voice-friendly starting point at 16 kHz.
const float LOWPASS_ALPHA = 0.65f;

// Noise gate thresholds in signed 16-bit sample units. Raise these if room hum
// is still heard during silence; lower them if quiet syllables get cut off.
const float GATE_OPEN_THRESHOLD = 900.0f;
const float GATE_CLOSE_THRESHOLD = 450.0f;
const float CLOSED_GATE_GAIN = 0.08f;

// Soft limiter starts before full-scale clipping so loud sounds rattle less.
const float LIMIT_START = 28000.0f;
const float LIMIT_KNEE_GAIN = 0.20f;

// Channel handlers
i2s_chan_handle_t tx_chan;
i2s_chan_handle_t rx_chan;

// --- VOLUME CONTROL STATE ---
int volumeLevel = 5; // Scale: 0 to 10
const unsigned long debounceDelay = 250;

// --- DSP STATE ---
float previousInput = 0.0f;
float highPassState = 0.0f;
float lowPassState = 0.0f;
float envelope = 0.0f;
float gateGain = 1.0f;

void setupButtons() {
  pinMode(VOL_UP_PIN, INPUT_PULLUP);
  pinMode(VOL_DOWN_PIN, INPUT_PULLUP);
}

void updateVolumeButtons() {
  static int previousUpState = HIGH;
  static int previousDownState = HIGH;
  static unsigned long lastButtonTime = 0;

  const unsigned long now = millis();
  const int upState = digitalRead(VOL_UP_PIN);
  const int downState = digitalRead(VOL_DOWN_PIN);

  if (now - lastButtonTime > debounceDelay) {
    if (previousUpState == HIGH && upState == LOW && volumeLevel < 10) {
      volumeLevel++;
      lastButtonTime = now;
    }

    if (previousDownState == HIGH && downState == LOW && volumeLevel > 0) {
      volumeLevel--;
      lastButtonTime = now;
    }
  }

  previousUpState = upState;
  previousDownState = downState;
}

float softLimit(float sample) {
  if (sample > LIMIT_START) {
    sample = LIMIT_START + (sample - LIMIT_START) * LIMIT_KNEE_GAIN;
  } else if (sample < -LIMIT_START) {
    sample = -LIMIT_START + (sample + LIMIT_START) * LIMIT_KNEE_GAIN;
  }

  if (sample > 32767.0f) return 32767.0f;
  if (sample < -32768.0f) return -32768.0f;
  return sample;
}

int16_t cleanVoiceSample(int16_t rawSample, float gain) {
  const float input = (float)rawSample;

  // DC blocker / high-pass filter.
  const float highPassed = input - previousInput + HPF_COEFF * highPassState;
  previousInput = input;
  highPassState = highPassed;

  // Gentle low-pass filter for hiss and digital hash.
  lowPassState += LOWPASS_ALPHA * (highPassed - lowPassState);

  // Smooth envelope follower for a click-free noise gate.
  const float level = fabsf(lowPassState);
  const float envelopeAlpha = level > envelope ? 0.20f : 0.01f;
  envelope += envelopeAlpha * (level - envelope);

  float targetGateGain = gateGain;
  if (envelope > GATE_OPEN_THRESHOLD) {
    targetGateGain = 1.0f;
  } else if (envelope < GATE_CLOSE_THRESHOLD) {
    targetGateGain = CLOSED_GATE_GAIN;
  }

  const float gateAlpha = targetGateGain > gateGain ? 0.08f : 0.003f;
  gateGain += gateAlpha * (targetGateGain - gateGain);

  const float cleaned = softLimit(lowPassState * gain * gateGain);
  return (int16_t)cleaned;
}

void setupI2S() {
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  i2s_new_channel(&chan_cfg, &tx_chan, &rx_chan);

  i2s_std_config_t std_cfg = {
    .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
    .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
    .gpio_cfg = {
      .mclk = I2S_GPIO_UNUSED,
      .bclk = (gpio_num_t)I2S_BCLK_PIN,
      .ws   = (gpio_num_t)I2S_WS_PIN,
      .dout = (gpio_num_t)DAC_DIN_PIN,
      .din  = (gpio_num_t)MIC_SD_PIN,
    },
  };

  i2s_channel_init_std_mode(tx_chan, &std_cfg);
  i2s_channel_init_std_mode(rx_chan, &std_cfg);

  i2s_channel_enable(tx_chan);
  i2s_channel_enable(rx_chan);
}

void setup() {
  Serial.begin(115200);
  setupButtons();
  setupI2S();

  Serial.println("Clean Hearing Booster Online");
  Serial.println("If the mic is silent, change MIC_CHANNEL_INDEX from 0 to 1.");
}

void loop() {
  int16_t audioBuffer[BUFFER_FRAMES * 2]; // Stereo: left/right interleaved
  size_t bytesRead = 0;
  size_t bytesWritten = 0;

  i2s_channel_read(rx_chan, audioBuffer, sizeof(audioBuffer), &bytesRead, portMAX_DELAY);
  updateVolumeButtons();

  const float gain = volumeLevel * 0.2f;
  const int framesToProcess = bytesRead / (sizeof(int16_t) * 2);

  for (int frame = 0; frame < framesToProcess; frame++) {
    const int sourceIndex = frame * 2 + MIC_CHANNEL_INDEX;
    const int16_t cleanedSample = cleanVoiceSample(audioBuffer[sourceIndex], gain);

    // Feed the same cleaned mono voice to both headphone channels.
    audioBuffer[frame * 2] = cleanedSample;
    audioBuffer[frame * 2 + 1] = cleanedSample;
  }

  const size_t bytesToWrite = framesToProcess * 2 * sizeof(int16_t);
  i2s_channel_write(tx_chan, audioBuffer, bytesToWrite, &bytesWritten, portMAX_DELAY);

  static int lastPrintedVolume = -1;
  if (volumeLevel != lastPrintedVolume) {
    Serial.printf("Current Volume Level: %d / 10 (Gain: %.1fx)\n", volumeLevel, gain);
    lastPrintedVolume = volumeLevel;
  }
}
