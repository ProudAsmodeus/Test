#include <Arduino.h>
#include <driver/i2s_std.h>
#include <math.h>

// ============================================================
// ESP32-N16R8 TWO-MIC HEARING BOOSTER
// Hardware:
// - Two INMP441 / ICS-43434 I2S microphones sharing SD/BCLK/WS
// - Left mic L/R pin  -> GND
// - Right mic L/R pin -> 3.3V
// - UDA1334A I2S DAC for headphone output
// Goal:
// - Clear amplified talking voice
// - Less low rumble, hiss, steady room noise, and sudden harsh peaks
// ============================================================

// --- PIN DEFINITIONS ---
#define MIC_SD_PIN     15  // Shared SD/DOUT from both I2S microphones
#define DAC_DIN_PIN    18  // UDA1334A DIN
#define I2S_BCLK_PIN   16  // Shared BCLK/SCK
#define I2S_WS_PIN     17  // Shared WS/LRCLK

#define VOL_UP_PIN      4
#define VOL_DOWN_PIN    5

// --- AUDIO CONFIGURATION ---
#define SAMPLE_RATE    44100
#define BUFFER_SIZE    64  // Low latency: about 1.45 ms at 44.1 kHz

// If left/right headphones sound swapped, change this to 1.
#define SWAP_MIC_CHANNELS 0

// --- PROFESSIONAL VOICE TUNING ---
// Voice band: removes rumble and very high hiss while preserving speech clarity.
const float VOICE_HIGHPASS_HZ = 120.0f;
const float VOICE_LOWPASS_HZ = 7200.0f;
const float PRESENCE_HZ = 2600.0f;
const float PRESENCE_GAIN_DB = 3.0f;

// Mid/side focus: reduces sound that is very different between left and right.
// 1.0 = normal stereo, 0.0 = mono center focus. Start around 0.45-0.70.
const float SIDE_KEEP = 0.55f;

// Adaptive noise reducer. It never mutes fully, so speech does not chop.
const float NOISE_FLOOR_START = 180.0f;
const float NOISE_FLOOR_MIN = 60.0f;
const float NOISE_REDUCTION_MIN_GAIN = 0.22f;
const float NOISE_OPEN_MULTIPLIER = 1.7f;
const float NOISE_FULL_MULTIPLIER = 5.0f;

// Hearing-booster AGC/compressor.
const float TARGET_SPEECH_LEVEL = 7200.0f;   // Higher = louder quiet voices
const float MAX_AGC_GAIN = 3.8f;             // Prevents amplifying room noise too much
const float COMPRESSOR_THRESHOLD = 12500.0f; // Loud sound control starts here
const float COMPRESSOR_RATIO = 3.2f;         // Higher = stronger loud sound control

// Final DAC protection / anti-rattle limiter.
const float LIMIT_START = 26000.0f;
const float LIMIT_KNEE_GAIN = 0.16f;

// --- I2S HANDLES ---
i2s_chan_handle_t tx_chan = NULL;
i2s_chan_handle_t rx_chan = NULL;

// --- VOLUME CONTROL ---
volatile bool volumeUpRequested = false;
volatile bool volumeDownRequested = false;
int volumeLevel = 5; // 0 to 10
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 220;

// --- BIQUAD FILTER ---
struct Biquad {
  float b0 = 1.0f;
  float b1 = 0.0f;
  float b2 = 0.0f;
  float a1 = 0.0f;
  float a2 = 0.0f;
  float z1 = 0.0f;
  float z2 = 0.0f;
};

struct ChannelDSP {
  Biquad highpass;
  Biquad lowpass;
  Biquad presence;
};

ChannelDSP leftDSP;
ChannelDSP rightDSP;

// Linked stereo control states. Linked processing avoids the image jumping
// left/right while the AGC and noise reducer work.
float noiseFloor = NOISE_FLOOR_START;
float expanderGain = 1.0f;
float agcGain = 1.0f;
float compressorGain = 1.0f;

void IRAM_ATTR handleVolumeUp() {
  volumeUpRequested = true;
}

void IRAM_ATTR handleVolumeDown() {
  volumeDownRequested = true;
}

float clampFloat(float value, float low, float high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

float maxFloat(float a, float b) {
  return a > b ? a : b;
}

void setBiquad(Biquad &filter, float b0, float b1, float b2, float a0, float a1, float a2) {
  filter.b0 = b0 / a0;
  filter.b1 = b1 / a0;
  filter.b2 = b2 / a0;
  filter.a1 = a1 / a0;
  filter.a2 = a2 / a0;
  filter.z1 = 0.0f;
  filter.z2 = 0.0f;
}

void setHighpass(Biquad &filter, float frequencyHz, float q) {
  const float w0 = 2.0f * PI * frequencyHz / SAMPLE_RATE;
  const float cw = cosf(w0);
  const float sw = sinf(w0);
  const float alpha = sw / (2.0f * q);

  const float b0 = (1.0f + cw) * 0.5f;
  const float b1 = -(1.0f + cw);
  const float b2 = (1.0f + cw) * 0.5f;
  const float a0 = 1.0f + alpha;
  const float a1 = -2.0f * cw;
  const float a2 = 1.0f - alpha;

  setBiquad(filter, b0, b1, b2, a0, a1, a2);
}

void setLowpass(Biquad &filter, float frequencyHz, float q) {
  const float w0 = 2.0f * PI * frequencyHz / SAMPLE_RATE;
  const float cw = cosf(w0);
  const float sw = sinf(w0);
  const float alpha = sw / (2.0f * q);

  const float b0 = (1.0f - cw) * 0.5f;
  const float b1 = 1.0f - cw;
  const float b2 = (1.0f - cw) * 0.5f;
  const float a0 = 1.0f + alpha;
  const float a1 = -2.0f * cw;
  const float a2 = 1.0f - alpha;

  setBiquad(filter, b0, b1, b2, a0, a1, a2);
}

void setPresenceBoost(Biquad &filter, float frequencyHz, float q, float gainDb) {
  const float a = powf(10.0f, gainDb / 40.0f);
  const float w0 = 2.0f * PI * frequencyHz / SAMPLE_RATE;
  const float cw = cosf(w0);
  const float sw = sinf(w0);
  const float alpha = sw / (2.0f * q);

  const float b0 = 1.0f + alpha * a;
  const float b1 = -2.0f * cw;
  const float b2 = 1.0f - alpha * a;
  const float a0 = 1.0f + alpha / a;
  const float a1 = -2.0f * cw;
  const float a2 = 1.0f - alpha / a;

  setBiquad(filter, b0, b1, b2, a0, a1, a2);
}

float processBiquad(Biquad &filter, float input) {
  const float output = filter.b0 * input + filter.z1;
  filter.z1 = filter.b1 * input - filter.a1 * output + filter.z2;
  filter.z2 = filter.b2 * input - filter.a2 * output;
  return output;
}

void configureVoiceFilters(ChannelDSP &channel) {
  setHighpass(channel.highpass, VOICE_HIGHPASS_HZ, 0.707f);
  setLowpass(channel.lowpass, VOICE_LOWPASS_HZ, 0.707f);
  setPresenceBoost(channel.presence, PRESENCE_HZ, 1.0f, PRESENCE_GAIN_DB);
}

float processVoiceBand(ChannelDSP &channel, float sample) {
  sample = processBiquad(channel.highpass, sample);
  sample = processBiquad(channel.lowpass, sample);
  sample = processBiquad(channel.presence, sample);
  return sample;
}

void updateVolumeButtons() {
  bool up = false;
  bool down = false;

  noInterrupts();
  up = volumeUpRequested;
  down = volumeDownRequested;
  volumeUpRequested = false;
  volumeDownRequested = false;
  interrupts();

  const unsigned long now = millis();
  if (now - lastDebounceTime <= debounceDelay) return;

  if (up && volumeLevel < 10) {
    volumeLevel++;
    lastDebounceTime = now;
  } else if (down && volumeLevel > 0) {
    volumeLevel--;
    lastDebounceTime = now;
  }
}

void updateNoiseFloor(float level) {
  // Track steady low-level background faster than sudden voice peaks.
  const float speechGuard = noiseFloor * 2.8f;
  if (level < speechGuard) {
    noiseFloor = 0.996f * noiseFloor + 0.004f * level;
  } else {
    noiseFloor = 0.9997f * noiseFloor + 0.0003f * level;
  }

  if (noiseFloor < NOISE_FLOOR_MIN) {
    noiseFloor = NOISE_FLOOR_MIN;
  }
}

float calculateExpanderGain(float level) {
  const float openLevel = maxFloat(NOISE_FLOOR_MIN, noiseFloor * NOISE_OPEN_MULTIPLIER);
  const float fullLevel = maxFloat(openLevel + 1.0f, noiseFloor * NOISE_FULL_MULTIPLIER);

  float targetGain = 1.0f;
  if (level <= openLevel) {
    targetGain = NOISE_REDUCTION_MIN_GAIN;
  } else if (level < fullLevel) {
    float t = (level - openLevel) / (fullLevel - openLevel);
    t = t * t * (3.0f - 2.0f * t); // smoothstep
    targetGain = NOISE_REDUCTION_MIN_GAIN + (1.0f - NOISE_REDUCTION_MIN_GAIN) * t;
  }

  // Open quickly for speech, close slowly to avoid choppy syllables.
  const float smoothing = targetGain > expanderGain ? 0.045f : 0.0025f;
  expanderGain += smoothing * (targetGain - expanderGain);
  return expanderGain;
}

float calculateAgcGain(float level) {
  float targetGain = 1.0f;

  // Only boost when the signal is clearly above the learned background.
  if (level > noiseFloor * 2.2f) {
    targetGain = TARGET_SPEECH_LEVEL / maxFloat(level, 1.0f);
    targetGain = clampFloat(targetGain, 0.45f, MAX_AGC_GAIN);
  }

  // Reduce gain fast for loud sounds, restore slowly for comfort.
  const float smoothing = targetGain < agcGain ? 0.035f : 0.0012f;
  agcGain += smoothing * (targetGain - agcGain);
  return agcGain;
}

float calculateCompressorGain(float predictedPeak) {
  float targetGain = 1.0f;

  if (predictedPeak > COMPRESSOR_THRESHOLD) {
    const float compressedPeak = COMPRESSOR_THRESHOLD +
                                 (predictedPeak - COMPRESSOR_THRESHOLD) / COMPRESSOR_RATIO;
    targetGain = compressedPeak / predictedPeak;
  }

  const float smoothing = targetGain < compressorGain ? 0.080f : 0.0035f;
  compressorGain += smoothing * (targetGain - compressorGain);
  return compressorGain;
}

float softLimit(float sample) {
  if (sample > LIMIT_START) {
    sample = LIMIT_START + (sample - LIMIT_START) * LIMIT_KNEE_GAIN;
  } else if (sample < -LIMIT_START) {
    sample = -LIMIT_START + (sample + LIMIT_START) * LIMIT_KNEE_GAIN;
  }

  return clampFloat(sample, -32768.0f, 32767.0f);
}

void processStereoFrame(float rawLeft, float rawRight, float &outLeft, float &outRight) {
#if SWAP_MIC_CHANNELS
  float temp = rawLeft;
  rawLeft = rawRight;
  rawRight = temp;
#endif

  float left = processVoiceBand(leftDSP, rawLeft);
  float right = processVoiceBand(rightDSP, rawRight);

  // Mid/side speech focus. Common centered speech stays strong; side-heavy
  // distractions are reduced but not removed completely.
  const float mid = 0.5f * (left + right);
  const float side = 0.5f * (left - right) * SIDE_KEEP;
  left = mid + side;
  right = mid - side;

  const float level = 0.5f * (fabsf(left) + fabsf(right));
  updateNoiseFloor(level);

  const float noiseGain = calculateExpanderGain(level);
  const float boosterGain = calculateAgcGain(level);
  const float userGain = volumeLevel == 0 ? 0.0f : (0.35f + 0.17f * volumeLevel);

  float predictedPeak = maxFloat(fabsf(left), fabsf(right)) * noiseGain * boosterGain * userGain;
  const float loudSoundGain = calculateCompressorGain(predictedPeak);

  const float finalGain = noiseGain * boosterGain * loudSoundGain * userGain;
  outLeft = softLimit(left * finalGain);
  outRight = softLimit(right * finalGain);
}

void setup() {
  Serial.begin(115200);
  delay(300);

  configureVoiceFilters(leftDSP);
  configureVoiceFilters(rightDSP);

  pinMode(VOL_UP_PIN, INPUT_PULLUP);
  pinMode(VOL_DOWN_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(VOL_UP_PIN), handleVolumeUp, FALLING);
  attachInterrupt(digitalPinToInterrupt(VOL_DOWN_PIN), handleVolumeDown, FALLING);

  // 1. Full-duplex I2S channel allocation.
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  i2s_new_channel(&chan_cfg, &tx_chan, &rx_chan);

  // 2. Stereo I2S:
  //    left mic  L/R = GND
  //    right mic L/R = 3.3V
  i2s_std_config_t std_cfg = {
    .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
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

  Serial.println("ESP32-N16R8 Professional Two-Mic Hearing Booster Online");
  Serial.println("Left mic L/R=GND, Right mic L/R=3.3V, shared SD/BCLK/WS");
}

void loop() {
  int16_t audioBuffer[BUFFER_SIZE * 2]; // Interleaved stereo: L, R, L, R...
  size_t bytesRead = 0;
  size_t bytesWritten = 0;

  i2s_channel_read(rx_chan, audioBuffer, sizeof(audioBuffer), &bytesRead, portMAX_DELAY);
  updateVolumeButtons();

  const int totalSamples = bytesRead / sizeof(int16_t);
  const int frames = totalSamples / 2;

  for (int frame = 0; frame < frames; frame++) {
    const int leftIndex = frame * 2;
    const int rightIndex = leftIndex + 1;

    float processedLeft = 0.0f;
    float processedRight = 0.0f;

    processStereoFrame((float)audioBuffer[leftIndex],
                       (float)audioBuffer[rightIndex],
                       processedLeft,
                       processedRight);

    audioBuffer[leftIndex] = (int16_t)processedLeft;
    audioBuffer[rightIndex] = (int16_t)processedRight;
  }

  i2s_channel_write(tx_chan, audioBuffer, frames * 2 * sizeof(int16_t), &bytesWritten, portMAX_DELAY);

  static unsigned long lastDebugPrint = 0;
  const unsigned long now = millis();
  if (now - lastDebugPrint > 1000) {
    Serial.printf("Vol:%d/10 Noise:%.1f Exp:%.2f AGC:%.2f Comp:%.2f\n",
                  volumeLevel, noiseFloor, expanderGain, agcGain, compressorGain);
    lastDebugPrint = now;
  }
}
