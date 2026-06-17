#include <Arduino.h>
#include <driver/i2s_std.h>
#include <math.h>

// ============================================================
// ESP32-N16R8 FOUR-MIC DIRECTIONAL HEARING BOOSTER
//
// Hardware:
// - 4x INMP441 / ICS-43434 I2S MEMS microphones
// - Front-left  mic L/R -> GND,  SD -> MIC_FRONT_SD_PIN
// - Front-right mic L/R -> 3.3V, SD -> MIC_FRONT_SD_PIN
// - Back-left   mic L/R -> GND,  SD -> MIC_BACK_SD_PIN
// - Back-right  mic L/R -> 3.3V, SD -> MIC_BACK_SD_PIN
// - All microphones share BCLK and WS/LRCLK
// - UDA1334A I2S DAC for headphone output
//
// I2S layout:
// - I2S0 master full-duplex: front mic pair in, DAC out, generates BCLK/WS
// - I2S1 slave RX: back mic pair in, listens to the same BCLK/WS
//
// Goal:
// - Favor the direction with the strongest voice-like signal
// - Amplify speech clearly for a person who does not hear well
// - Reduce rumble, hiss, steady room sound, side distractions, and harsh peaks
// ============================================================

// --- PIN DEFINITIONS ---
#define MIC_FRONT_SD_PIN 15  // front-left + front-right pair
#define MIC_BACK_SD_PIN  19  // back-left + back-right pair
#define DAC_DIN_PIN      18  // UDA1334A DIN
#define I2S_BCLK_PIN     16  // shared BCLK/SCK
#define I2S_WS_PIN       17  // shared WS/LRCLK

#define VOL_UP_PIN        4
#define VOL_DOWN_PIN      5

// --- AUDIO CONFIGURATION ---
#define SAMPLE_RATE      44100
#define BUFFER_SIZE      64  // about 1.45 ms at 44.1 kHz

// If a mic pair is reversed, change the matching value to 1.
#define SWAP_FRONT_LR     0
#define SWAP_BACK_LR      0

// --- VOICE / DIRECTION TUNING ---
const float VOICE_HIGHPASS_HZ = 120.0f;
const float VOICE_LOWPASS_HZ = 7200.0f;
const float PRESENCE_HZ = 2600.0f;
const float PRESENCE_GAIN_DB = 3.0f;

// Direction focus:
// - Higher = stronger focus on the loudest speaking direction
// - Lower = more natural room/stereo sound
const float DIRECTION_FOCUS_POWER = 2.2f;
const float DIRECTION_MIN_WEIGHT = 0.16f;
const float DIRECTION_ATTACK = 0.10f;
const float DIRECTION_RELEASE = 0.015f;

// Final stereo image:
// 0.0 = clean mono in both ears, best clarity
// 1.0 = stronger left/right spatial feeling
const float OUTPUT_STEREO_WIDTH = 0.20f;

// Adaptive noise reducer. It lowers steady background sound but never fully
// mutes, so syllables are less likely to chop.
const float NOISE_FLOOR_START = 180.0f;
const float NOISE_FLOOR_MIN = 60.0f;
const float NOISE_REDUCTION_MIN_GAIN = 0.22f;
const float NOISE_OPEN_MULTIPLIER = 1.7f;
const float NOISE_FULL_MULTIPLIER = 5.0f;

// Hearing-booster AGC/compressor.
const float TARGET_SPEECH_LEVEL = 7600.0f;
const float MAX_AGC_GAIN = 3.8f;
const float COMPRESSOR_THRESHOLD = 12500.0f;
const float COMPRESSOR_RATIO = 3.2f;

// Final DAC protection / anti-rattle limiter.
const float LIMIT_START = 26000.0f;
const float LIMIT_KNEE_GAIN = 0.16f;

// --- I2S HANDLES ---
i2s_chan_handle_t tx_chan = NULL;
i2s_chan_handle_t rx_front_chan = NULL;
i2s_chan_handle_t rx_back_chan = NULL;

// --- VOLUME CONTROL ---
volatile bool volumeUpRequested = false;
volatile bool volumeDownRequested = false;
int volumeLevel = 5; // 0 to 10
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 220;

// --- FILTERS ---
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

enum MicIndex {
  MIC_FL = 0,
  MIC_FR = 1,
  MIC_BL = 2,
  MIC_BR = 3,
  MIC_COUNT = 4
};

ChannelDSP micDSP[MIC_COUNT];

// Small block buffers. Keeping these global avoids putting them on the stack.
float micFrame[MIC_COUNT][BUFFER_SIZE];
float micEnergy[MIC_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
float directionWeight[MIC_COUNT] = {1.0f, 1.0f, 1.0f, 1.0f};

// Linked control states.
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

  setBiquad(filter,
            (1.0f + cw) * 0.5f,
            -(1.0f + cw),
            (1.0f + cw) * 0.5f,
            1.0f + alpha,
            -2.0f * cw,
            1.0f - alpha);
}

void setLowpass(Biquad &filter, float frequencyHz, float q) {
  const float w0 = 2.0f * PI * frequencyHz / SAMPLE_RATE;
  const float cw = cosf(w0);
  const float sw = sinf(w0);
  const float alpha = sw / (2.0f * q);

  setBiquad(filter,
            (1.0f - cw) * 0.5f,
            1.0f - cw,
            (1.0f - cw) * 0.5f,
            1.0f + alpha,
            -2.0f * cw,
            1.0f - alpha);
}

void setPresenceBoost(Biquad &filter, float frequencyHz, float q, float gainDb) {
  const float a = powf(10.0f, gainDb / 40.0f);
  const float w0 = 2.0f * PI * frequencyHz / SAMPLE_RATE;
  const float cw = cosf(w0);
  const float sw = sinf(w0);
  const float alpha = sw / (2.0f * q);

  setBiquad(filter,
            1.0f + alpha * a,
            -2.0f * cw,
            1.0f - alpha * a,
            1.0f + alpha / a,
            -2.0f * cw,
            1.0f - alpha / a);
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
    t = t * t * (3.0f - 2.0f * t);
    targetGain = NOISE_REDUCTION_MIN_GAIN + (1.0f - NOISE_REDUCTION_MIN_GAIN) * t;
  }

  const float smoothing = targetGain > expanderGain ? 0.045f : 0.0025f;
  expanderGain += smoothing * (targetGain - expanderGain);
  return expanderGain;
}

float calculateAgcGain(float level) {
  float targetGain = 1.0f;

  if (level > noiseFloor * 2.2f) {
    targetGain = TARGET_SPEECH_LEVEL / maxFloat(level, 1.0f);
    targetGain = clampFloat(targetGain, 0.45f, MAX_AGC_GAIN);
  }

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

void updateDirectionWeights(int frames) {
  float maxEnergy = 1.0f;

  for (int mic = 0; mic < MIC_COUNT; mic++) {
    float sumSquares = 0.0f;
    for (int i = 0; i < frames; i++) {
      sumSquares += micFrame[mic][i] * micFrame[mic][i];
    }

    micEnergy[mic] = sqrtf(sumSquares / maxFloat((float)frames, 1.0f));
    if (micEnergy[mic] > maxEnergy) {
      maxEnergy = micEnergy[mic];
    }
  }

  for (int mic = 0; mic < MIC_COUNT; mic++) {
    const float normalizedEnergy = clampFloat(micEnergy[mic] / maxEnergy, 0.0f, 1.0f);
    float targetWeight = DIRECTION_MIN_WEIGHT +
                         (1.0f - DIRECTION_MIN_WEIGHT) *
                         powf(normalizedEnergy, DIRECTION_FOCUS_POWER);

    const float smoothing = targetWeight > directionWeight[mic] ? DIRECTION_ATTACK : DIRECTION_RELEASE;
    directionWeight[mic] += smoothing * (targetWeight - directionWeight[mic]);
  }
}

void processFourMicFrame(int frame, float &outLeft, float &outRight) {
  const float fl = micFrame[MIC_FL][frame] * directionWeight[MIC_FL];
  const float fr = micFrame[MIC_FR][frame] * directionWeight[MIC_FR];
  const float bl = micFrame[MIC_BL][frame] * directionWeight[MIC_BL];
  const float br = micFrame[MIC_BR][frame] * directionWeight[MIC_BR];

  const float leftWeight = directionWeight[MIC_FL] + directionWeight[MIC_BL] + 0.001f;
  const float rightWeight = directionWeight[MIC_FR] + directionWeight[MIC_BR] + 0.001f;
  const float totalWeight = leftWeight + rightWeight;

  const float leftFocused = (fl + bl) / leftWeight;
  const float rightFocused = (fr + br) / rightWeight;
  const float monoFocused = (fl + fr + bl + br) / totalWeight;

  // Mostly mono for clarity, with a small stereo cue left in.
  float left = monoFocused * (1.0f - OUTPUT_STEREO_WIDTH) + leftFocused * OUTPUT_STEREO_WIDTH;
  float right = monoFocused * (1.0f - OUTPUT_STEREO_WIDTH) + rightFocused * OUTPUT_STEREO_WIDTH;

  const float level = 0.5f * (fabsf(left) + fabsf(right));
  updateNoiseFloor(level);

  const float noiseGain = calculateExpanderGain(level);
  const float boosterGain = calculateAgcGain(level);
  const float userGain = volumeLevel == 0 ? 0.0f : (0.35f + 0.17f * volumeLevel);

  const float predictedPeak = maxFloat(fabsf(left), fabsf(right)) *
                              noiseGain * boosterGain * userGain;
  const float loudSoundGain = calculateCompressorGain(predictedPeak);
  const float finalGain = noiseGain * boosterGain * loudSoundGain * userGain;

  outLeft = softLimit(left * finalGain);
  outRight = softLimit(right * finalGain);
}

void setupI2S() {
  // I2S1: slave RX only. It listens to the same BCLK/WS pins and reads the
  // back mic pair on its own SD pin. Configure it before the master so the
  // master is the last peripheral to set BCLK/WS as driven outputs.
  i2s_chan_config_t back_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_SLAVE);
  i2s_new_channel(&back_chan_cfg, NULL, &rx_back_chan);

  i2s_std_config_t back_std_cfg = {
    .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
    .gpio_cfg = {
      .mclk = I2S_GPIO_UNUSED,
      .bclk = (gpio_num_t)I2S_BCLK_PIN,
      .ws   = (gpio_num_t)I2S_WS_PIN,
      .dout = I2S_GPIO_UNUSED,
      .din  = (gpio_num_t)MIC_BACK_SD_PIN,
    },
  };

  i2s_channel_init_std_mode(rx_back_chan, &back_std_cfg);

  // I2S0: master full-duplex. It drives BCLK/WS, reads the front pair,
  // and writes processed output to the UDA1334A DAC.
  i2s_chan_config_t front_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  i2s_new_channel(&front_chan_cfg, &tx_chan, &rx_front_chan);

  i2s_std_config_t front_std_cfg = {
    .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
    .gpio_cfg = {
      .mclk = I2S_GPIO_UNUSED,
      .bclk = (gpio_num_t)I2S_BCLK_PIN,
      .ws   = (gpio_num_t)I2S_WS_PIN,
      .dout = (gpio_num_t)DAC_DIN_PIN,
      .din  = (gpio_num_t)MIC_FRONT_SD_PIN,
    },
  };

  i2s_channel_init_std_mode(tx_chan, &front_std_cfg);
  i2s_channel_init_std_mode(rx_front_chan, &front_std_cfg);

  // Enable the slave receiver first so it is ready when I2S0 starts clocks.
  i2s_channel_enable(rx_back_chan);
  i2s_channel_enable(tx_chan);
  i2s_channel_enable(rx_front_chan);
}

void setup() {
  Serial.begin(115200);
  delay(300);

  for (int mic = 0; mic < MIC_COUNT; mic++) {
    configureVoiceFilters(micDSP[mic]);
  }

  pinMode(VOL_UP_PIN, INPUT_PULLUP);
  pinMode(VOL_DOWN_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(VOL_UP_PIN), handleVolumeUp, FALLING);
  attachInterrupt(digitalPinToInterrupt(VOL_DOWN_PIN), handleVolumeDown, FALLING);

  setupI2S();

  Serial.println("ESP32-N16R8 Four-Mic Directional Hearing Booster Online");
  Serial.println("Front SD=GPIO15, Back SD=GPIO19, BCLK=GPIO16, WS=GPIO17, DAC DIN=GPIO18");
}

void loop() {
  int16_t frontBuffer[BUFFER_SIZE * 2]; // front-left, front-right, ...
  int16_t backBuffer[BUFFER_SIZE * 2];  // back-left, back-right, ...
  int16_t outputBuffer[BUFFER_SIZE * 2];

  size_t frontBytesRead = 0;
  size_t backBytesRead = 0;
  size_t bytesWritten = 0;

  i2s_channel_read(rx_front_chan, frontBuffer, sizeof(frontBuffer), &frontBytesRead, portMAX_DELAY);
  i2s_channel_read(rx_back_chan, backBuffer, sizeof(backBuffer), &backBytesRead, portMAX_DELAY);
  updateVolumeButtons();

  const int frontFrames = frontBytesRead / (sizeof(int16_t) * 2);
  const int backFrames = backBytesRead / (sizeof(int16_t) * 2);
  const int frames = frontFrames < backFrames ? frontFrames : backFrames;

  for (int frame = 0; frame < frames; frame++) {
    int frontLeftIndex = frame * 2;
    int frontRightIndex = frontLeftIndex + 1;
    int backLeftIndex = frame * 2;
    int backRightIndex = backLeftIndex + 1;

#if SWAP_FRONT_LR
    int tempFront = frontLeftIndex;
    frontLeftIndex = frontRightIndex;
    frontRightIndex = tempFront;
#endif

#if SWAP_BACK_LR
    int tempBack = backLeftIndex;
    backLeftIndex = backRightIndex;
    backRightIndex = tempBack;
#endif

    micFrame[MIC_FL][frame] = processVoiceBand(micDSP[MIC_FL], (float)frontBuffer[frontLeftIndex]);
    micFrame[MIC_FR][frame] = processVoiceBand(micDSP[MIC_FR], (float)frontBuffer[frontRightIndex]);
    micFrame[MIC_BL][frame] = processVoiceBand(micDSP[MIC_BL], (float)backBuffer[backLeftIndex]);
    micFrame[MIC_BR][frame] = processVoiceBand(micDSP[MIC_BR], (float)backBuffer[backRightIndex]);
  }

  updateDirectionWeights(frames);

  for (int frame = 0; frame < frames; frame++) {
    float processedLeft = 0.0f;
    float processedRight = 0.0f;

    processFourMicFrame(frame, processedLeft, processedRight);

    outputBuffer[frame * 2] = (int16_t)processedLeft;
    outputBuffer[frame * 2 + 1] = (int16_t)processedRight;
  }

  i2s_channel_write(tx_chan, outputBuffer, frames * 2 * sizeof(int16_t), &bytesWritten, portMAX_DELAY);

  static unsigned long lastDebugPrint = 0;
  const unsigned long now = millis();
  if (now - lastDebugPrint > 1000) {
    Serial.printf("Vol:%d Noise:%.1f W FL:%.2f FR:%.2f BL:%.2f BR:%.2f AGC:%.2f Comp:%.2f\n",
                  volumeLevel,
                  noiseFloor,
                  directionWeight[MIC_FL],
                  directionWeight[MIC_FR],
                  directionWeight[MIC_BL],
                  directionWeight[MIC_BR],
                  agcGain,
                  compressorGain);
    lastDebugPrint = now;
  }
}
