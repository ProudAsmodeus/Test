#include <Arduino.h>
#include <driver/i2s_std.h>

// --- PIN DEFINITIONS FOR YOUR V1 BOARD ---
#define MIC_SD_PIN     15  // INMP441/ICS-43434 SD/DOUT
#define DAC_DIN_PIN    18  // UDA1334A DIN
#define I2S_BCLK_PIN   16  // Shared BCLK/SCK
#define I2S_WS_PIN     17  // Shared WS/LRCLK

#define VOL_UP_PIN      4
#define VOL_DOWN_PIN    5

// --- AUDIO CONFIGURATION ---
#define SAMPLE_RATE    44100
#define BUFFER_SIZE    64

// --- CLEANUP TUNING ---
// This version intentionally keeps the same 16-bit I2S bus format as your
// original working sketch. The filters process both stereo slots so it does
// not matter whether the INMP441 L/R pin selects left or right.
const float HPF_COEFF = 0.995f;
const float LOWPASS_ALPHA = 0.60f;
const float LIMIT_START = 28000.0f;
const float LIMIT_KNEE_GAIN = 0.20f;

// Channel handlers
i2s_chan_handle_t tx_chan;
i2s_chan_handle_t rx_chan;

// --- VOLUME CONTROL STATE ---
volatile int volumeLevel = 5; // Scale: 0 to 10
volatile unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 250;

// --- FILTER STATE ---
float previousInput[2] = {0.0f, 0.0f};
float highPassState[2] = {0.0f, 0.0f};
float lowPassState[2] = {0.0f, 0.0f};

void IRAM_ATTR handleVolumeUp() {
  unsigned long currentTime = millis();

  if (currentTime - lastDebounceTime > debounceDelay) {
    if (volumeLevel < 10) volumeLevel++;
    lastDebounceTime = currentTime;
  }
}

void IRAM_ATTR handleVolumeDown() {
  unsigned long currentTime = millis();

  if (currentTime - lastDebounceTime > debounceDelay) {
    if (volumeLevel > 0) volumeLevel--;
    lastDebounceTime = currentTime;
  }
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

int16_t cleanSample(int16_t rawSample, int channel, float gain) {
  float input = (float)rawSample;

  // High-pass / DC blocker: removes DC offset, handling noise, and low rumble.
  float highPassed = input - previousInput[channel] + HPF_COEFF * highPassState[channel];
  previousInput[channel] = input;
  highPassState[channel] = highPassed;

  // Gentle low-pass: reduces hiss and sharp digital rattling.
  lowPassState[channel] += LOWPASS_ALPHA * (highPassed - lowPassState[channel]);

  float amplified = lowPassState[channel] * gain;
  amplified = softLimit(amplified);

  return (int16_t)amplified;
}

void setup() {
  Serial.begin(115200);

  pinMode(VOL_UP_PIN, INPUT_PULLUP);
  pinMode(VOL_DOWN_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(VOL_UP_PIN), handleVolumeUp, FALLING);
  attachInterrupt(digitalPinToInterrupt(VOL_DOWN_PIN), handleVolumeDown, FALLING);

  // 1. Setup the Channel Architecture
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  i2s_new_channel(&chan_cfg, &tx_chan, &rx_chan);

  // 2. Configure the same 16-bit I2S format as the original working sketch.
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

  // 3. Initialize both Tx and Rx with the shared pin configuration
  i2s_channel_init_std_mode(tx_chan, &std_cfg);
  i2s_channel_init_std_mode(rx_chan, &std_cfg);

  // 4. Start I2S
  i2s_channel_enable(tx_chan);
  i2s_channel_enable(rx_chan);

  Serial.println("Hearing Booster System Online - Cleaner 16-bit Bus Version");
}

void loop() {
  int16_t audioBuffer[BUFFER_SIZE * 2]; // Stereo buffer: left/right interleaved
  size_t bytesRead = 0;
  size_t bytesWritten = 0;

  i2s_channel_read(rx_chan, audioBuffer, sizeof(audioBuffer), &bytesRead, portMAX_DELAY);

  float gain = volumeLevel * 0.2f;
  int samplesToProcess = bytesRead / sizeof(int16_t);

  for (int i = 0; i < samplesToProcess; i++) {
    int channel = i % 2;
    audioBuffer[i] = cleanSample(audioBuffer[i], channel, gain);
  }

  i2s_channel_write(tx_chan, audioBuffer, bytesRead, &bytesWritten, portMAX_DELAY);

  static int lastPrintedVolume = -1;
  if (volumeLevel != lastPrintedVolume) {
    Serial.printf("Current Volume Level: %d / 10 (Gain: %.1fx)\n", volumeLevel, gain);
    lastPrintedVolume = volumeLevel;
  }
}
