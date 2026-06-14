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
#define BUFFER_FRAMES  64

// INMP441/ICS-43434 mics output 24-bit audio in a 32-bit I2S slot.
// If your mic L/R or SEL pin is wired for right channel, keep this at 1.
// If you get no voice, change it to 0.
#define MIC_CHANNEL_INDEX 1  // 0 = left slot, 1 = right slot

// --- CLEANUP TUNING ---
// Higher = less bass/rumble/hum, but too high makes voices thin.
const float HPF_COEFF = 0.995f;

// Lower = smoother/less hiss, higher = brighter/more natural.
const float LOWPASS_ALPHA = 0.45f;

// Start soft limiting before full-scale clipping to reduce harsh rattling.
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
float previousInput = 0.0f;
float highPassState = 0.0f;
float lowPassState = 0.0f;

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

int16_t cleanVoiceSample(int32_t rawI2SSample, float gain) {
  // 24-bit MEMS mic data is normally left-aligned inside the 32-bit word.
  // Shifting by 16 converts it to a signed 16-bit working sample.
  float input = (float)(rawI2SSample >> 16);

  // High-pass / DC blocker: removes DC offset, handling noise, and low rumble.
  float highPassed = input - previousInput + HPF_COEFF * highPassState;
  previousInput = input;
  highPassState = highPassed;

  // Gentle low-pass: reduces hiss and sharp digital rattling.
  lowPassState += LOWPASS_ALPHA * (highPassed - lowPassState);

  float amplified = lowPassState * gain;
  amplified = softLimit(amplified);

  return (int16_t)amplified;
}

int32_t sample16ToI2S32(int16_t sample) {
  // Put the 16-bit cleaned output in the high bits for the I2S DAC.
  return ((int32_t)sample) << 16;
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

  // 2. Configure 32-bit I2S.
  // This is important for INMP441/ICS-43434 because they are 24-bit I2S mics.
  i2s_std_config_t std_cfg = {
    .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
    .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
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

  Serial.println("Hearing Booster System Online - INMP441/UDA1334A Cleaner Version");
  Serial.println("If there is rattling but no voice, change MIC_CHANNEL_INDEX between 1 and 0.");
}

void loop() {
  int32_t audioBuffer[BUFFER_FRAMES * 2]; // Stereo buffer: left/right interleaved
  size_t bytesRead = 0;
  size_t bytesWritten = 0;

  i2s_channel_read(rx_chan, audioBuffer, sizeof(audioBuffer), &bytesRead, portMAX_DELAY);

  float gain = volumeLevel * 0.2f;
  int framesToProcess = bytesRead / (sizeof(int32_t) * 2);

  for (int frame = 0; frame < framesToProcess; frame++) {
    int micIndex = frame * 2 + MIC_CHANNEL_INDEX;

    int16_t cleanedSample = cleanVoiceSample(audioBuffer[micIndex], gain);
    int32_t outputSample = sample16ToI2S32(cleanedSample);

    // Send the selected mic as clean mono to both DAC/headphone channels.
    audioBuffer[frame * 2] = outputSample;
    audioBuffer[frame * 2 + 1] = outputSample;
  }

  size_t bytesToWrite = framesToProcess * 2 * sizeof(int32_t);
  i2s_channel_write(tx_chan, audioBuffer, bytesToWrite, &bytesWritten, portMAX_DELAY);

  static int lastPrintedVolume = -1;
  if (volumeLevel != lastPrintedVolume) {
    Serial.printf("Current Volume Level: %d / 10 (Gain: %.1fx)\n", volumeLevel, gain);
    lastPrintedVolume = volumeLevel;
  }
}
