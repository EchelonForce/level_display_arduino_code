#include <avr/io.h>

// Change this to speed up/slow down the initialization tests.
#define TIME_BETWEEN_TEST_STEPS 100
// Change this to 1 if the display is upside down, otherwise set to 0.
#define UPSIDE_DOWN 0
//Time between LED updates, samples are still gathered [ms]
#define LED_UPDATE_PERIOD 1
// How many samples to allow before decreasing the saved peak value. Decrease for faster peak update.
#define SAMPLES_PER_PEAK_DECREMENT 1000

// Number of digitial pins connected to LEDs. Rework required if changed.
#define PIN_CNT 16

void setup()
{
  static const uint8_t pins[] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, A0, A1, A2, A3, A5}; // A5 for timing see below
  //PD2,PD3,PD4,PD5,PD6,PD7,  PB0,PB1,PB2,PB3,PB4,PB5  PC0,PC1,PC2,PC3,  PC5

  for (byte i = 0; i <= PIN_CNT; i++)
  {
    pinMode(pins[i], OUTPUT);
  }

  PORTC = (PORTC & ~0x20); //set timing pin 0
  Serial.begin(115200);

  led_test();
  adc_setup();
}

/**
 * Setup the ADC hardware for continuous measurement with an interrupt.
 */
void adc_setup(void)
{
  ADCSRA = 0;
  ADCSRB = 0;
  ADMUX |= (6 & 0x07);   // Start conversion with A6 analog input pin
  ADMUX |= (1 << REFS0); // set reference voltage
  // The prsecaler chosen below limits the ADC to less than 10 bits.
  // We'll use 8bits, so align the ADC so we can read that in a single instruction.
  ADMUX |= (1 << ADLAR); // left align ADC value to 8 bits from ADCH register

  // sampling rate is [ADC clock] / [prescaler] / [conversion clock cycles]
  // for Arduino Uno ADC clock is 16 MHz and a conversion takes 13 clock cycles
  ADCSRA |= (1 << ADPS2) | (1 << ADPS0); // 32 prescaler for 38.5 KHz
  ADCSRA |= (1 << ADATE);                // enable auto trigger
  ADCSRA |= (1 << ADIE);                 // enable interrupts when measurement complete
  ADCSRA |= (1 << ADEN);                 // enable ADC
  ADCSRA |= (1 << ADSC);                 // start ADC measurements
}

//These vars used in ADC ISR and loop().
static uint8_t latest_sample = 0;
static uint8_t numSamples = 0;
/**
 * Interrupt handler for the ADC.
 */
ISR(ADC_vect)
{
  ADMUX = ADMUX ^ 0x01; //swap channels 6/7 every time, seems to limit signal for some reason
  if (numSamples > 0)
  {
    latest_sample = max(latest_sample, ADCH); // read 8 bit value from ADC
  }
  else
  {
    latest_sample = ADCH; // read 8 bit value from ADC
  }
  numSamples++;
}

static uint8_t peak = 0;
static uint8_t max_sample_between_led_updates = 0;
static unsigned long t_last_led_update = 0;
void loop()
{
  if (numSamples > 0)
  {
    noInterrupts();
    numSamples = 0;
    int sample = latest_sample;
    interrupts();
    sample = abs(sample - 127);
    max_sample_between_led_updates = max(sample, max_sample_between_led_updates);
    peak = peak_update(max_sample_between_led_updates, peak);

    //update the leds
    unsigned long t_now = millis();
    if (t_now - t_last_led_update >= LED_UPDATE_PERIOD)
    {
      t_last_led_update = t_now;
      //PORTC = (PORTC | 0x20); //timing
      unsigned long a = convert(max_sample_between_led_updates, peak);
      //PORTC = (PORTC ^ 0x20); //timing
      display_val(a);
      max_sample_between_led_updates = 0;
    }
  }
}

static unsigned int peak_update_skips = 0;
/**
 * Increases the peak if sample is larger than the current peak. Reduces peak by 1 if the
 * sample has been lower than the peak for SAMPLES_PER_PEAK_DECREMENT samples
 * (if peak hasn't changed).
 */
uint8_t peak_update(uint8_t sample, uint8_t peak) //2us
{
  if (sample >= peak)
  {
    peak_update_skips = 0;
    return sample;
  }
  else if (peak_update_skips == SAMPLES_PER_PEAK_DECREMENT) //tweak to make the peak last longer.
  {
    peak_update_skips = 0;
    return peak - 1;
  }
  else
  {
    peak_update_skips++;
    return peak;
  }
}

/**
 * Convert the latest sample and peak into a 16bit value that is later used to update LEDs.
 * Uses two look up tables that coorespond to thresholds of dB see lut_gen.py
 */
uint16_t convert(uint8_t sample, uint8_t peak) //3us w/o reverse, 8us w/ reverse
{
  static const uint16_t lookup_sample[] = {
      0, 0, 0, 0, 0,                                                                                                                                                          // 0-4
      0x1,                                                                                                                                                                    // 5-5
      0x3,                                                                                                                                                                    // 6-6
      0x7, 0x7,                                                                                                                                                               // 7-8
      0xf, 0xf,                                                                                                                                                               // 9-10
      0x1f, 0x1f,                                                                                                                                                             // 11-12
      0x3f, 0x3f, 0x3f, 0x3f,                                                                                                                                                 // 13-16
      0x7f, 0x7f, 0x7f, 0x7f,                                                                                                                                                 // 17-20
      0xff, 0xff, 0xff, 0xff, 0xff,                                                                                                                                           // 21-25
      0x1ff, 0x1ff, 0x1ff, 0x1ff, 0x1ff, 0x1ff, 0x1ff,                                                                                                                        // 26-32
      0x3ff, 0x3ff, 0x3ff, 0x3ff, 0x3ff, 0x3ff, 0x3ff, 0x3ff,                                                                                                                 // 33-40
      0x7ff, 0x7ff, 0x7ff, 0x7ff, 0x7ff, 0x7ff, 0x7ff, 0x7ff, 0x7ff, 0x7ff,                                                                                                   // 41-50
      0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff,                                                                       // 51-64
      0x1fff, 0x1fff, 0x1fff, 0x1fff, 0x1fff, 0x1fff, 0x1fff, 0x1fff, 0x1fff, 0x1fff, 0x1fff, 0x1fff, 0x1fff, 0x1fff, 0x1fff, 0x1fff,                                         // 65-80
      0x3fff, 0x3fff, 0x3fff, 0x3fff, 0x3fff, 0x3fff, 0x3fff, 0x3fff, 0x3fff, 0x3fff, 0x3fff, 0x3fff, 0x3fff, 0x3fff, 0x3fff, 0x3fff, 0x3fff, 0x3fff, 0x3fff, 0x3fff, 0x3fff, // 81-101
      0x7fff, 0x7fff, 0x7fff, 0x7fff, 0x7fff, 0x7fff, 0x7fff, 0x7fff, 0x7fff, 0x7fff, 0x7fff, 0x7fff, 0x7fff,                                                                 // 102-114
      0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff                                                                  // 115-126
  };
  static const uint16_t lookup_peak[] = {
      0, 0, 0, 0, 0,                                                                                                                                                          // 0-4
      0x1,                                                                                                                                                                    // 5-5
      0x2,                                                                                                                                                                    // 6-6
      0x4, 0x4,                                                                                                                                                               // 7-8
      0x8, 0x8,                                                                                                                                                               // 9-10
      0x10, 0x10,                                                                                                                                                             // 11-12
      0x20, 0x20, 0x20, 0x20,                                                                                                                                                 // 13-16
      0x40, 0x40, 0x40, 0x40,                                                                                                                                                 // 17-20
      0x80, 0x80, 0x80, 0x80, 0x80,                                                                                                                                           // 21-25
      0x100, 0x100, 0x100, 0x100, 0x100, 0x100, 0x100,                                                                                                                        // 26-32
      0x200, 0x200, 0x200, 0x200, 0x200, 0x200, 0x200, 0x200,                                                                                                                 // 33-40
      0x400, 0x400, 0x400, 0x400, 0x400, 0x400, 0x400, 0x400, 0x400, 0x400,                                                                                                   // 41-50
      0x800, 0x800, 0x800, 0x800, 0x800, 0x800, 0x800, 0x800, 0x800, 0x800, 0x800, 0x800, 0x800, 0x800,                                                                       // 51-64
      0x1000, 0x1000, 0x1000, 0x1000, 0x1000, 0x1000, 0x1000, 0x1000, 0x1000, 0x1000, 0x1000, 0x1000, 0x1000, 0x1000, 0x1000, 0x1000,                                         // 65-80
      0x2000, 0x2000, 0x2000, 0x2000, 0x2000, 0x2000, 0x2000, 0x2000, 0x2000, 0x2000, 0x2000, 0x2000, 0x2000, 0x2000, 0x2000, 0x2000, 0x2000, 0x2000, 0x2000, 0x2000, 0x2000, // 81-101
      0x4000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000,                                                                 // 102-114
      0x8000, 0x8000, 0x8000, 0x8000, 0x8000, 0x8000, 0x8000, 0x8000, 0x8000, 0x8000, 0x8000, 0x8000, 0x8000                                                                  // 115-126
  };
#if UPSIDE_DOWN == 0
  return lookup_sample[min(127, sample)] | lookup_peak[min(127, peak)];
#else
  return reverse_bits(lookup_sample[min(127, sample)] | lookup_peak[min(127, peak)]);
#endif
}

#if UPSIDE_DOWN != 0
/**
 * Reverses a 16 bit value's bits.
 */
uint16_t reverse_bits(uint16_t b) //5us
{
  b = ((b & 0x5555) << 1) | ((b & 0xAAAA) >> 1);
  b = ((b & 0x3333) << 2) | ((b & 0xCCCC) >> 2);
  b = ((b & 0x0F0F) << 4) | ((b & 0xF0F0) >> 4);
  return ((b & 0x00FF) << 8) | ((b & 0xFF00) >> 8);
}
#endif

/**
 * Map a 16 bit value to the LEDs.
 * Very specific to the Arduino Pro Mini, optimized for fast digital output.
 */
void display_val(uint16_t a) //4us
{
  PORTD = (PORTD & 0x03) | ((a & 0x3F) << 2);
  PORTB = (PORTB & 0xC0) | ((a & 0xFC0) >> 6);
  PORTC = (PORTC & 0xF0) | ((a & 0xF000) >> 12);
}

/**
 * Test all the functions and LEDs, also makes a nice boot sequence.
 */
void led_test(void)
{
  uint8_t thresholds[] = {5, 6, 7, 9, 11, 13, 17, 21, 26, 33, 41, 51, 65, 81, 102, 115};
  //                     -30,-28,-26,-24,-22,-20,-18,-16,-14,-12,-10,-8,-6, -4, -2, -1 dB  | 127=0dB

  //test all the thresholds
  int a = 0;
  for (int8_t i = 0; i < PIN_CNT; i++)
  {
    a = convert(thresholds[i], thresholds[i]);
    display_val(a);
    delay(TIME_BETWEEN_TEST_STEPS);
  }

  //test peaks in reverse
  for (int8_t i = PIN_CNT - 1; i >= 0; i--)
  {
    a = convert(0, thresholds[i]);
    display_val(a);
    delay(TIME_BETWEEN_TEST_STEPS);
  }

  //all on
  a = 0xFFFF;
  display_val(a);
  delay(TIME_BETWEEN_TEST_STEPS);

  //all off
  a = 0;
  display_val(a);
  delay(TIME_BETWEEN_TEST_STEPS);
}