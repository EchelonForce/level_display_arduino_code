#include <avr/io.h>

// Change this to speed up/slow down the initialization tests.
#define TIME_BETWEEN_TEST_STEPS 100
// Change this to 1 if the display is upside down, otherwise set to 0.
#define UPSIDE_DOWN 0
// Time between LED updates, samples are still gathered [ms]
#define LED_UPDATE_PERIOD 1
// How many samples to allow before decreasing the saved peak value. Decrease for faster peak update.
#define SAMPLES_PER_PEAK_DECREMENT 1000

// Number of digitial pins connected to LEDs. Rework required if changed.
#define PIN_CNT 16

uint8_t find_nearest_note(unsigned long period);
void display_val(uint16_t a);
void led_test(void);

void setup()
{
  static const uint8_t pins[] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, A0, A1, A2, A3, A5}; // A5 for timing see below
  // PD2,PD3,PD4,PD5,PD6,PD7,  PB0,PB1,PB2,PB3,PB4,PB5  PC0,PC1,PC2,PC3,  PC5

  for (byte i = 0; i <= PIN_CNT; i++)
  {
    pinMode(pins[i], OUTPUT);
  }

  PORTC = (PORTC & ~0x20); // set timing pin 0
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

// These vars used in ADC ISR and loop().
static volatile uint16_t latest_sample = 0;
static volatile uint8_t numSamples = 0;
/**
 * Interrupt handler for the ADC.
 */
ISR(ADC_vect)
{
  // ADMUX = ADMUX ^ 0x01; //swap channels 6/7 every time, seems to limit signal for some reason
  if (numSamples > 0)
  {
    latest_sample += ADCH; // read 8 bit value from ADC
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
unsigned long last_crossing = 0;
unsigned long crossing = 0;
boolean going_up = true;
int last_sample = 0;
int sample = 0;
uint8_t latest_sample_cnt;
bool last_was_crossing = false;

int temp = 0;
void loop()
{
  if (numSamples > 0)
  {
    noInterrupts();
    latest_sample_cnt = numSamples;
    numSamples = 0;
    sample = latest_sample;
    interrupts();
    sample = (sample / latest_sample_cnt) - 127;

    if (sample < last_sample)
    {
      going_up = false;
    }
    else if (sample > last_sample)
    {
      going_up = true;
    }

    if (going_up)
    {
      // issues here with low frequencies because of high frequency noise.
      if (last_sample <= 0 && sample >= 0 && last_sample != sample && !last_was_crossing)
      {
        last_was_crossing = true;
        crossing = micros();
      }
      else
      {
        last_was_crossing = false;
      }
    }

    last_sample = sample;

    if (crossing != last_crossing)
    {
      unsigned long period = (unsigned long)crossing - (unsigned long)last_crossing;

      uint8_t idx = find_nearest_note(period);
      uint16_t a = (uint16_t)1 << (idx % 12);
      display_val(a);
      last_crossing = crossing;
    }
  }
}
//                          C, C#, D, D#, E, F, F#, G, G#, A, A#, B
static uint16_t notes2[] = {61152, 57728, 54480, 51424, 48544, 45808, 43248, 40816, 38528, 36368, 34320, 32400,
                            30576, 28864, 27240, 25712, 24272, 22904, 21624, 20408, 19264, 18184, 17160, 16200,
                            15288, 14432, 13620, 12856, 12136, 11452, 10812, 10204, 9632, 9092, 8580, 8100,
                            7644, 7216, 6810, 6428, 6068, 5726, 5406, 5102, 4816, 4546, 4290, 4050,
                            3822, 3608, 3405, 3214, 3034, 2863, 2703, 2551, 2408, 2273, 2145, 2025,
                            1911, 1804, 1703, 1607, 1517, 1432, 1351, 1276, 1204, 1136, 1073, 1012,
                            956, 902, 851, 804, 758, 716, 676, 638, 602, 568, 536, 506,
                            478, 451, 426, 402, 379, 358, 338, 319, 301, 284, 268, 253,
                            239, 225, 213, 201, 190, 179, 169, 159, 150, 142, 134, 127};

#define NUM_NOTES 108
uint8_t find_nearest_note(unsigned long period)
{
  uint8_t i;
  if (period >= notes2[0])
  {
    return 0;
  }
  if (period <= notes2[NUM_NOTES - 1])
  {
    return NUM_NOTES - 1;
  }
  for (i = 1; i < NUM_NOTES; i++)
  {
    if (period == notes2[i])
    {
      return i;
    }
    if (period > notes2[i])
    {
      // was between i-1 and i
      if ((period - notes2[i]) <= (notes2[i - 1] - period))
      {
        return i;
      }
      else
      {
        return i - 1;
      }
    }
  }
  return i;
}

#if UPSIDE_DOWN != 0
/**
 * Reverses a 16 bit value's bits.
 */
uint16_t
reverse_bits(uint16_t b) // 5us
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
void display_val(uint16_t a) // 4us
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
  // test forward
  int a = 0;

  // test leds
  for (int8_t i = PIN_CNT - 1; i >= 0; i--)
  {
    a = (uint16_t)1 << i;
    display_val(a);
    delay(TIME_BETWEEN_TEST_STEPS);
  }

  // all on
  a = 0xFFFF;
  display_val(a);
  delay(TIME_BETWEEN_TEST_STEPS);

  // all off
  a = 0;
  display_val(a);
  delay(TIME_BETWEEN_TEST_STEPS);

  test(0, find_nearest_note(61151), "idx=0,period=61151");
  test(0, find_nearest_note(61152), "idx=0,period=61152");
  test(0, find_nearest_note(61153), "idx=0,period=61153");
  test(NUM_NOTES - 1, find_nearest_note(127), "idx=NUM_NOTES-1");
  test(NUM_NOTES - 1, find_nearest_note(126), "idx=NUM_NOTES-1,period=126");
  test(NUM_NOTES - 1, find_nearest_note(128), "idx=NUM_NOTES-1,period=128");
  test(1, find_nearest_note(57728), "idx=1,period=57728");
  test(1, find_nearest_note(57727), "idx=1,period=57727");
  test(1, find_nearest_note(57729), "idx=1,period=57729");
  test(NUM_NOTES - 2, find_nearest_note(133), "idx=NUM_NOTES-2,period=133");
  test(NUM_NOTES - 2, find_nearest_note(134), "idx=NUM_NOTES-2,period=134");
  test(NUM_NOTES - 2, find_nearest_note(135), "idx=NUM_NOTES-2,period=135");
  test(11, find_nearest_note(32399), "idx=11,period=32399");
  test(11, find_nearest_note(32400), "idx=11,period=32400");
  test(11, find_nearest_note(32401), "idx=11,period=32401");
}

//                          C, C#, D, D#, E, F, F#, G, G#, A, A#, B
// static uint16_t notes2[] = {61152, 57728, 54480, 51424, 48544, 45808, 43248, 40816, 38528, 36368, 34320, 32400,
//                             30576, 28864, 27240, 25712, 24272, 22904, 21624, 20408, 19264, 18184, 17160, 16200,
//                             15288, 14432, 13620, 12856, 12136, 11452, 10812, 10204, 9632, 9092, 8580, 8100,
//                             7644, 7216, 6810, 6428, 6068, 5726, 5406, 5102, 4816, 4546, 4290, 4050,
//                             3822, 3608, 3405, 3214, 3034, 2863, 2703, 2551, 2408, 2273, 2145, 2025,
//                             1911, 1804, 1703, 1607, 1517, 1432, 1351, 1276, 1204, 1136, 1073, 1012,
//                             956, 902, 851, 804, 758, 716, 676, 638, 602, 568, 536, 506,
//                             478, 451, 426, 402, 379, 358, 338, 319, 301, 284, 268, 253,
//                             239, 225, 213, 201, 190, 179, 169, 159, 150, 142, 134, 127};

// #define NUM_NOTES 108
// uint8_t find_nearest_note(uint16_t period)

void test(int actual, int expected, const char *const test_name)
{
  if (actual != expected)
  {
    while (true)
    {
      Serial.print(test_name);
      Serial.print(": FAILED : ");
      Serial.print(actual);
      Serial.print("!=");
      Serial.print(expected);
      Serial.println("");
      delay(2000);
    }
  }
  else
  {
    Serial.print(test_name);
    Serial.println(": PASSED");
  }
}