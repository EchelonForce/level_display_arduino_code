#include <avr/io.h>

static byte pins[] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, A0, A1, A2, A3}; //, A5};
//                    PD2,PD3,PD4,PD5,PD6,PD7,  PB0,PB1,PB2,PB3,PB4,PB5  PC0,PC1,PC2,PC3,PC4,PC5
static const int thresholds[] = {5, 6, 7, 9, 11, 13, 17, 21, 26, 33, 41, 51, 65, 81, 102, 115};
//                             -30,-28,-26,-24,-22,-20,-18,-16,-14,-12,-10,-8,-6, -4, -2, -1 dB  | 127=0dB

#define PIN_CNT 16
void setup()
{
  for (byte i = 0; i <= PIN_CNT; i++)
  {
    pinMode(pins[i], OUTPUT);
  }
  Serial.begin(115200);

  led_test();
  adc_setup();
}

void adc_setup(void)
{
  ADCSRA = 0;            // clear ADCSRA register
  ADCSRB = 0;            // clear ADCSRB register
  ADMUX |= (6 & 0x07);   // set A0 analog input pin
  ADMUX |= (1 << REFS0); // set reference voltage
  ADMUX |= (1 << ADLAR); // left align ADC value to 8 bits from ADCH register

  // sampling rate is [ADC clock] / [prescaler] / [conversion clock cycles]
  // for Arduino Uno ADC clock is 16 MHz and a conversion takes 13 clock cycles
  //ADCSRA |= (1 << ADPS2) | (1 << ADPS0);    // 32 prescaler for 38.5 KHz
  //ADCSRA |= (1 << ADPS2); // 16 prescaler for 76.9 KHz
  ADCSRA |= (1 << ADPS1) | (1 << ADPS0); // 8 prescaler for 153.8 KHz

  ADCSRA |= (1 << ADATE); // enable auto trigger
  ADCSRA |= (1 << ADIE);  // enable interrupts when measurement complete
  ADCSRA |= (1 << ADEN);  // enable ADC
  ADCSRA |= (1 << ADSC);  // start ADC measurements
}

static byte latest_sample = 0;
static byte numSamples = 0;
static byte time_toggle = HIGH;

ISR(ADC_vect)
{
  latest_sample = ADCH; // read 8 bit value from ADC
  numSamples++;
}

static unsigned long a = 0;
static int sample = 0;
static unsigned int peak = 0;
static unsigned int sample_peaks = 0;
static int t_last = 0;

void loop()
{
  if (numSamples > 0)
  {
    numSamples = 0;
    sample = latest_sample;
    sample = abs(sample - 127);
    sample_peaks = max(sample, sample_peaks);
    peak = peak_check(sample_peaks, peak);

    int t_now = millis();
    if (t_now - t_last > 0)
    {
      t_last = t_now;
      a = convert(sample_peaks, peak);
      // digitalWrite(A5, time_toggle);
      // time_toggle = !time_toggle;
      display_val(a);
      sample_peaks = 0;
    }
  }
}

static int skips = 0;
int peak_check(int sample, int peak)
{
  if (sample >= peak)
  {
    skips = 0;
    return sample;
  }
  else if (skips == 500) //tweak to make the peak last longer.
  {
    skips = 0;
    return peak - 1;
  }
  else
  {
    skips++;
  }
}

long convert(int sample, int peak) //78us
{
  long b = 0;
  int8_t idx_peak = -1;
  for (int8_t i = PIN_CNT - 1; i >= 0; i--)
  {
    if (peak >= thresholds[i] && idx_peak < 0)
    {
      idx_peak = i;
    }

    if (sample >= thresholds[i])
    {
      b = ((1 << (i + 1)) - 1);
      break;
    }
  }
  if (idx_peak >= 0)
  {
    b |= (1 << idx_peak);
  }
  return b;
}

void display_val(long a) //50us
{
  PORTD = (PORTD & 0x03) | ((a & 0x3F) << 2);
  PORTB = (PORTB & 0xC0) | ((a & 0xFC0) >> 6);
  PORTC = (PORTC & 0xE0) | ((a & 0x0F000) >> 12);
}

void led_test(void)
{
  //test all the thresholds
  int a = 0;
  for (int8_t i = 0; i < PIN_CNT; i++)
  {
    a = convert(thresholds[i] + 1, thresholds[i] + 1);
    display_val(a);
    delay(100);
  }

  //test peaks in reverse
  for (int8_t i = PIN_CNT - 1; i >= 0; i--)
  {
    a = convert(0, thresholds[i] + 1);
    display_val(a);
    delay(100);
  }
  //all on
  a = 0xFFFF;
  display_val(a);
  delay(100);

  //all of
  a = 0x0;
  display_val(a);
  delay(100);
}