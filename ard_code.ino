#include <avr/io.h>

static byte pins[] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, A0, A1, A2, A3}; // and A5 for timing see setup()
//                    PD2,PD3,PD4,PD5,PD6,PD7,  PB0,PB1,PB2,PB3,PB4,PB5  PC0,PC1,PC2,PC3,  PC5
static const int thresholds[] = {5, 6, 7, 9, 11, 13, 17, 21, 26, 33, 41, 51, 65, 81, 102, 115};
//                             -30,-28,-26,-24,-22,-20,-18,-16,-14,-12,-10,-8,-6, -4, -2, -1 dB  | 127=0dB

#define PIN_CNT 16
void setup()
{
  for (byte i = 0; i <= PIN_CNT; i++)
  {
    pinMode(pins[i], OUTPUT);
  }

  pinMode(A5, OUTPUT);    //for timing
  PORTC = (PORTC | 0x20); //timing

  Serial.begin(115200);

  led_test();
  adc_setup();
}

void adc_setup(void)
{
  ADCSRA = 0;            // clear ADCSRA register
  ADCSRB = 0;            // clear ADCSRB register
  ADMUX |= (6 & 0x07);   // set A6 analog input pin
  ADMUX |= (1 << REFS0); // set reference voltage
  ADMUX |= (1 << ADLAR); // left align ADC value to 8 bits from ADCH register

  // sampling rate is [ADC clock] / [prescaler] / [conversion clock cycles]
  // for Arduino Uno ADC clock is 16 MHz and a conversion takes 13 clock cycles
  ADCSRA |= (1 << ADPS2) | (1 << ADPS0); // 32 prescaler for 38.5 KHz
  ADCSRA |= (1 << ADATE);                // enable auto trigger
  ADCSRA |= (1 << ADIE);                 // enable interrupts when measurement complete
  ADCSRA |= (1 << ADEN);                 // enable ADC
  ADCSRA |= (1 << ADSC);                 // start ADC measurements
}

static byte latest_sample = 0;
static byte numSamples = 0;
static byte time_toggle = HIGH;

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
      PORTC = (PORTC | 0x20); //timing
      display_val(a);
      PORTC = (PORTC ^ 0x20); //timing
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
  else if (skips == 1000) //tweak to make the peak last longer.
  {
    skips = 0;
    return peak - 1;
  }
  else
  {
    skips++;
  }
}

uint16_t convert(int sample, int peak) //23us w/o reverse, 85 w/ reverse, 29 w/ reverse 2
{
  if (sample >= thresholds[PIN_CNT - 1])
  {
    return 0xFFFF;
  }
  uint16_t b = 0;
  int8_t idx_peak = (peak >= thresholds[PIN_CNT - 1]) ? PIN_CNT - 1 : -1;
  for (int8_t i = PIN_CNT - 2; i >= 0; i--)
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
  //return (b);
  return reverse_bits_2(b); //use if upsidedown.
}

uint16_t reverse_bits(uint16_t a) //62 us
{
  uint16_t b = 0;
  for (byte i = 0; i < PIN_CNT; i++)
  {
    b = b << 1 | ((a >> i) & 1);
  }
  return b;
}

uint16_t reverse_bits_2(uint16_t b) //6us
{
  b = ((b & 0x5555) << 1) | ((b & 0xAAAA) >> 1);
  b = ((b & 0x3333) << 2) | ((b & 0xCCCC) >> 2);
  b = ((b & 0x0F0F) << 4) | ((b & 0xF0F0) >> 4);
  return ((b & 0x00FF) << 8) | ((b & 0xFF00) >> 8);
}

void display_val(uint16_t a) //4us
{
  PORTD = (PORTD & 0x03) | ((a & 0x3F) << 2);
  PORTB = (PORTB & 0xC0) | ((a & 0xFC0) >> 6);
  PORTC = (PORTC & 0xF0) | ((a & 0xF000) >> 12);
}

void led_test(void)
{
#define TIME_BETWEEN 300
  //test all the thresholds
  int a = 0;
  for (int8_t i = 0; i < PIN_CNT; i++)
  {
    a = convert(thresholds[i], thresholds[i]);
    display_val(a);
    delay(TIME_BETWEEN);
  }

  //test peaks in reverse
  for (int8_t i = PIN_CNT - 1; i >= 0; i--)
  {
    a = convert(0, thresholds[i]);
    display_val(a);
    delay(TIME_BETWEEN);
  }

  //all on
  a = 0xFFFF;
  display_val(a);
  delay(TIME_BETWEEN);

  //all off
  a = 0;
  display_val(a);
  delay(TIME_BETWEEN);
}