
static byte pins[] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, A0, A1, A2, A3}; //, A5};
                                                                               // PD2,PD3,PD4,PD5,PD6,PD7,  PB0,PB1,PB2,PB3,PB4,PB5  PC0,PC1,PC2,PC3,PC4,PC5
#include <avr/io.h>
static const int thresholds[] = {17, 21, 26, 33, 41, 52, 65, 82, 103, 129, 162, 204, 257, 324, 407, 457};
//                              -30,-28,-26,-24,-22,-20,-18,-16, -14, -12, -10,  -8,  -6,  -4,  -2,  -1 dB  | 512=0dB
#define PIN_CNT 16
void setup()
{
  for (byte i = 0; i <= PIN_CNT; i++)
  {
    pinMode(pins[i], OUTPUT);
  }
#define FASTADC 1
#if FASTADC
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
  // set prescale to 16
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);
#endif
  Serial.begin(115200);

  led_test();
}

static unsigned long a = 0;
static byte time_toggle = HIGH;
static unsigned int sample = 0;
static unsigned int peak = 0;
static unsigned int sample_peaks = 0;
static int t_last = 0;

void loop()
{
  digitalWrite(A5, time_toggle);
  time_toggle = !time_toggle;

  sample = abs(analogRead(A6) - 512);

  digitalWrite(A5, time_toggle);
  time_toggle = !time_toggle;

  sample_peaks = max(sample, sample_peaks);
  peak = peak_check(sample_peaks, peak);

  int t_now = millis();
  if (t_now - t_last > 1000)
  {
    t_last = t_now;
    a = convert(sample_peaks, peak);
    display_val(a); //30us
    sample_peaks = 0;
  }
}

static byte skips = 0;
int peak_check(int sample, int peak)
{
  if (sample >= peak)
  {
    skips = 0;
    return sample;
  }
  else if (skips == 50)
  {
    skips = 0;
    return peak - 1;
  }
  else
  {
    skips++;
  }
}

long convert(int sample, int peak) //25us
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

void display_val(long a)
{
  PORTD = (PORTD & 0x03) | ((a & 0x3F) << 2);
  PORTB = (PORTB & 0xC0) | ((a & 0xFC0) >> 6);
  PORTC = (PORTC & 0xE0) | ((a & 0x0F000) >> 12);
}

void led_test(void)
{
  //test all
  int a = 0;
  for (int8_t i = 0; i < PIN_CNT; i++)
  {
    a = convert(thresholds[i] + 1, thresholds[i] + 1);
    display_val(a);
    delay(100);
  }

  for (int8_t i = PIN_CNT - 1; i >= 0; i--)
  {
    a = convert(0, thresholds[i] + 1);
    display_val(a);
    delay(100);
  }

  a = 0xFFFF;
  display_val(a);
  delay(100);

  a = 0x0;
  display_val(a);
  delay(100);
}