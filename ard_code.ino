
static byte pins[] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, A0, A1, A2, A3, A4}; //, A5};
                                                                                   // PD2,PD3,PD4,PD5,PD6,PD7,  PB0,PB1,PB2,PB3,PB4,PB5  PC0,PC1,PC2,PC3,PC4,PC5
#include <avr/io.h>
static const int thresholds[] = {1, 2, 4, 8, 16, 32, 64, 96, 128, 160, 192, 224, 256, 320, 384, 448, 500};

#define PIN_CNT 17
void setup()
{
  for (byte i = 0; i <= PIN_CNT; i++)
  {
    pinMode(pins[i], OUTPUT);
  }
#define FASTADC 1
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif
#if FASTADC
  // set prescale to 16
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);
#endif
  //Serial.begin(115200);
}

static unsigned long a = 0;
static byte time_toggle = HIGH;
static unsigned int sample = 0;
static unsigned int peak = 0;
void loop()
{
  // digitalWrite(LED_BUILTIN, HIGH); // turn the LED on (HIGH is the voltage level)
  // delay(250);                      // wait for a second
  // digitalWrite(LED_BUILTIN, LOW);  // turn the LED off by making the voltage LOW
  //delay(500); // wait for a second

  //~120us per read, standard setup
  //~34us per read, FASTADC=1
  sample = analogRead(A6);

  digitalWrite(A5, time_toggle);
  time_toggle = !time_toggle;
  if (sample >= 512)
  {
    sample -= 512;
  }
  else
  {
    sample = 512 - sample;
  }
  peak = peak_check(sample, peak);
  //a=convert(sample);//50-100us
  //a = convert2(sample); //20-40us
  a = convert3(sample, peak); //50us

  digitalWrite(A5, time_toggle);
  time_toggle = !time_toggle;
  // //test all
  //   for (byte i = 0; i < PIN_CNT; i++)
  //   {
  //     a |= 1 << i;
  //   }
  //display_val_bin(a); //300us
  display_val_bin2(a); //30us
  // if (a == 0)
  // {
  //   a = 1;
  // }
  // else if (a > ((long)1 << 19))
  // {
  //   a = 0;
  // }
  // else
  // {
  //   a = a << 1;
  // }
  // digitalWrite(A5, time_toggle);
  // time_toggle = !time_toggle;
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

long convert(int a)
{
  long b = 0;
  long c = 0;
  for (byte i = 0; i < PIN_CNT; i++)
  {
    if (a >= thresholds[i])
    {
      b |= 1 << i;
    }
    else
    {
      c |= 1 << i;
    }
  }
  return b;
}

long convert2(int a)
{
  // #define T1 1
  // #define T2 10
  // #define T3 20
  // #define T4 30
  // #define T5 40
  // #define T6 50
  // #define T7 60
  // #define T8 70
  // #define T9 80
  // #define T10 90
  // #define T11 100
  // #define T12 110
  // #define T13 120
  // #define T14 130
  // #define T15 140
  // #define T16 150
  // #define T17 160

  long b = 0;
  for (byte i = 0; i < PIN_CNT; i++)
  {
    if (a < thresholds[i])
    {
      b = (1 << i) - 1;
      break;
    }
  }
  return b;
}

long convert3(int sample, int peak)
{
  long b = 0;
  int8_t idx_peak = 0;
  for (int8_t i = PIN_CNT - 1; i >= 0; i--)
  {
    if (peak > thresholds[i] && idx_peak == 0)
    {
      idx_peak = i;
    }

    if (sample > thresholds[i])
    {
      b = ((1 << i) - 1) | (1 << idx_peak);
      break;
    }
  }
  return b;
}

void display_val_bin(long a)
{
  for (byte i = 0; i < PIN_CNT; i++)
  {
    digitalWrite(pins[i], ((a >> i) & 1));
  }
}

void display_val_bin2(long a)
{
  PORTD = (PORTD & 0x03) | ((a & 0x3F) << 2);
  PORTB = (PORTB & 0xC0) | ((a & 0xFC0) >> 6);
  PORTC = (PORTC & 0xE0) | ((a & 0x1F000) >> 12);
  // for (byte i = 0; i < PIN_CNT; i++)
  // {
  //   digitalWrite(pins[i], ((a >> i) & 1));
  // }
}

//static byte pins[] = {2,  3,  4,  5,  6,  7,    8,  9, 10,   11, 12, 13, A0, A1, A2, A3, A4}; //, A5};
// PD2,PD3,PD4,PD5,PD6,PD7,  PB0,PB1,PB2,PB3,PB4,PB5  PC0,PC1,PC2,PC3,PC4,PC5