
static byte pins[] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, A0, A1, A2, A3, A4}; //, A5};
                                                                                   // PD2,PD3,PD4,PD5,PD6,PD7,  PB0,PB1,PB2,PB3,PB4,PB5  PC0,PC1,PC2,PC3,PC4,PC5
#include <avr/io.h>
static const int thresholds[] = {1, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 150, 200, 300, 500, 600, 700, 900};

#define PIN_CNT 17
void setup()
{
  PORTB = 12;
  for (byte i = 0; i <= PIN_CNT; i++)
  {
    pinMode(pins[i], OUTPUT);
  }
  Serial.begin(115200);
}

static unsigned long a = 0;
static byte time_toggle = HIGH;
void loop()
{
  // digitalWrite(LED_BUILTIN, HIGH); // turn the LED on (HIGH is the voltage level)
  // delay(250);                      // wait for a second
  // digitalWrite(LED_BUILTIN, LOW);  // turn the LED off by making the voltage LOW
  //delay(500); // wait for a second

  a = analogRead(A6); ///100us per read

  if (a >= 512)
  {
    a -= 512;
  }
  else
  {
    a = 512 - a;
  }

  a = convert(a); //100 us

  digitalWrite(A5, time_toggle);
  time_toggle = !time_toggle;
  // //test all
  //   for (byte i = 0; i < PIN_CNT; i++)
  //   {
  //     a |= 1 << i;
  //   }
  //display_val_bin(a); //300us
  display_val_bin2(a); //30us
  digitalWrite(A5, time_toggle);
  time_toggle = !time_toggle;
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

long convert(long a)
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