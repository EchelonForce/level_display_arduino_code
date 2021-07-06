
static byte pins[] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, A0, A1, A2, A3, A4, A5};
static const int thresholds[] = {1, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 150, 200, 300, 500, 600, 700, 900};

#define PIN_CNT 18
void setup()
{
  for (byte i = 0; i < PIN_CNT; i++)
  {
    pinMode(pins[i], OUTPUT);
  }
  Serial.begin(115200);
}

static unsigned long a = 0;
void loop()
{
  // digitalWrite(LED_BUILTIN, HIGH); // turn the LED on (HIGH is the voltage level)
  // delay(250);                      // wait for a second
  // digitalWrite(LED_BUILTIN, LOW);  // turn the LED off by making the voltage LOW
  //delay(500); // wait for a second

  a = analogRead(A6);

  if (a >= 512)
  {
    a -= 512;
  }
  else
  {
    a = 512 - a;
  }

  Serial.println(a, DEC);
  a = convert(a);

  // //test all
  //   for (byte i = 0; i < PIN_CNT; i++)
  //   {
  //     a |= 1 << i;
  //   }
  display_val_bin(a);
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
}

long convert(long a)
{
  long b = 0;
  for (byte i = 0; i < PIN_CNT; i++)
  {
    if (a >= thresholds[i])
    {
      b |= 1 << i;
    }
    else
    {
      break;
    }
  }
  return b;
}

void display_val_bin(long a)
{
  for (int i = 0; i < PIN_CNT; i++)
  {
    digitalWrite(pins[i], ((a >> i) & 1) ? HIGH : LOW);
  }
}
