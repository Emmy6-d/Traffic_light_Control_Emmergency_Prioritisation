/************************************************************
 * Emergency Vehicle Transmitter
 * Toggle Button Version
 * MCU: Arduino UNO / Nano
 ************************************************************/

#define BTN_S 2
#define BTN_E 3
#define BTN_N 4
#define BTN_W 5

#define TX_INTERVAL 200

char activeLane = 0;

bool lastS = HIGH;
bool lastE = HIGH;
bool lastN = HIGH;
bool lastW = HIGH;

unsigned long lastTxTime = 0;

void setup()
{
  pinMode(BTN_S, INPUT_PULLUP);
  pinMode(BTN_E, INPUT_PULLUP);
  pinMode(BTN_N, INPUT_PULLUP);
  pinMode(BTN_W, INPUT_PULLUP);

  pinMode(13, OUTPUT);

  Serial.begin(9600);
}

void loop()
{
  checkButton(BTN_S, 'S', lastS);
  checkButton(BTN_E, 'E', lastE);
  checkButton(BTN_N, 'N', lastN);
  checkButton(BTN_W, 'W', lastW);

  if (activeLane != 0)
  {
    if (millis() - lastTxTime >= TX_INTERVAL)
    {
      sendEmergencyPacket(activeLane);
      lastTxTime = millis();
    }
  }
}

/************************************************************
 * BUTTON HANDLER
 ************************************************************/
void checkButton(uint8_t pin, char lane, bool &lastState)
{
  bool currentState = digitalRead(pin);

  if (lastState == HIGH && currentState == LOW)
  {
    delay(20); // debounce

    if (activeLane == lane)
    {
      activeLane = 0;      // stop transmission
      digitalWrite(13, LOW);
    }
    else
    {
      activeLane = lane;   // start/switch transmission
      digitalWrite(13, HIGH);
    }
  }

  lastState = currentState;
}

/************************************************************
 * SEND PACKET
 ************************************************************/
void sendEmergencyPacket(char lane)
{
  Serial.print("$EMG,");
  Serial.print(lane);
  Serial.print("#");
}