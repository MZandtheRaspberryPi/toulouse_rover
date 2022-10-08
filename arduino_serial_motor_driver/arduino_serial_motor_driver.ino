#define SLP_PIN 8

#define PWM_1 11
#define PWM_2 10
#define PWM_3 6
#define PWM_4 5
#define leftInteruptPin 2
#define rightInteruptPin 3

volatile byte leftInteruptCount = 0;
volatile byte rightInteruptCount = 0;
int leftPriorState = HIGH;
int rightPriorState = HIGH;

byte PWM_1_VALUE = 0;
byte PWM_2_VALUE = 0;
byte PWM_3_VALUE = 0;
byte PWM_4_VALUE = 0;
bool ENABLE_MOTOR = false;

// using SLIP protocol:
// https://en.wikipedia.org/wiki/Serial_Line_Internet_Protocol
const byte END_BYTE = 0xC0;
const byte ESC_BYTE = 0xDB;
const byte ESC_END_BYTE = 0xDC;
const byte ESC_ESC_BYTE = 0xDD;

const byte NUM_BYTES = 100;
byte receivedBytes[NUM_BYTES];
byte numReceived = 0;

boolean newDataRcv = false;
boolean newDataSnd = false;

byte sendBytes[NUM_BYTES];
byte numSend = 0;

byte transformedSendBytesBuffer[NUM_BYTES];


void recvBytesWithStartEndMarkers();
void processReceivedData();
void sendBytesWithStartAndEndMarkets();
void incrementLeftInterupt();
void incrementRightInterupt();
byte addByteToArrRemovingEscSequences(const byte& in_byte, byte* byte_arr, byte& byte_arr_size);
byte addByteToArrAddingEscSequences(const byte& in_byte, byte* byte_arr, byte& byte_arr_size);


void setup() {
  Serial.begin(9600);
  
  pinMode(SLP_PIN, OUTPUT);
  pinMode(PWM_1, OUTPUT);
  pinMode(PWM_2, OUTPUT);
  pinMode(PWM_3, OUTPUT);
  pinMode(PWM_4, OUTPUT);

  // ir encoder open collectors
  pinMode(leftInteruptPin, INPUT_PULLUP);
  pinMode(rightInteruptPin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(leftInteruptPin), incrementLeftInterupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(rightInteruptPin), incrementRightInterupt, FALLING);

}

void loop() {

/*
  if (digitalRead(leftInteruptPin) == LOW && leftPriorState == HIGH)
  {
    leftInteruptCount++;
  }
  leftPriorState = digitalRead(leftInteruptPin);
  if (digitalRead(rightInteruptPin) == LOW && rightPriorState == HIGH)
  {
    rightInteruptCount++;
  }
  rightPriorState = digitalRead(rightInteruptPin);
*/

  recvBytesWithEndMarker();
  processReceivedData();
  sendBytesWithEndMarker();
  
  digitalWrite(SLP_PIN, ENABLE_MOTOR);
  analogWrite(PWM_1, PWM_1_VALUE);
  analogWrite(PWM_2, PWM_2_VALUE);
  analogWrite(PWM_3, PWM_3_VALUE);
  analogWrite(PWM_4, PWM_4_VALUE);
}

void recvBytesWithEndMarker() {
    static byte ndx = 0;
    byte rb;
   
    while (Serial.available() > 0 && newDataRcv == false) {
        rb = Serial.read();
        if (rb != END_BYTE)
        {
          addByteToArrRemovingEscSequences(rb, receivedBytes, ndx);
        }
        else
        {
          ndx = 0;
          // save to use when sending out
          numReceived = ndx;
          newDataRcv = true;
        }
    }
}

void processReceivedData() {
    if (newDataRcv == true) {
        ENABLE_MOTOR = receivedBytes[0];
        PWM_1_VALUE = receivedBytes[1];
        PWM_2_VALUE = receivedBytes[2];
        PWM_3_VALUE = receivedBytes[3];
        PWM_4_VALUE = receivedBytes[4];

        byte leftEncoderToSend = leftInteruptCount;
        leftInteruptCount = 0;
        byte rightEncoderToSend = rightInteruptCount;
        rightInteruptCount = 0;

        numSend = 0;
        sendBytes[0] = ENABLE_MOTOR;
        sendBytes[1] = PWM_1_VALUE;
        sendBytes[2] = PWM_2_VALUE;
        sendBytes[3] = PWM_3_VALUE;
        sendBytes[4] = PWM_4_VALUE;
        sendBytes[5] = leftEncoderToSend;
        sendBytes[6] = rightEncoderToSend;

        for (int i = 0; i < 7; i++)
        {
          addByteToArrAddingEscSequences(sendBytes[i], transformedSendBytesBuffer, numSend);
        }

        newDataRcv = false;
        newDataSnd = true;
    }
}

void sendBytesWithEndMarker()
{
    static byte ndxSend = 0;
   
    while (Serial.availableForWrite() > 0 && newDataSnd == true) {

        if (ndxSend < numSend)
        {
          Serial.write(transformedSendBytesBuffer[ndxSend]);
          ndxSend++;
        }
        else
        {
          Serial.write(END_BYTE);
          ndxSend = 0;
          newDataSnd = false;
        }
    }
}

void incrementLeftInterupt()
{
  leftInteruptCount++;
}
void incrementRightInterupt()
{
  rightInteruptCount++;
}

byte addByteToArrRemovingEscSequences(const byte& in_byte, byte* byte_arr, byte& byte_arr_size)
{
  if (byte_arr_size >= NUM_BYTES)
  {
    // overwrite last piece of data
    byte_arr_size--;
  }

  // if we had an escape sequence before this byte, it could be
  // special
  if (byte_arr_size > 1 && byte_arr[byte_arr_size - 1] == ESC_BYTE)
  {
    switch (in_byte)
    {
      case ESC_END_BYTE:
        byte_arr[byte_arr_size - 1] = END_BYTE;
        break;
      case ESC_ESC_BYTE:
        byte_arr[byte_arr_size - 1] = ESC_BYTE;
        break;
      default:
        byte_arr[byte_arr_size] = in_byte;
        byte_arr_size++;
    }   
  }
  else
  {
    byte_arr[byte_arr_size] = in_byte;
    byte_arr_size++;
  }
}


byte addByteToArrAddingEscSequences(const byte& in_byte, byte* byte_arr, byte& byte_arr_size)
{

  if (byte_arr_size >= NUM_BYTES)
  {
    // overwrite last piece of data
    byte_arr_size--;
  }

  switch (in_byte)
  {
    case END_BYTE:
      byte_arr[byte_arr_size] = ESC_BYTE;
      byte_arr_size++;
      byte_arr[byte_arr_size] = ESC_END_BYTE;
      byte_arr_size++;
      break;
    case ESC_BYTE:
      byte_arr[byte_arr_size] = ESC_BYTE;
      byte_arr_size++;
      byte_arr[byte_arr_size] = ESC_ESC_BYTE;
      byte_arr_size++;
      break;
    default:
      byte_arr[byte_arr_size] = in_byte;
      byte_arr_size++;
  }
  
}
