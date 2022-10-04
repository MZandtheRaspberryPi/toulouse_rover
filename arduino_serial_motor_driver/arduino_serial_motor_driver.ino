#define SLP_PIN 8

#define PWM_1 11
#define PWM_2 10
#define PWM_3 9
#define PWM_4 3

byte PWM_1_VALUE = 0;
byte PWM_2_VALUE = 0;
byte PWM_3_VALUE = 0;
byte PWM_4_VALUE = 0;
bool ENABLE_MOTOR = false;

// our pwm for the wheels will
// always be greater than 20
// as the wheels need a certain
// voltage to spin. hence start 
// of 3 and end of 4.
const byte startMarker = 0x3;
const byte endMarker = 0x4;
const byte byteArrTerminator = 0x5;

const byte numBytes = 12;
byte receivedBytes[numBytes];
byte numReceived = 0;

boolean newDataRcv = false;
boolean newDataSnd = false;

byte sendBytes[numBytes];
byte numSend = 0;
void recvBytesWithStartEndMarkers();
void processReceivedData();
void sendBytesWithStartAndEndMarkets();


void setup() {
  // start serial port at 115200 bps and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  pinMode(SLP_PIN, OUTPUT);

  pinMode(PWM_1, OUTPUT);
  pinMode(PWM_2, OUTPUT);
  pinMode(PWM_3, OUTPUT);
  pinMode(PWM_4, OUTPUT);

}

void loop() {
  recvBytesWithStartEndMarkers();
  processReceivedData();
  sendBytesWithStartAndEndMarkets();
  digitalWrite(SLP_PIN, ENABLE_MOTOR);
  analogWrite(PWM_1, PWM_1_VALUE);
  analogWrite(PWM_2, PWM_2_VALUE);
  analogWrite(PWM_3, PWM_3_VALUE);
  analogWrite(PWM_4, PWM_4_VALUE);
}

void recvBytesWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    byte rb;
   
    while (Serial.available() > 0 && newDataRcv == false) {
        rb = Serial.read();

        if (recvInProgress == true) {
            if (rb != endMarker) {
                receivedBytes[ndx] = rb;
                ndx++;
                if (ndx >= numBytes) {
                    ndx = numBytes - 1;
                }
            }
            else {
                receivedBytes[ndx] = byteArrTerminator; // terminate the string
                recvInProgress = false;
                numReceived = ndx;  // save the number for use when printing
                ndx = 0;
                newDataRcv = true;
            }
        }

        else if (rb == startMarker) {
          
            recvInProgress = true;
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
        newDataRcv = false;
        newDataSnd = true;

        for (int i = 0; i < numBytes; i++)
        {
          sendBytes[i] = receivedBytes[i];
          if (receivedBytes[i] == byteArrTerminator)
          {
            break;
          }
        }
    }
}

void sendBytesWithStartAndEndMarkets()
{
    static boolean sendInProgress = false;
    static byte ndxSend = 0;
    byte rb;
   

    while (Serial.availableForWrite() > 0 && newDataSnd == true) {

        if (sendInProgress == true) {
            if (sendBytes[ndxSend] != byteArrTerminator ) {
                Serial.write(sendBytes[ndxSend]);
                ndxSend++;
            }
            else {
                Serial.write(endMarker);
                sendInProgress = false;
                numSend = ndxSend;  // save the number for use when printing
                ndxSend = 0;
                newDataSnd = false;
            }
        }

        else {
            Serial.write(startMarker);
            sendInProgress = true;
        }
    }
}
