class WheelController {

public:

  WheelController(int forwardPwmPin, int forwardLowPin, int interuptPin) : forwardPwmPin(forwardPwmPin),
                                                          forwardLowPin(forwardLowPin), interuptPin(interuptPin) {
    commandRadPerSec = 0;
    // setup wiring pi interupt for this wheel
    // assume wiring pi setup has been called
    wiringPiISR (0, INT_EDGE_FALLING, &wheelInterupt);
  }

  int setRadPerSec(int radPerSec) {
    // set direction for wheels encoder interupt
    // set new goal for PID, start PID
  }

  int update() {
    // check the delta t and calc error from commands
    // return new pwm command
  }

  int interuptPin;
  int forwardPwmPin;
  int forwardLowPin;
  int commandRadPerSec;
  volatile int encoderCounts = 0;
private:
  // function to convert radians per second of wheel rotation into
  // encoder counts per second of rotation
  int convertRadToEnc(int rps) {};
  void wheelInterupt() {
    if (direction > 0) {
      encoderCounts++;
    }
    else {
      encoderCounts--;
    }
  int encTicksPerRotation = 20;
  bool oppositeSide;
};
