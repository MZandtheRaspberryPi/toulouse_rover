#include <iostream>
#include <wiringPi.h>
#include <signal.h>

void handleKeyboardInterupt(int s) {
    std::printf("Caught keyboard interupt %d, exiting...\n", s);
    pwmWrite(1, 0);
    exit(0);
}


int main() {
    wiringPiSetup();
    pinMode(1, PWM_OUTPUT);

    pinMode(24, OUTPUT);
    delay(100);

    digitalWrite(24, LOW);

    struct sigaction act;
    act.sa_handler = handleKeyboardInterupt;
    sigaction(SIGINT, &act, NULL);

    while(1) {
      delay(1000);
      std::printf("toggling\n");
      pwmWrite(1, 1023);
      delay(1000);
      pwmWrite(1, 900);
    }

}
