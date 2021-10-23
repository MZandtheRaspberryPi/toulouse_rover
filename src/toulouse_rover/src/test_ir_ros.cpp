#include <ros/ros.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <wiringPi.h>


// globalCounter:
//      Global variable to count interrupts
//      Should be declared volatile to make sure the compiler doesn't cache it.

static volatile int globalCounter [4] ;


/*
 * myInterrupt:
 *********************************************************************************
 */

void myInterrupt0 (void) { ++globalCounter [0] ; }
void myInterrupt1 (void) { ++globalCounter [1] ; }
void myInterrupt2 (void) { ++globalCounter [2] ; }
void myInterrupt3 (void) { ++globalCounter [3] ; }

/*
 *********************************************************************************
 * main
 *********************************************************************************
 */

int main (int argc, char **argv)
{
  int gotOne, pin ;
  int myCounter [4] ;

  for (pin = 0 ; pin < 4 ; ++pin)
    globalCounter [pin] = myCounter [pin] = 0 ;

  wiringPiSetup () ;

  wiringPiISR (0, INT_EDGE_FALLING, &myInterrupt0) ;
  wiringPiISR (25, INT_EDGE_FALLING, &myInterrupt1) ;
  wiringPiISR (2, INT_EDGE_FALLING, &myInterrupt2) ;
  wiringPiISR (3, INT_EDGE_FALLING, &myInterrupt3) ;

  ros::init(argc, argv, "test_ir_ros");
  ros::NodeHandle nh;

  while (ros::ok())
  {
    gotOne = 0 ;
    printf ("Waiting ... ") ; fflush (stdout) ;

    while (ros::ok())
    {
      for (pin = 0 ; pin < 4 ; ++pin)
      {
        if (globalCounter [pin] != myCounter [pin])
        {
          printf (" Int on pin %d: Counter: %5d\n", pin, globalCounter [pin]) ;
          myCounter [pin] = globalCounter [pin] ;
          ++gotOne ;
        }
      }
      if (gotOne != 0)
        break ;
    }
  }

  return 0 ;
}
