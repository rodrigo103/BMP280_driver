// https://stackoverflow.com/questions/1157209/is-there-an-alternative-sleep-function-in-c-to-milliseconds

#include <time.h>   // for nanosleep
#include "stdio.h"

void sleep_ms(int milliseconds) // cross-platform sleep function
{
    struct timespec ts;
    ts.tv_sec = milliseconds / 1000;
    ts.tv_nsec = (milliseconds % 1000) * 1000000;
    nanosleep(&ts, NULL);
}

int main (void) {
  sleep_ms(2000);
  printf("holas");
  sleep_ms(2000);
  printf("holas");
  sleep_ms(2000);
  return 0;
}