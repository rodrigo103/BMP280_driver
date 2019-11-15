// https://stackoverflow.com/questions/1157209/is-there-an-alternative-sleep-function-in-c-to-milliseconds

#include <unistd.h> // for usleep
#include "stdio.h"

void sleep_ms(int milliseconds) // cross-platform sleep function
{
  usleep(milliseconds * 1000);
}

int main (void) {
  sleep_ms(2000);
  printf("holas");
  sleep_ms(2000);
  printf("holas");
  sleep_ms(2000);
  return 0;
}