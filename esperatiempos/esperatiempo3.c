#include <time.h>
#include <errno.h>
#include <stdio.h>

int msleep(long msec)
{
  struct timespec ts;
  int res;

  if (msec < 0)
  {
    errno = EINVAL;
    return -1;
  }

  ts.tv_sec = msec / 1000;
  ts.tv_nsec = (msec % 1000) * 1000000;

  do
  {
    res = nanosleep(&ts, &ts);
  } while (res && errno == EINTR);

  return res;
}

int main (void) {
  printf("holas");
  msleep(2000);
  printf("holas");
  msleep(2000);
  printf("holas");
  msleep(2000);
  return 0;
}