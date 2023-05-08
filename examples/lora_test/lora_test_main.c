/**
 * cansat_apps/lora_test/lora_test_main.c
 *
 * LoRa transmission test program. Sends "245 Marco Hi!" every second.
 *
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <assert.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include "nuttx/rf/rfm95.h"

#define DEV_NAME "/dev/radio0"

int main(int argc, FAR char *argv[])
{
  int ret;
  printf("RFM95 driver test app\n");

  int fd = open(DEV_NAME, O_RDWR);
  if (fd < 0)
    {
      int errcode = errno;
      printf("ERROR: Failed to open device %s: %d\n", DEV_NAME, errcode);
    }

  ret = ioctl(fd, RFM95_IOCTL_INIT, 0);
  if (ret < 0)
    {
      printf("failed to change init: %d!\n", ret);
    }

  printf("Init done!\n");

  char msg[5] = {"Ciao!"};

  while(true) {
   ret = write(fd, "\xf5Marco, hi!", 11);
   if (ret < 0)
      {
         printf("failed to send message: %d!\n", ret);
      }

   printf("Message sent!\n");
   sleep(1);
  }

  close(fd);
  return 0;
}
