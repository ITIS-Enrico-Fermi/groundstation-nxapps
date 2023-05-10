#include <nuttx/config.h>
#include <stdio.h>
#include <stdbool.h>
#include <assert.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include "nuttx/rf/rfm95.h"
#include "flight_senutils.h"

#define DEV_NAME "/dev/radio0"

#pragma pack 1
struct lora_packet pkt;
size_t pkt_len;


int main(int argc, FAR char *argv[]) {
  int ret;
  printf("LoRa receiver\n");

  int fd = open(DEV_NAME, O_RDWR);
  if (fd < 0)
    {
    int errcode = errno;
    printf("ERROR: Failed to open device %s: %d\n", DEV_NAME, errcode);
    }

  ret = ioctl(fd, RFM95_IOCTL_INIT, 0);
  if (ret < 0) {
    printf("failed to change init: %d!\n", ret);
    }

  printf("Init done!\n");

  while(true) {
    pkt_len = read(fd, &pkt, sizeof(struct lora_packet));

    printf("Received packet of %d bytes\n", pkt_len);

    printf(
      "{"
        "'pressure': %.2f,"
        "'accel': {"
          "'x': %d,"
          "'y': %d,"
          "'z': %d,"
        "}, 'roto': {"
          "'x': %d,"
          "'y': %d,"
          "'z': %d,"
        "}, 'lat': %.5f,"
        "'lon': %.5f,"
        "'uv': %d,"
        "'counter': %d"
      "}\n",
      pkt.pressure,
      pkt.gyro.accel.x, pkt.gyro.accel.y, pkt.gyro.accel.z,
      pkt.gyro.roto.x, pkt.gyro.roto.y, pkt.gyro.roto.z,
      pkt.gps.latitude, pkt.gps.longitude,
      pkt.uv, pkt.counter
    );

    usleep(100000);
  }

  close(fd);
  return 0;
}
