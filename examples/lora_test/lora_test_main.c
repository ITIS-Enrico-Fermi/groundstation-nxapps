/****************************************************************************
 * cansat_apps/lora_test/lora_test_main.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <assert.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "nuttx/rf/rfm95.h"

#define DEV_NAME "/dev/radio0"


#pragma pack 1
struct pkt_t {
  uint8_t id;
  char msg[10];
};

/*
 * Test SPI driver before writing a stable library
 * library will offer a fast and easy way to use rfm95 device.
 */
int main(int argc, FAR char *argv[]) {
  int ret;
  printf("RFM95 driver test app\n");

  int fd = open(DEV_NAME, O_RDWR);
  if (fd < 0) {
    int errcode = errno;
    printf("ERROR: Failed to open device %s: %d\n", DEV_NAME, errcode);
  }

  ret = ioctl(fd, RFM95_IOCTL_INIT, 0);
  if (ret < 0) {
    printf("failed to change init: %d!\n", ret);
  }

  printf("Init done!\n");

  struct pkt_t pkt = {
    .id = 245,
    .msg = "marco, azz"
  };

  while(true) {
    ret = write(fd, &pkt, sizeof(pkt));
    if (ret < 0) {
      printf("failed to send message: %d!\n", ret);
    }

    printf("Message sent!\n");
    sleep(1);
  }

  close(fd);
  return 0;
}