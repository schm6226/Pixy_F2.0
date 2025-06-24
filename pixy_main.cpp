/* This code uses a flywheel to point the 
 * Pixy camera.
 * Created by Andrew Schmit for UMN 
 * Ballooning team 
 */

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <lgpio.h>
#include "libpixyusb2.h"
#include <PIDLoop.h>

// GPIO pins using BCM numbering
#define STP 23  // Step pin
#define DIR 24  // Direction pin

Pixy2 pixy;
PIDLoop panLoop(0.5, 0.01, 0.2, true);  // Tune these values!
static bool run_flag = true;

int h;  // lgpio handle

void handle_SIGINT(int unused) {
  run_flag = false;
}

int16_t acquireBlock() {
  if (pixy.ccc.numBlocks && pixy.ccc.blocks[0].m_age > 30)
    return pixy.ccc.blocks[0].m_index;
  return -1;
}

Block* trackBlock(uint8_t index) {
  for (uint8_t i = 0; i < pixy.ccc.numBlocks; i++) {
    if (index == pixy.ccc.blocks[i].m_index)
      return &pixy.ccc.blocks[i];
  }
  return NULL;
}

void spinFlywheel(int speed) {
  speed = std::clamp(speed, -300, 300);  // C++17 std::clamp

  if (speed == 0) return;

  lgGpioWrite(h, DIR, speed > 0 ? 1 : 0);
  speed = abs(speed);

  for (int i = 0; i < speed; i++) {
    lgGpioWrite(h, STP, 1);
    usleep(500);
    lgGpioWrite(h, STP, 0);
    usleep(500);
  }
}

int main() {
  int i = 0;
  int16_t index = -1;
  int32_t panOffset;
  Block* block = NULL;

  signal(SIGINT, handle_SIGINT);

  // Open GPIO chip
  h = lgGpiochipOpen(0);  // Use GPIO chip 0 (/dev/gpiochip0)
  if (h < 0) {
    perror("lgGpiochipOpen failed");
    return 1;
  }

  // Set up GPIO pins as output
  if (lgGpioClaimOutput(h, 0, STP, 0) < 0 ||
      lgGpioClaimOutput(h, 0, DIR, 0) < 0) {
    perror("Failed to claim GPIO pins");
    return 1;
  }

  // Initialize Pixy2
  pixy.init();
  pixy.changeProg("color_connected_components");

  while (run_flag) {
    pixy.ccc.getBlocks();

    if (index == -1) {
      printf("Searching for block...\n");
      index = acquireBlock();
      if (index >= 0)
        printf("Found block!\n");
    }

    if (index >= 0)
      block = trackBlock(index);

    if (block) {
      i++;
      if (i % 60 == 0)
        printf("Tracking target... frame %d\n", i);

      // Calculate horizontal error
      panOffset = (int32_t)pixy.frameWidth / 2 - (int32_t)block->m_x;

      // Use PID to generate motor speed
      int speed = panLoop.update(panOffset);

      // Spin flywheel
      spinFlywheel(speed);
    } else {
      spinFlywheel(5); //spin the stepper motor untill sun is found
    }
  }

  printf("Exiting...\n");
  lgGpiochipClose(h);
  return 0;
}
