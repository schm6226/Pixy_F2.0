/*this code uses a flywheel to point the 
 * Pixy camera.
 * created by Andrew Schmit for UMN 
 * Ballooning team*/
 
#include <signal.h>
#include <wiringPi.h>
#include "libpixyusb2.h"
#include <PIDLoop.h>

#define STP 23  // GPIO pin for step (BCM 23)
#define DIR 24  // GPIO pin for direction (BCM 24)

Pixy2 pixy;
PIDLoop panLoop(0.5, 0.01, 0.2, true);  // Tune these values!
static bool run_flag = true;

void setup() {
  pinMode(STP, OUTPUT);
  pinMode(DIR, OUTPUT);
}

void handle_SIGINT(int unused) {
  run_flag = false;
}

// Convert Pixy x-offset to flywheel speed
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

// Spin flywheel proportionally to control signal
void spinFlywheel(int speed) {
  speed = constrain(speed, -300, 300);  // limit speed (adjust as needed)

  if (speed == 0) return;

  digitalWrite(DIR, speed > 0 ? HIGH : LOW);
  speed = abs(speed);

  for (int i = 0; i < speed; i++) {
    digitalWrite(STP, HIGH);
    delayMicroseconds(500);
    digitalWrite(STP, LOW);
    delayMicroseconds(500);
  }
}

int main() {
  int i = 0;
  int16_t index = -1;
  int32_t panOffset;
  Block* block = NULL;

  wiringPiSetupGpio();  // use BCM pin numbering
  setup();
  signal(SIGINT, handle_SIGINT);
  
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
      index = -1;
      // If no target, stop spinning
    }
  }

  printf("Exiting...\n");
  return 0;
}
