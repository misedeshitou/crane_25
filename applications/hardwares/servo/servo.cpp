#include "servo.hpp"

#include "cmsis_os.h"
#include "io/servo/servo.hpp"

namespace servo
{
sp::Servo servo(&htim1, TIM_CHANNEL_1, 168e6f, 180.0f);
HardwareMode servo_mode = HardwareMode::DISABLE;

void claw_open()
{
  servo_mode = HardwareMode::POSITION;
  servo.set(0.0f);  // Open claw
  osDelay(1000);    // Wait for 1 second
}
void claw_close()
{
  servo_mode = HardwareMode::POSITION;
  servo.set(90.0f);  // Close claw
  osDelay(1000);     // Wait for 1 second
}
void servo_disable() { servo_mode = HardwareMode::DISABLE; }

}  // namespace servo