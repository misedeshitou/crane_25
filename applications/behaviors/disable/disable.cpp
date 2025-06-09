#include "disable.hpp"

// hardwares
#include "hardwares/frame/frame.hpp"

void disable()
{
  frame::disable_x();
  frame::disable_z();
}