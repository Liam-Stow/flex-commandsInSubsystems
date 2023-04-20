#pragma once

#include <frc2/command/button/Trigger.h>
#include <frc/DigitalInput.h>

class DigitalInputTrigger : public frc::DigitalInput, public frc2::Trigger {
 public:
  DigitalInputTrigger(int channel);
};
