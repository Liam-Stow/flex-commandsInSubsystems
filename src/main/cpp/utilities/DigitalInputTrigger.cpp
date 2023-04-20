#include "utilities/DigitalInputTrigger.h"

DigitalInputTrigger::DigitalInputTrigger(int channel)
    : DigitalInput(channel), Trigger([&] { return Get(); }){};
