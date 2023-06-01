
#pragma once

namespace canivore {
  constexpr int frontLeftDrive = 7;
  constexpr int frontLeftTurn = 8;
  constexpr int frontLeftEncoder = 10;

  constexpr int frontRightDrive = 5;
  constexpr int frontRightTurn = 6;
  constexpr int frontRightEncoder = 12;

  constexpr int backLeftDrive = 1;
  constexpr int backLeftTurn = 2;
  constexpr int backLeftEncoder = 11;

  constexpr int backRightDrive = 3;
  constexpr int backRightTurn = 4;
  constexpr int backRightEncoder = 9;
}

namespace canid {
  // Arm id: 10 - 19
  constexpr int armBottom = 10;
  constexpr int armBottomFollow = 11;
  constexpr int armTop = 13;
  constexpr int armTopFollow = 12;

  //Intake id: 20-29
  constexpr int intake = 20;
}
 
namespace dio {
    constexpr int armBottomSensor = 9;
    constexpr int breakModeSwitch = 6;
    constexpr int armTopSensor = 1;
    constexpr int coneSensor = 2;
    constexpr int clawClosedSensor = 5;
    constexpr int clawOpenSensor = 4;
    constexpr int intakeSensor = 8;
}

namespace pwm {
  constexpr int LEDs = 0;
}