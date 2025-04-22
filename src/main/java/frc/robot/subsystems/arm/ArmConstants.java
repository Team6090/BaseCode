// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.arm;

public class ArmConstants {
  public static final int armCanId = 41;
  public static final int armShaftEncoderId = 43;

  public static final double gearRatioMotorToCANCoder =
      (52.0 / 12.0) * (64.0 / 16.0) * (24.0 / 50.0);
  public static final double gearRatioCANCoderToMechanism = 50.0 / 24.0;

  public static final int armStatorCurrentLimit = 200;
  public static final boolean armStatorCurrentLimitEnable = true;
  public static final int armSupplyCurrentLimit = 100;
  public static final boolean armSupplyCurrentLimitEnable = true;

  public static final double armForwardVoltageLimit = 12.0;
  public static final double armReverseVoltageLimit = -12.0;

  public static final double armkS = 0.81096;
  public static final double armkG = -0.81021;
  public static final double armkV = 0.81336;
  public static final double armkA = 1.242;
  public static final double armkP = 130.0;
  public static final double armkI = 0.0;
  public static final double armkD = 1.5;

  public static final double armMotionMagicCruiseVelocity = 0.5;
  public static final double armMotionMagicAcceleration = 2;
  public static final double armMotionMagicJerk = 3;

  public static final double armCANCoderMagnetOffset = 0.3425;

  // TODO Get real values from arm for each objective
  public static final double l4Angle = 326.5; // good
  public static final double l2And3Angle = 295; // good
  public static final double l1Angle = 295; // good
  public static final double intakeCoralAngle = 115; // good
  public static final double intakeCoralTouchingStationAngle = 100; // good
  public static final double intakeLowAlgaeAngle = 347.0;
  public static final double intakeUpAlgaeAngle = 347.0;
  public static final double processorAngle = 121.25;
  public static final double processorMidwayAngle = 145.0;
  public static final double outOfClimberAngle = 125.0;
  public static final double bargeAngle = 280.0;
  public static final double bargePrimeAngle = 280.0;
  public static final double startAngle = 80; // good
  public static final double homeAngle = 115.0;
}
