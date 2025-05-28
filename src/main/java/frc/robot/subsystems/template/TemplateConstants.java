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

package frc.robot.subsystems.template;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class TemplateConstants {
  public static final int templateMotorCANId = 41;
  public static final int templateEncoderCANId = 43;
  public static final String canbusName = "rio";

  public static final int templateStatorCurrentLimit = 200;
  public static final boolean templateStatorCurrentLimitEnable = true;
  public static final int templateSupplyCurrentLimit = 100;
  public static final boolean templateSupplyCurrentLimitEnable = true;

  public static final NeutralModeValue templateMotorNeutralControlMode = NeutralModeValue.Brake;
  public static final InvertedValue templateMotorInverted = InvertedValue.Clockwise_Positive;

  public static final FeedbackSensorSourceValue templateMotorFeedbackSensorMode =
      FeedbackSensorSourceValue.FusedCANcoder;
  public static final double gearRatioMotorToCANCoder =
      (52.0 / 12.0) * (64.0 / 16.0) * (24.0 / 50.0);
  public static final double gearRatioCANCoderToMechanism = 50.0 / 24.0;

  public static final double templatekS = 0.81096;
  public static final double templatekG = -0.81021;
  public static final double templatekV = 0.81336;
  public static final double templatekA = 1.242;
  public static final double templatekP = 130.0;
  public static final double templatekI = 0.0;
  public static final double templatekD = 1.5;

  public static final double templateForwardVoltageLimit = 12.0;
  public static final double templateReverseVoltageLimit = -12.0;

  public static final double templateMotionMagicCruiseVelocity = 0.5;
  public static final double templateMotionMagicAcceleration = 2;
  public static final double templateMotionMagicJerk = 3;

  public static final double templateEncoderMagnetOffset = 0.3425;
  public static final SensorDirectionValue templateEncoderSensorDirection =
      SensorDirectionValue.Clockwise_Positive;
  public static final double templateEncoderAbsoluteDiscontinuityPoint = 1;

  public static final double zeroPos = 0.1; // good
  public static final double onePos = 0.5; // good
  public static final double twoPos = 0.7; // good
  public static final double threePos = 0.9;
}
