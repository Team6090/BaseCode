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

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public boolean armConnected = false;

    public double armPositionDeg = 0.0;
    public double armTargetPositionDeg = 0.0;
    public double armVelocityDegPerSec = 0.0;
    public double armMotorTemp = 0.0;
    public double armAppliedVolts = 0.0;
    public double armCurrentAmps = 0.0;
  }

  /** Update the set of loggable inputs. */
  public default void updateInputs(ArmIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  public default void runToAngle(double angle) {}

  public default double getCANCoderAngle() {
    return 0;
  }

  public default double getActualAngle() {
    return 0;
  }

  public static double getActualAngleAsAStatic() {
    return 0;
  }

  public default double getMotorAngle() {
    return 0;
  }

  public default double getMotorPos() {
    return 0;
  }

  public default void adjustCANcoderOffset(double value) {}

  public default void setCANcoderOffset(double value) {}
}
