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

import org.littletonrobotics.junction.AutoLog;

public interface TemplateIO {
  @AutoLog
  public static class TemplateIOInputs {
    public boolean templateConnected = false;

    public double templateMotorPositionRot = 0.0;
    public double templateMotorVelocityRotPerSec = 0.0;
    public double templateEncoderPositionRot = 0.0;
    public double templateEncoderVelocityRotPerSec = 0.0;
    public double templateTargetPositionRot = 0.0;
    public double templateMotorTemp = 0.0;
    public double templateAppliedVolts = 0.0;
    public double templateCurrentAmps = 0.0;
  }

  /** Update the set of loggable inputs. */
  public default void updateInputs(TemplateIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void runVoltage(double volts) {}

  /** Run closed loop to the specified position */
  public default void runToPos(double rot) {}

  /** Get value of motor rotations */
  public default double getMotorRot() {
    return 0;
  }

  /** Get value of encoder rotations */
  public default double getEncoderRot() {
    return 0;
  }

  /** Get value of mechanism rotations */
  public default double getActualRot() {
    return 0;
  }
}
