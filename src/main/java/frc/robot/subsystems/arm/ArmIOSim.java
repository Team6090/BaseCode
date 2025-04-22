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

import static frc.robot.subsystems.arm.ArmConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ArmIOSim implements ArmIO {
  private DCMotorSim sim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX60(1), 0.004, gearRatioMotorToCANCoder),
          DCMotor.getKrakenX60(1));

  private PIDController armController = new PIDController(5, 0, 0.1);
  private boolean closedLoop = false;
  private double appliedVolts;

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    if (closedLoop) {
      appliedVolts = armController.calculate(sim.getAngularPositionRad());
    } else {
      armController.reset();
    }

    sim.setInputVoltage(MathUtil.clamp(appliedVolts, -12, 12));
    sim.update(0.02);

    inputs.armConnected = true;
    inputs.armPositionDeg = sim.getAngularPositionRad();
    inputs.armVelocityDegPerSec = sim.getAngularVelocityRadPerSec();
    inputs.armAppliedVolts = appliedVolts;
    inputs.armCurrentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    this.appliedVolts = volts;
  }

  @Override
  public void runToAngle(double angle) {
    closedLoop = true;
    armController.setSetpoint(angle);
  }

  @Override
  public double getCANCoderAngle() {
    return this.sim.getAngularPosition().magnitude();
  }
}
