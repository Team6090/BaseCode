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

import static frc.robot.subsystems.template.TemplateConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class TemplateIOSim implements TemplateIO {
  private DCMotorSim sim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX60(1), 0.004, gearRatioMotorToCANCoder),
          DCMotor.getKrakenX60(1));

  private PIDController templateController = new PIDController(5, 0, 0.1);
  private boolean closedLoop = false;
  private double appliedVolts;
  private double targetRot = 0;

  @Override
  public void updateInputs(TemplateIOInputs inputs) {
    if (closedLoop) {
      appliedVolts = templateController.calculate(sim.getAngularPositionRad());
    } else {
      templateController.reset();
    }

    sim.setInputVoltage(MathUtil.clamp(appliedVolts, -12, 12));
    sim.update(0.02);

    inputs.templateConnected = true;
    inputs.templateMotorPositionRot = sim.getAngularPositionRotations();
    inputs.templateMotorVelocityRotPerSec = sim.getAngularVelocityRPM() / 60;
    inputs.templateTargetPositionRot = targetRot;
    inputs.templateAppliedVolts = appliedVolts;
    inputs.templateCurrentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void runVoltage(double volts) {
    closedLoop = false;
    this.appliedVolts = volts;
  }

  @Override
  public void runToPos(double rot) {
    closedLoop = true;
    templateController.setSetpoint(rot);
  }

  @Override
  public double getMotorRot() {
    return sim.getAngularPositionRotations();
  }

  @Override
  public double getEncoderRot() {
    return 0;
  }

  @Override
  public double getActualRot() {
    return targetRot;
  }
}
