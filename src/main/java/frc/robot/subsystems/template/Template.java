package frc.robot.subsystems.template;
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

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volt;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class Template extends SubsystemBase {
  public static double mechanismPosition;
  Mechanism2d mechanismBase = new Mechanism2d(0, 0, new Color8Bit("#05b4ff"));
  MechanismRoot2d mechanismRoot = mechanismBase.getRoot("root", 0, 0);
  MechanismLigament2d mechanismLigament =
      mechanismRoot.append(
          new MechanismLigament2d("ligament", 1, 180, 10, new Color8Bit("#ff8a05")));
  private final TemplateIO io;
  private final TemplateIOInputsAutoLogged inputs = new TemplateIOInputsAutoLogged();

  public Template(TemplateIO io) {
    this.io = io;
    SmartDashboard.putNumber("MechanismLength", 0.1);
  }

  SysIdRoutine sysId =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Velocity.ofBaseUnits(0.5, VelocityUnit.combine(Volt, Seconds)),
              Units.Volts.of(3),
              null,
              (state) -> Logger.recordOutput("Template/SysIdState", state.toString())),
          // (state) -> SignalLogger.writeString("SysIDState", state.toString())),
          new SysIdRoutine.Mechanism((voltage) -> runVoltage(voltage.in(Volt)), null, this));

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runVoltage(0.0)).withTimeout(1.0).andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runVoltage(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Template", inputs);
    SmartDashboard.putData("Mechanism", mechanismBase);
    mechanismLigament.setAngle(90);
    mechanismLigament.setLength(SmartDashboard.getNumber("MechanismLength", 0.1));
    SmartDashboard.putNumber("TemplateMotorRot", getMotorRot());
    SmartDashboard.putNumber("TemplateEncoderRot", getEncoderRot());
    SmartDashboard.putNumber("TemplateActualRot", getActualRot());
    SmartDashboard.putString("TargetedTemplateRot", "" + TemplateTargetPos.getTargetPos());
  }

  public void runVoltage(double voltage) {
    io.runVoltage(voltage);
  }

  public void runToPos(double rot) {
    io.runToPos(rot);
  }

  public double getMotorRot() {
    return io.getMotorRot();
  }

  public double getEncoderRot() {
    return io.getEncoderRot();
  }

  public double getActualRot() {
    return io.getActualRot();
  }
}
