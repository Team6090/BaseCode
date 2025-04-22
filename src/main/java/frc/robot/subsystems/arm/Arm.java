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

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volt;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.elevator.Elevator;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  public static double mechanismPosition;
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  private final MechanismLigament2d armLigament =
      new MechanismLigament2d("arm", 0.3, 0, 3, new Color8Bit(0, 255, 0));

  public Arm(ArmIO io) {
    this.io = io;
  }

  SysIdRoutine sysId =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Velocity.ofBaseUnits(0.5, VelocityUnit.combine(Volt, Seconds)),
              Units.Volts.of(3),
              null,
              (state) -> Logger.recordOutput("Arm/SysIdState", state.toString())),
          // (state) -> SignalLogger.writeString("SysIDState", state.toString())),
          new SysIdRoutine.Mechanism((voltage) -> driveVoltage(voltage.in(Volt)), null, this));

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runPercent(0.0)).withTimeout(1.0).andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runPercent(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
    armLigament.setAngle(180 + getCANCoderAngle());
    SmartDashboard.putNumber("CANCoderArmAngle", getCANCoderAngle());
    SmartDashboard.putString("MechanismPosition", String.format("%.2f", mechanismPosition));
    SmartDashboard.putString("TargetedArmAngle", "" + ArmTargetAngle.getTargetArmPos());
    // SmartDashboard.putNumber("TargetedArmID", ArmTargetAngle.getTargetArmPos().getId());
    mechanismPosition =
        (getActualAngle() % 360) > 0 ? (getActualAngle() % 360) : (getActualAngle() % 360) + 360;
  }

  public Command runPercent(double percent) {
    return runEnd(() -> io.setVoltage(percent * 12.0), () -> io.setVoltage(0.0));
  }

  public void driveVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public Command runToAngle(double angle) {
    return run(() -> io.runToAngle(angle));
  }

  public void runToAngleVoid(double angle) {
    io.runToAngle(angle);
  }

  public Command runTeleop(DoubleSupplier forward, DoubleSupplier reverse) {
    return runEnd(
        () -> io.setVoltage((forward.getAsDouble() - reverse.getAsDouble()) * 12.0),
        () -> io.setVoltage(0.0));
  }

  public double getCANCoderAngle() {
    return io.getCANCoderAngle();
  }

  public double getActualAngle() {
    return io.getActualAngle();
  }

  public double getMotorAngle() {
    return io.getMotorAngle();
  }

  public double getMotorPos() {
    return io.getMotorPos();
  }

  public void armAppend() {
    Elevator.eleRightLigament.append(armLigament);
  }

  public void adjustCANcoderOffset(double value) {
    io.adjustCANcoderOffset(value);
  }

  public void setCANcoderOffset(double value) {
    io.setCANcoderOffset(value);
  }
}
