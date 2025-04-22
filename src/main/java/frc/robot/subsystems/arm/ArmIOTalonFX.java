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
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.arm.ArmTargetAngle.TargetedArmAngle;

/** This arm implementation is for a Talon FX driving a motor like the Falon 500 or Kraken X60. */
public class ArmIOTalonFX implements ArmIO {
  private final TalonFX arm = new TalonFX(armCanId, "Drivetrain");
  private final CANcoder armShaftEncoder = new CANcoder(armShaftEncoderId, "Drivetrain");

  private final StatusSignal<Angle> armPositionRot = armShaftEncoder.getPosition();
  private final StatusSignal<AngularVelocity> armVelocityRotPerSec = armShaftEncoder.getVelocity();
  private double armMotorTemp =
      arm.getDeviceTemp().getValue().in(edu.wpi.first.units.Units.Fahrenheit);
  private final StatusSignal<Voltage> armAppliedVolts = arm.getMotorVoltage();
  private final StatusSignal<Current> armCurrentAmps = arm.getSupplyCurrent();

  private final MotionMagicVoltage mmv = new MotionMagicVoltage(0);
  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  private double armFeedforward = 0.0;
  private double armTarget;

  public ArmIOTalonFX() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = armStatorCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = armStatorCurrentLimitEnable;
    config.CurrentLimits.SupplyCurrentLimit = armSupplyCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = armSupplyCurrentLimitEnable;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.Feedback.FeedbackRemoteSensorID = armShaftEncoderId;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    config.Feedback.RotorToSensorRatio = gearRatioMotorToCANCoder;
    config.Feedback.SensorToMechanismRatio = gearRatioCANCoderToMechanism;
    config.ClosedLoopGeneral.ContinuousWrap = true;

    config.Slot0.kP = armkP;
    config.Slot0.kI = armkI;
    config.Slot0.kD = armkD;

    config.Voltage.PeakForwardVoltage = armForwardVoltageLimit;
    config.Voltage.PeakReverseVoltage = armReverseVoltageLimit;

    config.MotionMagic.MotionMagicCruiseVelocity = armMotionMagicCruiseVelocity;
    config.MotionMagic.MotionMagicAcceleration = armMotionMagicAcceleration;
    config.MotionMagic.MotionMagicJerk = armMotionMagicJerk;

    tryUntilOk(5, () -> arm.getConfigurator().apply(config, 0.25));

    CANcoderConfiguration armCANcoderConfig = new CANcoderConfiguration();

    armCANcoderConfig.MagnetSensor.MagnetOffset = armCANCoderMagnetOffset;
    armCANcoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    armCANcoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;

    tryUntilOk(5, () -> armShaftEncoder.getConfigurator().apply(armCANcoderConfig, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, armPositionRot, armVelocityRotPerSec, armAppliedVolts, armCurrentAmps);
    arm.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        armPositionRot, armVelocityRotPerSec, armAppliedVolts, armCurrentAmps);

    armShaftEncoder.getPosition().refresh();
    arm.getPosition().refresh();
    inputs.armPositionDeg = Units.rotationsToDegrees(armPositionRot.getValueAsDouble());
    inputs.armTargetPositionDeg = ArmTargetAngle.getTargetArmAngle();
    inputs.armVelocityDegPerSec = Units.rotationsToDegrees(armVelocityRotPerSec.getValueAsDouble());
    inputs.armMotorTemp = armMotorTemp;
    inputs.armAppliedVolts = armAppliedVolts.getValueAsDouble();
    inputs.armCurrentAmps = armCurrentAmps.getValueAsDouble();

    SmartDashboard.putNumber("ArmFeedforward", armFeedforward);

    if (ArmTargetAngle.getTargetArmPos() != TargetedArmAngle.NONE) {
      armFeedforward =
          (new ArmFeedforward(armkS, armkG, armkV, armkA)
              .calculate(Units.degreesToRadians(Arm.mechanismPosition), 0));
    }
  }

  @Override
  public void setVoltage(double volts) {
    arm.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void runToAngle(double degrees) {
    armTarget = degrees;
    SmartDashboard.putNumber("CANcoderTarget", degrees);
    double cancoderTarget = Units.degreesToRotations(degrees);
    arm.setControl(mmv.withPosition(cancoderTarget).withSlot(0).withFeedForward(armFeedforward));
  }

  @Override
  public double getCANCoderAngle() {
    return Units.rotationsToDegrees(armShaftEncoder.getPosition().getValueAsDouble());
  }

  @Override
  public double getActualAngle() {
    return Units.rotationsToDegrees(
        armShaftEncoder.getPosition().getValueAsDouble() / gearRatioCANCoderToMechanism);
  }

  @Override
  public double getMotorPos() {
    return arm.getPosition().getValueAsDouble();
  }

  public double degreesToCANcoder(double deg) {
    return deg * gearRatioCANCoderToMechanism;
  }

  @Override
  public void adjustCANcoderOffset(double value) {
    armShaftEncoder.setPosition(
        armShaftEncoder.getPosition().getValueAsDouble()
            + Units.degreesToRotations(value * gearRatioCANCoderToMechanism));
  }

  @Override
  public void setCANcoderOffset(double value) {
    armShaftEncoder.setPosition(Units.degreesToRotations(value * gearRatioCANCoderToMechanism));
  }
}
