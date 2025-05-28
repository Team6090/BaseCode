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
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/**
 * This template implementation is for a Talon FX driving a motor like the Falon 500 or Kraken X60.
 */
public class TemplateIOTalonFX implements TemplateIO {
  private final TalonFX templateMotor = new TalonFX(templateMotorCANId, canbusName);
  private final CANcoder templateEncoder = new CANcoder(templateEncoderCANId, canbusName);

  private final StatusSignal<Angle> templateMotorPositionRot = templateMotor.getPosition();
  private final StatusSignal<AngularVelocity> templateMotorVelocityRotPerSec =
      templateMotor.getVelocity();
  private final StatusSignal<Angle> templateEncoderPositionRot = templateEncoder.getPosition();
  private final StatusSignal<AngularVelocity> templateEncoderVelocityRotPerSec =
      templateEncoder.getVelocity();
  private final StatusSignal<Temperature> templateMotorTemp = templateMotor.getDeviceTemp();
  private final StatusSignal<Voltage> templateAppliedVolts = templateMotor.getMotorVoltage();
  private final StatusSignal<Current> templateCurrentAmps = templateMotor.getSupplyCurrent();

  private final MotionMagicVoltage mmvRequest = new MotionMagicVoltage(0);
  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  private double templateTarget;

  public TemplateIOTalonFX() {
    TalonFXConfiguration templateMotorConfig = new TalonFXConfiguration();
    templateMotorConfig.CurrentLimits.StatorCurrentLimit = templateStatorCurrentLimit;
    templateMotorConfig.CurrentLimits.StatorCurrentLimitEnable = templateStatorCurrentLimitEnable;
    templateMotorConfig.CurrentLimits.SupplyCurrentLimit = templateSupplyCurrentLimit;
    templateMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = templateSupplyCurrentLimitEnable;

    templateMotorConfig.MotorOutput.NeutralMode = templateMotorNeutralControlMode;
    templateMotorConfig.MotorOutput.Inverted = templateMotorInverted;

    templateMotorConfig.Feedback.FeedbackRemoteSensorID = templateEncoderCANId;
    templateMotorConfig.Feedback.FeedbackSensorSource = templateMotorFeedbackSensorMode;
    templateMotorConfig.Feedback.RotorToSensorRatio = gearRatioMotorToCANCoder;
    templateMotorConfig.Feedback.SensorToMechanismRatio = gearRatioCANCoderToMechanism;

    templateMotorConfig.Slot0.kS = templatekS;
    templateMotorConfig.Slot0.kG = templatekG;
    templateMotorConfig.Slot0.kV = templatekV;
    templateMotorConfig.Slot0.kA = templatekA;
    templateMotorConfig.Slot0.kP = templatekP;
    templateMotorConfig.Slot0.kI = templatekI;
    templateMotorConfig.Slot0.kD = templatekD;

    templateMotorConfig.Voltage.PeakForwardVoltage = templateForwardVoltageLimit;
    templateMotorConfig.Voltage.PeakReverseVoltage = templateReverseVoltageLimit;

    templateMotorConfig.MotionMagic.MotionMagicCruiseVelocity = templateMotionMagicCruiseVelocity;
    templateMotorConfig.MotionMagic.MotionMagicAcceleration = templateMotionMagicAcceleration;
    templateMotorConfig.MotionMagic.MotionMagicJerk = templateMotionMagicJerk;

    tryUntilOk(5, () -> templateMotor.getConfigurator().apply(templateMotorConfig, 0.25));

    CANcoderConfiguration templateCANcoderConfig = new CANcoderConfiguration();

    templateCANcoderConfig.MagnetSensor.MagnetOffset = templateEncoderMagnetOffset;
    templateCANcoderConfig.MagnetSensor.SensorDirection = templateEncoderSensorDirection;
    templateCANcoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint =
        templateEncoderAbsoluteDiscontinuityPoint;

    tryUntilOk(5, () -> templateEncoder.getConfigurator().apply(templateCANcoderConfig, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        templateMotorPositionRot,
        templateMotorVelocityRotPerSec,
        templateEncoderPositionRot,
        templateEncoderVelocityRotPerSec,
        templateMotorTemp,
        templateAppliedVolts,
        templateCurrentAmps);
    templateMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(TemplateIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        templateMotorPositionRot,
        templateMotorVelocityRotPerSec,
        templateEncoderPositionRot,
        templateEncoderVelocityRotPerSec,
        templateMotorTemp,
        templateAppliedVolts,
        templateCurrentAmps);
    inputs.templateMotorPositionRot = templateMotorPositionRot.getValueAsDouble();
    inputs.templateMotorVelocityRotPerSec = templateMotorVelocityRotPerSec.getValueAsDouble();
    inputs.templateEncoderPositionRot = templateMotorPositionRot.getValueAsDouble();
    inputs.templateEncoderVelocityRotPerSec = templateMotorVelocityRotPerSec.getValueAsDouble();
    inputs.templateTargetPositionRot = TemplateTargetPos.getTargetTemplatePos();
    inputs.templateMotorTemp = templateMotorTemp.getValueAsDouble();
    inputs.templateAppliedVolts = templateAppliedVolts.getValueAsDouble();
    inputs.templateCurrentAmps = templateCurrentAmps.getValueAsDouble();
  }

  @Override
  public void runVoltage(double volts) {
    templateMotor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void runToPos(double rot) {
    templateTarget = rot;
    templateMotor.setControl(mmvRequest.withPosition(rot).withSlot(0));
  }

  @Override
  public double getMotorRot() {
    return templateMotor.getPosition().getValueAsDouble();
  }

  @Override
  public double getEncoderRot() {
    return templateEncoder.getPosition().getValueAsDouble();
  }

  @Override
  public double getActualRot() {
    return templateEncoder.getPosition().getValueAsDouble() / gearRatioCANCoderToMechanism;
  }
}
