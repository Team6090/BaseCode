package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ArmConstants.processorMidwayAngle;

import frc.robot.subsystems.grabber.GrabberSettings;
import frc.robot.subsystems.grabber.GrabberSettings.GrabberPieceMode;

public class ArmTargetAngle {

  private static ArmTargetAngle m_TargetLevel = new ArmTargetAngle();

  public static ArmTargetAngle getInstance() {
    return m_TargetLevel;
  }

  private static TargetedArmAngle targetArmAngle = TargetedArmAngle.NONE;

  public static enum TargetedArmAngle {
    NONE(0),
    INTAKECORAL(1),
    INTAKELOWALGAE(2),
    INTAKEUPPERALGAE(3),
    PROCESSOR(4),
    REEF1(5),
    REEF2AND3(6),
    REEF4(7),
    HOME(8),
    PROCESSORMIDWAY(9),
    OUTOFCLIMBER(10),
    BARGE(11),
    BARGEPRIME(12),
    INTAKECORALTOUCHINGSTATION(13);

    private int id;

    TargetedArmAngle(int id) {
      this.id = id;
    }

    public int getId() {
      return id;
    }
  }

  public static TargetedArmAngle getTargetArmPos() {
    return targetArmAngle;
  }

  public static void setArmTargetAngle(TargetedArmAngle targetArmAngle) {
    ArmTargetAngle.targetArmAngle = targetArmAngle;
  }

  // TODO Force Arm To Use Target Angle FeedForward
  public static double getTargetArmAngle() {
    switch (targetArmAngle.id) {
      case 0: /* None; Init Angle */
        return 0;
      case 1: /* Intake Coral Angle */
        if (Arm.mechanismPosition > ArmConstants.intakeCoralAngle + 170) {
          return Arm.mechanismPosition - 170;
        } else {
          return ArmConstants.intakeCoralAngle;
        }
      case 2: /* Intake Lower Algae Angle */
        if (Arm.mechanismPosition < ArmConstants.intakeLowAlgaeAngle - 170) {
          return Arm.mechanismPosition + 170;
        } else {
          return ArmConstants.intakeLowAlgaeAngle;
        }
      case 3: /* Intake Upper Algae Angle */
        if (Arm.mechanismPosition < ArmConstants.intakeUpAlgaeAngle - 170) {
          return Arm.mechanismPosition + 170;
        } else {
          return ArmConstants.intakeUpAlgaeAngle;
        }
      case 4: /* Processor Angle */
        if (Arm.mechanismPosition > ArmConstants.processorAngle + 170) {
          return Arm.mechanismPosition - 170;
        } else {
          return ArmConstants.processorAngle;
        }
      case 5: /* Level 1 Reef Angle */
        if (Arm.mechanismPosition > ArmConstants.l1Angle + 170) {
          return Arm.mechanismPosition - 170;
        } else {
          return ArmConstants.l1Angle;
        }
      case 6: /* Level 2 and 3 Reef Angle */
        if (Arm.mechanismPosition < ArmConstants.l2And3Angle - 170) {
          return Arm.mechanismPosition + 170;
        } else {
          return ArmConstants.l2And3Angle;
        }
      case 7: /* Level 4 Reef Angle */
        if (Arm.mechanismPosition < ArmConstants.l4Angle - 170) {
          return Arm.mechanismPosition + 170;
        } else {
          return ArmConstants.l4Angle;
        }
      case 8: /* Home Angle */
        if (Arm.mechanismPosition > ArmConstants.homeAngle + 170) {
          return Arm.mechanismPosition - 170;
        } else if (GrabberSettings.getPieceInGrabber() == GrabberPieceMode.ALGAE) {
          return ArmConstants.processorAngle;
        } else {
          return ArmConstants.homeAngle;
        }
      case 9: /* Processor Midway Angle */
        if (Arm.mechanismPosition > processorMidwayAngle + 170) {
          return Arm.mechanismPosition - 170;
        } else {
          return processorMidwayAngle;
        }
      case 10: /* Out Of Climber Angle */
        if (Arm.mechanismPosition > ArmConstants.outOfClimberAngle + 170) {
          return Arm.mechanismPosition - 170;
        } else {
          return ArmConstants.outOfClimberAngle;
        }
      case 11: /* Barge Angle */
        if (Arm.mechanismPosition < ArmConstants.bargeAngle - 170) {
          return Arm.mechanismPosition + 170;
        } else {
          return ArmConstants.bargeAngle;
        }
      case 12: /* Barge Prime Angle */
        if (Arm.mechanismPosition < ArmConstants.bargePrimeAngle - 170) {
          return Arm.mechanismPosition + 170;
        } else {
          return ArmConstants.bargePrimeAngle;
        }
      case 13: /* Intake Coral Angle Touching Station */
        if (Arm.mechanismPosition > ArmConstants.intakeCoralTouchingStationAngle + 170) {
          return Arm.mechanismPosition - 170;
        } else {
          return ArmConstants.intakeCoralTouchingStationAngle;
        }
      default:
        if (Arm.mechanismPosition > ArmConstants.homeAngle + 170) {
          return Arm.mechanismPosition - 170;
        } else if (GrabberSettings.getPieceInGrabber() == GrabberPieceMode.ALGAE) {
          return ArmConstants.processorAngle;
        } else {
          return ArmConstants.homeAngle;
        }
    }
  }
}
