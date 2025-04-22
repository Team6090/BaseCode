package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.ArmTargetAngle;
import frc.robot.subsystems.arm.ArmTargetAngle.TargetedArmAngle;
import frc.robot.subsystems.elevator.ElevatorTargetPosition;
import frc.robot.subsystems.elevator.ElevatorTargetPosition.TargetedElevatorPosition;
import frc.robot.subsystems.grabber.GrabberSettings;

public class SetEleArmTargetPos extends Command {

  private TargetedElevatorPosition targetElePos;
  private TargetedArmAngle targetArmAngle;
  private boolean done = false;

  /**
   * Set target positions for the elevator and the arm
   *
   * @param targetElePos : Targeted elevator height
   * @param targetArmAngle : Targeted arm angle
   */
  public SetEleArmTargetPos(
      TargetedElevatorPosition targetElePos, TargetedArmAngle targetArmAngle) {
    this.targetElePos = targetElePos;
    this.targetArmAngle = targetArmAngle;
    addRequirements();
  }

  @Override
  public void initialize() {
    if (targetElePos.getId() == 0) {

    } else {
      ElevatorTargetPosition.setEleTargetPos(targetElePos);
    }

    if (targetArmAngle.getId() == 0) {

    } else {
      ArmTargetAngle.setArmTargetAngle(targetArmAngle);
      GrabberSettings.setGrabberModeFromArmMode();
    }
  }

  @Override
  public void execute() {
    done = true;
  }

  @Override
  public void end(boolean interrupted) {
    done = false;
  }

  @Override
  public boolean isFinished() {
    return done;
  }
}
