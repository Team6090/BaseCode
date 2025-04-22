package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmTargetAngle;

public class MoveArmToPos extends Command {

  private Arm arm;

  /**
   * Run arm to position set inside of ArmTargetAngle
   *
   * @param arm : Arm subsystem
   */
  public MoveArmToPos(Arm arm) {
    this.arm = arm;
    addRequirements(arm);
  }

  @Override
  public void execute() {
    if (ArmTargetAngle.getTargetArmPos().getId() == 0) /* If no position is set */ {
      // Don't run anything
    } else /* If a position is set */ {
      // Run arm to position; this will constantly update if the target position changes as long as
      // the command is running
      arm.runToAngleVoid(ArmTargetAngle.getTargetArmAngle());
    }
  }
}
