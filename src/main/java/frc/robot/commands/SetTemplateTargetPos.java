package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.template.TemplateTargetPos;
import frc.robot.subsystems.template.TemplateTargetPos.TargetedPos;

public class SetTemplateTargetPos extends Command {

  private TargetedPos targetPos;
  private boolean done = false;

  /**
   * Set target position for the template system
   *
   * @param targetPos : Targeted position
   */
  public SetTemplateTargetPos(TargetedPos targetPos) {
    this.targetPos = targetPos;
    addRequirements();
  }

  @Override
  public void initialize() {
    TemplateTargetPos.setTemplateTargetPos(targetPos);
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
