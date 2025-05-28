package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.template.Template;
import frc.robot.subsystems.template.TemplateTargetPos;

public class RunTemplateToPos extends Command {

  private Template template;

  /**
   * Run template system to position set inside of TemplateTargetPos
   *
   * @param template : Template subsystem
   */
  public RunTemplateToPos(Template template) {
    this.template = template;
    addRequirements(template);
  }

  @Override
  public void execute() {
    template.runToPos(TemplateTargetPos.getTargetTemplatePos());
  }
}
