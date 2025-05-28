package frc.robot.util;

import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.commands.SetTemplateTargetPos;
import frc.robot.subsystems.template.Template;
import frc.robot.subsystems.template.TemplateTargetPos.TargetedPos;

public class NamedCommandSetup {

  public static void setupNamedCommands(Template template) {
    NamedCommands.registerCommand("ZeroPos", new SetTemplateTargetPos(TargetedPos.NONE));
    NamedCommands.registerCommand("OnePos", new SetTemplateTargetPos(TargetedPos.ONE));
    NamedCommands.registerCommand("TwoPos", new SetTemplateTargetPos(TargetedPos.TWO));
    NamedCommands.registerCommand("ThreePos", new SetTemplateTargetPos(TargetedPos.THREE));
  }
}
