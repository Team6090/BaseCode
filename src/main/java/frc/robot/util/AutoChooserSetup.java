package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutoChooserSetup {

  public static final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser());

  public static void setUpSysIdPaths(Drive drive) {
    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "RightSideStartJToOuterCoralStationToReefK",
        new PathPlannerAuto("StartToJToOuterCoralStationToK", true));
    autoChooser.addOption("RightSideStartToI", new PathPlannerAuto("StartToI", true));
    autoChooser.addOption(
        "RightSideStartToIToOuterStationToK",
        new PathPlannerAuto("StartToIToOuterCoralStationToK", true));
    autoChooser.addOption(
        "RightSideFastStartToIToOuterCoralStationToKToOuterCoralStationToL",
        new PathPlannerAuto("FastStartToIToOuterCoralStationToKToOuterCoralStationToL", true));
    autoChooser.addOption(
        "RightSideStartToIToOuterCoralStationToKToOuterCoralStation",
        new PathPlannerAuto("StartToIToOuterCoralStationToKToOuterCoralStation", true));
  }
}
