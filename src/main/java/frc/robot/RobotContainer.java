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

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.frontLeftLimelightName;
import static frc.robot.subsystems.vision.VisionConstants.frontRightLimelightName;
import static frc.robot.subsystems.vision.VisionConstants.rearLimelightName;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AlignToAprilTag;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.RunTemplateToPos;
import frc.robot.commands.SetTemplateTargetPos;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.template.Template;
import frc.robot.subsystems.template.TemplateIO;
import frc.robot.subsystems.template.TemplateIOSim;
import frc.robot.subsystems.template.TemplateIOTalonFX;
import frc.robot.subsystems.template.TemplateTargetPos.TargetedPos;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionConstants.SideOfTag;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.util.AutoChooserSetup;
import frc.robot.util.NamedCommandSetup;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Template template;
  private final Vision vision;

  // Controller
  private final CommandXboxController mainController = new CommandXboxController(0);
  private final CommandXboxController secondaryController = new CommandXboxController(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        template = new Template(new TemplateIOTalonFX());
        vision =
            new Vision(
                drive,
                new VisionIOLimelight(frontLeftLimelightName, drive::getRotation),
                new VisionIOLimelight(frontRightLimelightName, drive::getRotation),
                new VisionIOLimelight(rearLimelightName, drive::getRotation));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        drive.setPose(new Pose2d(3.2, 4, new Rotation2d()));
        template = new Template(new TemplateIOSim());
        vision = new Vision(drive);
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        template = new Template(new TemplateIO() {});
        vision = new Vision(drive);
        break;
    }

    NamedCommandSetup.setupNamedCommands(template);
    AutoChooserSetup.setUpSysIdPaths(drive);
    template.templateAppend();
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -mainController.getLeftY(),
            () -> -mainController.getLeftX(),
            () -> -mainController.getRightX()));

    template.setDefaultCommand(new RunTemplateToPos(template));

    mainController.y().onTrue(new SetTemplateTargetPos(TargetedPos.THREE));

    mainController.x().onTrue(new SetTemplateTargetPos(TargetedPos.TWO));

    mainController.b().onTrue(new SetTemplateTargetPos(TargetedPos.ONE));

    mainController.a().onTrue(new SetTemplateTargetPos(TargetedPos.NONE));

    mainController.leftBumper().onTrue(new AlignToAprilTag(drive, SideOfTag.LEFT));

    mainController.rightBumper().onTrue(new AlignToAprilTag(drive, SideOfTag.RIGHT));

    mainController.back().onTrue(new AlignToAprilTag(drive, SideOfTag.MIDDLE));

    mainController.leftStick().onTrue(Commands.runOnce(() -> VisionConstants.FREEME = true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return AutoChooserSetup.autoChooser.get();
  }
}
