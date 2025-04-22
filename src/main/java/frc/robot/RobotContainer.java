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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AlignToAprilTag;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.GrabberIntake;
import frc.robot.commands.GrabberScore;
import frc.robot.commands.MoveArmToPos;
import frc.robot.commands.MoveEleToPos;
import frc.robot.commands.SetBargeTargetPos;
import frc.robot.commands.SetEleArmTargetPos;
import frc.robot.commands.SetProcessorTargetPos;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmIOTalonFX;
import frc.robot.subsystems.arm.ArmTargetAngle.TargetedArmAngle;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOTalonFX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.elevator.ElevatorTargetPosition.TargetedElevatorPosition;
import frc.robot.subsystems.grabber.Grabber;
import frc.robot.subsystems.grabber.GrabberIO;
import frc.robot.subsystems.grabber.GrabberIOTalonFX;
import frc.robot.subsystems.grabber.GrabberSettings;
import frc.robot.subsystems.grabber.GrabberSettings.GrabberMode;
import frc.robot.subsystems.grabber.GrabberSettings.GrabberPieceMode;
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
  private final Climber climber;
  private final Drive drive;
  private final Elevator elevator;
  private final Arm arm;
  private final Grabber grabber;
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
        climber = new Climber(new ClimberIOTalonFX());
        elevator = new Elevator(new ElevatorIOTalonFX());
        arm = new Arm(new ArmIOTalonFX());
        grabber = new Grabber(new GrabberIOTalonFX());
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
        elevator = new Elevator(new ElevatorIOSim());
        arm = new Arm(new ArmIOSim());
        grabber = new Grabber(new GrabberIO() {});
        climber = new Climber(new ClimberIO() {});
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
        elevator = new Elevator(new ElevatorIO() {});
        arm = new Arm(new ArmIO() {});
        grabber = new Grabber(new GrabberIO() {});
        climber = new Climber(new ClimberIO() {});
        vision = new Vision(drive);
        break;
    }

    NamedCommandSetup.setupNamedCommands(arm, drive, elevator, grabber);
    AutoChooserSetup.setUpSysIdPaths(drive);
    // TODO Figure out how to show the trajectory of the chosen path on the smart dashboard
    // Configure the button bindings
    elevator.append();
    arm.armAppend();
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

    arm.setDefaultCommand(new MoveArmToPos(arm));
    elevator.setDefaultCommand(new MoveEleToPos(elevator));

    mainController
        .y()
        .onTrue(new SetEleArmTargetPos(TargetedElevatorPosition.REEF4, TargetedArmAngle.REEF4));
    mainController
        .x()
        .onTrue(new SetEleArmTargetPos(TargetedElevatorPosition.REEF3, TargetedArmAngle.REEF2AND3));
    mainController
        .b()
        .onTrue(new SetEleArmTargetPos(TargetedElevatorPosition.REEF2, TargetedArmAngle.REEF2AND3));
    mainController
        .start()
        .onTrue(
            new SetEleArmTargetPos(
                TargetedElevatorPosition.INTAKECORALTOUCHINGSTATION,
                TargetedArmAngle.INTAKECORALTOUCHINGSTATION));

    mainController
        .povRight()
        .onTrue(
            new SetEleArmTargetPos(
                TargetedElevatorPosition.INTAKECORAL, TargetedArmAngle.INTAKECORAL));
    mainController
        .povUp()
        .onTrue(
            new SetEleArmTargetPos(
                TargetedElevatorPosition.INTAKEUPPERALGAE, TargetedArmAngle.INTAKEUPPERALGAE));
    mainController
        .povLeft()
        .onTrue(
            new SetEleArmTargetPos(
                TargetedElevatorPosition.INTAKELOWALGAE, TargetedArmAngle.INTAKEUPPERALGAE));
    mainController
        .povDown()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> GrabberSettings.setGrabberMode(GrabberMode.SCOREPROCESSOR)),
                new SetProcessorTargetPos()));

    mainController
        .leftTrigger(0.1)
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> GrabberSettings.setGrabberMode(GrabberMode.SCOREBARGE)),
                new SetBargeTargetPos()));

    mainController.rightTrigger(0.1).onTrue(GrabberSettings.grabberCommandFromSetMode(grabber));

    mainController.leftBumper().onTrue(new AlignToAprilTag(drive, SideOfTag.LEFT));

    mainController.rightBumper().onTrue(new AlignToAprilTag(drive, SideOfTag.RIGHT));

    mainController.back().onTrue(new AlignToAprilTag(drive, SideOfTag.MIDDLE));

    mainController
        .a()
        .onTrue(
            new SetEleArmTargetPos(
                TargetedElevatorPosition.INTAKECORAL, TargetedArmAngle.BARGEPRIME));

    mainController.leftStick().onTrue(Commands.runOnce(() -> VisionConstants.FREEME = true));

    secondaryController
        .a()
        .onTrue(
            new SetEleArmTargetPos(
                TargetedElevatorPosition.OUTOFCLIMBER, TargetedArmAngle.OUTOFCLIMBER));
    secondaryController.b().whileTrue(new GrabberScore(grabber, GrabberPieceMode.OVERRIDE));
    secondaryController.x().whileTrue(new GrabberIntake(grabber, GrabberPieceMode.OVERRIDE));
    secondaryController.y().whileTrue(climber.runPercent(0.2));
    secondaryController.y().onFalse(climber.runPercent(0));

    secondaryController
        .leftStick()
        .and(secondaryController.rightStick())
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> elevator.setElePos(ElevatorConstants.startHeight))
                    .ignoringDisable(true),
                Commands.runOnce(() -> arm.setCANcoderOffset(ArmConstants.startAngle))
                    .ignoringDisable(true)));
    secondaryController
        .back()
        .onTrue(new SetEleArmTargetPos(TargetedElevatorPosition.REEF3, TargetedArmAngle.NONE));
    secondaryController
        .leftBumper()
        .onTrue(
            Commands.runOnce(
                () ->
                    drive.setPose(
                        new Pose2d(
                            drive.getPose().getTranslation(),
                            new Rotation2d(Units.degreesToRadians(0))))));

    secondaryController
        .povUp()
        .onTrue(Commands.runOnce(() -> elevator.changeElePos(0.025)).ignoringDisable(true));
    secondaryController
        .povDown()
        .onTrue(Commands.runOnce(() -> elevator.changeElePos(-0.025)).ignoringDisable(true));
    secondaryController
        .povLeft()
        .onTrue(Commands.runOnce(() -> arm.adjustCANcoderOffset(1)).ignoringDisable(true));
    secondaryController
        .povRight()
        .onTrue(Commands.runOnce(() -> arm.adjustCANcoderOffset(-1)).ignoringDisable(true));

    climberUp()
        .whileTrue(
            Commands.run(
                () ->
                    climber.runVoltageFromPercent(() -> -secondaryController.getRightTriggerAxis()),
                climber));
    climberUp().onFalse(climber.runPercent(0));

    climberDown()
        .whileTrue(
            Commands.run(
                () -> climber.runVoltageFromPercent(() -> secondaryController.getLeftTriggerAxis()),
                climber));
    climberDown().onFalse(climber.runPercent(0));

    // killClimber().onTrue(climber.runPercent(0));
  }

  private Trigger climberDown() {
    return secondaryController
        .leftTrigger(0.1)
        .and(() -> (!Climber.hitLowerLimit || mainController.rightStick().getAsBoolean()));
  }

  private Trigger climberUp() {
    return secondaryController
        .rightTrigger(0.1)
        .and(() -> (!Climber.hitUpperLimit || mainController.rightStick().getAsBoolean()));
  }

  private Trigger killClimber() {
    return new Trigger(() -> (climber.climberAtUpperLimit() || climber.climberAtLowerLimit()));
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
