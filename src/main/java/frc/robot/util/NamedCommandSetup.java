package frc.robot.util;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.GrabberIntake;
import frc.robot.commands.GrabberScore;
import frc.robot.commands.SetBargeTargetPos;
import frc.robot.commands.SetEleArmTargetPos;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmTargetAngle.TargetedArmAngle;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorTargetPosition.TargetedElevatorPosition;
import frc.robot.subsystems.grabber.Grabber;
import frc.robot.subsystems.grabber.GrabberSettings;
import frc.robot.subsystems.grabber.GrabberSettings.GrabberMode;
import frc.robot.subsystems.grabber.GrabberSettings.GrabberPieceMode;

public class NamedCommandSetup {

  public static void setupNamedCommands(Arm arm, Drive drive, Elevator elevator, Grabber grabber) {

    NamedCommands.registerCommand(
        "L4Prep", new SetEleArmTargetPos(TargetedElevatorPosition.REEF4, TargetedArmAngle.REEF4));
    NamedCommands.registerCommand(
        "L4Arm", new SetEleArmTargetPos(TargetedElevatorPosition.NONE, TargetedArmAngle.REEF4));

    NamedCommands.registerCommand(
        "L3Prep",
        new SetEleArmTargetPos(TargetedElevatorPosition.REEF3, TargetedArmAngle.REEF2AND3));

    NamedCommands.registerCommand(
        "IntakeCoralPrep",
        new SetEleArmTargetPos(TargetedElevatorPosition.INTAKECORAL, TargetedArmAngle.INTAKECORAL));

    NamedCommands.registerCommand(
        "LowerAlgaePrep",
        new SetEleArmTargetPos(
            TargetedElevatorPosition.INTAKELOWALGAE, TargetedArmAngle.INTAKELOWALGAE));

    NamedCommands.registerCommand(
        "UpperAlgaePrep",
        new SetEleArmTargetPos(
            TargetedElevatorPosition.INTAKEUPPERALGAE, TargetedArmAngle.INTAKEUPPERALGAE));

    NamedCommands.registerCommand(
        "ProcessorPrep",
        new SetEleArmTargetPos(TargetedElevatorPosition.PROCESSOR, TargetedArmAngle.PROCESSOR));

    NamedCommands.registerCommand(
        "HomePos", new SetEleArmTargetPos(TargetedElevatorPosition.HOME, TargetedArmAngle.HOME));

    NamedCommands.registerCommand("XMode", Commands.runOnce(() -> drive.stopWithX(), drive));

    NamedCommands.registerCommand(
        "HoldCoral", GrabberSettings.useGrabberCommand(grabber, () -> GrabberMode.HOLD));

    NamedCommands.registerCommand(
        "IntakeCoral", GrabberSettings.useGrabberCommand(grabber, () -> GrabberMode.INTAKECORAL));
    NamedCommands.registerCommand(
        "IntakeAlgae", GrabberSettings.useGrabberCommand(grabber, () -> GrabberMode.INTAKEALGAE));
    NamedCommands.registerCommand(
        "ScoreCoral", GrabberSettings.useGrabberCommand(grabber, () -> GrabberMode.SCOREREEF));
    NamedCommands.registerCommand(
        "ProcessorScore",
        GrabberSettings.useGrabberCommand(grabber, () -> GrabberMode.SCOREPROCESSOR));

    NamedCommands.registerCommand(
        "SetStartHeight",
        Commands.sequence(
            Commands.runOnce(() -> elevator.setElePos(ElevatorConstants.startHeight))
                .ignoringDisable(true),
            Commands.runOnce(() -> arm.setCANcoderOffset(ArmConstants.startAngle))
                .ignoringDisable(true)));

    NamedCommands.registerCommand(
        "OutOfStartHeight",
        new SetEleArmTargetPos(TargetedElevatorPosition.REEF3, TargetedArmAngle.NONE));

    NamedCommands.registerCommand(
        "IntakeCoralOverride", new GrabberIntake(grabber, GrabberPieceMode.OVERRIDE));

    NamedCommands.registerCommand("StopIntake", new GrabberIntake(grabber, GrabberPieceMode.NONE));

    NamedCommands.registerCommand("BargePrep", new SetBargeTargetPos());

    NamedCommands.registerCommand("BargeScore", new GrabberScore(grabber, GrabberPieceMode.BARGE));

    NamedCommands.registerCommand(
        "ArmOutOfWay",
        new SetEleArmTargetPos(TargetedElevatorPosition.INTAKELOWALGAE, TargetedArmAngle.BARGE));
  }
}
