package frc.robot.commands;

import static frc.robot.subsystems.vision.VisionConstants.poseFromTagFromTagId;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionConstants.SideOfTag;

public class AlignToAprilTag extends Command {
  private PIDController xController, yController, rotController;
  private Pose2d targetPoseFromTag, tagPose, adjustedPose;
  private int tagId;
  private boolean setByConstructor;
  private SideOfTag alignmentSide;
  private Timer stopTimer;
  private Drive drive;

  /**
   * Drive robot to specific pose based on AprilTag and designated side of tag
   *
   * @param drive : Drive subsystem
   * @param alignmentSide : Side of AprilTag to align with (left, middle, right)
   * @param tagId : Optional; Force-align to specific tag instead of using Limelight
   */
  public AlignToAprilTag(Drive drive, SideOfTag alignmentSide, int... tagId) {
    // Setup PID Controllers to control the robot's speed
    xController = new PIDController(12.5, 0, 0.5);
    yController = new PIDController(12.5, 0, 0.5);
    rotController = new PIDController(12.5, 0, 0.5);
    rotController.enableContinuousInput(0, 2 * Math.PI);
    this.alignmentSide = alignmentSide;
    this.drive = drive;
    if (tagId.length > 0) /* If tagId has an entry */ {
      // Use given tagId
      this.tagId = tagId[0];
      this.setByConstructor = true;
    } else /* If tagId has no entries */ {
      // Set tagId to a placeholder to update in initialize
      this.tagId = -1;
      this.setByConstructor = false;
    }
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    if (!setByConstructor) /* If tagId not manually set */ {
      // Use current seen tagId
      this.tagId = Vision.currentSeenTagId;
    }
    // Get pose adjustment from tag pose to target pose
    this.targetPoseFromTag =
        VisionConstants.poseFromTag(poseFromTagFromTagId(this.tagId, this.alignmentSide));
    // Get pose of seen AprilTag
    this.tagPose = VisionConstants.tagPoses[tagId - 1];
    // Add pose adjustment and tag pose to obtain target pose
    this.adjustedPose =
        new Pose2d(
            tagPose.getTranslation().plus(targetPoseFromTag.getTranslation()),
            Vision.rotationOppositeToTagRotation2d(tagId));
    // Log target pose
    Field2d field2d = new Field2d();
    SmartDashboard.putData("Field2d", field2d);
    field2d.setRobotPose(adjustedPose);
    // Start timer that will end the command
    this.stopTimer = new Timer();
    stopTimer.start();

    // Set targeted values for each PIDController
    rotController.setSetpoint(adjustedPose.getRotation().getRadians());
    rotController.setTolerance(0.1);

    xController.setSetpoint(adjustedPose.getX());
    xController.setTolerance(0.01);

    yController.setSetpoint(adjustedPose.getY());
    yController.setTolerance(0.01);
    SmartDashboard.putBoolean("AlignedFully?", false);
  }

  @Override
  public void execute() {
    SmartDashboard.putNumber("TargetPoseX", adjustedPose.getX());
    SmartDashboard.putNumber("TargetPoseY", adjustedPose.getY());
    SmartDashboard.putNumber("TargetPoseRot", adjustedPose.getRotation().getRadians());
    // Get robot speed in each direction
    // Multiplied by a constant to manually slow it down
    double xSpeed = 0.21 * xController.calculate(drive.getPose().getX());
    double ySpeed = 0.21 * yController.calculate(drive.getPose().getY());
    double rotSpeed = 0.21 * rotController.calculate(drive.getPose().getRotation().getRadians());
    SmartDashboard.putNumber("CalcXSpeed", xSpeed);
    SmartDashboard.putNumber("CalcYSpeed", ySpeed);
    SmartDashboard.putNumber("CalcRotSpeed", rotSpeed);

    // Drive the robot field-relative to the targeted speeds
    drive.runVelocityField(new ChassisSpeeds(xSpeed, ySpeed, rotSpeed));

    // End timer if at setpoint
    if (!rotController.atSetpoint() || !xController.atSetpoint() || !rotController.atSetpoint()) {
      stopTimer.reset();
    }
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("AlignedFully?", true);
    // Reset boolean for future usage and put wheels into x-mode
    VisionConstants.FREEME = false;
    drive.stopWithX();
  }

  @Override
  public boolean isFinished() {
    // Check the timer for completion or a boolean to manually end command
    return this.stopTimer.hasElapsed(0.1) || this.tagId == 23 || VisionConstants.FREEME;
  }
}
