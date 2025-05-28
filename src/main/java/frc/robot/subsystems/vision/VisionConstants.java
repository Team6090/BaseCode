// Copyright 2021-2024 FRC 6328
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

package frc.robot.subsystems.vision;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
  public static PoseFromTag poseFromTag = PoseFromTag.NONE;

  public static boolean FREEME = false;

  // Camera names, must match names configured on coprocessor
  public static String frontLeftLimelightName = "limelight-frontlf";
  public static String frontRightLimelightName = "limelight-frontrt";
  public static String rearLimelightName = "limelight-rear";

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.09; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0, // Camera 1
        1.0 // Camera 2
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available

  public static PathPlannerPath ppPath =
      new PathPlannerPath(
          PathPlannerPath.waypointsFromPoses(new Pose2d(), new Pose2d()),
          new PathConstraints(1, 1, 1, 1),
          null,
          new GoalEndState(0, new Rotation2d()));

  public static Pose2d[] tagPoses = {
    new Pose2d(16.697198, 0.65532, new Rotation2d(Math.toRadians(126))), // Tag 1
    new Pose2d(16.697198, 7.39648, new Rotation2d(Math.toRadians(234))), // Tag 2
    new Pose2d(11.56081, 8.05561, new Rotation2d(Math.toRadians(270))), // Tag 3
    new Pose2d(9.27608, 6.137656, new Rotation2d(Math.toRadians(0))), // Tag 4
    new Pose2d(9.27608, 1.914906, new Rotation2d(Math.toRadians(0))), // Tag 5
    new Pose2d(13.474446, 3.306318, new Rotation2d(Math.toRadians(300))), // Tag 6
    new Pose2d(13.890498, 4.0259, new Rotation2d(Math.toRadians(0))), // Tag 7
    new Pose2d(13.474446, 4.745482, new Rotation2d(Math.toRadians(60))), // Tag 8
    new Pose2d(12.643358, 4.745482, new Rotation2d(Math.toRadians(120))), // Tag 9
    new Pose2d(12.227306, 4.0259, new Rotation2d(Math.toRadians(180))), // Tag 10
    new Pose2d(12.643358, 3.306318, new Rotation2d(Math.toRadians(240))), // Tag 11
    new Pose2d(0.851154, 0.65532, new Rotation2d(Math.toRadians(54))), // Tag 12
    new Pose2d(0.851154, 7.39648, new Rotation2d(Math.toRadians(306))), // Tag 13
    new Pose2d(8.272272, 6.137656, new Rotation2d(Math.toRadians(180))), // Tag 14
    new Pose2d(8.272272, 1.914906, new Rotation2d(Math.toRadians(180))), // Tag 15
    new Pose2d(5.987542, -0.00381, new Rotation2d(Math.toRadians(90))), // Tag 16
    new Pose2d(4.073906, 3.306318, new Rotation2d(Math.toRadians(240))), // Tag 17
    new Pose2d(3.6576, 4.0259, new Rotation2d(Math.toRadians(180))), // Tag 18
    new Pose2d(4.073906, 4.745482, new Rotation2d(Math.toRadians(120))), // Tag 19
    new Pose2d(4.90474, 4.745482, new Rotation2d(Math.toRadians(60))), // Tag 20
    new Pose2d(5.321046, 4.0259, new Rotation2d(Math.toRadians(0))), // Tag 21
    new Pose2d(4.90474, 3.306318, new Rotation2d(Math.toRadians(300))), // Tag 22
    new Pose2d(-1, -1, new Rotation2d()) // No Tag Seen
  };

  public static enum PoseFromTag {
    NONE(0),
    TAG19AND9LEFT(1),
    TAG19AND9RIGHT(2),
    TAG20AND8LEFT(3),
    TAG20AND8RIGHT(4),
    TAG21AND7LEFT(5),
    TAG21AND7RIGHT(6),
    TAG22AND6LEFT(7),
    TAG22AND6RIGHT(8),
    TAG17AND11LEFT(9),
    TAG17AND11RIGHT(10),
    TAG18AND10LEFT(11),
    TAG18AND10RIGHT(12),
    TAG19AND9MIDDLE(13),
    TAG20AND8MIDDLE(14),
    TAG21AND7MIDDLE(15),
    TAG22AND6MIDDLE(16),
    TAG17AND11MIDDLE(17),
    TAG18AND10MIDDLE(18);

    private int id;

    PoseFromTag(int id) {
      this.id = id;
    }

    public int getId() {
      return id;
    }
  }

  public static Pose2d poseFromTag(PoseFromTag poseFromTag) {
    switch (poseFromTag.id) {
      case 0: /* Pose From Ids Dir */
        return new Pose2d();
      case 1: /* Pose From 19 And 9 Left */
        return new Pose2d(-0.073906, 0.544518, null);
      case 2: /* Pose From 19 And 9 Right */
        return new Pose2d(-0.392863156214, 0.360368, null);
      case 13: /* Pose From 19 And 9 Middle */
        return new Pose2d(-0.233384578107, 0.452443, null);
      case 3: /* Pose From 20 And 8 Left */
        return new Pose2d(0.392863156214, 0.360368, null);
      case 4: /* Pose From 20 And 8 Right */
        return new Pose2d(0.103906, 0.504518, null); // x 0.073906 y 0.544518
      case 14: /* Pose From 20 And 8 Middle */
        return new Pose2d(0.248384578107, 0.432443, null);
      case 5: /* Pose From 21 And 7 Left */
        return new Pose2d(0.456954, -0.1629, null); // x 0.506954 y -0.1629
      case 6: /* Pose From 21 And 7 Right */
        return new Pose2d(0.456954, 0.1673, null); // x 0.506954 y 0.1673
      case 15: /* Pose From 21 And 7 Middle */
        return new Pose2d(0.456954, 0.0022, null); // x 0.506954 y -0.1629
      case 7: /* Pose From 22 And 6 Left */
        return new Pose2d(0.073906, -0.544518, null);
      case 8: /* Pose From 22 And 6 Right*/
        return new Pose2d(0.392863156214, -0.360368, null); // x 0.392863156214 y -0.360368
      case 16: /* Pose From 22 And 6 Middle */
        return new Pose2d(0.233384578107, -0.452443, null);
      case 9: /* Pose From 17 And 11 Left */
        return new Pose2d(-0.382863156214, -0.330368, null); // x -0.392863156214 y -0.360368
      case 10: /* Pose From 17 And 11 Right */
        return new Pose2d(-0.103906, -0.504518, null);
      case 17: /* Pose From 17 And 11 Middle */
        return new Pose2d(-0.243384578107, -0.417443, null);
      case 11: /* Pose From 18 And 10 Left */
        return new Pose2d(-0.506954, 0.1673, null);
      case 12: /* Pose From 18 And 10 Right */
        return new Pose2d(-0.506954, -0.1629, null);
      case 18: /* Pose From 18 And 10 Right */
        return new Pose2d(-0.506954, 0.0022, null);
      default:
        return new Pose2d();
    }
  }

  public static enum SideOfTag {
    LEFT(),
    MIDDLE(),
    RIGHT();
  }

  public static PoseFromTag poseFromTagFromTagId(int tagId, SideOfTag tagSide) {
    switch (tagId) {
      case 19, 9:
        if (tagSide == SideOfTag.LEFT) {
          return PoseFromTag.TAG19AND9LEFT;
        } else if (tagSide == SideOfTag.RIGHT) {
          return PoseFromTag.TAG19AND9RIGHT;
        } else {
          return PoseFromTag.TAG19AND9MIDDLE;
        }
      case 20, 8:
        if (tagSide == SideOfTag.LEFT) {
          return PoseFromTag.TAG20AND8LEFT;
        } else if (tagSide == SideOfTag.RIGHT) {
          return PoseFromTag.TAG20AND8RIGHT;
        } else {
          return PoseFromTag.TAG20AND8MIDDLE;
        }
      case 21, 7:
        if (tagSide == SideOfTag.LEFT) {
          return PoseFromTag.TAG21AND7LEFT;
        } else if (tagSide == SideOfTag.RIGHT) {
          return PoseFromTag.TAG21AND7RIGHT;
        } else {
          return PoseFromTag.TAG21AND7MIDDLE;
        }
      case 22, 6:
        if (tagSide == SideOfTag.LEFT) {
          return PoseFromTag.TAG22AND6LEFT;
        } else if (tagSide == SideOfTag.RIGHT) {
          return PoseFromTag.TAG22AND6RIGHT;
        } else {
          return PoseFromTag.TAG22AND6MIDDLE;
        }
      case 17, 11:
        if (tagSide == SideOfTag.LEFT) {
          return PoseFromTag.TAG17AND11LEFT;
        } else if (tagSide == SideOfTag.RIGHT) {
          return PoseFromTag.TAG17AND11RIGHT;
        } else {
          return PoseFromTag.TAG17AND11MIDDLE;
        }
      case 18, 10:
        if (tagSide == SideOfTag.LEFT) {
          return PoseFromTag.TAG18AND10LEFT;
        } else if (tagSide == SideOfTag.RIGHT) {
          return PoseFromTag.TAG18AND10RIGHT;
        } else {
          return PoseFromTag.TAG18AND10MIDDLE;
        }
      default:
        return PoseFromTag.NONE;
    }
  }
}
