package frc.robot.other;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;
import frc.robot.constants.GeneralConstants;

/**
 * Utility class for robot-related operations, such as alliance-specific transformations,
 * pose manipulations, and perspective adjustments.
 */
public class RobotUtils {

    /**
     * Checks if the robot is on the red alliance.
     *
     * @return True if the robot is on the red alliance, false otherwise.
     */
    public static boolean onRedAlliance() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }

    /**
     * Inverts a given Pose2d across the field's center. This is useful for transforming
     * poses from one alliance's perspective to the other.
     *
     * @param pose The pose to invert.
     * @return The inverted pose.
     */
    public static Pose2d invertPose(Pose2d pose) {
        return new Pose2d(
            GeneralConstants.fieldLength - pose.getX(),
            GeneralConstants.fieldWidth - pose.getY(),
            pose.getRotation().rotateBy(Rotation2d.k180deg)
        );
    }

    /**
     * Inverts a given Pose2d to the current alliance's perspective. If the robot is on the red alliance,
     * the pose is inverted; otherwise, it is returned as-is.
     *
     * @param pose The pose to invert.
     * @return The pose transformed to the current alliance's perspective.
     */
    public static Pose2d invertPoseToAlliance(Pose2d pose) {
        if (onRedAlliance()) {
            return invertPose(pose);
        } else {
            return pose;
        }
    }

    /**
     * Inverts a given Translation3d across the field's center. This is useful for transforming
     * 3D translations from one alliance's perspective to the other.
     *
     * @param trans The 3D translation to invert.
     * @return The inverted 3D translation.
     */
    public static Translation3d invertTrans3d(Translation3d trans) {
        return new Translation3d(
            GeneralConstants.fieldLength - trans.getX(),
            GeneralConstants.fieldWidth - trans.getY(),
            trans.getZ()
        );
    }

    /**
     * Inverts a given Translation3d to the current alliance's perspective. If the robot is on the red alliance,
     * the translation is inverted; otherwise, it is returned as-is.
     *
     * @param trans The 3D translation to invert.
     * @return The 3D translation transformed to the current alliance's perspective.
     */
    public static Translation3d invertTrans3dToAlliance(Translation3d trans) {
        if (onRedAlliance()) {
            return invertTrans3d(trans);
        } else {
            return trans;
        }
    }

    /**
     * Determines the forward direction based on the robot's perspective. In simulation, the forward
     * direction is 90 degrees counterclockwise. On the red alliance, the forward direction is 180 degrees.
     * On the blue alliance, the forward direction is 0 degrees.
     *
     * @return The forward direction as a Rotation2d.
     */
    public static Rotation2d getPerspectiveForward() {
        if (Robot.isSimulation()) {
            return Rotation2d.kCCW_90deg;
        } else if (onRedAlliance()) {
            return Rotation2d.k180deg;
        } else {
            return Rotation2d.kZero;
        }
    }

    /**
     * Finds the closest Pose2d from a list of poses to a given reference pose. The reference pose
     * is first transformed to the current alliance's perspective before calculating distances.
     *
     * @param pose The reference pose.
     * @param others An array of poses to compare against.
     * @return The closest pose from the list.
     */
    public static Pose2d getClosestPoseToPose(Pose2d pose, Pose2d[] others) {
        pose = RobotUtils.invertPoseToAlliance(pose);
        Pose2d closest = null;
        double closestDistance = Double.MAX_VALUE;
        for (Pose2d other : others) {
            Transform2d transform = pose.minus(other);
            double distance = Math.hypot(transform.getX(), transform.getY());
            if (distance < closestDistance) {
                closestDistance = distance;
                closest = other;
            }
        }
        return closest;
    }
}