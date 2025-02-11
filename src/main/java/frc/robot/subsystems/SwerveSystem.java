package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.SwerveSystemConstants;
import frc.robot.other.RobotUtils;

/**
 * The `ArmMechanism` class is responsible for visualizing the arm, elevator, and wrist mechanisms
 * using a 2D mechanism representation on the SmartDashboard. This class extends `SubsystemBase`
 * and updates the visualization periodically based on the current state of the arm, elevator, and wrist.
 */
public class SwerveSystem extends SubsystemBase {
    /**
     * Represents a complete state of the arm system, including positions for
     * the arm, elevator, and wrist.
     */
    public static class SwerveSystemState {
        public final double armPosition;
        public final double elevatorPosition;
        public final double wristPosition;
        public final Pose2d botPosition;

        public SwerveSystemState(double armPosition, double elevatorPosition, double wristPosition, Pose2d botPosition) {
            this.armPosition = armPosition;
            this.elevatorPosition = elevatorPosition;
            this.wristPosition = wristPosition;
            this.botPosition = botPosition;
        }

        public SwerveSystemState withPose(Pose2d pose){
            return new SwerveSystemState(armPosition, elevatorPosition, wristPosition, pose);
        }
    }

    // Subsystems
    private Arm arm;        // The arm subsystem
    private Elevator elevator; // The elevator subsystem
    private Wrist wrist;    // The wrist subsystem
    public final Swerve swerve; // The swerve subsystem

    private StructPublisher<Pose2d> botPosePublisher = NetworkTableInstance.getDefault().getStructTopic("SmartDashboard/driveState/pose", Pose2d.struct).publish();
    private StructPublisher<Pose2d> targetBotPosePublisher = NetworkTableInstance.getDefault().getStructTopic("SmartDashboard/driveState/targetPose", Pose2d.struct).publish();
    private StructPublisher<Pose3d> firstStagePosePublisher = NetworkTableInstance.getDefault().getStructTopic("SmartDashboard/sim/elevatorFirst", Pose3d.struct).publish();
    private StructPublisher<Pose3d> secondStagePosePublisher = NetworkTableInstance.getDefault().getStructTopic("SmartDashboard/sim/elevatorSecond", Pose3d.struct).publish();
    private StructPublisher<Pose3d> armPosePublisher = NetworkTableInstance.getDefault().getStructTopic("SmartDashboard/sim/arm", Pose3d.struct).publish();
    private StructPublisher<Pose3d> wristPosePublisher = NetworkTableInstance.getDefault().getStructTopic("SmartDashboard/sim/wrist", Pose3d.struct).publish();

    /**
     * Constructor for the `ArmMechanism` class.
     *
     * @param arm      The arm subsystem.
     * @param elevator The elevator subsystem.
     * @param wrist    The wrist subsystem.
     */
    public SwerveSystem(Arm arm, Elevator elevator, Wrist wrist, Swerve swerve) {
        this.arm = arm;
        this.elevator = elevator;
        this.wrist = wrist;
        this.swerve = swerve;
    }

    /**
     * This method is called periodically (every 20ms by default) and updates the visualization
     * of the arm, elevator, and wrist mechanisms based on their current positions.
     */
    @Override
    public void periodic() {
        // Update robot pose visualization
        botPosePublisher.set(swerve.getPose());
        targetBotPosePublisher.set(swerve.getTargetPose());

        // Calculate elevator positions
        double elevatorHeight = elevator.getMeasurement();
        firstStagePosePublisher.set(new Pose3d(0, 0, elevatorHeight/2, new Rotation3d()));
        
        // Calculate elevator end position
        Translation3d elevatorTrans = new Translation3d(0, 0, elevatorHeight);
        Pose3d elevatorPose = new Pose3d(elevatorTrans, new Rotation3d());
        secondStagePosePublisher.set(elevatorPose);

        // Calculate arm pivot position (offset from elevator end)
        Translation3d armPivotTrans = elevatorTrans.plus(SwerveSystemConstants.armTransOffset);
        
        // Calculate arm end position (arm angle is now relative to vertical)
        double armAngle = arm.getMeasurement();
        double armX = ArmConstants.armLength * -Math.sin(armAngle); // negative because forward is negative X
        double armZ = ArmConstants.armLength * Math.cos(armAngle);  // cosine for vertical component
        Translation3d armEndTrans = armPivotTrans.plus(new Translation3d(armX, 0, armZ));
        
        // Publish arm pose
        Pose3d armPose = new Pose3d(armPivotTrans, new Rotation3d(0, -armAngle, 0));
        armPosePublisher.set(armPose);

        // Calculate and publish wrist pose
        double wristAngle = wrist.getAdjustedMeasurement();
        Pose3d wristPose = new Pose3d(armEndTrans, new Rotation3d(0, -wristAngle, 0));
        wristPosePublisher.set(wristPose);
    }

    /**
     * Calculate the distance between two poses, including rotation
     * Uses a weighted sum of translational and rotational distance
     */
    private double calculateDistance(Pose2d pose1, Pose2d pose2) {
        return RobotUtils.getWeightedPoseDistance(pose1, pose2, SwerveSystemConstants.rotationWeight);
    }

    /**
     * Sets the arm system to a specific state.
     * 
     * @param state The state to set the arm system to.
     * @param targetPose The target robot pose to use if state's position is null
     */
    public void setState(SwerveSystemState state) {
        arm.setGoal(state.armPosition);
        elevator.setGoal(state.elevatorPosition);
        wrist.setGoal(state.wristPosition);
        swerve.setTargetPose(state.botPosition);
    }

    public SwerveSystemState getClosestState(SwerveSystemState[] states){
        if (states == null || states.length == 0) {
            throw new IllegalArgumentException("States array cannot be empty");
        }

        // Get current robot pose
        Pose2d currentPose = swerve.getPose();
        
        // Find the closest state based on robot position
        SwerveSystemState closestState = states[0];
        double minDistance = Double.MAX_VALUE;
        
        for (SwerveSystemState state : states) {
            // Use targetPose if state's position is null
            double distance = calculateDistance(currentPose, state.botPosition);
            if (distance < minDistance) {
                minDistance = distance;
                closestState = state;
            }
        }

        return closestState;
    }

    /**
     * Checks if all mechanisms (arm, elevator, wrist) are at their setpoints.
     * 
     * @return true if all mechanisms are at their setpoints, false otherwise.
     */
    public boolean atSetpoint() {
        return arm.atSetpoint() && elevator.atSetpoint() && wrist.atSetpoint() && swerve.atTarget();
    }
}