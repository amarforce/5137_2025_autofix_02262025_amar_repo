package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.MechanismConstants;
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
    }

    // Subsystems
    private Arm arm;        // The arm subsystem
    private Elevator elevator; // The elevator subsystem
    private Wrist wrist;    // The wrist subsystem
    public final Swerve swerve; // The swerve subsystem

    // Mechanism2d visualization components
    private final Mechanism2d mech2d = new Mechanism2d(MechanismConstants.mechWidth, MechanismConstants.mechHeight);
    private final MechanismRoot2d mech2dRoot = mech2d.getRoot("Elevator Root", MechanismConstants.mechWidth / 2, 0);
    private final MechanismLigament2d elevatorMech2d = mech2dRoot.append(new MechanismLigament2d("Elevator", 0, 90));
    private final MechanismLigament2d armMech2d = elevatorMech2d.append(new MechanismLigament2d("Arm", MechanismConstants.armLength, 0));
    private final MechanismLigament2d wristMech2d = armMech2d.append(new MechanismLigament2d("Wrist", MechanismConstants.wristLength, 0));

    private StructPublisher<Pose3d> firstStagePosePublisher = NetworkTableInstance.getDefault().getStructTopic("SmartDashboard/elevator/firstStage", Pose3d.struct).publish();
    private StructPublisher<Pose3d> secondStagePosePublisher = NetworkTableInstance.getDefault().getStructTopic("SmartDashboard/elevator/secondStage", Pose3d.struct).publish();
    private StructPublisher<Pose3d> armPosePublisher = NetworkTableInstance.getDefault().getStructTopic("SmartDashboard/arm/pose", Pose3d.struct).publish();
    private StructPublisher<Pose3d> wristPosePublisher = NetworkTableInstance.getDefault().getStructTopic("SmartDashboard/wrist/pose", Pose3d.struct).publish();

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

        // Set the colors for the mechanism components
        elevatorMech2d.setColor(MechanismConstants.elevatorColor);
        armMech2d.setColor(MechanismConstants.armColor);
        wristMech2d.setColor(MechanismConstants.wristColor);

        // Display the mechanism on the SmartDashboard
        SmartDashboard.putData("armSystem", mech2d);
    }

    /**
     * This method is called periodically (every 20ms by default) and updates the visualization
     * of the arm, elevator, and wrist mechanisms based on their current positions.
     */
    @Override
    public void periodic() {
        // Update the length of the elevator visualization based on its current height
        elevatorMech2d.setLength((elevator.getMeasurement() / (ElevatorConstants.maxHeight * 2) + 0.25) * MechanismConstants.mechHeight);

        // Update the angle of the arm visualization based on its current angle
        armMech2d.setAngle(Units.radiansToDegrees(arm.getMeasurement() - Math.PI / 2));

        // Update the angle of the wrist visualization based on its current angle
        wristMech2d.setAngle(Units.radiansToDegrees(wrist.getMeasurement() - Math.PI / 2));

        firstStagePosePublisher.set(new Pose3d(0,0,elevator.getMeasurement()/2,new Rotation3d()));
        Translation3d elevatorTrans=new Translation3d(0,0,elevator.getMeasurement());
        Pose3d elevatorPose=new Pose3d(elevatorTrans,new Rotation3d());
        secondStagePosePublisher.set(elevatorPose);
        Translation3d armTrans=elevatorTrans.plus(SwerveSystemConstants.armTransOffset);
        Pose3d armPose=new Pose3d(armTrans,new Rotation3d(0,arm.getMeasurement(),0));
        armPosePublisher.set(armPose);
        Translation3d wristTrans=armTrans.plus(new Translation3d(-ArmConstants.armLength*Math.cos(arm.getMeasurement()), 0, ArmConstants.armLength*Math.sin(arm.getMeasurement())));
        Pose3d wristPose=new Pose3d(wristTrans,new Rotation3d(0,arm.getMeasurement()+wrist.getMeasurement(),0));
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
    public void setState(SwerveSystemState state, Pose2d targetPose) {
        arm.setGoal(state.armPosition);
        elevator.setGoal(state.elevatorPosition);
        wrist.setGoal(state.wristPosition);
        swerve.setTargetPose(state.botPosition == null ? targetPose : state.botPosition);
    }

    /**
     * Sets the arm system to a specific state, using current pose for null positions.
     * 
     * @param state The state to set the arm system to.
     */
    public void setState(SwerveSystemState state) {
        setState(state, swerve.getPose());
    }

    /**
     * Moves to the closest state from an array of possible states
     * 
     * @param states Array of possible states to choose from
     * @param targetPose The target robot pose to use for states with null positions
     * @throws IllegalArgumentException if states array is empty
     */
    public void moveToClosestState(SwerveSystemState[] states, Pose2d targetPose) {
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
            Pose2d statePosition = state.botPosition == null ? targetPose : state.botPosition;
            double distance = calculateDistance(currentPose, statePosition);
            if (distance < minDistance) {
                minDistance = distance;
                closestState = state;
            }
        }
        
        setState(closestState, targetPose);
    }

    /**
     * Moves to the closest state from an array of possible states, using current pose for null positions
     * 
     * @param states Array of possible states to choose from
     * @throws IllegalArgumentException if states array is empty
     */
    public void moveToClosestState(SwerveSystemState[] states) {
        moveToClosestState(states, swerve.getPose());
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