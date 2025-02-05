package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.MechanismConstants;
import frc.robot.constants.ArmSystemConstants;

/**
 * The `ArmMechanism` class is responsible for visualizing the arm, elevator, and wrist mechanisms
 * using a 2D mechanism representation on the SmartDashboard. This class extends `SubsystemBase`
 * and updates the visualization periodically based on the current state of the arm, elevator, and wrist.
 */
public class ArmSystem extends SubsystemBase {
    /**
     * Represents a complete state of the arm system, including positions for
     * the arm, elevator, and wrist.
     */
    public static class ArmSystemState {
        public final double armPosition;
        public final double elevatorPosition;
        public final double wristPosition;

        public ArmSystemState(double armPosition, double elevatorPosition, double wristPosition) {
            this.armPosition = armPosition;
            this.elevatorPosition = elevatorPosition;
            this.wristPosition = wristPosition;
        }
    }

    // Subsystems
    private Arm arm;        // The arm subsystem
    private Elevator elevator; // The elevator subsystem
    private Wrist wrist;    // The wrist subsystem

    // Mechanism2d visualization components
    private final Mechanism2d mech2d = new Mechanism2d(MechanismConstants.mechWidth, MechanismConstants.mechHeight);
    private final MechanismRoot2d mech2dRoot = mech2d.getRoot("Elevator Root", MechanismConstants.mechWidth / 2, 0);
    private final MechanismLigament2d elevatorMech2d = mech2dRoot.append(new MechanismLigament2d("Elevator", 0, 90));
    private final MechanismLigament2d armMech2d = elevatorMech2d.append(new MechanismLigament2d("Arm", MechanismConstants.armLength, 0));
    private final MechanismLigament2d wristMech2d = armMech2d.append(new MechanismLigament2d("Wrist", MechanismConstants.wristLength, 0));

    /**
     * Constructor for the `ArmMechanism` class.
     *
     * @param arm      The arm subsystem.
     * @param elevator The elevator subsystem.
     * @param wrist    The wrist subsystem.
     */
    public ArmSystem(Arm arm, Elevator elevator, Wrist wrist) {
        this.arm = arm;
        this.elevator = elevator;
        this.wrist = wrist;

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
    }

    /**
     * Moves all mechanisms (arm, elevator, wrist) to a named position.
     * 
     * @param stateName The name of the state to move to (e.g., "groundIntake", "default", "source", "algae", "score1", etc.)
     * @throws IllegalArgumentException if the state name is not found
     */
    public void moveTo(String stateName) {
        ArmSystemState state = ArmSystemConstants.states.get(stateName);
        if (state == null) {
            throw new IllegalArgumentException("Unknown state name: " + stateName);
        }
        setState(state);
    }

    /**
     * Sets the arm system to a specific state.
     * 
     * @param state The state to set the arm system to.
     */
    private void setState(ArmSystemState state) {
        arm.setGoal(state.armPosition);
        elevator.setGoal(state.elevatorPosition);
        wrist.setGoal(state.wristPosition);
    }

    /**
     * Checks if all mechanisms (arm, elevator, wrist) are at their setpoints.
     * 
     * @return true if all mechanisms are at their setpoints, false otherwise.
     */
    public boolean atSetpoint() {
        return arm.atSetpoint() && elevator.atSetpoint() && wrist.atSetpoint();
    }
}