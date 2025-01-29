package frc.robot.other;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.MechanismConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

/**
 * The `ArmMechanism` class is responsible for visualizing the arm, elevator, and wrist mechanisms
 * using a 2D mechanism representation on the SmartDashboard. This class extends `SubsystemBase`
 * and updates the visualization periodically based on the current state of the arm, elevator, and wrist.
 */
public class ArmMechanism extends SubsystemBase {
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
    public ArmMechanism(Arm arm, Elevator elevator, Wrist wrist) {
        this.arm = arm;
        this.elevator = elevator;
        this.wrist = wrist;

        // Set the colors for the mechanism components
        elevatorMech2d.setColor(MechanismConstants.elevatorColor);
        armMech2d.setColor(MechanismConstants.armColor);
        wristMech2d.setColor(MechanismConstants.wristColor);

        // Display the mechanism on the SmartDashboard
        SmartDashboard.putData("Scoring System", mech2d);
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
}