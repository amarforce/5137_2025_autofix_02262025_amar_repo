package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Hang;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

/**
 * The MultiCommands class is responsible for creating complex commands that involve multiple subsystems.
 * These commands are typically composed of several simpler commands that run in parallel or sequence.
 */
public class MultiCommands {
    // Subsystems
    private Arm arm;
    private Elevator elevator;
    private Wrist wrist;
    private Swerve swerve;
    private Intake intake;
    private Hang hang;

    // Command groups for each subsystem
    private ArmCommands armCommands;
    private ElevatorCommands elevatorCommands;
    private WristCommands wristCommands;
    private SwerveCommands swerveCommands;
    private IntakeCommands intakeCommands;
    private HangCommand hangCommand;

    /**
     * Constructor for MultiCommands.
     * 
     * @param arm The Arm subsystem.
     * @param elevator The Elevator subsystem.
     * @param wrist The Wrist subsystem.
     * @param swerve The Swerve subsystem.
     * @param intake The Intake subsystem.
     * @param hang The Hang subsystem.
     * @param armCommands The ArmCommands command group.
     * @param elevatorCommands The ElevatorCommands command group.
     * @param wristCommands The WristCommands command group.
     * @param swerveCommands The SwerveCommands command group.
     * @param intakeCommands The IntakeCommands command group.
     * @param hangCommand The HangCommand command group.
     */
    public MultiCommands(Arm arm, Elevator elevator, Wrist wrist, Swerve swerve, Intake intake, Hang hang,
                         ArmCommands armCommands, ElevatorCommands elevatorCommands, WristCommands wristCommands,
                         SwerveCommands swerveCommands, IntakeCommands intakeCommands, HangCommand hangCommand) {
        this.arm = arm;
        this.elevator = elevator;
        this.wrist = wrist;
        this.swerve = swerve;
        this.intake = intake;
        this.hang = hang;
        this.armCommands = armCommands;
        this.elevatorCommands = elevatorCommands;
        this.wristCommands = wristCommands;
        this.intakeCommands = intakeCommands;
        this.swerveCommands = swerveCommands;
        this.hangCommand = hangCommand;
    }

    /**
     * Command to move the arm, elevator, and wrist to the ground intake position.
     * 
     * @return A ParallelCommandGroup that moves the arm, elevator, and wrist to the ground intake position.
     */
    public Command moveToGroundIntake() {
        return new ParallelCommandGroup(
            armCommands.moveToGroundIntake(),
            elevatorCommands.moveToGroundIntake(),
            wristCommands.toPos1()
        );
    }

    /**
     * Command to move the arm, elevator, and wrist to the default position.
     * 
     * @return A ParallelCommandGroup that moves the arm, elevator, and wrist to the default position.
     */
    public Command moveToDefault() {
        return new ParallelCommandGroup(
            armCommands.moveToDefault(),
            elevatorCommands.moveToDefault(),
            wristCommands.toPos1()
        );
    }

    /**
     * Command to move the arm, elevator, and wrist to the source position.
     * 
     * @return A ParallelCommandGroup that moves the arm, elevator, and wrist to the source position.
     */
    public Command moveToSource() {
        return new ParallelCommandGroup(
            armCommands.moveToSource(),
            elevatorCommands.moveToSource(),
            wristCommands.toPos2()
        );
    }

    /**
     * Command to move the arm, elevator, and wrist to the algae position.
     * 
     * @return A ParallelCommandGroup that moves the arm, elevator, and wrist to the algae position.
     */
    public Command moveToAlgae() {
        return new ParallelCommandGroup(
            armCommands.moveToAlgae(),
            elevatorCommands.moveToAlgae(),
            wristCommands.toPos2()
        );
    }

    /**
     * Command to move the arm, elevator, and wrist to a specific goal position.
     * 
     * @param goal The goal position to move to.
     * @return A ParallelCommandGroup that moves the arm, elevator, and wrist to the specified goal position.
     */
    public Command moveToGoal(int goal) {
        return new ParallelCommandGroup(
            armCommands.moveToGoal(goal),
            elevatorCommands.moveToGoal(goal),
            wristCommands.toPos2()
        );
    }

    /**
     * Command to retrieve coral based on the robot's current pose.
     * 
     * @param pose The current pose of the robot.
     * @return A ParallelCommandGroup that drives the robot to the specified pose and moves the arm, elevator, and wrist accordingly.
     */
    public Command getCoral(Pose2d pose) {
        if (pose == null) {
            return new InstantCommand(); // Do nothing if the pose is null
        } else {
            Command moveTo=null;
            if (pose.getY() > 1.75 && pose.getY() < 6.3) {
                // If the robot is within a specific Y range, move to the ground intake position
                moveTo = new ParallelCommandGroup(
                    swerveCommands.driveToPose(() -> pose),
                    moveToGroundIntake()
                );
            } else {
                // Otherwise, move to the source position
                moveTo = new ParallelCommandGroup(
                    swerveCommands.driveToPose(() -> pose),
                    moveToSource()
                );
            }
            return new SequentialCommandGroup(moveTo,intakeCommands.intakeUntilSwitched());
        }
    }

    /**
     * Getter for the SwerveCommands.
     * 
     * @return The SwerveCommands command group.
     */
    public SwerveCommands getSwerveCommands() {
        return swerveCommands;
    }

    /**
     * Getter for the SwerveCommands.
     * 
     * @return The SwerveCommands command group.
     */
    public IntakeCommands getIntakeCommands() {
        return intakeCommands;
    }
}