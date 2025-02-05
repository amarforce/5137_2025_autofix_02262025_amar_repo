package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.elastic.Reef;
import frc.robot.subsystems.ArmSystem;

/**
 * The MultiCommands class is responsible for creating complex commands that involve multiple subsystems.
 * These commands are typically composed of several simpler commands that run in parallel or sequence.
 */
public class MultiCommands {
    // Command groups for each subsystem
    private ArmSystem armSystem;
    private SwerveCommands swerveCommands;
    private IntakeCommands intakeCommands;
    @SuppressWarnings("unused")
    private HangCommand hangCommand;
    private Reef reef;

    /**
     * Constructor for MultiCommands.
     */
    public MultiCommands(ArmSystem armSystem, SwerveCommands swerveCommands, IntakeCommands intakeCommands,
                         HangCommand hangCommand, Reef reef) {
        this.armSystem = armSystem;
        this.intakeCommands = intakeCommands;
        this.swerveCommands = swerveCommands;
        this.hangCommand = hangCommand;
        this.reef = reef;
    }

    /**
     * Command to move the arm system to a named position.
     * @param stateName The name of the state to move to (e.g. "groundIntake", "default", "source", "algae", "L1", "L2", etc.)
     */
    public Command moveTo(Supplier<String> stateName) {
        return new FunctionalCommand(
            () -> {},
            () -> {
                armSystem.moveTo(stateName.get());
            },
            (e) -> {},
            () -> armSystem.atSetpoint(),
            armSystem

        );
    }


    /**
     * Command to move the arm system to a specific goal level.
     * @param level The level number (1-4)
     */
    public Command moveToGoal(Supplier<Integer> level) {
        return moveTo(() -> "L" + level.get());
    }

    /**
     * Command to retrieve coral based on the robot's current pose.
     */
    public Command getCoral(Pose2d pose) {
        if (pose == null) {
            return new InstantCommand(); // Do nothing if the pose is null
        } else {
            Command moveTo = null;
            if (pose.getY() > 1.75 && pose.getY() < 6.3) {
                // If the robot is within a specific Y range, move to the ground intake position
                moveTo = new ParallelCommandGroup(
                    swerveCommands.driveToPose(() -> pose),
                    moveTo(()->"groundIntake")
                );
            } else {
                // Otherwise, move to the source position
                moveTo = new ParallelCommandGroup(
                    swerveCommands.driveToPose(() -> pose),
                    moveTo(()->"source")
                );
            }
            return new SequentialCommandGroup(moveTo, intakeCommands.intakeUntilSwitched());
        }
    }

    

    public Command placeCoral(int branch) {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                swerveCommands.driveToBranch(branch),
                moveToGoal(() -> reef.getLevel(branch))
            ),
            intakeCommands.outtake()
        );
    }

    public Command getAlgae(int branch) {
        Command moveTo;
        if (reef.isAlgaeLow(branch)) {
            moveTo = new ParallelCommandGroup(
                moveTo(()->"algaeLow")
                //,swerveCommands.driveToPose(()->pose)
                );
        } else {
            moveTo = new ParallelCommandGroup(
                moveTo(()->"algaeHigh")
                //,swerveCommands.driveToPose(()->pose)
                );
        }
        
        return new SequentialCommandGroup(
            moveTo, 
            intakeCommands.intakeUntilSwitched(),
            swerveCommands.functionalDrive(()->0.8, ()->0, ()->0, ()->false));
    }

    public Command placeAlgae() {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                swerveCommands.driveToProcessor(),
                moveTo(() -> "processor")
            ),
            intakeCommands.outtake()
        );
    }

    /**
     * Getter for the SwerveCommands.
     */
    public SwerveCommands getSwerveCommands() {
        return swerveCommands;
    }

    /**
     * Getter for the IntakeCommands.
     */
    public IntakeCommands getIntakeCommands() {
        return intakeCommands;
    }
}