package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.SwerveSystemConstants;
import frc.robot.subsystems.SwerveSystem;

public class SwerveSystemCommands {
    private SwerveSystem swerveSystem;

    public SwerveSystemCommands(SwerveSystem swerveSystem) {
        this.swerveSystem = swerveSystem;
    }

    /**
     * Creates a command that moves to the closest state from an array of states
     * 
     * @param states The array of states to choose from
     * @param targetPose Optional target pose for states with null positions
     */
    private Command moveToClosestStateCommand(SwerveSystem.SwerveSystemState[] states, Supplier<Pose2d> targetPose) {
        return new ParallelRaceGroup(
            new FunctionalCommand(
                () -> {},
                () -> swerveSystem.moveToClosestState(states, targetPose.get()),
                (interrupted) -> {},
                () -> swerveSystem.atSetpoint(),
                swerveSystem
            ),
            new WaitCommand(SwerveSystemConstants.timeout)
        );
    }

    /**
     * Creates a command that moves to a specific state
     * 
     * @param state The state to move to
     * @param targetPose Optional target pose for states with null positions
     */
    private Command moveToStateCommand(SwerveSystem.SwerveSystemState state, Supplier<Pose2d> targetPose) {
        return new ParallelRaceGroup(
            new FunctionalCommand(
                () -> {},
                () -> swerveSystem.setState(state, targetPose.get()),
                (interrupted) -> {},
                () -> swerveSystem.atSetpoint(),
                swerveSystem
            ),
            new WaitCommand(SwerveSystemConstants.timeout)
        );
    }

    /**
     * Command to move to the closest scoring state for a specific level
     * 
     * @param level The scoring level (0=L1, 1=L2, 2=L3, 3=L4)
     */
    public Command moveToScoringState(Supplier<Integer> level) {
        return new FunctionalCommand(
            () -> {},
            () -> {
                int l = level.get();
                if (l < 0 || l >= 4) {
                    throw new IllegalArgumentException("Invalid scoring level: " + l);
                }
                swerveSystem.moveToClosestState(SwerveSystemConstants.scoringStates[l]);
            },
            (interrupted) -> {},
            () -> swerveSystem.atSetpoint(),
            swerveSystem
        );
    }

    /**
     * Command to move to the closest source state
     */
    public Command moveToSourceState() {
        return moveToClosestStateCommand(SwerveSystemConstants.sourceStates, () -> swerveSystem.swerve.getPose());
    }

    /**
     * Command to move to the closest algae state
     */
    public Command moveToAlgaeState() {
        return moveToClosestStateCommand(SwerveSystemConstants.algaeStates, () -> swerveSystem.swerve.getPose());
    }

    /**
     * Command to move to the processor state
     */
    public Command moveToProcessor() {
        return moveToStateCommand(SwerveSystemConstants.processor, () -> swerveSystem.swerve.getPose());
    }

    /**
     * Command to move to the ground intake state
     * 
     * @param targetPose The target robot pose to use (required since ground intake has no position)
     */
    public Command moveToGroundIntake(Supplier<Pose2d> targetPose) {
        return moveToStateCommand(SwerveSystemConstants.groundIntake, targetPose);
    }

    /**
     * Command to move to the ground intake state, using current pose
     */
    public Command moveToGroundIntake() {
        return moveToGroundIntake(() -> swerveSystem.swerve.getPose());
    }

    /**
     * Command to move to the default state
     * 
     * @param targetPose The target robot pose to use (required since default has no position)
     */
    public Command moveToDefault(Supplier<Pose2d> targetPose) {
        return moveToStateCommand(SwerveSystemConstants.defaultState, targetPose);
    }

    /**
     * Command to move to the default state, using current pose
     */
    public Command moveToDefault() {
        return moveToDefault(() -> swerveSystem.swerve.getPose());
    }
}
