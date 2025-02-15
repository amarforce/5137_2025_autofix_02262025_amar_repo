package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * The MultiCommands class is responsible for creating complex commands that involve multiple subsystems.
 * These commands are typically composed of several simpler commands that run in parallel or sequence.
 */
public class MultiCommands {
    // Command groups for each subsystem
    private SwerveSystemCommands swerveSystemCommands;
    private IntakeCommands intakeCommands;
    private SwerveCommands swerveCommands;
    @SuppressWarnings("unused")
    private HangCommands hangCommand;


    /**
     * Constructor for MultiCommands.
     */
    public MultiCommands(SwerveSystemCommands swerveSystemCommands, SwerveCommands swerveCommands, IntakeCommands intakeCommands,
                         HangCommands hangCommand) {
        this.swerveSystemCommands = swerveSystemCommands;
        this.swerveCommands = swerveCommands;
        this.intakeCommands = intakeCommands;
        this.hangCommand = hangCommand;
    }

    /**
     * Command to retrieve coral based on the robot's current pose.
     */
    public Command getCoralFromSource() {
        return new SequentialCommandGroup(
            swerveSystemCommands.moveToSource(),
            new ParallelCommandGroup(intakeCommands.intake(),swerveSystemCommands.simIntake("coral"))
        );
    }

    public Command getCoralFromGround(Supplier<Pose2d> pose) {
        return new SequentialCommandGroup(
            swerveSystemCommands.moveToGround(pose),
            new ParallelCommandGroup(intakeCommands.intake(),swerveSystemCommands.simIntake("coral"))
        );
    }

    public Command placeCoral(Supplier<Integer> level,Supplier<Integer> branch) {
        return new SequentialCommandGroup(
            swerveSystemCommands.moveToBranch(level,branch),
            new ParallelCommandGroup(intakeCommands.outtake(),swerveSystemCommands.simOuttake())
        );
    }

    public Command getAlgae(Supplier<Integer> side) {
        return new SequentialCommandGroup(
            swerveSystemCommands.moveToAlgae(side), 
            new ParallelCommandGroup(intakeCommands.intake(),swerveSystemCommands.simIntake("algae")),
            swerveCommands.driveBack()
        );
    }

    public Command placeAlgae() {
        return new SequentialCommandGroup(
            swerveSystemCommands.moveToProcessor(),
            new ParallelCommandGroup(intakeCommands.outtake(),swerveSystemCommands.simOuttake())
        );
    }
}