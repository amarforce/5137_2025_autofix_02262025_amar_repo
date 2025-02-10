package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.elastic.Reef;

/**
 * The MultiCommands class is responsible for creating complex commands that involve multiple subsystems.
 * These commands are typically composed of several simpler commands that run in parallel or sequence.
 */
public class MultiCommands {
    // Command groups for each subsystem
    private SwerveSystemCommands swerveSystemCommands;
    private IntakeCommands intakeCommands;
    @SuppressWarnings("unused")
    private HangCommand hangCommand;
    private Reef reef;


    /**
     * Constructor for MultiCommands.
     */
    public MultiCommands(SwerveSystemCommands swerveSystemCommands, IntakeCommands intakeCommands,
                         HangCommand hangCommand, Reef reef) {
        this.swerveSystemCommands = swerveSystemCommands;
        this.intakeCommands = intakeCommands;
        this.hangCommand = hangCommand;
        this.reef = reef;

    }

    /**
     * Command to retrieve coral based on the robot's current pose.
     */
    public Command getCoral(Pose2d pose) {
        if (pose == null) {
            return new InstantCommand(); // Do nothing if the pose is null
        } else {
            String goalName = (pose.getY() > 1.75 && pose.getY() < 6.3) ? "groundIntake" : "source";
            return new SequentialCommandGroup(new ParallelCommandGroup(swerveCommands.driveToPoseStatic(()->pose),armSystemCommands.moveTo(()->goalName)), intakeCommands.intakeUntilSwitched());
        }
    }

    

    public Command placeCoral(Supplier<Integer> level,Supplier<Integer> branch) {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                swerveCommands.driveToBranch(branch),
                armSystemCommands.moveToGoal(level)
            ),
            intakeCommands.outtake()
        );
    }

    public Command placeCoralNearest(int level,int branch){
        return placeCoral(()->reef.getNearestLevel(level-2, branch)+2,()->reef.getNearestBranch(level-2, branch));
    }

    public Command getAlgae(int side) {
        String goal=reef.isAlgaeLow(side)?"algaeLow":"algaeHigh";
        Command moveTo=new ParallelCommandGroup(
            armSystemCommands.moveTo(()->goal),
            swerveCommands.driveToAlgae(()->side)
        );
        return new SequentialCommandGroup(
            moveTo, 
            intakeCommands.intakeUntilSwitched(),
            swerveCommands.driveBack()
        );
    }

    public Command placeAlgae() {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                swerveCommands.driveToProcessor(),
                armSystemCommands.moveTo(() -> "processor")
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

    /**
     * Getter for the IntakeCommands.
     */
    public ArmSystemCommands getArmSystemCommands() {
        return armSystemCommands;
    }
}