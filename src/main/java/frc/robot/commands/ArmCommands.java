package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Arm;
import frc.robot.constants.ArmConstants;

/**
 * The `ArmCommands` class provides a set of commands for controlling the arm subsystem.
 * These commands are used to set or adjust the arm's goal position, move to predefined positions,
 * and perform system identification (SysId) routines.
 */
public class ArmCommands {

    private Arm arm; // The arm subsystem that these commands will control.

    /**
     * Constructs an `ArmCommands` object.
     *
     * @param arm The arm subsystem to be controlled by these commands.
     */
    public ArmCommands(Arm arm) {
        this.arm = arm;
    }

    /**
     * Returns a command that sets the arm's goal position to a specified value.
     *
     * @param goal A `DoubleSupplier` that provides the goal position for the arm.
     * @return A command that sets the arm's goal position.
     */
    public Command setGoal(DoubleSupplier goal) {
        return new InstantCommand(() -> arm.setGoal(goal.getAsDouble()), arm);
    }

    /**
     * Returns a command that adjusts the arm's goal position by a specified amount.
     *
     * @param change A `DoubleSupplier` that provides the amount to change the arm's goal position.
     * @return A command that adjusts the arm's goal position.
     */
    public Command changeGoal(DoubleSupplier change) {
        return new InstantCommand(() -> arm.setGoal(arm.getGoal() + change.getAsDouble()), arm);
    }

    /**
     * Returns a command that moves the arm to a predefined goal position based on an index.
     *
     * @param goal The index of the predefined goal position in the `ArmConstants.goals` array.
     * @return A command that moves the arm to the specified goal position.
     */
    public Command moveToGoal(int goal) {
        return new InstantCommand(() -> arm.setGoal(ArmConstants.goals[goal - 1]), arm);
    }

    /**
     * Returns a command that moves the arm to the predefined "source" position.
     *
     * @return A command that moves the arm to the source position.
     */
    public Command moveToSource() {
        return new InstantCommand(() -> arm.setGoal(ArmConstants.sourceGoal), arm);
    }

    /**
     * Returns a command that moves the arm to the predefined "ground intake" position.
     *
     * @return A command that moves the arm to the ground intake position.
     */
    public Command moveToGroundIntake() {
        return new InstantCommand(() -> arm.setGoal(ArmConstants.groundIntakeGoal), arm);
    }

    /**
     * Returns a command that moves the arm to the predefined "default" position.
     *
     * @return A command that moves the arm to the default position.
     */
    public Command moveToDefault() {
        return new InstantCommand(() -> arm.setGoal(ArmConstants.defaultGoal), arm);
    }

    /**
     * Returns a command that moves the arm to the predefined "algae" position.
     *
     * @return A command that moves the arm to the algae position.
     */
    public Command moveToAlgae() {
        return new InstantCommand(() -> arm.setGoal(ArmConstants.algaeGoal), arm);
    }

    /**
     * Returns a command that runs a quasi-static system identification routine on the arm.
     *
     * @param dir The direction of the SysId routine (forward or reverse).
     * @return A command that runs the quasi-static SysId routine.
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction dir) {
        return arm.getRoutine().quasistatic(dir);
    }

    /**
     * Returns a command that runs a dynamic system identification routine on the arm.
     *
     * @param dir The direction of the SysId routine (forward or reverse).
     * @return A command that runs the dynamic SysId routine.
     */
    public Command sysIdDynamic(SysIdRoutine.Direction dir) {
        return arm.getRoutine().dynamic(dir);
    }
}