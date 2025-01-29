package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.WristConstants;
import frc.robot.subsystems.Wrist;

/**
 * The `WristCommands` class provides a set of commands for controlling the wrist subsystem.
 * These commands include moving the wrist to specific positions and running system identification routines.
 */
public class WristCommands {
    private Wrist wrist; // The wrist subsystem that these commands will control.

    /**
     * Constructs a new `WristCommands` object.
     *
     * @param wrist The wrist subsystem that this set of commands will control.
     */
    public WristCommands(Wrist wrist) {
        this.wrist = wrist;
    }

    /**
     * Returns a command that moves the wrist to position 1.
     *
     * @return A command that sets the wrist's goal position to `WristConstants.pos1`.
     */
    public Command toPos1() {
        return new InstantCommand(() -> wrist.setGoal(WristConstants.pos1));
    }

    /**
     * Returns a command that moves the wrist to position 2.
     *
     * @return A command that sets the wrist's goal position to `WristConstants.pos2`.
     */
    public Command toPos2() {
        return new InstantCommand(() -> wrist.setGoal(WristConstants.pos2));
    }

    /**
     * Returns a command that runs a quasi-static system identification routine on the wrist.
     *
     * @param dir The direction of the system identification routine (forward or reverse).
     * @return A command that runs the quasi-static system identification routine.
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction dir) {
        return wrist.getRoutine().quasistatic(dir);
    }

    /**
     * Returns a command that runs a dynamic system identification routine on the wrist.
     *
     * @param dir The direction of the system identification routine (forward or reverse).
     * @return A command that runs the dynamic system identification routine.
     */
    public Command sysIdDynamic(SysIdRoutine.Direction dir) {
        return wrist.getRoutine().dynamic(dir);
    }
}