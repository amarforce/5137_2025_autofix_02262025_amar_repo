package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Wrist;

/**
 * The `WristCommands` class provides a set of commands for controlling the wrist subsystem.
 * These commands include running system identification routines.
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