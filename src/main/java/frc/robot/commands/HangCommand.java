package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.HangConstants;
import frc.robot.subsystems.Hang;

/**
 * The `HangCommand` class is a `SequentialCommandGroup` that sequences a series of commands
 * to control the hanging mechanism of the robot. This command is used to perform a sequence
 * of actions: deactivating the clamp, extending the climb mechanism, activating the clamp,
 * and then retracting the climb mechanism.
 */
public class HangCommand extends SequentialCommandGroup {

    /**
     * Constructs a new `HangCommand` that sequences the necessary commands to operate the hang mechanism.
     *
     * @param hangSubsystem The `Hang` subsystem that this command will operate on.
     */
    public HangCommand(Hang hangSubsystem) {
        // Add a sequence of commands to be executed in order
        addCommands(
            // Deactivate the clamp mechanism
            Commands.runOnce(() -> hangSubsystem.clampDeactivate()),
            
            // Wait for the specified deactivation time
            new WaitCommand(HangConstants.clampDeactivationTime),
            
            // Extend the climb mechanism
            Commands.runOnce(() -> hangSubsystem.climbExtend()),
            
            // Wait for the specified extension time
            new WaitCommand(HangConstants.climbExtensionTime),
            
            // Activate the clamp mechanism
            Commands.runOnce(() -> hangSubsystem.clampActivate()),
            
            // Wait for the specified activation time
            new WaitCommand(HangConstants.clampActivationTime),
            
            // Retract the climb mechanism
            Commands.runOnce(() -> hangSubsystem.climbRetract())
        );
    }
}