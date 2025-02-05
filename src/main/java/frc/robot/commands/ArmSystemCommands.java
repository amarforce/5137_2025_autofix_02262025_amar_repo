package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.ArmSystem;

public class ArmSystemCommands {
    private ArmSystem armSystem;

    public ArmSystemCommands(ArmSystem armSystem){
        this.armSystem=armSystem;
    }
    
    /**
     * 
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
}
