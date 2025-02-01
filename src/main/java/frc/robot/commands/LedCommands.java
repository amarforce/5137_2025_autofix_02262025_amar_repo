package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.*;

public class LEDCommands 
{

    private LED led;
    public LEDCommands(LED led)
    {
        this.led = led;
    }
    public Command red()
    {
        return new InstantCommand(() -> led.red(), led);
    }
    public Command blue()
    {
        return new InstantCommand(() -> led.blue(), led);
    }
    public Command fire()    
    {
        return new InstantCommand(() -> led.fire(),led);
    }
    public Command green()
    {
        return new InstantCommand(()->led.green(),led);
    }
}