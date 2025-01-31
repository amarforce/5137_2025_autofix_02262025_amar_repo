package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.LEDConstants;
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
        return new InstantCommand(() -> led.set(LEDConstants.red), led);
    }
    public Command blue()
    {
        return new InstantCommand(() -> led.set(LEDConstants.blue), led);
    }
    public Command rainbow()    
    {
        return new InstantCommand(() -> led.set(LEDConstants.rainbow),led);
    }
    public Command gradient()
    {
        return new InstantCommand(()->led.set(LEDConstants.gradient),led);
    }
    public Command steps()
    {
        return new InstantCommand(()->led.setReverse(LEDConstants.steps),led);
    }


                                                                                                                                                                                                                                                                                                                                        }
