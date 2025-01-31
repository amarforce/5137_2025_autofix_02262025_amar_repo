package frc.robot.commands;


import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Map;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.LedConstants;
import frc.robot.subsystems.*;

public class LedCommands 
{

    private LED led;
    public LedCommands(LED led)
    {
        this.led = led;
    }
    public Command red()
    {
        return new InstantCommand(() -> led.set(LedConstants.red), led);
    }
    public Command blue()
    {
        return new InstantCommand(() -> led.set(LedConstants.blue), led);
    }
    public Command rainbow()    
    {
        return new InstantCommand(() -> led.set(LedConstants.rainbow),led);
    }
    public Command gradient()
    {
        return new InstantCommand(()->led.set(LedConstants.gradient),led);
    }
    public Command steps()
    {
        return new InstantCommand(()->led.setReverse(LedConstants.steps),led);
    }


                                                                                                                                                                                                                                                                                                                                        }
