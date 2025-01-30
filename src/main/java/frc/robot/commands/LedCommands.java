package frc.robot.commands;


import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Map;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.*;

public class LedCommands 
{

    private Led led;
    public LedCommands(Led led)
    {
        this.led = led;
    }
    public Command red()
    {
        return new InstantCommand(() -> led.set(LEDPattern.solid(Color.kRed)), led);
    }
    public Command blue()
    {
        return new InstantCommand(() -> led.set(LEDPattern.solid(Color.kBlue)), led);
    }
    public Command rainbow()    
    {
        return new InstantCommand(() -> led.set(LEDPattern.rainbow(255, 100).scrollAtAbsoluteSpeed(MetersPerSecond.of(1), Meters.of(1/120.0))),led);
    }
    public Command gradient()
    {
        return new InstantCommand(()->led.set(LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kRed,Color.kBlue)),led);
    }
    public Command steps()
    {
        return new InstantCommand(()->led.setReverse(LEDPattern.steps(Map.of(0,Color.kRed,0.3,Color.kBlue,0.6,Color.kGreen))),led);
    }


                                                                                                                                                                                                                                                                                                                                        }
