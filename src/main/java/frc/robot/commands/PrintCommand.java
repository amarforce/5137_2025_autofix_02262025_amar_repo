package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class PrintCommand extends InstantCommand{
    public PrintCommand(Supplier<String> p){
        super(()->System.out.println(p.get()));
    }
}
