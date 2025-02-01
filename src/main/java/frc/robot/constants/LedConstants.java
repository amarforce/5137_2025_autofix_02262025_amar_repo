package frc.robot.constants;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Map;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;

public class LEDConstants {
    public static final LEDPattern red = LEDPattern.solid(Color.kRed);
    public static final LEDPattern blue = LEDPattern.solid(Color.kBlue);
    public static final LEDPattern rainbow = LEDPattern.rainbow(255, 100).scrollAtAbsoluteSpeed(MetersPerSecond.of(1), Meters.of(1/120.0));
    public static final LEDPattern gradient = LEDPattern.gradient(GradientType.kContinuous, Color.kRed,Color.kBlue).scrollAtAbsoluteSpeed(MetersPerSecond.of(1), Meters.of(1/120.0));
    public static final LEDPattern steps = LEDPattern.steps(Map.of(0,Color.kRed,0.3,Color.kBlue,0.6,Color.kGreen));
    public static final int LEDLength = 30;
}
