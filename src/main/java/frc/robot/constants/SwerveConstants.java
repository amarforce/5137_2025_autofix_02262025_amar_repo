package frc.robot.constants;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import com.pathplanner.lib.path.PathConstraints;

public class SwerveConstants {
    public static final double translationalDeadband = 0.1;
    public static final double rotationalDeadband = 0.1;
    public static final double odometryFrequency = 20; // ms
    public static final double simLoopPeriod = 0.005; // seconds

    public static final double translationKP = 5.0;
    public static final double translationKI = 0.0;
    public static final double translationKD = 0.0;
    public static final double rotationKP = 5.0;
    public static final double rotationKI = 0.0;
    public static final double rotationKD = 0.0;

    public static final PathConstraints constraints = new PathConstraints(
        MetersPerSecond.of(5.0),
        MetersPerSecondPerSecond.of(5.0),
        RadiansPerSecond.of(1.5*Math.PI),
        RadiansPerSecondPerSecond.of(Math.PI));

    public static final double coralExpirationTime = 5;
}
