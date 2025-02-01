package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.other.MotorTransform;

public class WristConstants {
    // Motor ID
    public static final int motorId = 24;

    // Encoder transform
    public static final double wristOffset = 0.0;
    public static final double gearRatio = 100.0;
    public static final MotorTransform transform = new MotorTransform((2*Math.PI)/gearRatio, wristOffset);
    

    // PID constants
    public static final double kP = 30;
    public static final double kI = 0;
    public static final double kD = 1;

    // Tolerance
    public static final double wristTolerance = 0.1;

    // Positions
    public static final double pos1 = Units.degreesToRadians(0); // down
    public static final double pos2 = Units.degreesToRadians(90); // straight

    // Simulation constants
    public static final double momentOfInertia = 0.0155;
    public static final double wristLength = 0.1524;
    public static final double minAngle = Units.degreesToRadians(0);
    public static final double maxAngle = Units.degreesToRadians(90);
    public static final DCMotor motorSim = DCMotor.getKrakenX60(1);
}