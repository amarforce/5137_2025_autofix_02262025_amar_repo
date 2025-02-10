package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.other.MotorTransform;

public class WristConstants {
    // Motor ID
    public static final int motorId = Robot.isSimulation()?23:3;

    // Limits
    public static final double minAngle = Units.degreesToRadians(-90);
    public static final double maxAngle = Units.degreesToRadians(90);

    // Encoder transform
    public static final double wristOffset = 0.0;
    public static final double gearRatio = 31.5;
    public static final MotorTransform transform = new MotorTransform((2*Math.PI)/gearRatio, wristOffset);
    
    // PID constants
    public static final double kP = 30;
    public static final double kI = 0;
    public static final double kD = 1;

    // Feedforward constants
    public static final double kS = 0.0;
    public static final double kG = 0.1;
    public static final double kV = 0.62;
    public static final double kA = 0.0;

    // Tolerance
    public static final double wristTolerance = Units.degreesToRadians(1);

    // Simulation constants
    public static final double momentOfInertia = 0.0155;
    public static final double wristLength = 0.1524;
    public static final DCMotor motorSim = DCMotor.getKrakenX60(1);
}