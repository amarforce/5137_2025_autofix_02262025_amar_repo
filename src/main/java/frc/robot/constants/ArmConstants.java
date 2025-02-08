package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.other.MotorTransform;

public class ArmConstants {
    // Motor ID
    public static final int motorId = 2;

    // Encoder transform
    public static final double armOffset = -Math.PI/2;
    public static final double gearRatio = 31.5;
    public static final MotorTransform transform = new MotorTransform((2*Math.PI)/gearRatio, armOffset);
    
    // PID constants
    public static final double kP = 1;
    public static final double kI = 0;
    public static final double kD = 1;

    // Feedforward constants
    public static final double kS = 0.0;
    public static final double kG = 0.0;
    public static final double kV = 0.0;

    // Arm tolerance
    public static final double armTolerance = 0.1;

    // Arm goals
    public static final double[] goals = {Units.degreesToRadians(135),Units.degreesToRadians(135),Units.degreesToRadians(135),Units.degreesToRadians(120)};
    public static final double sourceGoal = Units.degreesToRadians(45);
    public static final double groundIntakeGoal = Units.degreesToRadians(0);
    public static final double algaeGoal = Units.degreesToRadians(120);
    public static final double defaultGoal = Units.degreesToRadians(90);
    
    // Simulation constants
    public static final double minAngle = Units.degreesToRadians(0);
    public static final double maxAngle = Units.degreesToRadians(135);
    public static final double momentOfInertia = 1.2;
    public static final double armLength = 0.594;
    public static final DCMotor motorSim = DCMotor.getKrakenX60(1);

    // Mechanism constants
    public static final int mechWidth = 20;
    public static final int mechHeight = 50;
}
