package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.other.MotorTransform;


public class ElevatorConstants {
    // Motor IDs
    public static final int leftMotorId = 21; // TODO: Change to 1
    public static final int rightMotorId = 22; // TODO: Change to 2

    // Encoder transform
    public static final double elevatorOffset = 0;
    public static final double metersPerRotation = 0.01397;
    public static final MotorTransform transform = new MotorTransform(metersPerRotation, elevatorOffset);

    // PID constants
    public static final double kP = 30;
    public static final double kI = 0;
    public static final double kD = 1;

    // Feedforward constants
    public static final double kS = 0;
    public static final double kG = 0.18;
    public static final double kV = 0.88;
    public static final double kA = 0.02;

    // Elevator tolerance
    public static final double elevatorTol = 0.1;

    // Elevator goals
    public static final double[] goals = {0.06,0.26,0.56,1.26};
    public static final double sourceGoal = 0.76;
    public static final double groundIntakeGoal = 0.26;
    public static final double algaeGoal = 0.35;
    public static final double defaultGoal = 0;

    // Simulation constants
    public static final double gearRatio = 20.0; // gear ratio
    public static final double carriageMass = 13.0; // in kg
    public static final double drumRadius = metersPerRotation*gearRatio/(2*Math.PI); // in meters
    public static final double minHeight = 0; // in meters
    public static final double maxHeight = 1.27; // in meters
    public static final DCMotor motorSim = DCMotor.getFalcon500(2);

    // Mech constants
    public static final double mechWidth = 20;
    public static final double mechHeight = 50;
}
