package frc.robot.constants;

import frc.robot.Robot;

public class IntakeConstants {
    public static final int motorId = Robot.isSimulation()?24:4;
    public static final int switchChannel = 0; // TODO change
    public static final double intakeSpeed = 0.3;
    public static final double outtakeTime = 1;
    public static final double intakeTimeout = 4;
}
