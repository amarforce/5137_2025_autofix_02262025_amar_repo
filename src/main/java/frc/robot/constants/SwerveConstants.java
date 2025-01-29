package frc.robot.constants;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

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

    public static final Pose2d processor = new Pose2d(new Translation2d(6.25, 0.65), new Rotation2d(3*Math.PI/2));

    public static final Pose2d leftCage = new Pose2d(new Translation2d(8.5, 7.25), new Rotation2d(3*Math.PI/2));
    public static final Pose2d centerCage = new Pose2d(new Translation2d(8.5, 6.15), new Rotation2d(3*Math.PI/2));
    public static final Pose2d rightCage = new Pose2d(new Translation2d(8.5, 5.05), new Rotation2d(3*Math.PI/2));
    public static final Pose2d[] cages = {leftCage, centerCage, rightCage};

    public static final Pose2d stationA = new Pose2d(new Translation2d(1.55, 0.75), new Rotation2d(Units.degreesToRadians(54)));
    public static final Pose2d stationB = new Pose2d(new Translation2d(0.725, 1.375), new Rotation2d(Units.degreesToRadians(54)));
    public static final Pose2d stationC = new Pose2d(new Translation2d(0.725, 6.675), new Rotation2d(Units.degreesToRadians(306)));
    public static final Pose2d stationD = new Pose2d(new Translation2d(1.55, 7.3), new Rotation2d(Units.degreesToRadians(306)));
    public static final Pose2d[] stations = {stationA, stationB, stationC, stationD};

    public static final Pose2d leftPickup = new Pose2d(new Translation2d(1.8, 5.825), new Rotation2d(0.0));
    public static final Pose2d centerPickup = new Pose2d(new Translation2d(1.8, 4.025), new Rotation2d(0.0));
    public static final Pose2d rightPickup = new Pose2d(new Translation2d(1.8, 2.225), new Rotation2d(0.0));
    public static final Pose2d[] pickups = {leftPickup, centerPickup, rightPickup};

    public static final Translation2d reefCenter = new Translation2d(4.4958,4.0259);
    public static final double d1 = 1.3;
    public static final double d2 = 0.16;

    // public static final Pose2d reefA = new Pose2d(new Translation2d(3.15, 4.19), new Rotation2d(0.0));
    // public static final Pose2d reefB = new Pose2d(new Translation2d(3.15, 3.86), new Rotation2d(0.0));
    // public static final Pose2d reefC = new Pose2d(new Translation2d(3.685, 2.98), new Rotation2d(Math.PI/3));
    // public static final Pose2d reefD = new Pose2d(new Translation2d(3.965, 2.82), new Rotation2d(Math.PI/3));
    // public static final Pose2d reefE = new Pose2d(new Translation2d(4.985, 2.82), new Rotation2d(2*Math.PI/3));
    // public static final Pose2d reefF = new Pose2d(new Translation2d(5.265, 2.98), new Rotation2d(2*Math.PI/3));
    // public static final Pose2d reefG = new Pose2d(new Translation2d(5.8, 3.86), new Rotation2d(Math.PI));
    // public static final Pose2d reefH = new Pose2d(new Translation2d(5.8, 4.19), new Rotation2d(Math.PI));
    // public static final Pose2d reefI = new Pose2d(new Translation2d(5.265, 5.07), new Rotation2d(4*Math.PI/3));
    // public static final Pose2d reefJ = new Pose2d(new Translation2d(4.985, 5.23), new Rotation2d(4*Math.PI/3));
    // public static final Pose2d reefK = new Pose2d(new Translation2d(3.965, 5.23), new Rotation2d(5*Math.PI/3));
    // public static final Pose2d reefL = new Pose2d(new Translation2d(3.685, 5.07), new Rotation2d(5*Math.PI/3));
    // public static final Pose2d reefAB = new Pose2d(new Translation2d(3.15, 4.025), new Rotation2d(0.0));
    // public static final Pose2d reefCD = new Pose2d(new Translation2d(3.825, 2.9), new Rotation2d(Math.PI/3));
    // public static final Pose2d reefEF = new Pose2d(new Translation2d(5.125, 2.9), new Rotation2d(2*Math.PI/3));
    // public static final Pose2d reefGH = new Pose2d(new Translation2d(5.8, 4.025), new Rotation2d(Math.PI));
    // public static final Pose2d reefIJ = new Pose2d(new Translation2d(5.125, 5.15), new Rotation2d(4*Math.PI/3));
    // public static final Pose2d reefKL = new Pose2d(new Translation2d(3.825, 5.15), new Rotation2d(5*Math.PI/3));

    public static final Pose2d[] leftReef = new Pose2d[GeneralConstants.sides];
    public static final Pose2d[] centerReef = new Pose2d[GeneralConstants.sides];
    public static final Pose2d[] rightReef = new Pose2d[GeneralConstants.sides];
    public static final Pose2d[] allReef = new Pose2d[GeneralConstants.sides*2];
    static{
        for(int i=0;i<GeneralConstants.sides;i++){
            Rotation2d forward=new Rotation2d(2*i*Math.PI/GeneralConstants.sides);
            Rotation2d angle=forward.rotateBy(Rotation2d.k180deg);
            Translation2d algae=reefCenter.plus(new Translation2d(d1, angle));
            centerReef[i]=new Pose2d(algae,forward);
            Translation2d left=algae.plus(new Translation2d(d2, angle.rotateBy(Rotation2d.kCW_90deg)));
            leftReef[i]=new Pose2d(left,forward);
            allReef[2*i]=leftReef[i];
            Translation2d right=algae.plus(new Translation2d(d2, angle.rotateBy(Rotation2d.kCCW_90deg)));
            rightReef[i]=new Pose2d(right,forward);
            allReef[2*i+1]=rightReef[i];
        }
    }
}
