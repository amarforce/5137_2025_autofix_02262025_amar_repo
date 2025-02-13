package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.elastic.Reef;
import frc.robot.other.RobotUtils;

public class GeneralConstants {
    public static final double fieldLength = 17.55;
    public static final double fieldWidth = 8.05;
    public static final int sides = 6;
    public static final int numAuto = 6;
    public static final double simPeriod = 0.02;

    private static final double d1 = 1.3;
    private static final double d2 = 0.165;
    private static final double dShift = -0.21;

    private static final double branchd1 = 0.75;
    private static final double branchd2 = 0.16;
    private static final double l2d1 = 0.1;
    private static final double l2height = 0.740;
    private static final double l3d1 = 0.1;
    private static final double l3height = 1.143;
    private static final double l4d1 = 0.2;
    private static final double l4height = 1.6;

    private static final double algaed1 = 0.66;
    private static final double algaeLowHeight = 0.88;
    private static final double algaeHighHeight = 1.28;

    // Base positions (without alliance inversion)
    private static final Pose2d baseProcessor;
    private static final Pose2d[] baseCages;
    private static final Pose2d[] baseStations;
    private static final Pose2d[] basePickups;
    private static final Pose2d[] baseCenterReef;
    private static final Pose2d[] baseBranchReef;
    private static final Translation3d[][] baseCoralPositions;
    private static final Translation3d[] baseAlgaePositions;

    static {
        // Initialize processor
        baseProcessor = new Pose2d(new Translation2d(6.25, 1), new Rotation2d(3*Math.PI/2));

        // Initialize cages
        Pose2d leftCage = new Pose2d(new Translation2d(8.5, 7.25), new Rotation2d(3*Math.PI/2));
        Pose2d centerCage = new Pose2d(new Translation2d(8.5, 6.15), new Rotation2d(3*Math.PI/2));
        Pose2d rightCage = new Pose2d(new Translation2d(8.5, 5.05), new Rotation2d(3*Math.PI/2));
        baseCages = new Pose2d[]{leftCage, centerCage, rightCage};

        // Initialize stations
        Pose2d stationRR = new Pose2d(new Translation2d(1.8, 1), new Rotation2d(Units.degreesToRadians(234)));
        Pose2d stationRL = new Pose2d(new Translation2d(0.975, 1.625), new Rotation2d(Units.degreesToRadians(234)));
        Pose2d stationLR = new Pose2d(new Translation2d(0.975, 6.555), new Rotation2d(Units.degreesToRadians(126)));
        Pose2d stationLL = new Pose2d(new Translation2d(1.8, 7.05), new Rotation2d(Units.degreesToRadians(126)));
        baseStations = new Pose2d[]{stationRR, stationRL, stationLR, stationLL};

        // Initialize pickups
        Pose2d leftPickup = new Pose2d(new Translation2d(1.8, 5.825), new Rotation2d(0.0));
        Pose2d centerPickup = new Pose2d(new Translation2d(1.8, 4.025), new Rotation2d(0.0));
        Pose2d rightPickup = new Pose2d(new Translation2d(1.8, 2.225), new Rotation2d(0.0));
        basePickups = new Pose2d[]{leftPickup, centerPickup, rightPickup};

        // Initialize reef positions
        Translation2d reefCenter = new Translation2d(4.4958, 4.0259);
        baseCenterReef = new Pose2d[sides];
        baseBranchReef = new Pose2d[sides*2];
        
        for(int i = 0; i < sides; i++) {
            Rotation2d forward = new Rotation2d(2*i*Math.PI/sides).rotateBy(Rotation2d.k180deg);
            Translation2d outer = reefCenter.plus(new Translation2d(d1, forward));
            Rotation2d sidewaysAngle = forward.rotateBy(Rotation2d.kCW_90deg);
            Translation2d algae = outer.plus(new Translation2d(dShift, sidewaysAngle));
            baseCenterReef[i] = new Pose2d(algae, forward);
            Translation2d left = algae.plus(new Translation2d(d2, sidewaysAngle));
            baseBranchReef[2*i] = new Pose2d(left, forward);
            Translation2d right = algae.plus(new Translation2d(-d2, sidewaysAngle));
            baseBranchReef[2*i+1] = new Pose2d(right, forward);
        }

        // Initialize coral positions
        baseCoralPositions = new Translation3d[sides*2][3];
        for(int i = 0; i < sides*2; i++) {
            Rotation2d angle = new Rotation2d(2*(i/2)*Math.PI/sides).rotateBy(Rotation2d.k180deg);
            Translation2d center = reefCenter.plus(new Translation2d(branchd1, angle));
            Translation2d side = center.plus(new Translation2d(branchd2, angle.rotateBy(i%2==0?Rotation2d.kCW_90deg:Rotation2d.kCCW_90deg)));
            baseCoralPositions[i][0] = new Translation3d(side.plus(new Translation2d(l2d1, angle))).plus(new Translation3d(0, 0, l2height));
            baseCoralPositions[i][1] = new Translation3d(side.plus(new Translation2d(l3d1, angle))).plus(new Translation3d(0, 0, l3height));
            baseCoralPositions[i][2] = new Translation3d(side.plus(new Translation2d(l4d1, angle))).plus(new Translation3d(0, 0, l4height));
        }

        baseAlgaePositions = new Translation3d[sides*2];
        for(int i=0;i<sides; i++){
            Rotation2d angle = new Rotation2d(2*i*Math.PI/sides);
            Translation2d center = reefCenter.plus(new Translation2d(algaed1, angle));
            baseAlgaePositions[i]=new Translation3d(center).plus(new Translation3d(0, 0, Reef.isAlgaeLow(i)?algaeLowHeight:algaeHighHeight));
        }

        for(int i=sides;i<2*sides;i++){
            baseAlgaePositions[i]=RobotUtils.invertTrans3d(baseAlgaePositions[i-sides]);
        }
    }

    public static Pose2d getProcessor() {
        return RobotUtils.invertToAlliance(baseProcessor);
    }

    public static Pose2d[] getCages() {
        return RobotUtils.invertToAlliance(baseCages);
    }

    public static Pose2d[] getStations() {
        return RobotUtils.invertToAlliance(baseStations);
    }

    public static Pose2d[] getPickups() {
        return RobotUtils.invertToAlliance(basePickups);
    }

    public static Pose2d[] getCenterReef() {
        return RobotUtils.invertToAlliance(baseCenterReef);
    }

    public static Pose2d[] getBranchReef() {
        return RobotUtils.invertToAlliance(baseBranchReef);
    }

    public static Translation3d[][] getCoralPositions() {
        return RobotUtils.invertToAlliance(baseCoralPositions);
    }

    public static Translation3d[] getAlgaePositions() {
        return baseAlgaePositions;
    }
}
