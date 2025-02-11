package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.other.RobotUtils;

public class GeneralConstants {
    public static final double fieldLength = 17.55;
    public static final double fieldWidth = 8.05;
    public static final int sides = 6;
    public static final int numAuto = 6;
    public static final double simPeriod = 0.02;

    private static final Pose2d processor=new Pose2d(new Translation2d(6.25, 0.65), new Rotation2d(3*Math.PI/2));
    public static final Pose2d processor(){
        return RobotUtils.invertToAlliance(processor);
    }

    private static final Pose2d leftCage = new Pose2d(new Translation2d(8.5, 7.25), new Rotation2d(3*Math.PI/2));
    private static final Pose2d centerCage = new Pose2d(new Translation2d(8.5, 6.15), new Rotation2d(3*Math.PI/2));
    private static final Pose2d rightCage = new Pose2d(new Translation2d(8.5, 5.05), new Rotation2d(3*Math.PI/2));
    private static final Pose2d[] cages={leftCage,centerCage,rightCage};
    public static Pose2d[] cages(){
        return RobotUtils.invertToAlliance(cages);
    }

    private static final Pose2d stationRR = new Pose2d(new Translation2d(1.55, 0.75), new Rotation2d(Units.degreesToRadians(54)));
    private static final Pose2d stationRL = new Pose2d(new Translation2d(0.725, 1.375), new Rotation2d(Units.degreesToRadians(54)));
    private static final Pose2d stationLR = new Pose2d(new Translation2d(0.725, 6.675), new Rotation2d(Units.degreesToRadians(306)));
    private static final Pose2d stationLL = new Pose2d(new Translation2d(1.55, 7.3), new Rotation2d(Units.degreesToRadians(306)));
    private static final Pose2d[] stations = {stationRR,stationRL,stationLR,stationLL};
    public static Pose2d[] stations(){
        return RobotUtils.invertToAlliance(stations);
    }
    

    private static final Pose2d leftPickup = new Pose2d(new Translation2d(1.8, 5.825), new Rotation2d(0.0));
    private static final Pose2d centerPickup = new Pose2d(new Translation2d(1.8, 4.025), new Rotation2d(0.0));
    private static final Pose2d rightPickup = new Pose2d(new Translation2d(1.8, 2.225), new Rotation2d(0.0));
    private static final Pose2d[] pickups = {leftPickup, centerPickup, rightPickup};
    public static Pose2d[] pickups(){
        return RobotUtils.invertToAlliance(pickups);
    }

    private static final Translation2d reefCenter = new Translation2d(4.4958,4.0259);

    private static final double d1 = 1.3;
    private static final double d2 = 0.16;
    private static final double dShift = -0.13;

    private static final Pose2d[] leftReef = new Pose2d[GeneralConstants.sides];
    public static Pose2d[] leftReef(){
        return RobotUtils.invertToAlliance(leftReef);
    }

    private static final Pose2d[] centerReef = new Pose2d[GeneralConstants.sides];
    public static Pose2d[] centerReef(){
        return RobotUtils.invertToAlliance(centerReef);
    }

    private static final Pose2d[] rightReef = new Pose2d[GeneralConstants.sides];
    public static Pose2d[] rightReef(){
        return RobotUtils.invertToAlliance(rightReef);
    }

    private static final Pose2d[] allReef = new Pose2d[GeneralConstants.sides*2];
    public static Pose2d[] allReef(){
        return RobotUtils.invertToAlliance(allReef);
    }

    static{
        for(int i=0;i<GeneralConstants.sides;i++){
            Rotation2d forward=new Rotation2d(2*i*Math.PI/GeneralConstants.sides);
            Rotation2d angle=forward.rotateBy(Rotation2d.k180deg);
            Translation2d outer=reefCenter.plus(new Translation2d(d1, angle));
            Rotation2d sidewaysAngle=angle.rotateBy(Rotation2d.kCW_90deg);
            Translation2d algae=outer.plus(new Translation2d(dShift,sidewaysAngle));
            centerReef[i]=RobotUtils.invertToAlliance(new Pose2d(algae,forward));
            Translation2d left=algae.plus(new Translation2d(d2, sidewaysAngle));
            leftReef[i]=RobotUtils.invertToAlliance(new Pose2d(left,forward));
            allReef[2*i]=leftReef[i];
            Translation2d right=algae.plus(new Translation2d(-d2, sidewaysAngle));
            rightReef[i]=RobotUtils.invertToAlliance(new Pose2d(right,forward));
            allReef[2*i+1]=rightReef[i];
        }
    }

    private static final Translation3d[][] coralPositions = new Translation3d[GeneralConstants.sides*2][3];
    public static Translation3d[][] coralPositions(){
        return RobotUtils.invertToAlliance(coralPositions);
    }

    private static final double branchd1 = 0.75;
    private static final double branchd2 = 0.16;
    private static final double l2d1 = 0.1;
    private static final double l2height = 0.740;
    private static final double l3d1 = 0.1;
    private static final double l3height = 1.143;
    private static final double l4d1 = 0.2;
    private static final double l4height = 1.6;
    static{
        for(int i=0;i<GeneralConstants.sides*2;i++){
            Rotation2d angle=new Rotation2d(2*(i/2)*Math.PI/GeneralConstants.sides).rotateBy(Rotation2d.k180deg);
            Translation2d center=reefCenter.plus(new Translation2d(branchd1, angle));
            Translation2d side=center.plus(new Translation2d(branchd2, angle.rotateBy(i%2==0?Rotation2d.kCW_90deg:Rotation2d.kCCW_90deg)));
            coralPositions[i][0]=new Translation3d(side.plus(new Translation2d(l2d1, angle))).plus(new Translation3d(0, 0, l2height));
            coralPositions[i][1]=new Translation3d(side.plus(new Translation2d(l3d1, angle))).plus(new Translation3d(0, 0, l3height));
            coralPositions[i][2]=new Translation3d(side.plus(new Translation2d(l4d1, angle))).plus(new Translation3d(0, 0, l4height));
        }
    }


}
