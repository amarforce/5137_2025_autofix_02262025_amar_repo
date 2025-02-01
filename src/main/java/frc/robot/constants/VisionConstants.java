package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class VisionConstants {
    public static final Transform3d robotToFrontCamera = new Transform3d(new Translation3d(0.35, 0, 0.2), new Rotation3d(0, -Math.PI/6, 0));
    public static final Transform3d robotToLeftCamera = new Transform3d(new Translation3d(0, 0.35, 0.2), new Rotation3d(0, -Math.PI/6, Math.PI/2));
    public static final Transform3d robotToFrontObjectCamera = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));

    public static final String[] classNames = {"Algae","Coral"};

    public static final double objectMarginOfError = 0.1;

    // public static final Translation2d reefA = new Translation2d(3.75, 4.19);
    // public static final Translation2d reefB = new Translation2d(3.75, 3.86);
    // public static final Translation2d reefC = new Translation2d(4.00, 3.50);
    // public static final Translation2d reefD = new Translation2d(4.25, 3.30);
    // public static final Translation2d reefE = new Translation2d(4.70, 3.30);
    // public static final Translation2d reefF = new Translation2d(4.95, 3.50);
    // public static final Translation2d reefG = new Translation2d(5.25, 3.86);
    // public static final Translation2d reefH = new Translation2d(5.25, 4.19);
    // public static final Translation2d reefI = new Translation2d(4.95, 4.60);
    // public static final Translation2d reefJ = new Translation2d(4.70, 4.80);
    // public static final Translation2d reefK = new Translation2d(4.25, 4.80);
    // public static final Translation2d reefL = new Translation2d(4.00, 4.60);
    
}
