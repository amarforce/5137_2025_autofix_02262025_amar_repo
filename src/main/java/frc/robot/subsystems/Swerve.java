package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.constants.SwerveConstants;
import frc.robot.other.DetectedObject;
import frc.robot.other.RobotUtils;
import frc.robot.other.SwerveFactory;

import static edu.wpi.first.units.Units.*;

import java.io.File;
import java.util.List;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * The Swerve subsystem controls the swerve drivetrain of the robot.
 * It handles driving, odometry, vision integration, and system identification (SysId) routines.
 */
public class Swerve extends SubsystemBase {

    private SwerveDrivetrain<TalonFX, TalonFX, CANcoder> swerve; // Swerve drivetrain instance
    private Vision vision; // Vision subsystem for pose estimation and object detection

    private double maxSpeed; // Maximum translational speed of the robot
    private double maxAngularSpeed; // Maximum rotational speed of the robot

    private Field2d field; // Field visualization for SmartDashboard

    // Swerve control requests
    private SwerveRequest.FieldCentric fieldOrientedDrive; // Field-oriented driving request
    private SwerveRequest.RobotCentric robotOrientedDrive; // Robot-oriented driving request
    private SwerveRequest.ApplyRobotSpeeds setChassisSpeeds; // Request to set chassis speeds directly
    private SwerveRequest.SwerveDriveBrake lock; // Request to lock the swerve modules in place

    private StringLogEntry log;

    // Target pose
    private Pose2d targetPose;

    /* Mechanisms to represent the swerve module states */
    private final Mechanism2d[] moduleMechanisms = new Mechanism2d[] {
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
    };
    /* A direction and length changing ligament for speed representation */
    private final MechanismLigament2d[] moduleSpeeds = new MechanismLigament2d[] {
        moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
    };
    /* A direction changing and length constant ligament for module direction */
    private final MechanismLigament2d[] moduleDirections = new MechanismLigament2d[] {
        moduleMechanisms[0].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        moduleMechanisms[1].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        moduleMechanisms[2].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        moduleMechanisms[3].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
    };

    private StructPublisher<Pose2d> currentPosePublisher = NetworkTableInstance.getDefault().getStructTopic("SmartDashboard/driveState/pose", Pose2d.struct).publish();
    private StructPublisher<Pose2d> targetPosePublisher = NetworkTableInstance.getDefault().getStructTopic("SmartDashboard/driveState/targetPose", Pose2d.struct).publish();

    /**
     * Constructor for the Swerve subsystem.
     *
     * @param file    The configuration file for the swerve drivetrain.
     * @param vision  The vision subsystem for pose estimation.
     */
    public Swerve(File file, Vision vision,StringLogEntry log) {
        SwerveFactory factory = new SwerveFactory(file);
        swerve = factory.create(); // Create the swerve drivetrain using the factory
        this.vision = vision;

        maxSpeed = factory.getMaxSpeed(); // Get max speed from factory
        maxAngularSpeed = factory.getMaxAngularSpeed(); // Get max angular speed from factory

        // Configure field-oriented and robot-oriented driving requests with deadbands
        fieldOrientedDrive = new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeed * SwerveConstants.translationalDeadband)
            .withRotationalDeadband(maxAngularSpeed * SwerveConstants.rotationalDeadband)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        robotOrientedDrive = new SwerveRequest.RobotCentric()
            .withDeadband(maxSpeed * SwerveConstants.translationalDeadband)
            .withRotationalDeadband(maxAngularSpeed * SwerveConstants.rotationalDeadband)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        setChassisSpeeds = new SwerveRequest.ApplyRobotSpeeds(); // Request to set chassis speeds
        lock = new SwerveRequest.SwerveDriveBrake(); // Request to lock the swerve modules

        // Configure PathPlanner AutoBuilder for autonomous driving
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        AutoBuilder.configure(
                this::getPose, // Method to get the current pose
                this::resetPose, // Method to reset the pose
                this::getCurrentSpeeds, // Method to get the current chassis speeds
                (speeds, feedforwards) -> drive(speeds), // Method to drive the robot
                new PPHolonomicDriveController(
                        new PIDConstants(SwerveConstants.translationKP, SwerveConstants.translationKI, SwerveConstants.translationKD),
                        new PIDConstants(SwerveConstants.rotationKP, SwerveConstants.rotationKI, SwerveConstants.rotationKD)
                ),
                config,
                () -> RobotUtils.onRedAlliance(), // Method to check if the robot is on the red alliance
                this
        );

        

        // Start simulation thread if in simulation
        if (Robot.isSimulation()) {
            startSimThread();
        }

        // Initialize telemetry and field visualization
        swerve.registerTelemetry(this::telemetry);

        field = new Field2d();
        SmartDashboard.putData("field", field);

        for(int i=0;i<moduleMechanisms.length;i++){
            SmartDashboard.putData("module" + i, moduleMechanisms[i]);
        }

        this.log=log;
    }

    /**
     * Sets the control request for the swerve drivetrain.
     *
     * @param request The swerve control request to apply.
     */
    public void setControl(SwerveRequest request) {
        swerve.setControl(request);
    }

    /**
     * Resets the robot's pose to the specified pose.
     *
     * @param pose The new pose to set.
     */
    public void resetPose(Pose2d pose) {
        swerve.resetPose(pose);
    }

    /**
     * Gets the current pose of the robot.
     *
     * @return The current pose of the robot.
     */
    public Pose2d getPose() {
        return swerve.getState().Pose;
    }

    /**
     * Drives the robot with the specified chassis speeds.
     *
     * @param speeds The chassis speeds to apply.
     */
    public void drive(ChassisSpeeds speeds) {
        setControl(setChassisSpeeds.withSpeeds(speeds));
    }

    /**
     * Gets the current chassis speeds of the robot.
     *
     * @return The current chassis speeds.
     */
    public ChassisSpeeds getCurrentSpeeds() {
        return swerve.getState().Speeds;
    }

    /**
     * Drives the robot with a percentage of maximum speed.
     *
     * @param dx          The percentage of maximum speed in the x direction.
     * @param dy          The percentage of maximum speed in the y direction.
     * @param dtheta      The percentage of maximum angular speed.
     * @param fieldRelative Whether the drive is field-relative or robot-relative.
     */
    public void setPercentDrive(double dx, double dy, double dtheta, boolean fieldRelative) {
        double absSpeedX = dx*maxSpeed;
        double absSpeedY = dy*maxSpeed;
        double absRot = dtheta*maxAngularSpeed;
        if (fieldRelative) {
            setControl(fieldOrientedDrive
                .withVelocityX(absSpeedX)
                .withVelocityY(absSpeedY)
                .withRotationalRate(absRot)
            );
        } else {
            setControl(robotOrientedDrive
                .withVelocityX(absSpeedX)
                .withVelocityY(absSpeedY)
                .withRotationalRate(absRot)
            );
        }
    }

    /**
     * Gets the closest pose from a list of poses.
     *
     * @param poses The list of poses to compare.
     * @return The closest pose to the current robot pose.
     */
    public Pose2d getClosest(Pose2d[] poses) {
        return RobotUtils.getClosestPoseToPose(this.getPose(), poses);
    }

    /**
     * Resets the gyro and reseeds the field-centric drive.
     */
    public void resetGyro() {
        swerve.setOperatorPerspectiveForward(RobotUtils.getPerspectiveForward());
        swerve.seedFieldCentric();
    }

    /**
     * Locks the swerve modules in place.
     */
    public void lock() {
        this.setControl(lock);
    }

    public List<DetectedObject> getGroundCoral(){
        return vision.getGroundCoral(SwerveConstants.coralExpirationTime);
    }

    public Pose2d getTargetPose(){
        return targetPose;
    }

    public void setTargetPose(Pose2d target){
        if(target!=targetPose){
            targetPose=target;
            Command auto = AutoBuilder.pathfindToPose(targetPose, SwerveConstants.constraints);
            auto.addRequirements(this);
            auto.schedule();
        }
    }

    public boolean atTarget(){
        Pose2d currentPose=getPose();
        double dist=currentPose.getTranslation().getDistance(targetPose.getTranslation());
        if(dist>SwerveConstants.transTol){
            return false;
        }
        double rotDist=currentPose.getRotation().minus(targetPose.getRotation()).getRadians();
        if(rotDist>SwerveConstants.rotTol){
            return false;
        }
        return true;
    }

    /**
     * Periodic method called every robot loop cycle.
     * Updates vision measurements, processes new objects, and updates field visualization.
     */
    @Override
    public void periodic() {
        try{
            List<EstimatedRobotPose> newPoses = vision.getNewPoses();
            for (EstimatedRobotPose newPose : newPoses) {
                swerve.addVisionMeasurement(newPose.estimatedPose.toPose2d(), newPose.timestampSeconds);
            }

            vision.processNewObjects(this.getPose());
            field.setRobotPose(this.getPose());
            if(Robot.isSimulation()){
                vision.updateSim(this.getPose());
            }
        }catch(Exception e){
            log.append("Periodic error: "+RobotUtils.getError(e));
        }
    }

    private void telemetry(SwerveDriveState state){
        SmartDashboard.putNumber("driveState/odometryPeriod", state.OdometryPeriod);

        currentPosePublisher.set(state.Pose);

        for (int i = 0; i < 4; ++i) {
            SmartDashboard.putNumberArray("driveState/moduleStates/"+i, new double[]{state.ModuleStates[i].speedMetersPerSecond,state.ModuleStates[i].angle.getRadians()});
            SmartDashboard.putNumberArray("driveState/moduleTargets/"+i, new double[]{state.ModuleTargets[i].speedMetersPerSecond,state.ModuleStates[i].angle.getRadians()});
        }

        /* Telemeterize the module states to a Mechanism2d */
        for (int i = 0; i < moduleMechanisms.length; ++i) {
            moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
            moduleDirections[i].setAngle(state.ModuleStates[i].angle);
            moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * maxSpeed));
        }

        if(targetPose!=null){
            targetPosePublisher.set(targetPose);
        }
    }

    // SysId Routines for system characterization

    /**
     * SysId routine for characterizing translation.
     * This is used to find PID gains for the drive motors.
     */
    public final SysIdRoutine sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null        // Use default timeout (10 s)
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(new SwerveRequest.SysIdSwerveTranslation().withVolts(output)),
            null,
            this
        )
    );

    /**
     * SysId routine for characterizing steer.
     * This is used to find PID gains for the steer motors.
     */
    public final SysIdRoutine sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null        // Use default timeout (10 s)
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(new SwerveRequest.SysIdSwerveSteerGains().withVolts(volts)),
            null,
            this
        )
    );

    /**
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     */
    public final SysIdRoutine sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(Math.PI / 6).per(Second), // Ramp rate in radians per second²
            Volts.of(Math.PI), // Dynamic voltage in radians per second
            null // Use default timeout (10 s)
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                setControl(new SwerveRequest.SysIdSwerveRotation().withRotationalRate(output.in(Volts)));
            },
            null,
            this
        )
    );

    private SysIdRoutine routine = sysIdRoutineTranslation; // Current SysId routine to test

    /**
     * Sets the current SysId routine.
     *
     * @param routine The SysId routine to set.
     */
    public void setRoutine(SysIdRoutine routine) {
        this.routine = routine;
    }

    /**
     * Gets the current SysId routine.
     *
     * @return The current SysId routine.
     */
    public SysIdRoutine getRoutine() {
        return routine;
    }

    // Simulation
    private Notifier simNotifier; // Notifier for simulation thread
    private double lastSimTime; // Last simulation time

    /**
     * Starts the simulation thread.
     */
    private void startSimThread() {
        lastSimTime = Utils.getCurrentTimeSeconds();

        // Run simulation at a faster rate for better PID behavior
        simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;

            // Update simulation state with measured time delta and battery voltage
            swerve.updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        simNotifier.startPeriodic(SwerveConstants.simLoopPeriod);
    }
}