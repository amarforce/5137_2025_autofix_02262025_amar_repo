package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.constants.SwerveConstants;
import frc.robot.other.RobotUtils;
import frc.robot.other.SwerveFactory;
import frc.robot.other.Telemetry;

import static edu.wpi.first.units.Units.*;

import java.io.File;
import java.util.List;
import java.util.function.Consumer;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.SignalLogger;
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
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Swerve extends SubsystemBase {

    private SwerveDrivetrain<TalonFX, TalonFX, CANcoder> swerve;

    private Vision vision;

    private double maxSpeed;
    private double maxAngularSpeed;

    private Telemetry logger;
    private Field2d field;

    private SwerveRequest.FieldCentric fieldOrientedDrive;
    private SwerveRequest.RobotCentric robotOrientedDrive;
    private SwerveRequest.ApplyRobotSpeeds setChassisSpeeds;
    private SwerveRequest.SwerveDriveBrake lock;

    private SendableChooser<Integer> cageChoice;

    // Data log
    private DataLog dataLog;

    public Swerve(File file, Vision vision, DataLog dataLog) {
        SwerveFactory factory = new SwerveFactory(file);
        swerve=factory.create();
        this.vision = vision;

        maxSpeed = factory.getMaxSpeed();
        maxAngularSpeed = factory.getMaxAngularSpeed();
                
        fieldOrientedDrive = new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeed * SwerveConstants.translationalDeadband).withRotationalDeadband(maxAngularSpeed * SwerveConstants.rotationalDeadband)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        
        robotOrientedDrive = new SwerveRequest.RobotCentric()
            .withDeadband(maxSpeed * SwerveConstants.translationalDeadband).withRotationalDeadband(maxAngularSpeed * SwerveConstants.rotationalDeadband)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        setChassisSpeeds = new SwerveRequest.ApplyRobotSpeeds();
        
        lock = new SwerveRequest.SwerveDriveBrake();

        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        AutoBuilder.configure(
                this::getPose,
                this::resetPose,
                this::getCurrentSpeeds,
                (speeds, feedforwards) -> drive(speeds),
                new PPHolonomicDriveController(
                        new PIDConstants(SwerveConstants.translationKP, SwerveConstants.translationKI, SwerveConstants.translationKD),
                        new PIDConstants(SwerveConstants.rotationKP, SwerveConstants.rotationKI, SwerveConstants.rotationKD)
                ),
                config,
                () -> RobotUtils.onRedAlliance(),
                this
        );
        
        cageChoice = new SendableChooser<Integer>();
        cageChoice.addOption("Left", 0);
        cageChoice.addOption("Center", 1);
        cageChoice.addOption("Right", 2);
        cageChoice.setDefaultOption("Center", 1);
        SmartDashboard.putData("Cage Choice", cageChoice);
        
        if (Robot.isSimulation()) {
            startSimThread();
        }

        // Might remove telemtry, keep for now
        logger = new Telemetry(maxSpeed);
        swerve.registerTelemetry(logger::telemeterize);

        field = new Field2d();
        SmartDashboard.putData("Field", field);

        this.dataLog = dataLog;
    }

    public void setControl(SwerveRequest request) {
        swerve.setControl(request);
    }

    public void resetPose(Pose2d pose) {
        swerve.resetPose(pose);
    }

    public Pose2d getPose() {
        return swerve.getState().Pose;
    }

    public void drive(ChassisSpeeds speeds) {
        setControl(setChassisSpeeds.withSpeeds(speeds));
    }

    public ChassisSpeeds getCurrentSpeeds() {
        return swerve.getState().Speeds;
    }

    public void setPercentDrive(double dx, double dy, double dtheta, boolean fieldRelative) {
        if (fieldRelative) {
            setControl(fieldOrientedDrive
                .withVelocityX(dx*maxSpeed)
                .withVelocityY(dy*maxSpeed)
                .withRotationalRate(dtheta*maxAngularSpeed)
            );
        } else {
            setControl(robotOrientedDrive
                .withVelocityX(dx*maxSpeed)
                .withVelocityY(dy*maxSpeed)
                .withRotationalRate(dtheta*maxAngularSpeed)
            );
        }
    }

    public Pose2d getClosest(Pose2d[] poses){
        return RobotUtils.getClosestPoseToPose(this.getPose(), poses);
    }

    public int getCage() {
        return cageChoice.getSelected();
    }

    public void resetGyro() {
        swerve.setOperatorPerspectiveForward(RobotUtils.getPerspectiveForward());
        swerve.seedFieldCentric();
    }

    public void lock() {
        this.setControl(lock);
    }

    // Reef Visualization using Object Detection

    @Override
    public void periodic() {
        List<EstimatedRobotPose> newPoses=vision.getNewPoses();
        for(EstimatedRobotPose newPose:newPoses){
            swerve.addVisionMeasurement(newPose.estimatedPose.toPose2d(), newPose.timestampSeconds);
        }

        vision.processNewObjects(this.getPose());
        field.setRobotPose(this.getPose());
        vision.updateSim(this.getPose());
    }

    // CTRE Generated SysId Routines

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    public final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(new SwerveRequest.SysIdSwerveTranslation().withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    public final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(new SwerveRequest.SysIdSwerveSteerGains().withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    public final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(new SwerveRequest.SysIdSwerveRotation().withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine routine = m_sysIdRoutineTranslation;

    public void setRoutine(SysIdRoutine routine) {
        this.routine = routine;
    }

    public SysIdRoutine getRoutine() {
        return routine;
    }

    // CTRE Telemtry Simulation

    private static final double kSimLoopPeriod = 0.005;
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            swerve.updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public void registerTelemetry(Consumer<SwerveDriveState> telemetryFunction) {
        swerve.registerTelemetry(telemetryFunction);
    }

    public void log(){
        // TODO add logs
    }
}
