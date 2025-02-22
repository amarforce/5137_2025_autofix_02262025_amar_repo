package frc.robot;

import java.io.File;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.elastic.*;
import frc.robot.gamepieces.Gamepieces;
import frc.robot.other.AutoFactory;
import frc.robot.other.CageChoice;
import frc.robot.other.RobotUtils;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.constants.HangConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.constants.SwerveSystemConstants;

@SuppressWarnings("unused")
public class RobotContainer {
	// Controllers
	private CommandPS5Controller driver;
	private CommandPS5Controller operator;
	private CommandPS5Controller sysIdTest;

	// Subsystems and their commands
	private Vision vision;
	private Swerve swerve;
	private SwerveCommands swerveCommands;
	private SwerveSystem swerveSystem;
	private SwerveSystemCommands swerveSystemCommands;

	private Elevator elevator;
	private ElevatorCommands elevatorCommands;

	private Arm arm;
	private ArmCommands armCommands;

	private Wrist wrist;
	private WristCommands wristCommands;

	private Intake intake;
	private IntakeCommands intakeCommands;

	private Hang hang;
	private HangCommands hangCommands;

	private LED led;

	private MultiCommands multiCommands;

	// Additional components
	private Reef reef;
	private ReefScoring reefScoring;
	private CageChoice cageChoice;
	private AutoFactory autoFactory;
	private Gamepieces gamepieces;

	/**
	 * Constructor for RobotContainer.
	 * Initializes all subsystems, commands, and binds controls.
	 */
	public RobotContainer() {
		// Start data logging
		DataLogManager.start();
		DriverStation.startDataLog(DataLogManager.getLog());

		try {
			initControllers();
			
			// Configure emergency stop - this should always be available
			driver.touchpad().onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));
			
			initReef();
			initGamepieces();
			
			// Initialize subsystems
			//initVision();
			initSwerve();
			initElevator();
			initArm();
			initWrist();
			initIntake();
			initHang();
			//initLED();
			
			// Initialize combined systems and commands
			initSwerveSystem();
			//initMultiCommands();
			//initAdditionalComponents();

			// Configure SysId bindings for elevator
			//configureSysIdBindings(elevatorCommands);

			//arm.resetPos();
			//wrist.resetPos();
			//elevator.resetPos();
		} catch (Exception e) {
			DataLogManager.log("Error while initializing: " + RobotUtils.getError(e));
		}
	}

	private void initControllers() {
		driver = new CommandPS5Controller(0);
		operator = new CommandPS5Controller(1);
		//sysIdTest = new CommandPS5Controller(2);
	}

	private void initReef() {
		reef = new Reef();
		reefScoring = new ReefScoring(reef);
		SmartDashboard.putData("reef", reef);
		SmartDashboard.putData("reefScoring", reefScoring);
	}

	private void initGamepieces() {
		gamepieces=new Gamepieces();
	}

	private void initVision() {
		vision = new Vision(reef);
	}

	private void initSwerve() {
		swerve = new Swerve(new File(Filesystem.getDeployDirectory(), "swerve.json"), vision);
		swerveCommands = new SwerveCommands(swerve);

		// Configure swerve bindings
		swerve.setDefaultCommand(swerveCommands.drive(
			() -> -driver.getLeftY(), 
			() -> -driver.getLeftX(), 
			() -> driver.getRightX(),
			() -> true)
		);

		//driver.cross().whileTrue(swerveCommands.lock());
		driver.options().onTrue(swerveCommands.resetGyro());
	}

	private void initElevator() {
		elevator = new Elevator();
		elevatorCommands = new ElevatorCommands(elevator);

		// Configure elevator bindings
		elevator.setDefaultCommand(elevatorCommands.changeGoal(() -> -MathUtil.applyDeadband(operator.getLeftY(),0.1) / 50));
	}

	private void initArm() {
		arm = new Arm();
		armCommands = new ArmCommands(arm);

		// Configure arm bindings
		arm.setDefaultCommand(armCommands.changeGoal(() -> -operator.getRightX() / 50));
	}

	private void initWrist() {
		wrist = new Wrist(arm);
		wristCommands = new WristCommands(wrist);

		// Configure wrist bindings
		operator.L1().whileTrue(wristCommands.changeGoal(()->0.1));
		operator.R1().whileTrue(wristCommands.changeGoal(()->-0.1));
	}

	private void initIntake() {
		intake = new Intake();
		intakeCommands = new IntakeCommands(intake);

		// Configure intake bindings
		operator.L2().or(driver.L2())
			.onTrue(intakeCommands.setSpeed(()->-IntakeConstants.intakeSpeed))
			.onFalse(intakeCommands.stop());

		operator.R2().or(driver.R2())
			.onTrue(intakeCommands.setSpeed(()->IntakeConstants.intakeSpeed))
			.onFalse(intakeCommands.stop());
	}

	private void initHang() {
		hang = new Hang();
		hangCommands = new HangCommands(hang);
		hang.setDefaultCommand(hangCommands.setSpeed(()->{
			return MathUtil.applyDeadband(operator.getLeftX(),0.1)*HangConstants.hangSpeed;
		}));
	}

	private void initLED() {
		led = new LED();
	}

	private void initSwerveSystem() {
		swerveSystem = new SwerveSystem(arm, elevator, wrist, swerve, gamepieces);
		swerveSystemCommands = new SwerveSystemCommands(swerveSystem);

		// Configure swerve system bindings
		//driver.triangle().onTrue(swerveSystemCommands.moveToSource());
		//driver.circle().onTrue(swerveSystemCommands.moveToProcessor());

		driver.triangle().onTrue(swerveSystemCommands.moveToLevel(3));
		driver.square().onTrue(swerveSystemCommands.moveToLevel(2));
		driver.circle().onTrue(swerveSystemCommands.moveToLevel(1));
		driver.cross().onTrue(swerveSystemCommands.moveToLevel(0));
		driver.povDown().onTrue(swerveSystemCommands.moveToDefault());

		driver.L1().onTrue(swerveSystemCommands.moveToState(()->SwerveSystemConstants.getGroundIntake()));
		driver.R1().onTrue(swerveSystemCommands.moveToState(()->SwerveSystemConstants.getSourceStates()[0]));

		// operator.povUp().onTrue(swerveSystemCommands.moveToGround(()->new Pose2d()));
		// operator.povDown().onTrue(swerveSystemCommands.moveToState(()->SwerveSystemConstants.getSourceStates()[0]));
		// operator.cross().onTrue(swerveSystemCommands.moveToBranch(()->0,()->0));
		// operator.triangle().onTrue(swerveSystemCommands.moveToBranch(()->1,()->0));
		// operator.square().onTrue(swerveSystemCommands.moveToBranch(()->2,()->0));
		// operator.circle().onTrue(swerveSystemCommands.moveToBranch(()->3,()->0));
	}

	private void initMultiCommands() {
		multiCommands = new MultiCommands(swerveSystemCommands, swerveCommands, intakeCommands, hangCommands);
		driver.triangle().onTrue(multiCommands.getCoralFromSource()); // e
		driver.square().onTrue(multiCommands.getAlgae(()->0)); // x
		driver.circle().onTrue(multiCommands.placeCoral(()->2,()->0)); // t
		driver.cross().onTrue(multiCommands.placeAlgae()); // p
		
	}

	private void initAdditionalComponents() {
		cageChoice = new CageChoice();
		autoFactory = new AutoFactory(multiCommands);
	}

	private void configureSysIdBindings(SysIdCommands subsystemCommands) {
		sysIdTest.cross()
			.onTrue(subsystemCommands.sysIdDynamic(Direction.kForward))
			.onFalse(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));

		sysIdTest.circle()
			.onTrue(subsystemCommands.sysIdDynamic(Direction.kReverse))
			.onFalse(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));

		sysIdTest.square()
			.onTrue(subsystemCommands.sysIdQuasistatic(Direction.kForward))
			.onFalse(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));

		sysIdTest.triangle()
			.onTrue(subsystemCommands.sysIdQuasistatic(Direction.kReverse))
			.onFalse(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));
	}

	/**
	 * Returns the autonomous command to be executed.
	 *
	 * @return The autonomous command
	 */
	public Command getAutonomousCommand() {
		if (autoFactory != null) {
			return autoFactory.getAuto();
		}
		return new InstantCommand();
	}
}