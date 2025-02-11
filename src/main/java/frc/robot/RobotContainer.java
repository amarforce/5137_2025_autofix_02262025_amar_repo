package frc.robot;

import java.io.File;
import java.util.function.Supplier;

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
import frc.robot.other.AutoFactory;
import frc.robot.other.CageChoice;
import frc.robot.other.RobotUtils;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.constants.SwerveConstants;

@SuppressWarnings("unused")
public class RobotContainer {
	// Controllers
	private CommandPS5Controller driver;
	private CommandPS5Controller operator;
	private CommandPS5Controller sysIdTest;

	// Subsystems and their commands
	private Vision vision;
	private Swerve swerve;
<<<<<<< HEAD
	private SwerveCommands swerveCommands;
	private SwerveSystem swerveSystem;
	private SwerveSystemCommands swerveSystemCommands;

=======
>>>>>>> origin/F/PID
	private Elevator elevator;
	private ElevatorCommands elevatorCommands;

	private Arm arm;
	private ArmCommands armCommands;

	private Wrist wrist;
	private WristCommands wristCommands;

	private Intake intake;
	private IntakeCommands intakeCommands;

	private Hang hang;
	private HangCommand hangCommand;

	private LED led;

<<<<<<< HEAD
=======
	// Commands for each subsystem
	private SwerveCommands swerveCommands;
	private ElevatorCommands elevatorCommands;
	private ArmCommands armCommands;
	private WristCommands wristCommands;
	private IntakeCommands intakeCommands;
	private HangCommand hangCommand;
	private ArmSystemCommands armSystemCommands;
>>>>>>> origin/F/PID
	private MultiCommands multiCommands;

	// Additional components
	private Reef reef;
	private ReefScoring reefScoring;
	private CageChoice cageChoice;
<<<<<<< HEAD
	private AutoFactory autoFactory;
=======

	// Factory for autonomous commands
	private AutoFactory autoFactory;

	private StringLogEntry log;
>>>>>>> origin/F/PID

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
			
			// Initialize subsystems
			initVision();
			initSwerve();
			initElevator();
			initArm();
			initWrist();
			initIntake();
			initHang();
			initLED();
			
			// Initialize combined systems and commands
			initSwerveSystem();
			initMultiCommands();
			initAdditionalComponents();
		} catch (Exception e) {
			DataLogManager.log("Error while initializing: " + RobotUtils.getError(e));
		}
	}

	private void initControllers() {
		driver = new CommandPS5Controller(0);
		operator = new CommandPS5Controller(1);
		sysIdTest = new CommandPS5Controller(2);
	}

	private void initReef() {
		reef = new Reef();
		reefScoring = new ReefScoring(reef);
		SmartDashboard.putData("reef", reef);
		SmartDashboard.putData("reefScoring", reefScoring);
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

		driver.cross().whileTrue(swerveCommands.lock());
		driver.options().onTrue(swerveCommands.resetGyro());
	}

	private void initElevator() {
		elevator = new Elevator();
		elevatorCommands = new ElevatorCommands(elevator);

		// Configure elevator bindings
		elevator.setDefaultCommand(elevatorCommands.changeGoal(() -> -operator.getLeftY() / 50));

		// Configure SysId bindings for elevator
		configureSysIdBindings(elevatorCommands);
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
		operator.L1().onTrue(wristCommands.changeGoal(() -> Math.toRadians(-5)));
		operator.R1().onTrue(wristCommands.changeGoal(() -> Math.toRadians(5)));
	}

	private void initIntake() {
		intake = new Intake();
		intakeCommands = new IntakeCommands(intake);

		// Configure intake bindings
		operator.L2()
			.onTrue(intakeCommands.outtake())
			.onFalse(intakeCommands.stop());

		operator.R2()
			.onTrue(intakeCommands.intake())
			.onFalse(intakeCommands.stop());
	}

	private void initHang() {
		hang = new Hang();
		hangCommand = new HangCommand(hang);

		// Configure hang bindings
		operator.touchpad().onTrue(hangCommand);
	}

	private void initLED() {
		led = new LED();
	}

	private void initSwerveSystem() {
		swerveSystem = new SwerveSystem(arm, elevator, wrist, swerve);
		swerveSystemCommands = new SwerveSystemCommands(swerveSystem);

		// Configure swerve system bindings
		driver.triangle().onTrue(swerveSystemCommands.moveToSource());
		driver.circle().onTrue(swerveSystemCommands.moveToProcessor());

		operator.triangle().onTrue(swerveSystemCommands.moveToLevel(3));
		operator.circle().onTrue(swerveSystemCommands.moveToLevel(2));
		operator.square().onTrue(swerveSystemCommands.moveToLevel(1));
		operator.cross().onTrue(swerveSystemCommands.moveToLevel(0));
	}

	private void initMultiCommands() {
		multiCommands = new MultiCommands(swerveSystemCommands, swerveCommands, intakeCommands, hangCommand);
	}

	private void initAdditionalComponents() {
		cageChoice = new CageChoice();
		autoFactory = new AutoFactory(multiCommands);
	}

	private void configureSysIdBindings(ElevatorCommands subsystemCommands) {
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