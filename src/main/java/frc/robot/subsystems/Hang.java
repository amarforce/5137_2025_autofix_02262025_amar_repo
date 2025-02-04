package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.HangConstants;
import frc.robot.other.RobotUtils;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.Compressor;

/**
 * The Hang subsystem controls the pneumatics for the robot's hanging mechanism.
 * This includes the clamp and climb solenoids, as well as the compressor that
 * provides the necessary air pressure.
 */
public class Hang extends SubsystemBase {
    
    // Solenoids for controlling the clamp and climb mechanisms
    private final Solenoid clampSolenoid;
    private final Solenoid climbSolenoid;

    // Compressor to maintain air pressure within the specified range
    private final Compressor compressor;

    private StringLogEntry log;

    /**
     * Constructs a new Hang subsystem.
     */
    public Hang(StringLogEntry log) {
        // Initialize the solenoids with the appropriate channels from HangConstants
        clampSolenoid = new Solenoid(PneumaticsModuleType.REVPH, HangConstants.clampSolenoid);
        climbSolenoid = new Solenoid(PneumaticsModuleType.REVPH, HangConstants.climbSolenoid);

        // Initialize the compressor and enable it to maintain pressure within the specified range
        compressor = new Compressor(PneumaticsModuleType.REVPH);
        compressor.enableAnalog(HangConstants.minPressure, HangConstants.maxPressure);

        this.log=log;
    }
    
    /**
     * Activates the clamp by extending the clamp solenoid.
     */
    public void clampActivate() {
        clampSolenoid.set(true);
    }

    /**
     * Deactivates the clamp by retracting the clamp solenoid.
     */
    public void clampDeactivate() {
        clampSolenoid.set(false);
    }

    /**
     * Checks if the clamp is currently activated.
     *
     * @return true if the clamp is activated, false otherwise.
     */
    public boolean isClampActivated() {
        return clampSolenoid.get();
    }

    /**
     * Extends the climb mechanism by activating the climb solenoid.
     */
    public void climbExtend() {
        climbSolenoid.set(true);
    }

    /**
     * Retracts the climb mechanism by deactivating the climb solenoid.
     */
    public void climbRetract() {
        climbSolenoid.set(false);
    }

    /**
     * Checks if the climb mechanism is currently extended.
     *
     * @return true if the climb mechanism is extended, false otherwise.
     */
    public boolean isClimbExtended() {
        return climbSolenoid.get();
    }

    /**
     * Logs relevant data from the Hang subsystem.
     * This method is currently a placeholder and should be implemented to log specific data.
     */
    private void telemetry(){
        SmartDashboard.putBoolean("hang/climbSolenoidEnabled",climbSolenoid.get());
        SmartDashboard.putBoolean("hang/climbSolenoidShorted",climbSolenoid.isDisabled());
        SmartDashboard.putBoolean("hang/clampSolenoidEnabled",clampSolenoid.get());
        SmartDashboard.putBoolean("hang/clampSolenoidShorted",clampSolenoid.isDisabled());
    }

    @Override
    public void periodic() {
        try{
            telemetry();
        }catch(Exception e){
            log.append("Periodic error: "+RobotUtils.getError(e));
        }
    }
}