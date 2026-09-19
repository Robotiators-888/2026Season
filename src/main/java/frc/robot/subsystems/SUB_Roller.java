package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Alert;

public class SUB_Roller extends SubsystemBase {
    /** Subsystem hardware components */
    private TalonFX roller;
    private final VelocityTorqueCurrentFOC velocityRequest = new VelocityTorqueCurrentFOC(0);
    private final TorqueCurrentFOC currentRequest = new TorqueCurrentFOC(0);
    private static SUB_Roller INSTANCE = null;

    /**
     * @return Single instance of the SUB_Roller subsystem
     */
    public static SUB_Roller getInstance (){
        if (INSTANCE == null) {
            INSTANCE = new SUB_Roller();
        } 
        return INSTANCE;
    }

    private SUB_Roller () {
        // Defines motor with ID from Constants
        roller = new TalonFX(Constants.Roller.kINTAKE_MOTOR_CANID);
        configureMotors();
    }

    private void configureMotors(){
        // Configure TalonFX motor controller with current limits and inversion
        TalonFXConfiguration talonConfig = new TalonFXConfiguration();

        // Supply Current Limit: 40A to prevent brownouts
        talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        talonConfig.CurrentLimits.SupplyCurrentLowerLimit = 20.0;
        talonConfig.CurrentLimits.SupplyCurrentLowerTime = 2.2;

        // Stator Current Limit: 60A to allow high acceleration torque without jams
        talonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        talonConfig.CurrentLimits.StatorCurrentLimit = 60.0;

        // Torque Current Limits
        talonConfig.TorqueCurrent.PeakForwardTorqueCurrent = 60.0; // Amperes
        talonConfig.TorqueCurrent.PeakReverseTorqueCurrent = -60.0; // Amperes

        // PID Configuration for TorqueCurrentFOC
        talonConfig.Slot0.kS = Constants.Roller.kROLLER_FLYWHEEL_kS;
        talonConfig.Slot0.kV = Constants.Roller.kROLLER_FLYWHEEL_kV;
        talonConfig.Slot0.kA = Constants.Roller.kROLLER_FLYWHEEL_kA;
        talonConfig.Slot0.kP = Constants.Roller.kROLLER_FLYWHEEL_kP;
        talonConfig.Slot0.kI = Constants.Roller.kROLLER_FLYWHEEL_kI;
        talonConfig.Slot0.kD = Constants.Roller.kROLLER_FLYWHEEL_kD;
        talonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        roller.getConfigurator().apply(talonConfig);
    }


    /** @param rpm Target velocity for the roller motor */
    public void setRPM(double rpm){
        roller.setControl(velocityRequest.withVelocity(rpm / 60.0));
    }

    /** @param amps Target torque current for the roller motor */
    public void setCurrent(double amps){
        roller.setControl(currentRequest.withOutput(amps));
    }

    /** @return Current velocity of the roller in RPM */
    public double rollerRPM(){
        return roller.getVelocity().getValue().baseUnitMagnitude();
    }

    @Override
    public void periodic() {
        // Telemetry logging for dashboard
        SmartDashboard.putNumber("Roller/RollerRPM", rollerRPM());
        SmartDashboard.putNumber("Roller/Roller Encoder Pos", roller.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Roller/Roller Stator Current", roller.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Roller/Roller Supply Current", roller.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Roller/Roller Torque Current", roller.getTorqueCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Roller/Roller Supply Voltage", roller.getSupplyVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Roller/Roller Motor Voltage", roller.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Roller/Roller Device Temp", roller.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putNumber("Roller/Roller Processor Temp", roller.getProcessorTemp().getValueAsDouble());

        Alert.alertKraken(roller);
    }

    
}
