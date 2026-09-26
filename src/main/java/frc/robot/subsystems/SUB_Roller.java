package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Alert;
import com.ctre.phoenix6.controls.CoastOut;

public class SUB_Roller extends SubsystemBase {
    /** Subsystem hardware components */
    private TalonFX roller;
    private final CoastOut coastRequest = new CoastOut();
    private final VelocityTorqueCurrentFOC velocityRequest =
            new VelocityTorqueCurrentFOC(0).withSlot(0);
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0).withEnableFOC(true);
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
        talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonConfig.CurrentLimits.SupplyCurrentLimit = 50;
        talonConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
        talonConfig.CurrentLimits.SupplyCurrentLowerTime = 1.0;        
        talonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        talonConfig.Slot0.withKS(5.0)
            .withKV(0.0)
            .withKA(0.0)
            .withKP(5.0)
            .withKI(0)
            .withKD(0);
        roller.getConfigurator().apply(talonConfig);


    }



    /** @param speed Stop */
    public void stop() {
        roller.setControl(coastRequest);
    }

    public void setRPM(int rpm) {
        roller.setControl(velocityRequest.withVelocity(RPM.of(rpm)));
    }
    
    /** @return Current velocity of the roller in RPM */
    public double rollerRPM(){
        return roller.getVelocity().getValue().in(RPM);
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
