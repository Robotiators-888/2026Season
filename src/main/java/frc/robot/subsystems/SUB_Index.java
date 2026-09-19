package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Alert;

public class SUB_Index extends SubsystemBase {
    /** Subsystem hardware components */
    private SparkMax index;
    private SparkMax meteringWheel;
    private SparkClosedLoopController indexController;
    private SparkClosedLoopController meteringController;
    
    private double targetMeteringRPM = 0;

    private static SUB_Index INSTANCE = null;

    /** @return Single instance of the SUB_Index subsystem */
    public static SUB_Index getInstance () {
        if (INSTANCE == null) {
            INSTANCE = new SUB_Index();
        }
        return INSTANCE;
    }
    
    private SUB_Index () {
        // Defines motors for indexing and metering
        index = new SparkMax(Constants.Index.KINDEX_MOTOR_CANID, MotorType.kBrushless);
        meteringWheel = new SparkMax(Constants.Index.kMETERING_WHEEL_CANID, MotorType.kBrushless);
        
        // Configure main indexer motor for Torque/Current control to ensure it pushes balls with constant force
        SparkMaxConfig indexConfig = new SparkMaxConfig();
        // Smart Current Limit: 40 Amps. High enough to push jammed balls but low enough to protect the NEO.
        indexConfig.smartCurrentLimit(40);
        indexConfig.inverted(true);
        index.configure(indexConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        indexController = index.getClosedLoopController();
        
        // Configure high-speed metering wheel with Feedforward-heavy tuning for stable speed holding
        SparkMaxConfig meteringConfig = new SparkMaxConfig();
        // Smart Current Limit: 40 Amps.
        meteringConfig.smartCurrentLimit(40);
        
        // PID/FF Config for RPM control.
        // Using very low P to prevent reactive current spikes. Most of the effort is handled by kFF.
        double kP = 0.00001; // Tiny proportional gain just to correct steady-state errors
        double kI = 0.0;
        double kD = 0.0; 
        double kFF = 0.0021; // FF based on NEO nominal free speed at 12V (~5676 RPM => 12V/5676 = ~0.0021)
        
        meteringConfig.closedLoop.pid(kP, kI, kD);
        meteringConfig.closedLoop.velocityFF(kFF);
        meteringConfig.encoder.uvwMeasurementPeriod(8);
        meteringConfig.encoder.uvwAverageDepth(2);
        meteringWheel.configure(meteringConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        
        meteringController = meteringWheel.getClosedLoopController();
    }
    
    /** @param speed Target percent output for indexing [-1.0, 1.0]. Converted to Current Request. */
    public void set(double speed){
        // Map -1.0 to 1.0 speed to -40A to 40A current request to push with constant torque
        indexController.setReference(speed * 40.0, ControlType.kCurrent);
    }
    
    /** @return Current velocity of the indexer in RPM */
    public double indexRPM(){
        return index.getEncoder().getVelocity();
    }
    
    /** @return Current velocity of the metering wheel in RPM */
    public double intakeMeteringRPM(){
        return meteringWheel.getEncoder().getVelocity(); 
    }

    /** @param speed Target percent output for metering [-1.0, 1.0] */
    public void setMeteringSpeed(double speed) {
        targetMeteringRPM = -1;
        meteringWheel.set(speed);
    }

    /** @param volts Target voltage for the index motor */
    public void setVolts(double volts) {
        // Fallback for voltage control mapping to equivalent torque limit if needed
        indexController.setReference((volts / 12.0) * 40.0, ControlType.kCurrent);
    }

    /** @param volts Target voltage for the metering motor */
    public void setMeteringVolts(double volts) {
        targetMeteringRPM = -1;
        meteringWheel.setVoltage(volts);
    }

    /** @param wheelRPM Target velocity for the metering wheel in RPM */
    public void setMeteringRPM(double wheelRPM) {
        targetMeteringRPM = wheelRPM;
        meteringController.setReference(wheelRPM, ControlType.kVelocity);
    }
    
    /** Stops the metering wheel entirely */
    public void stopMetering() {
        targetMeteringRPM = 0;
        meteringWheel.set(0);
    }

    @Override
    public void periodic() {
        // Telemetry logging for dashboard
        SmartDashboard.putNumber("Index/Index RPM", indexRPM());
        SmartDashboard.putNumber("Index/Index Output Current", index.getOutputCurrent());
        SmartDashboard.putNumber("Index/Metering Output Current", meteringWheel.getOutputCurrent());
        SmartDashboard.putNumber("Index/Metering RPM", intakeMeteringRPM());
        SmartDashboard.putNumber("Index/Metering Target RPM", targetMeteringRPM);
        SmartDashboard.putNumber("Index/Metering Bus Voltage", meteringWheel.getBusVoltage());
        SmartDashboard.putNumber("Index/Index Bus Voltage", index.getBusVoltage());
        SmartDashboard.putNumber("Index/Metering Encoder Pos", meteringWheel.getEncoder().getPosition());
        SmartDashboard.putNumber("Index/Index Encoder Pos", index.getEncoder().getPosition());
        SmartDashboard.putNumber("Index/Metering Motor Temp", meteringWheel.getMotorTemperature());
        SmartDashboard.putNumber("Index/Index Motor Temp", index.getMotorTemperature());

        Alert.alertNeoFaults(index);
        Alert.alertNeoWarnings(index);
        Alert.alertNeoFaults(meteringWheel);
        Alert.alertNeoWarnings(meteringWheel);
    }
}
