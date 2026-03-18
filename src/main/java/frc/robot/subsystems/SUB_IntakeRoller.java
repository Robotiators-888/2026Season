package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SUB_IntakeRoller extends SubsystemBase {
    // Initiliazes values and objects used in subsystem

    private TalonFX intake;
    private int periodicCountFault = 0;
    private static SUB_IntakeRoller INSTANCE = null;
    private boolean intakeArmAndRollersUntil = false;
    public static SUB_IntakeRoller getInstance (){
        if (INSTANCE == null) {
            INSTANCE = new SUB_IntakeRoller();
        } 
        return INSTANCE;
    }

    private SUB_IntakeRoller () {
        //Defines motors with IDs and what controller
        intake = new TalonFX(Constants.Intake.kINTAKE_MOTOR_CANID);

        configureMotors();
    }

    private void configureMotors(){
        //Creates config for motors
        
        TalonFXConfiguration talonConfig = new TalonFXConfiguration(); //Creates new TalonFX Config
        talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true; //enables supply current limit which is how much goes to motor controller
        talonConfig.CurrentLimits.SupplyCurrentLimit = 40; //Sets supply current limit in amps
        talonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // Makes it so positive values make the motor spin CC
        intake.getConfigurator().apply(talonConfig); //Applies Config to the intake roller
    }



    //Sets voltage of intake roller
    public void setVolts(double speed){
        intake.setVoltage(speed);
    }

    //Sets speed of intake roller
    public void set(double speed){
        intake.set(speed);
    }
    //Returns RPM of intake roller
    public double intakeRPM(){
        return intake.getVelocity().getValue().baseUnitMagnitude();
    }

    // Logs everything every periodic
    public void periodic() {
        SmartDashboard.putNumber("Intake/IntakeRPM", intakeRPM()); //puts Intake motor RPM into Smart Dashboard


        

        SmartDashboard.putNumber("Intake/Intake Encoder Pos", intake.getPosition().getValueAsDouble());



        SmartDashboard.putNumber("Intake/Intake Stator Current", intake.getStatorCurrent().getValueAsDouble()); //Return stator current of intake roller
        SmartDashboard.putNumber("Intake/Intake Supply Current", intake.getSupplyCurrent().getValueAsDouble());

        SmartDashboard.putNumber("Intake/Intake Supply Voltage", intake.getSupplyVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Intake Motor Voltage", intake.getMotorVoltage().getValueAsDouble());


        if (periodicCountFault > 0) {
            periodicCountFault--;
        }
    }

    

    //Puts intake down and then activates rollers
    public void intakeArmAndRollers() {
        setVolts(Constants.Intake.kINTAKE_MOTOR_VOLTAGE);
        if (!isArmDownReached() && !intakeArmAndRollersUntil) {
            // intakeArmDown();
            setArm(-.50);
        } else {
            arm.set(-.025);
            intakeArmAndRollersUntil = true;
        }
    }
}
