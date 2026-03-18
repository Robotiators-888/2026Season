package frc.robot.subsystems;


import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class SUB_IntakeArm extends SubsystemBase{
    public static boolean extended;
    private PIDController controller = new PIDController(.001, 0, 0);
    private SparkMax arm;
    private SparkMax armFollower;
    private boolean stickUp = false;
    private boolean stickDown = false;
    private int periodicCountFault = 0;
    private static SUB_IntakeArm INSTANCE = null;

    public static SUB_IntakeArm getInstance (){
        if (INSTANCE == null) {
                INSTANCE = new SUB_IntakeArm();
        }
        return INSTANCE;
    }

    private SUB_IntakeArm() {
        //defines motors with IDs and what controller
        arm = new SparkMax(Constants.Intake.kARM_MOTOR_CANID, MotorType.kBrushless);
        armFollower = new SparkMax(Constants.Intake.kARM_FOLLOWER_MOTOR_CANID, MotorType.kBrushless);
        configureMotors();
    }

    private void configureMotors(){
        SparkMaxConfig config = new SparkMaxConfig();
        config.encoder.positionConversionFactor(360.0/23); //Converst rotations to degrees, gearbox 23:1
        config.encoder.velocityConversionFactor((360.0 / 23) / 60.0); //Converst from RPM to deg/sec
        config.smartCurrentLimit(35); //sets current limit in amps
        config.inverted(true);
        arm.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        SparkMaxConfig followerConfig = new SparkMaxConfig(); //creates config from follower sparkmax
        followerConfig.follow(arm, true); //follows other arm but inverted
        followerConfig.smartCurrentLimit(35);//sets current limit to 35 amps
        armFollower.configure(followerConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
    }

    public boolean isForwardPressed() {
        return stickUp||Math.abs(arm.getEncoder().getPosition()-Constants.Intake.kINTAKE_ARM_TOP_SETPOINT)<10;
    }

    public boolean isReversePressed() {
        return stickDown||Math.abs(arm.getEncoder().getPosition()-Constants.Intake.kINTAKE_ARM_BOTTOM_SETPOINT)<10;
    }

    public void periodic() {

        SmartDashboard.putBoolean("Intake/Arm Forward Limit", isForwardPressed()); //Returns if the intake arm is down
        SmartDashboard.putBoolean("Intake/Arm Reverse Limit", isReversePressed()); //Returns if the intake arm is up

        SmartDashboard.putNumber("Intake/Arm Encoder Pos", arm.getEncoder().getPosition()); //Returns angle of intake arm

        SmartDashboard.putNumber("Intake/Arm Output Current", arm.getOutputCurrent()); //Returns how much current is going into the intake arm motors

        SmartDashboard.putNumber("Intake/Arm Bus Voltage", arm.getBusVoltage());

        SmartDashboard.putBoolean("Intake/Stick Up", stickUp);  
        SmartDashboard.putBoolean("Intake/Stick Down", stickDown);
    }

    public void set(double speed) {
        arm.set(speed);
        //**Only use if you know what you are doing */
    }
    //sets arm to speed put in method
    public void setArm(double speed) {
        if (arm.getOutputCurrent() > Constants.Intake.kIntake_ARM_FAULT_AMPS) {
            periodicCountFault+=2;
        }
        if (periodicCountFault > 12) {
            //if speed is going up when faults are high the arm is up
            if (speed > 0) {
                stickUp = true;
                stickDown = false;
                arm.getEncoder().setPosition(Constants.Intake.kINTAKE_ARM_TOP_SETPOINT);
            //If speed is negative when faults are high the arm is down
            } else if (speed < 0) {
                stickUp = false;
                stickDown = true;
                arm.getEncoder().setPosition(Constants.Intake.kINTAKE_ARM_BOTTOM_SETPOINT);
            }
            speed = 0;
        }
        //if arm is up and it moves down then it is set to no longer being up
        if (stickUp) {
            if (speed < 0) {
                stickUp = false;
            } else {
                speed = 0;
            }
        }
        //if arm is down and it moves up then it is set to being no longer down.
        if (stickDown) {
            if (speed > 0) {
                stickDown = false;
            } else {
                speed = 0;
            }
        }
        //sets speed of arm motor
        arm.set(speed);
        
    }

    //Retuns if arm is extended
    public boolean isExtended() {
        return extended;
    }

    //Makes arm go down based on PID
    public void intakeArmDown() {
        setArm(controller.calculate(arm.getEncoder().getPosition(), Constants.Intake.kINTAKE_ARM_BOTTOM_SETPOINT)); 
    }

    //Makes arm go up based on PID
    public void intakeArmUp() {
        setArm(controller.calculate(arm.getEncoder().getPosition(), Constants.Intake.kINTAKE_ARM_TOP_SETPOINT)); 
    }

    //Returns if arm is down
    public boolean isArmDownReached() {
        return Math.abs(arm.getEncoder().getPosition() - Constants.Intake.kINTAKE_ARM_BOTTOM_SETPOINT) < 3.0;
    }

    //Returns if arm is up
    public boolean isArmUpReached() {
        return Math.abs(arm.getEncoder().getPosition() - Constants.Intake.kINTAKE_ARM_TOP_SETPOINT) < 3.0;
    }
}
