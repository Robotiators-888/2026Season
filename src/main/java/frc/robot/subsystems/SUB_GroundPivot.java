package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.GroundIntake;
import frc.robot.Constants.GroundPivot;

public class SUB_GroundPivot extends SubsystemBase {
  private static SUB_GroundPivot INSTANCE = null;

  // I could use public SUB_GroundPivot () {} to set configs

  private SparkMax groundPivot = new SparkMax(GroundPivot.kGroundPivotCanID, MotorType.kBrushless);
  private SparkMaxConfig config = new SparkMaxConfig();
  private RelativeEncoder relativeEncoder = groundPivot.getEncoder();
  // private PIDController voltagePID = new PIDController(3, 0, 0); // TODO: Change
  public double activesetpoint = Constants.GroundPivot.kStowPos;
  private boolean runningAutomatic = true;

  public SUB_GroundPivot() {
    config.smartCurrentLimit(25);
    config.idleMode(IdleMode.kBrake);
    config.inverted(false);
    config.voltageCompensation(12);
    // voltagePID.setTolerance(5);
    groundPivot.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // Allows for changing the pivot angle of the ground intake and is needed to be able to pick up
  // coral and score also may not be needed
  public void setGroundPivot(double percent) {
    groundPivot.set(percent);
  }

  public void runGroundPivotManualVoltage(double volts) {
    groundPivot.setVoltage(volts);
  }

  public void changeSetpoint(double setpoint) {
    activesetpoint = setpoint;
    runningAutomatic = true;
  }

  // public void drivePivotPID() {
  // double outputVoltage = MathUtil.clamp(voltagePID.calculate(relativeEncoder.getPosition(),
  // activesetpoint), -6, 6);
  // runGroundPivotManualVoltage(outputVoltage);
  // }

  public void drivePivotConditionally(double manualAxis) {
    if (Math.abs(manualAxis) > GroundPivot.kPivotDeadband) {
      runningAutomatic = false;
      runGroundPivotManualVoltage(
          12 * Math.copySign(Math.pow(MathUtil.applyDeadband(manualAxis, GroundPivot.kPivotDeadband), 2), manualAxis)
              * GroundPivot.kPivotSpeed);
      return;
    }
    if (runningAutomatic) {
      driveBangBang();
      return;
    }
    runGroundPivotManualVoltage(0);
    return;
  }

  public void driveBangBang() {
    if (Math.abs(relativeEncoder.getPosition() - activesetpoint) < 4) {
      runGroundPivotManualVoltage(0);
      return;
    }
    if (relativeEncoder.getPosition() > activesetpoint) {

      if (relativeEncoder.getPosition() - activesetpoint > 25) {
        runGroundPivotManualVoltage(-12);
        return;
      }
      if (relativeEncoder.getPosition() - activesetpoint < 25) {
        runGroundPivotManualVoltage(-5);
        return;
      }
    }
    if (relativeEncoder.getPosition() < activesetpoint) {

      if (activesetpoint - relativeEncoder.getPosition() > 25) {
        runGroundPivotManualVoltage(12);
        return;
      }
      if (activesetpoint - relativeEncoder.getPosition() < 25) {
        runGroundPivotManualVoltage(5);
        return;
      }
    }

  }

  public void zeroEncoder() {
    relativeEncoder.setPosition(0);
  }

  public boolean nearIntakeSetpoint() {
    return relativeEncoder.getPosition() > GroundPivot.kIntakePos - GroundPivot.kIntakeThreshold;
  }

  public boolean shouldHold() {
    return relativeEncoder.getPosition() > GroundPivot.kScorePos - 5;
  }

  public static SUB_GroundPivot getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new SUB_GroundPivot();
    }
    return INSTANCE;
  }

  public void periodic() {
    SmartDashboard.putNumber("Ground Pivot Relative Encoder", relativeEncoder.getPosition());
    // SmartDashboard.putNumber("OutputVoltage",
    // MathUtil.clamp(voltagePID.calculate(relativeEncoder.getPosition(), activesetpoint), -6, 6));
  }

}
