package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import java.util.function.Supplier;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GroundIntake;

public class SUB_GroundIntake extends SubsystemBase{
    private static SUB_GroundIntake INSTANCE = null;
    private static SparkMaxConfig config = new SparkMaxConfig();
    private SparkMax groundIntake = new SparkMax(GroundIntake.kGroundIntakeCanID, MotorType.kBrushless);
    
    public SUB_GroundIntake() {
      config.smartCurrentLimit(45);
      config.inverted(false);
      config.voltageCompensation(12);
      groundIntake.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    // Allows for running the ground intake and can be used to intake and score the coral by using negative values
    public void setGroundIntake (double percent) {
        groundIntake.set(percent);
    }

    public void groundIntakeDetection(Supplier<Boolean> shouldIntake, Supplier<Boolean> shouldRun){
      if (shouldIntake.get()) {
        setGroundIntake(GroundIntake.kGroundIntakeSpeed);
        return;
      }
      if (shouldRun.get()){
        setGroundIntake(-.2);
        return;
      }
      else{
        setGroundIntake(0);
        return;
      }
    }

    public static SUB_GroundIntake getInstance() {
        if (INSTANCE == null) {
          INSTANCE = new SUB_GroundIntake();
        }
        return INSTANCE;
      }

}
