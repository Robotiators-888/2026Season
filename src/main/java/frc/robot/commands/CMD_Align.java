// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_PhotonVision;

public class CMD_Align extends RunCommand {
  private final SUB_PhotonVision photonVision;
  private final SUB_Drivetrain drivetrain;
  private Integer targetId;

  private final boolean isLeftAlign;
  private List<Integer> targetTagSet;
  private Pose2d targetPose = new Pose2d();

  HashMap<Integer, Translation2d> redLeft = new HashMap<>();
  HashMap<Integer, Translation2d> redRight = new HashMap<>();
  HashMap<Integer, Translation2d> blueLeft = new HashMap<>();
  HashMap<Integer, Translation2d> blueRight = new HashMap<>();

  private final PIDController xController = new PIDController(0.3, 0, 0.03);
  private final PIDController yController = new PIDController(0.3, 0, 0.03);
  private final PIDController robotAngleController = new PIDController(0.7, 0, 0.05);

  // private final double xMagnitude = Constants.Drivetrain.kXShiftMagnitude;
  // private final double yMagnitude = Constants.Drivetrain.kYShiftMagnitude;

  public CMD_Align(SUB_Drivetrain drivetrain, SUB_PhotonVision photonVision,
      boolean isLeftAlign) {
    super(() -> {
    }, drivetrain);

    this.drivetrain = drivetrain;
    this.photonVision = photonVision;
    this.isLeftAlign = isLeftAlign;
    robotAngleController.enableContinuousInput(-Math.PI, Math.PI);
    redRight.put(7, new Translation2d(14.341348, 4.2116375));
    redLeft.put(7, new Translation2d(14.341348, 3.8401625));
    redRight.put(8, new Translation2d(13.539017606564588, 5.228798303296214));
    redLeft.put(8, new Translation2d(13.860724393435412, 5.043060803296214));
    redRight.put(9, new Translation2d(12.257079606564588, 5.043060803296214));
    redLeft.put(9, new Translation2d(12.578786393435411, 5.228798303296214));
    redRight.put(10, new Translation2d(11.776455999999998, 3.8401625));
    redLeft.put(10, new Translation2d(11.776455999999998, 4.2116375));
    redRight.put(11, new Translation2d(12.578786393435411, 2.8230016967037854));
    redLeft.put(11, new Translation2d(12.257079606564588, 3.0087391967037855));
    redRight.put(6, new Translation2d(13.860724393435412, 3.0087391967037855));
    redLeft.put(6, new Translation2d(13.539017606564588, 2.8230016967037854));
    blueRight.put(21, new Translation2d(5.771896, 4.2116375));
    blueLeft.put(21, new Translation2d(5.771896, 3.8401625));
    blueRight.put(20, new Translation2d(4.969311606564587, 5.228798303296214));
    blueLeft.put(20, new Translation2d(5.2910183934354125, 5.043060803296214));
    blueRight.put(19, new Translation2d(3.687627606564587, 5.043060803296214));
    blueLeft.put(19, new Translation2d(4.009334393435411, 5.228798303296214));
    blueRight.put(18, new Translation2d(3.20675, 3.8401625));
    blueLeft.put(18, new Translation2d(3.20675, 4.2116375));
    blueRight.put(17, new Translation2d(4.00933439343541, 2.8230016967037854));
    blueLeft.put(17, new Translation2d(3.6876276065645865, 3.0087391967037855));
    blueRight.put(22, new Translation2d(5.2910183934354125, 3.0087391967037855));
    blueLeft.put(22, new Translation2d(4.969311606564588, 2.8230016967037854));
    
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    xController.setTolerance(Units.inchesToMeters(1.25));
    yController.setTolerance(Units.inchesToMeters(1.25));
    robotAngleController.setTolerance(Units.degreesToRadians(5.0));

    Pose2d tagPose = new Pose2d();
    Integer targetId = 7;


    List<Integer> targetTagSet;
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    HashMap<Integer, Translation2d> selectedMap;
    if (alliance.isPresent()) {
      targetTagSet =
          alliance.get() == DriverStation.Alliance.Red ? Arrays.asList(7, 8, 9, 10, 11, 6)
              : Arrays.asList(21, 20, 19, 18, 17, 22);

      if (isLeftAlign) {
        if (alliance.get() == DriverStation.Alliance.Red) {
          selectedMap = redLeft;
        } else {
          selectedMap = blueLeft;
        }
      } else {
        if (alliance.get() == DriverStation.Alliance.Red) {
          selectedMap = redRight;
        } else {
          selectedMap = blueRight;
        }
      }
    } else {
      return;
    }


    double minDistance = Double.MAX_VALUE;
    for (int tag : targetTagSet) {
      Pose2d pose = photonVision.at_field.getTagPose(tag).orElse(new Pose3d()).toPose2d();
      Translation2d translate = pose.minus(drivetrain.getPose()).getTranslation();
      double distance = translate.getNorm();

      if (distance < minDistance) {
        tagPose = pose;
        targetId = tag;
        minDistance = distance;
      }
    }

    Pose2d tPose = (DriverStation.getAlliance().equals(Optional.of(Alliance.Red)))
      ? photonVision.at_field.getTagPose(25).orElse(new Pose3d()).toPose2d()
      : photonVision.at_field.getTagPose(9).orElse(new Pose3d()).toPose2d();

    Pose2d currentPose = drivetrain.getPose();
    targetPose = new Pose2d(currentPose.getX(), currentPose.getY(),
        tPose.getRotation().plus(Rotation2d.fromRadians(Math.PI)));
    robotAngleController.reset();
    xController.reset();
    yController.reset();
  }

  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getPose();

    drivetrain.publisher1.set(targetPose);


    
    double omegaSpeed = robotAngleController.calculate(
        MathUtil.angleModulus(currentPose.getRotation().getRadians()),
        MathUtil.angleModulus(targetPose.getRotation().getRadians()));

    drivetrain.drive(0, 0, omegaSpeed, true, true);
    SmartDashboard.putNumber("X Error", currentPose.getX() - targetPose.getX());
    SmartDashboard.putNumber("Y Error", currentPose.getY() - targetPose.getY());
  }

  @Override
  public void end(boolean interrupted) {
    // No specific actions on end
  }

  @Override
  public boolean isFinished() {
    boolean atSetpoint =
        xController.atSetpoint() && yController.atSetpoint() && robotAngleController.atSetpoint();
    SmartDashboard.putBoolean("AlignCommandComplete!", atSetpoint);
    return atSetpoint;
  }
}