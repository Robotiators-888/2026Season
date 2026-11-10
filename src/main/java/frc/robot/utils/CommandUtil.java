package frc.robot.utils;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.commands.CMD_AimBotAuto;

import frc.robot.subsystems.SUB_Index;
import frc.robot.subsystems.SUB_PhotonVision;
import frc.robot.subsystems.SUB_Shooter;
import frc.robot.subsystems.SUB_Arm;
import frc.robot.subsystems.SUB_Roller;

public class CommandUtil {
    private CommandSwerveDrivetrain drivetrain;
    private SUB_Arm arm;
    private SUB_Roller roller;
    private SUB_Index index;
    private SUB_PhotonVision photonVision;
    private SUB_Shooter shooter;
    public CommandUtil (CommandSwerveDrivetrain drivetrain, SUB_Arm arm, SUB_Roller roller, SUB_Index index, SUB_PhotonVision photonVision, SUB_Shooter shooter){
        this.drivetrain = drivetrain;
        this.arm = arm;
        this.roller = roller;
        this.index = index;
        this.photonVision = photonVision;
        this.shooter = shooter;
    }
    public void registerAllNamedCommands(){
    }
    private Command getShakeyCommand () {
                
                Command c = new ParallelCommandGroup(
                        new RunCommand(()->roller.setRPM(1880), roller),
                        new SequentialCommandGroup(
                                new RunCommand(()->arm.setArm(.15), arm).withTimeout(.4),
                                new RunCommand(()->arm.setArm(-.13), arm).withTimeout(.4)
                        ).repeatedly()
                );
                return c;
        }
}
