package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
public class DropAuto extends SequentialCommandGroup {
    
    public DropAuto(Swerve s_Swerve, armSub m_arm, liftSub m_lift){
        

        addCommands(
       
        new littleUponLift(m_lift), 
        new InstantCommand(() -> m_arm.armUp()), 
        new InstantCommand(() -> m_lift.liftUp()), 
        new waitTime(), 
        new Move1(s_Swerve),  
        new InstantCommand(() -> m_arm.armDown()), 
        new waitTime(),
        new Move2(s_Swerve), 
        new InstantCommand(() -> m_arm.armUp()));

    }
}