/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import java.sql.Time;
import java.util.List;
import java.util.function.BooleanSupplier;

import javax.imageio.plugins.tiff.ExifGPSTagSet;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class AutoLevelCommand extends CommandBase {
private double looper;
  private final Timer m2_timer = new Timer();
  private double time_to_wait = 5;
private Swerve s_Swerve; 
private int runonce = 0;
private double GoToX;
private BooleanSupplier robotCentricSup;

  /*
   * Creates a new IntakeDumpCommand.
   */
  public AutoLevelCommand(Swerve s_Swerve){
    this.s_Swerve= s_Swerve;
    addRequirements(s_Swerve);
    this.robotCentricSup = robotCentricSup;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m2_timer.stop();
    m2_timer.reset();
    m2_timer.start();
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
      
        if(s_Swerve.gyro.getPitch() >= 10){
          GoToX = GoToX + .001;
        s_Swerve.drive(new Translation2d(GoToX,0), 
        0, 
        !robotCentricSup.getAsBoolean(), 
        true);
        }
        
        else if(s_Swerve.gyro.getPitch() <= -10){
          GoToX = GoToX - .001;
        s_Swerve.drive(new Translation2d(GoToX,0), 
        0, 
        !robotCentricSup.getAsBoolean(), 
        true);
        }

        else{
          s_Swerve.drive(new Translation2d(GoToX,0), 
        0, 
        !robotCentricSup.getAsBoolean(), 
        true);
        //Change
        }
        SmartDashboard.putNumber("GotoX", GoToX);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    SmartDashboard.putNumber("timer2", m2_timer.get());
    SmartDashboard.putNumber("SwerveX", s_Swerve.getPose().getX());

    if(looper >= .45){
      return true;
    }
    else{
      return false;
    }
    
   
  }
}
