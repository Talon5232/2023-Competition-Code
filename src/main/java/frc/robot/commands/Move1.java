/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class Move1 extends CommandBase {
private double looper;
  private final Timer m2_timer = new Timer();
  private double time_to_wait = 5;
private Swerve s_Swerve; 
private int runonce = 0;

  /*
   * Creates a new IntakeDumpCommand.
   */
  public Move1(Swerve s_Swerve){
    this.s_Swerve= s_Swerve;
    addRequirements(s_Swerve);
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
    
    TrajectoryConfig config = 
    new TrajectoryConfig(
            Constants.AutoConstants.kMaxSpeedMetersPerSecond,
            Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(Constants.Swerve.swerveKinematics);
        Trajectory exampleTrajectory5 =
        TrajectoryGenerator.generateTrajectory(
            // Start at node, put code to place cone before this
            new Pose2d(0, 0, new Rotation2d(0)),
            // move over charging station, moving 190in putting us in front of cone by a bit
            List.of(),//new Translation2d(2.95, 0)
            // drive onto charging station, reaching the theoretical center
            new Pose2d(1.5, 0, new Rotation2d(0)),
            config
            );
    var thetaController =
        new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    
    SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                exampleTrajectory5,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);
    
        if(runonce == 0){
         
        // An example trajectory to follow.  All units in meters.
       
        s_Swerve.resetOdometry(exampleTrajectory5.getInitialPose());
        runonce = 1;
        swerveControllerCommand.schedule();
        } /* 
        while(isFinished() == false){
          looper = s_Swerve.getPose().getX();
          SmartDashboard.putNumber("LooperGetXValue", looper);

        }
        */
        SmartDashboard.putNumber("timer3", m2_timer.get());
        
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
