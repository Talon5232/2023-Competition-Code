// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.List;

import javax.sound.midi.Track;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.littleUponLift;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.armSub;
import frc.robot.subsystems.intakeSub;
import frc.robot.subsystems.liftSub;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class NewScoreConeAuto extends SequentialCommandGroup {
  /** Creates a new NewScoreConeAuto. */
  public NewScoreConeAuto(Swerve s_Swerve, armSub m_arm, liftSub m_lift, intakeSub m_IntakeSub) {
    Trajectory trajectory_1 = GenerateTrajectory(-0.5, 0, 0, s_Swerve);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> s_Swerve.resetOdometry(trajectory_1.getInitialPose())),
      new littleUponLift(m_lift), 
      new InstantCommand(() -> m_arm.armUp()),
      new InstantCommand(() -> m_lift.liftUp()),
      new InstantCommand(() -> GenerateSwerveCommand(-0.5, 0, 0, s_Swerve)),
     // GenerateSwerveCommand(-0.5, 0, 0, s_Swerve),
    //  new InstantCommand(() -> m_IntakeSub.AutoIntakeOut()),
      new WaitCommand(2),
    //  new InstantCommand(() -> m_IntakeSub.AutoIntakeOff()),
      GenerateSwerveCommand(-1.5, 0, 0, s_Swerve)

    );
  }

  public SwerveControllerCommand GenerateSwerveCommand(
      double target_x_cord,
      double targey_y_cord,
      double target_rotation,
      Swerve s_Swerve) {
    SwerveControllerCommand swerve_controller_command;

    var thetaController = new ProfiledPIDController(
        Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
      
    swerve_controller_command =  new SwerveControllerCommand(
      GenerateTrajectory(target_x_cord, targey_y_cord, target_rotation, s_Swerve),
      s_Swerve::getPose,
      Constants.Swerve.swerveKinematics,
      new PIDController(Constants.AutoConstants.kPXController, 0, 0),
      new PIDController(Constants.AutoConstants.kPYController, 0, 0),
      thetaController,
      s_Swerve::setModuleStates,
      s_Swerve);

    return swerve_controller_command;

  }

  private Trajectory GenerateTrajectory(
  double target_x_cord,
  double targey_y_cord,
  double target_rotation,
  Swerve s_Swerve)
  {
    Trajectory trajectory;

    TrajectoryConfig config = 
    new TrajectoryConfig(
            Constants.AutoConstants.kMaxSpeedMetersPerSecond,
            Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(Constants.Swerve.swerveKinematics);    

    trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(s_Swerve.getPose().getX(), s_Swerve.getPose().getY(), new Rotation2d(s_Swerve.getYaw().getDegrees())),
      List.of(),
      new Pose2d(target_x_cord, targey_y_cord, new Rotation2d(target_rotation)),
      config
    );

    return trajectory;
  }

}
