// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.littleUponLift;
import frc.robot.subsystems.armSub;
import frc.robot.subsystems.intakeSub;
import frc.robot.subsystems.liftSub;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.littleUponLift;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.armSub;
import frc.robot.subsystems.intakeSub;
import frc.robot.subsystems.liftSub;
import frc.robot.subsystems.Vision.ObjectToTarget;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TrajTest extends SequentialCommandGroup {
  /** Creates a new TrajTest. */
  private armSub m_arm;
    private liftSub m_lift;
    private intakeSub m_IntakeSub;
    private Vision m_vision;
  public TrajTest(frc.robot.subsystems.Swerve s_Swerve, liftSub m_lift, intakeSub m_intake, armSub m_arm,
  Vision m_vision, double xdistance, double ydistance) {
    this.m_arm = m_arm;
    this.m_IntakeSub = m_IntakeSub;
    this.m_lift = m_lift;
    this.m_vision = m_vision;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    var thetaController = new PIDController(
      Constants.AutoConstants.kPThetaController,
      0,
      0);
thetaController.enableContinuousInput(-Math.PI, Math.PI);
    PathPlannerTrajectory traj3 = PathPlanner.generatePath(
                new PathConstraints(.25, .25),
                new PathPoint(new Translation2d(s_Swerve.getPose().getX(), s_Swerve.getPose().getY()),
                        new Rotation2d(0)), // position, heading
                new PathPoint(
                        new Translation2d(Math.abs(xdistance) - 1,
                               ydistance-0),
                       new Rotation2d(0)));
        SmartDashboard.putNumber("AutoDistance", m_vision.generateDistanceXToObject(ObjectToTarget.APRIL_TAG));
                //new Translation2d(.2, 0), new Rotation2d(0)));
               
                
                PPSwerveControllerCommand pP = new PPSwerveControllerCommand(
                traj3,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                true,
                s_Swerve);
    addCommands(pP);
  }
}
