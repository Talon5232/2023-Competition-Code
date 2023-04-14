// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.TestAutos;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.ObjectToTarget;

public class DriveToAndAlign extends CommandBase {
  /** Creates a new DriveToAndAlign. */
  private final static double TRANSLATION_TOLERANCE = .02;
  private final static double THETA_TOLERANCE = Units.degreesToRadians(2);
  
  private static final TrapezoidProfile.Constraints XY_CONSTRAINTS = new TrapezoidProfile.Constraints(
    Constants.Swerve.maxSpeed,
    Constants.Swerve.maxSpeed*.5
  );

  private static final TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(
    Constants.Swerve.maxAngularVelocity, 
    Constants.Swerve.maxAngularVelocity*.3
    );

  private final ProfiledPIDController xController = new ProfiledPIDController(3, 0, 0, XY_CONSTRAINTS);
  private final ProfiledPIDController yController = new ProfiledPIDController(3, 0, 0, XY_CONSTRAINTS);
  private final ProfiledPIDController thetaController = new ProfiledPIDController(4, 0, 0, THETA_CONSTRAINTS);

  private final Swerve m_Swerve;
  private final Vision m_Vision;
  private final Supplier<Pose2d> m_providedPose;
  private final Pose2d m_goalPose;
  private final ObjectToTarget m_ObjectToTarget;
  public DriveToAndAlign(Swerve swerve, Vision vision, Supplier<Pose2d> pose, Pose2d goalPose, ObjectToTarget object) {
    this.m_Swerve = swerve;
    this.m_Vision = vision;
    this.m_providedPose = pose;
    this.m_goalPose = goalPose;
    this.m_ObjectToTarget = object;

    addRequirements(swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  resetController();
  xController.setGoal(m_goalPose.getX());
  yController.setGoal(m_goalPose.getY());
  thetaController.setGoal((m_goalPose.getRotation().getRadians()));
  
  } 
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var robotPose = m_providedPose.get();
    if(m_ObjectToTarget != ObjectToTarget.NONE){
      robotPose = new Pose2d(m_Vision.generate2dPositionToObject(m_ObjectToTarget), new Rotation2d(0));
    }

    var xSpeed = xController.calculate(robotPose.getX());
    var ySpeed = yController.calculate(robotPose.getY());
    var thetaSpeed  = thetaController.calculate(robotPose.getRotation().getRadians());

    m_Swerve.drive(new Translation2d(xSpeed, ySpeed), thetaSpeed, false, false);
    SmartDashboard.putNumber("xSpeed", xSpeed);
    SmartDashboard.putNumber("ySpeed", ySpeed);
    SmartDashboard.putNumber("thetaSpeed", thetaSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return goalReached();
  }

 
  private void resetController(){
    var providedPose = m_providedPose.get();
    xController.reset(providedPose.getX());
    yController.reset(providedPose.getY());
    thetaController.reset(providedPose.getRotation().getRadians());
  }

  private boolean goalReached(){
    return (xController.atGoal() && yController.atGoal() && thetaController.atGoal());
  }

}
