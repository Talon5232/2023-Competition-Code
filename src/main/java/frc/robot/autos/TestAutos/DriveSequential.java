// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.TestAutos;

import java.sql.Time;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.waitTime;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.intakeSub;
import frc.robot.subsystems.liftSub;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Vision.ObjectToTarget;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveSequential extends SequentialCommandGroup {
  private final Swerve m_Swerve;
  private final Vision m_Vision;
  private final intakeSub m_intake;
  private final liftSub m_lift;
  private final armSub m_arm;
  //private final ObjectToTarget m_ObjectToTarget;

  /** Creates a new DriveSequential. */
  public DriveSequential(Swerve swerve, Vision vision, intakeSub intake, liftSub lift, armSub arm) {
    this.m_Swerve = swerve;
    this.m_intake = intake;
    this.m_lift = lift;
    this.m_Vision = vision;
    this.m_arm = arm;
   // this.m_ObjectToTarget = object;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> m_Swerve.zeroGyro()),
      new InstantCommand(() -> m_Swerve.zeroPosition(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)), 0)),
      new InstantCommand(() -> m_Vision.setObject(ObjectToTarget.FLOOR_CONE)),
     // new DriveToAndAlign(m_Swerve, m_Vision, m_Swerve::getPose, new Pose2d(new Translation2d(m_Vision.generateDistanceXToObject(ObjectToTarget.APRIL_TAG)-.4,m_Vision.generateDistanceYToObject(ObjectToTarget.APRIL_TAG)), new Rotation2d(0)), ObjectToTarget.NONE)
      new InstantCommand(() -> m_intake.AutoIntakeIn()),
      new WaitCommand(1),
      new InstantCommand(() -> m_intake.AutoIntakeOff()),
      new InstantCommand(() -> m_lift.liftLittleUp()),
      new DriveToAndAlign(m_Swerve, m_Vision, m_Swerve::getPose, new Pose2d(new Translation2d(-3,-.15), new Rotation2d(Math.PI)), ObjectToTarget.NONE, 0, 0, false),
      new InstantCommand(() -> m_arm.armDown()),
      new InstantCommand(() -> m_lift.liftDown()),

     // new DriveToAndAlign(m_Swerve, m_Vision, m_Swerve::getPose, new Pose2d(new Translation2d(0,0), new Rotation2d(Math.PI)), ObjectToTarget.FLOOR_CONE, 0, 0)
      new DriveToAndAlign(m_Swerve, m_Vision, m_Swerve::getPose, new Pose2d(new Translation2d(-4,0), new Rotation2d(Math.PI)), ObjectToTarget.FLOOR_CONE, .4, .2, true),
      new InstantCommand(() -> m_arm.armAuto()),
      new InstantCommand(() -> m_intake.AutoIntakeIn()),
      new WaitCommand(2),
      new InstantCommand(() -> m_intake.AutoIntakeOff())






      

     // new DriveToAndAlign(m_Swerve, m_Vision, m_Swerve::getPose, new Pose2d(new Translation2d(0,0), new Rotation2d(0)), ObjectToTarget.APRIL_TAG, .8, 0)
      // SmartDashboard.putData("Autonomous Chooser", autoChooser);      //new DriveTo(m_Swerve, new Translation2d(-1, 0), 0)
     // new DriveToAndAlign(m_Swerve, m_Vision, ObjectToTarget.APRIL_TAG)
    );
  }
}
