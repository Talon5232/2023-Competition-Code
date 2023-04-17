// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
 // private final Vision m_Vision = new Vision();
  Command autonomousCommand;

  public static CTREConfigs ctreConfigs;

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private EventLoop m_eventLoop = new EventLoop();

  

  @Override
  public void robotInit() {
    CameraServer.startAutomaticCapture();
    ctreConfigs = new CTREConfigs();
    m_robotContainer = new RobotContainer();

    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {


    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    m_robotContainer.exampleResetOdo(new Pose2d()); // reset our odo

    /* 
    s_swerve.resetOdometry(new Pose2d(new
    Translation2d(m_Vision.generateDistanceXToObject(ObjectToTarget.APRIL_TAG),
    m_Vision.generateDistanceYToObject(ObjectToTarget.APRIL_TAG)), new
    Rotation2d(0)));
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto Selected: " + m_autoSelected);
    */
  }
  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopInit() {
    if (Robot.isSimulation()) {
      NetworkTableInstance instance = NetworkTableInstance.getDefault();
      instance.stopServer();
      // set the NT server if simulating this code.
      // "localhost" for photon on desktop, or "photonvision.local" / "[ip-address]"
      // for coprocessor
      instance.setServer("localhost");
      instance.startClient4("myRobot");
    }
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
