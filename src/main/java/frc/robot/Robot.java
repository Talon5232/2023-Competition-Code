// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.io.ObjectInputStream.GetField;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.autos.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.blinkin;
import frc.robot.autos.ShortAuto;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  //SendableChooser<Command> autoChooser;
  private final Vison m_Vison = new Vison();
  Command autonomousCommand;
// private final Swerve s_Swerve = new Swerve();
 // private final ShortAuto a_ShortAuto = new ShortAuto(s_Swerve);
 // private final LongAuto a_LongAuto = new LongAuto(s_Swerve);
  public static CTREConfigs ctreConfigs;
 // private final SendableChooser<String> m_chooser = new SendableChooser<>();

 // private static final String ShortAuto = "ShortAuto";
 // private static final String LongAuto = "LongAuto";

 
  //private String m_autoSelected;

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    CameraServer.startAutomaticCapture();
    ctreConfigs = new CTREConfigs();
    m_robotContainer = new RobotContainer();
    AddressableLED m_led = new AddressableLED(4);
    AnalogInput ultraSonic = new AnalogInput(0);
    AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(60);
    m_led.setLength(m_ledBuffer.getLength());
    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      if(DriverStation.getAlliance() == Alliance.Blue){
        m_ledBuffer.setRGB(i, 0, 100, ultraSonic.getValue()*255);
      }
      else{
        m_ledBuffer.setRGB(i, ultraSonic.getValue()*255, 100, 0);
      }
     
      
   }
   
   m_led.setData(m_ledBuffer);
     // PWM port 9
    // Must be a PWM header, not MXP or DIO
    


    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    

    //autoChooser = new SendableChooser<Command>();
    //autoChooser.setDefaultOption("LongAuto", new LongAuto(s_Swerve));
    //autoChooser.addOption("ShortAuto", new ShortAuto(s_Swerve));
    //autoChooser.addOption("BalanceAuto", new BalanceAuto(s_Swerve));
   // SmartDashboard.putData("Autonomous Chooser", autoChooser);
    

    }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    m_Vison.periodic();
    
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();  
    }
   // m_autoSelected = m_chooser.getSelected();
    //System.out.println("Auto Selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    /*
    switch (m_autoSelected){
      case ShortAuto:
        new ShortAuto(s_Swerve);
      break;
      case LongAuto:
        default:
        new LongAuto(s_Swerve);
        break;

    }
    */

  }

  @Override
  public void teleopInit() {
    if (Robot.isSimulation()) {
      NetworkTableInstance instance = NetworkTableInstance.getDefault();
      instance.stopServer();
      // set the NT server if simulating this code.
      // "localhost" for photon on desktop, or "photonvision.local" / "[ip-address]" for coprocessor
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
  public void testPeriodic() {}
}
