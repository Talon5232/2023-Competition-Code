package frc.robot;

import java.sql.DriverAction;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final XboxController xcontrollers = new XboxController(0);
    private final Joystick driver = new Joystick(0);
    private final Joystick driver2 = new Joystick(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;
    
    
    

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton bbutton = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton abutton = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton xbutton = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton Startbutton = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton backButton = new JoystickButton(driver, XboxController.Button.kBack.value);


    private final JoystickButton leftBumper = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton rightBumper = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    //Thrust Master Buttons
    private final boolean trigger = driver2.getRawButton(1);

    /* Subsystems */
   // private final liftSub m_lift = new liftSub();
    private final armSub m_arm = new armSub();
    private final liftSub m_lift = new liftSub();
    private final Swerve s_Swerve = new Swerve();
    private final intakeSub m_Intake = new intakeSub();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        
        abutton.whileTrue(new InstantCommand(() -> m_arm.armUp())).whileFalse(new InstantCommand(() -> m_arm.armStop()));
        abutton.whileTrue(new InstantCommand(() -> m_arm.armMovement(0))).whileFalse(new InstantCommand(() -> m_arm.armStop()));

        bbutton.whileTrue(new InstantCommand(() -> m_arm.armDown())).whileFalse(new InstantCommand(() -> m_arm.armStop()));
        leftBumper.whileTrue(new InstantCommand(() -> m_lift.liftUp())).whileFalse(new InstantCommand(() -> m_lift.liftStop()));
        rightBumper.whileTrue(new InstantCommand(() -> m_lift.liftDown())).whileFalse(new InstantCommand(() -> m_lift.liftStop()));
        xbutton.whileTrue(new InstantCommand(() -> m_Intake.intakeDown())).whileFalse(new InstantCommand(() -> m_Intake.intakeStop()));
        Startbutton.whileTrue(new InstantCommand(() -> m_Intake.intakeUp())).whileFalse(new InstantCommand(() -> m_Intake.intakeStop()));
    

    if(trigger == true){
        new InstantCommand(() -> m_lift.liftUp());
    } else{
        new InstantCommand(() -> m_lift.liftDown());
    }
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
}
    

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(s_Swerve);
    }
}
