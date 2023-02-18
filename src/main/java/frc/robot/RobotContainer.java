package frc.robot;

import java.sql.DriverAction;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
@@ -25,8 +26,7 @@ public class RobotContainer {
    /* Controllers */
    private final XboxController xcontrollers = new XboxController(0);
    private final Joystick driver = new Joystick(0);
    //private final Joystick driver2 = new Joystick(1);

    private final Joystick driver2 = new Joystick(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
@@ -49,15 +49,15 @@ public class RobotContainer {
    private final JoystickButton rightBumper = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    //Thrust Master Buttons
    //private final boolean trigger = driver2.getRawButton(1);
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
@@ -84,26 +84,23 @@ private void configureButtonBindings() {
        /* Driver Buttons */

        abutton.whileTrue(new InstantCommand(() -> m_arm.armUp())).whileFalse(new InstantCommand(() -> m_arm.armStop()));
       // backButton.whileTrue(new InstantCommand(() -> m_arm.armMovement(double setpoint + .025)));
        abutton.whileTrue(new InstantCommand(() -> m_arm.armMovement(0))).whileFalse(new InstantCommand(() -> m_arm.armStop()));

        bbutton.whileTrue(new InstantCommand(() -> m_arm.armDown())).whileFalse(new InstantCommand(() -> m_arm.armStop()));
        leftBumper.whileTrue(new InstantCommand(() -> m_lift.liftUp())).whileFalse(new InstantCommand(() -> m_lift.liftStop()));
        rightBumper.whileTrue(new InstantCommand(() -> m_lift.liftDown())).whileFalse(new InstantCommand(() -> m_lift.liftStop()));
        xbutton.whileTrue(new InstantCommand(() -> m_Intake.intakeDown())).whileFalse(new InstantCommand(() -> m_Intake.intakeStop()));
        Startbutton.whileTrue(new InstantCommand(() -> m_Intake.intakeUp())).whileFalse(new InstantCommand(() -> m_Intake.intakeStop()));
        /* 
        if(trigger == true){
           new InstantCommand(() -> m_lift.liftUp());
        }
        else{
            new InstantCommand(() -> m_lift.liftDown());
        }
        */




        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

    if(trigger == true){
        new InstantCommand(() -> m_lift.liftUp());
    } else{
        new InstantCommand(() -> m_lift.liftDown());
    }
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
}


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
