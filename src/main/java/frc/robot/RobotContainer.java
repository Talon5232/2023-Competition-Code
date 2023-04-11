package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autos.BalanceAuto;
import frc.robot.autos.GenerateTrajAuto;
import frc.robot.autos.LongAuto;
import frc.robot.autos.LongTwoDropAuto;
import frc.robot.autos.NoAuto;
import frc.robot.autos.PlannerDrop;
import frc.robot.autos.ShortAuto;
import frc.robot.autos.TwoDropAuto;
import frc.robot.autos.VeryShortAuto;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Flipper;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.armSub;
import frc.robot.subsystems.blinkin;
import frc.robot.subsystems.intakeSub;
import frc.robot.subsystems.liftSub;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    /* Controllers */
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    private final Joystick driver = new Joystick(0);// xbox controller
    private final Joystick Thrustmaster = new Joystick(1);
    // private final Joystick driver2 = new Joystick(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    private final JoystickButton Abutton = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton Bbutton = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton Xbutton = new JoystickButton(driver, XboxController.Button.kX.value);

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton button8 = new JoystickButton(Thrustmaster, 8);
    private final JoystickButton button7 = new JoystickButton(Thrustmaster, 7);
    private final JoystickButton button2 = new JoystickButton(Thrustmaster, 2);

    private final JoystickButton button14 = new JoystickButton(Thrustmaster, 14);
    private final JoystickButton button15 = new JoystickButton(Thrustmaster, 15);
    private final JoystickButton button16 = new JoystickButton(Thrustmaster, 16);
    private final JoystickButton rightBummper = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton button4 = new JoystickButton(Thrustmaster, 4);
    private final JoystickButton button3 = new JoystickButton(Thrustmaster, 3);

    private final JoystickButton trigger = new JoystickButton(Thrustmaster, 1);
    private final JoystickButton backButton = new JoystickButton(driver, XboxController.Button.kBack.value);

    private final JoystickButton button9 = new JoystickButton(Thrustmaster, 9);
    private final JoystickButton button6 = new JoystickButton(Thrustmaster, 6);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    // Thrust Master Buttons
    // private final boolean trigger = driver2.getRawButton(1);
    /* Subsystems */
    // private final liftSub m_lift = new liftSub();
    private final armSub m_arm = new armSub();
    private final liftSub m_lift = new liftSub();
    private final Swerve s_Swerve = new Swerve();
    private final intakeSub m_Intake = new intakeSub();
    private final Flipper Flipper = new Flipper();
    private final blinkin blinkin = new blinkin();
    private final Vision m_vision = new Vision();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        s_Swerve.setDefaultCommand(

                new TeleopSwerve(
                        s_Swerve,
                        () -> (-driver.getRawAxis(translationAxis)), // *Math.abs(driver.getRawAxis(translationAxis))
                        () -> (-driver.getRawAxis(strafeAxis)), // *Math.abs(driver.getRawAxis(strafeAxis))
                        () -> (-driver.getRawAxis(rotationAxis)), // * Math.abs(driver.getRawAxis(rotationAxis))
                        () -> robotCentric.getAsBoolean())

        );
        // autoChooser = new SendableChooser<Command>();
        autoChooser.setDefaultOption("NoAuto", new NoAuto(s_Swerve));
        autoChooser.addOption("ShortAuto", new ShortAuto(s_Swerve));
        autoChooser.addOption("BalanceAuto", new BalanceAuto(s_Swerve));
        autoChooser.addOption("LongAuto", new LongAuto(s_Swerve));
        autoChooser.addOption("VeryShortAuto", new VeryShortAuto(s_Swerve));
        // autoChooser.addOption("DropAuto", new DropAuto(s_Swerve, m_arm, m_lift,
        // m_Intake));
        // d autoChooser.addOption("AutoLevel", new AutoLevel(s_Swerve, m_arm, m_lift));
        autoChooser.addOption("DropPathPlanner", new PlannerDrop(s_Swerve, m_lift, m_Intake, m_arm));
        autoChooser.addOption("Planner2Drop", new TwoDropAuto(s_Swerve, m_lift, m_Intake, m_arm));
        autoChooser.addOption("OppPlanner2Drop", new LongTwoDropAuto(s_Swerve, m_lift, m_Intake, m_arm));
        autoChooser.addOption("GenerateTrajJauto", new GenerateTrajAuto(s_Swerve, m_lift, m_Intake, m_arm, m_vision));
        // SmartDashboard.putData("Autonomous Chooser", autoChooser);
        Shuffleboard.getTab("user tab").add(autoChooser);

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

        /* Driver Buttons */
        // UP
        button16.whileTrue(new InstantCommand(() -> m_arm.armUp()));
        button16.whileTrue(new InstantCommand(() -> m_lift.liftUp()));
        // MIddle
        button15.whileTrue(new InstantCommand(() -> m_arm.armMiddle()));
        button15.whileTrue(new InstantCommand(() -> m_lift.liftMiddle()));
        // Down
        button14.whileTrue(new InstantCommand(() -> m_arm.armDown()));
        button14.whileTrue(new InstantCommand(() -> m_lift.liftDown()));
        button7.whileTrue(new InstantCommand(() -> m_lift.liftUpManual()))
                .whileFalse(new InstantCommand(() -> m_lift.liftUpManualStop()));
        button8.whileTrue(new InstantCommand(() -> m_lift.liftDownManual()))
                .whileFalse(new InstantCommand(() -> m_lift.liftDownManualStop()));
        button6.whileTrue(new InstantCommand(() -> m_arm.armUpManual()))
                .whileFalse(new InstantCommand(() -> m_arm.armUpManualStop()));
        button9.whileTrue(new InstantCommand(() -> m_arm.armDownManual()))
                .whileFalse(new InstantCommand(() -> m_arm.armDownManualStop()));
        ;
        button3.whileTrue(new InstantCommand(() -> Flipper.Flipperin()))
                .whileFalse(new InstantCommand(() -> Flipper.FlipperStop()));
        button4.whileTrue(new InstantCommand(() -> Flipper.Flipperout()))
                .whileFalse(new InstantCommand(() -> Flipper.FlipperStop()));

        // backButton.whileTrue(new InstantCommand(() -> m_arm.armMovement(double
        // setpoint + .025)));
        // button8.whileTrue(new InstantCommand(() -> m_lift.liftDown())).whileFalse(new
        // InstantCommand(() -> m_lift.liftStop()));
        // button7.whileTrue(new InstantCommand(() -> m_lift.liftUp())).whileFalse(new
        // InstantCommand(() -> m_lift.liftStop()));
        // button9.whileTrue(new InstantCommand(() ->
        // m_lift.liftMiddle())).whileFalse(new InstantCommand(() ->
        // m_lift.liftStop()));
        trigger.whileTrue(new InstantCommand(() -> m_Intake.intakeout()))
                .whileFalse(new InstantCommand(() -> m_Intake.intakeStop()));
        button2.whileTrue(new InstantCommand(() -> m_Intake.intakein()))
                .whileFalse(new InstantCommand(() -> m_Intake.intakeStop()));
        // button3.whileTrue(new InstantCommand(() ->
        // blinkin.givecone())).whileFalse(new InstantCommand(() ->
        // frc.robot.subsystems.blinkin.lightsNormal()));

        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return autoChooser.getSelected();
    }

    /**
     * Use this to update pose to april tag at start
     * 
     * @param pose Pass start pose from camera
     */
    public void exampleResetOdo(Pose2d pose){
        s_Swerve.resetOdometry(pose);
    }
}