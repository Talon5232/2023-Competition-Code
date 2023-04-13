
package frc.robot.autos;

import java.time.Instant;
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
import frc.robot.commands.*;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.armSub;
import frc.robot.subsystems.intakeSub;

import frc.robot.subsystems.liftSub;
import frc.robot.subsystems.Vision.ObjectToTarget;

public class GenTraj2 extends SequentialCommandGroup {

  private armSub m_arm;
  private liftSub m_lift;
  private intakeSub m_Intake;
  private Vision m_vision;
  private Swerve m_Swerve;

  List<PathPlannerTrajectory> totalPaths = new ArrayList<>();

  SequentialCommandGroup seq = new SequentialCommandGroup();
  SequentialCommandGroup followPath = new SequentialCommandGroup();

  public GenTraj2(frc.robot.subsystems.Swerve s_Swerve, liftSub m_lift, intakeSub m_intake, armSub m_arm,
      Vision m_vision) {
    this.m_Swerve = s_Swerve;
    this.m_arm = m_arm;
    this.m_Intake = m_Intake;
    this.m_lift = m_lift;
    this.m_vision = m_vision;
    // List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("2Drop",
    // new PathConstraints(3, 1));

    // This is just an example event map. It would be better to have a constant,
    // global event map
    // in your code that will be used by all path following commands.

    // eventMap.put("marker2", new PrintCommand("Passed marker 1"));

    // eventMap.put("intakeDown", new IntakeDown());

    // Create the AutoBuilder. This only needs to be created once when robot code
    // starts, not every time you want to create an auto command. A good place to
    // put this is in RobotContainer along with your subsystems.
    // SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
    // s_Swerve::getPose, // Pose2d supplier
    // s_Swerve::resetOdometry,
    // Constants.Swerve.swerveKinematics,
    // new PIDConstants(5, 0.0, 0.0), // PI constants to correct for translation
    // error (used to create the X and Y PID controllers)
    // new PIDConstants(4.5, 0.0, 0.0), // PID constants to correct for rotation
    // error (used to create the rotation controller)
    // s_Swerve::setModuleStates, // Module states consumer used to output to the
    // drive subsystem
    // eventMap,
    // true, // Should the path be automatically mirrored depending on alliance
    // color. Optional, defaults to true
    // s_Swerve // The drive subsystem. Used to properly set the requirements of
    // path following commands
    // );

    var thetaController = new PIDController(
        Constants.AutoConstants.kPThetaController,
        0,
        0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // Command fullAuto = autoBuilder.followPath(traj3);

    addCommands(
        new WaitCommand(0.1),
        getCommand());

  }

  private PPSwerveControllerCommand getCommand() {
    PathPlannerTrajectory traj = PathPlanner.generatePath(new PathConstraints(.25, .25),
        new PathPoint(new Translation2d(m_Swerve.getPose().getX(), m_Swerve.getPose().getY()),
            new Rotation2d(0)), // position, heading
        new PathPoint(
            new Translation2d(Math.abs(m_vision.generateDistanceXToObject(ObjectToTarget.APRIL_TAG)) - 1,
                m_vision.generateDistanceYToObject(ObjectToTarget.APRIL_TAG) - 0),
            new Rotation2d(0)));

    var thetaController = new PIDController(
        Constants.AutoConstants.kPThetaController,
        0,
        0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SmartDashboard.putNumber("AutoDistance", m_vision.generateDistanceXToObject(ObjectToTarget.APRIL_TAG));
    return new PPSwerveControllerCommand(traj,
        m_Swerve::getPose,
        Constants.Swerve.swerveKinematics,
        new PIDController(Constants.AutoConstants.kPXController, 0, 0),
        new PIDController(Constants.AutoConstants.kPYController, 0, 0),
        thetaController,
        m_Swerve::setModuleStates,
        true,
        m_Swerve);
  }

}
