package frc.robot.autos;

import java.time.Instant;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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

public class GenerateTrajAuto extends SequentialCommandGroup {
    
    private armSub m_arm; 
    private liftSub m_lift; 
    private intakeSub m_IntakeSub; 
    private Vision m_vision;
    // This will load the file "FullAuto.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
// for every path in the group

public GenerateTrajAuto(frc.robot.subsystems.Swerve s_Swerve, liftSub m_lift, intakeSub m_intake, armSub m_arm, Vision m_vision){
    this.m_arm = m_arm;
    this.m_IntakeSub = m_IntakeSub;
    this.m_lift = m_lift;
    this.m_vision = m_vision;
//List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("2Drop", new PathConstraints(3, 1));

// This is just an example event map. It would be better to have a constant, global event map
// in your code that will be used by all path following commands.

HashMap<String, Command> eventMap = new HashMap<>();

// eventMap.put("ResetOdo", new InstantCommand(() ->s_Swerve.resetOdometry(new Pose2d(new Translation2d(m_vision.generateDistanceXToObject(ObjectToTarget.APRIL_TAG), m_vision.generateDistanceYToObject(ObjectToTarget.APRIL_TAG)), new Rotation2d(0)))));
// eventMap.put("LittleUpOnLift", new littleUponLift(m_lift));
// eventMap.put("UpOnArm", new InstantCommand(() -> m_arm.armUp()));
// eventMap.put("UpOnLift", new InstantCommand(() -> m_lift.liftUp()));
// eventMap.put("Outake", new InstantCommand(() -> m_intake.AutoIntakeOut()));
// eventMap.put("Wait1", new WaitCommand(1));
// eventMap.put("StopOutake", new InstantCommand(() -> m_intake.AutoIntakeOff()));
// eventMap.put("DownOnLift", new InstantCommand(() -> m_lift.liftDown()));
// eventMap.put("DownOnArm", new InstantCommand(() -> m_arm.armDown()));
// eventMap.put("Intake", new InstantCommand(() -> m_intake.AutoIntakeIn()));
// eventMap.put("ArmVeryDown", new InstantCommand(() -> m_arm.armVeryDown()));
// eventMap.put("AutoArm", new InstantCommand(()-> m_arm.armAuto()));

// eventMap.put("Wait2", new WaitCommand(1));







//eventMap.put("marker2", new PrintCommand("Passed marker 1"));



//eventMap.put("intakeDown", new IntakeDown());

// Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
    s_Swerve::getPose, // Pose2d supplier
    s_Swerve::resetOdometry,
    Constants.Swerve.swerveKinematics,
    new PIDConstants(5, 0.0, 0.0), // PI constants to correct for translation error (used to create the X and Y PID controllers)
    new PIDConstants(4.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
    s_Swerve::setModuleStates, // Module states consumer used to output to the drive subsystem
    eventMap,
    true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
    s_Swerve // The drive subsystem. Used to properly set the requirements of path following commands
);
PathPlannerTrajectory traj3 = PathPlanner.generatePath(
    new PathConstraints(.25, .25),
    new PathPoint(new Translation2d(s_Swerve.getPose().getX(), s_Swerve.getPose().getY()), new Rotation2d(0)), // position, heading
    new PathPoint(new Translation2d(Math.abs(m_vision.generateDistanceXToObject(ObjectToTarget.APRIL_TAG)) - .3, m_vision.generateDistanceYToObject(ObjectToTarget.APRIL_TAG)+.5), new Rotation2d(0))) // position, heading

    ;
//FollowPathWithEvents command = new FollowPathWithEvents(getPath, null, eventMap



Command fullAuto = autoBuilder.followPath(traj3);
addCommands(
   fullAuto,
   new InstantCommand(() -> s_Swerve.drive(new Translation2d(0,0), 0, false, false)));

}

}
