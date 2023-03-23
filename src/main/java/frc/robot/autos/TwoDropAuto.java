package frc.robot.autos;

import java.time.Instant;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import frc.robot.subsystems.*;
import frc.robot.Constants;
import frc.robot.Constants.*;
import frc.robot.Constants.Swerve;
import frc.robot.commands.littleUponLift;

public class TwoDropAuto extends SequentialCommandGroup {
    private armSub m_arm; 
    private liftSub m_lift; 
    private intakeSub m_IntakeSub; 
    // This will load the file "FullAuto.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
// for every path in the group

public TwoDropAuto(frc.robot.subsystems.Swerve s_Swerve, liftSub m_lift, intakeSub m_intake, armSub m_arm){
    this.m_arm = m_arm;
    this.m_IntakeSub = m_IntakeSub;
    this.m_lift = m_lift;
List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("2Drop", new PathConstraints(3, 1));

// This is just an example event map. It would be better to have a constant, global event map
// in your code that will be used by all path following commands.
HashMap<String, Command> eventMap = new HashMap<>();
eventMap.put("ResetOdometry", new InstantCommand(() -> s_Swerve.zeroGyro()));
eventMap.put("LittleUpOnLift", new littleUponLift(m_lift));
eventMap.put("UpOnArm", new InstantCommand(() -> m_arm.armUp()));
eventMap.put("UpOnLift", new InstantCommand(() -> m_lift.liftUp()));
eventMap.put("Outake", new InstantCommand(() -> m_intake.AutoIntakeOut()));
eventMap.put("Wait1", new WaitCommand(.5));
eventMap.put("StopOutake", new InstantCommand(() -> m_intake.AutoIntakeOff()));
eventMap.put("DownOnLift", new InstantCommand(() -> m_lift.liftDown()));
eventMap.put("DownOnArm", new InstantCommand(() -> m_arm.armDown()));
eventMap.put("Intake", new InstantCommand(() -> m_intake.AutoIntakeIn()));
eventMap.put("ArmVeryDown", new InstantCommand(() -> m_arm.armVeryDown()));
eventMap.put("Wait2", new WaitCommand(1));
eventMap.put("MiddleOnLift", new InstantCommand(() -> m_lift.liftMiddle()));
eventMap.put("MiddleOnArm", new InstantCommand(() -> m_arm.armMiddle()));
eventMap.put("ArmAuto", new InstantCommand(() -> m_arm.armAuto()));







//eventMap.put("marker2", new PrintCommand("Passed marker 1"));



//eventMap.put("intakeDown", new IntakeDown());

// Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
    s_Swerve::getPose, // Pose2d supplier
    s_Swerve::resetOdometry,
    Constants.Swerve.swerveKinematics,
    new PIDConstants(5, 0.0, .0), // PI constants to correct for translation error (used to create the X and Y PID controllers)
    new PIDConstants(4.5, 0.0, .0), // PID constants to correct for rotation error (used to create the rotation controller)
    s_Swerve::setModuleStates, // Module states consumer used to output to the drive subsystem
    eventMap,
    true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
    s_Swerve // The drive subsystem. Used to properly set the requirements of path following commands
);

Command fullAuto = autoBuilder.fullAuto(pathGroup);
addCommands(
    fullAuto
);
}

}
