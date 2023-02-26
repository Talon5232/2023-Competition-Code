package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
public class OverlineBalance extends SequentialCommandGroup {
    public OverlineBalance(Swerve s_Swerve){
        TrajectoryConfig config = 
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at node, put code to place cone before this
                new Pose2d(0, 0, new Rotation2d(0)),
                // move over charging station, moving 190in putting us in front of cone by a bit
                List.of(new Translation2d(4.2, 0), new Translation2d(-1.618, 0)),//new Translation2d(2.95, 0)
                // drive onto charging station, reaching the theoretical center
                new Pose2d(2.581, 0, new Rotation2d(0)),
                config
                );
                /* 
         Trajectory exampleTrajectory2 =
            TrajectoryGenerator.generateTrajectory(
                // Start at node, put code to place cone before this
                new Pose2d(4.2, 0, new Rotation2d(0)),
                // move over charging station, moving 190in putting us in front of cone by a bit
                List.of(new Translation2d(-1.618, 0)),//new Translation2d(2.95, 0)
                // drive onto charging station, reaching the theoretical center
                new Pose2d(2.581, 0, new Rotation2d(0)),
                config
                );        
                var concatTraj = exampleTrajectory.concatenate(exampleTrajectory2);
                */
        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                exampleTrajectory,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);


        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose())),
            swerveControllerCommand
        );
    }
}