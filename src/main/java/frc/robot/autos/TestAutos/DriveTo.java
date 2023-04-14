// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.TestAutos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveTo extends InstantCommand {
  Swerve m_Swerve;
  Translation2d m_Translation2d;
  double m_Rotation;
  public DriveTo(Swerve swerve, Translation2d translation2d, double rotation) {
    this.m_Swerve = swerve;
    this.m_Translation2d = translation2d;
    this.m_Rotation = Units.degreesToRadians(rotation);
    addRequirements(swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Swerve.drive(m_Translation2d, m_Rotation,false, false);
  }

  // @Override
  // public void end(boolean interrupted){
  //   m_Swerve.drive(new Translation2d(0, 0) , 0, true, false);
  // }
}
