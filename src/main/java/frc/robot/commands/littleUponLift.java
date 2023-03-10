/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;


import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.*;

public class littleUponLift extends CommandBase {
  private final Timer m_timer = new Timer();
  private double time_to_wait = 1;
private liftSub m_lift; 

  /*
   * Creates a new IntakeDumpCommand.
   */
  public littleUponLift(liftSub m_lift){
    this.m_lift = m_lift;
    addRequirements(m_lift);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    m_lift.liftLittleUp();
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(time_to_wait <= m_timer.get()){
      return true;
    }
    return false;
    
  }
}
