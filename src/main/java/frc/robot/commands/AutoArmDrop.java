/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.sql.Time;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.*;

public class AutoArmDrop extends SequentialCommandGroup {
  private boolean endcommand = false;
  private final Timer m_timer = new Timer();
  private double time_to_wait = 5;
private armSub m_arm; 
private liftSub m_lift; 
private intakeSub m_IntakeSub; 

  /*
   * Creates a new IntakeDumpCommand.
   */
  public AutoArmDrop(armSub m_arm, liftSub m_lift, intakeSub m_IntakeSub){
    this.m_arm = m_arm;
    this.m_IntakeSub = m_IntakeSub;
    this.m_lift = m_lift;
    addRequirements(m_arm, m_lift, m_IntakeSub);
    addCommands(

    );
  }
}
  // Called when the command is initially scheduled.
 