// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.LinkedList;

import edu.wpi.first.wpilibj2.command.Command;

/** Add your docs here. */
public class WaypointActions {
    private int nodePoint;
    private Command commandToExecute;
    
    WaypointActions(int node, Command cmd){
        this.nodePoint = node;
        this.commandToExecute = cmd;
    }

}