// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.LinkedList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;


/** Add your docs here. */
public class Factory {
    private final Pose2d currPose;
    private final Pose2d goalPose;
    private final LinkedList<WaypointActions> actions;
    
    Factory(Pose2d current, Pose2d goal, LinkedList<WaypointActions> act){
        this.currPose = current;
        this.goalPose = goal;
        this.actions = act;
    }

    public void fillEmptyNodes(){
        
    }


}


class WaypointActions {
    private int nodePoint;
    private Command commandToExecute;
    
    WaypointActions(int node, Command cmd){
        this.nodePoint = node;
        this.commandToExecute = cmd;
    }

}