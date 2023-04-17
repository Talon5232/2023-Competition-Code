// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AutoBuilder;

import java.util.LinkedList;

import edu.wpi.first.math.geometry.Pose2d;

/** Add your docs here. */
public class PathBuilder {


}



    
    /*
     * Larger Subroutine 
     * 
     * Class called AutoFactory
     * Needs 2 constructors one which requires curr and goal pose
     * Second needs curr and vision target
     * new AutoFactory() creates List like
     * List.of(curr->Goal, curr->Vision)
     * 
     */


    /* Internal Subroutine for each Translation added */
    /* Node 0 == 0,0  */
    // start Pose
    /* Node 1 == First Translation end pont 0.75   */
    /* Node 2 == First Translation end pont 1.5   -- Midpoint !*/
    /* Node 3 == First Translation end pont 2.25  */
    /* Node 4 == First Translation end pont 3     */
    /* 
     * At each distance reach pop the respective node
     * List should now be 3 nodes remaining 0,1,2 which were 2,3,4
     * 
     * startTime = new Time
     * startDistance = getX + getY
     * Now find distance to object (Say arbitrary distance of 2 meters x, 0 meters y) = newGoal
     * we must now set new goals and add to the list
     * controllersNewGoal = newGoal.controller - (controllerSpeed * (currentTime - startTime))
     * controllers.setGoal()
     * conttrollersNewGoal / 5 (Do we divide by space to allow only 5 on the stack? or just always add 5?)
     * 
     * Node 0 2.25
     * Node 1 3
     * Node 2 3.7
     * Node 3 4.4
     * Node 4 5
     * 
     * 
     * 
     */

    // double calcStart = Timer.getFPGATimestamp();
    // double xStart = test.getX();
    // double yStart = test.getY();
    // double rotStart = test.getRotation().getRadians();
    
    // LinkedList<Pose2d> test = new LinkedList<>();
    // Pose2d targetPose = new Pose2d(new Translation2d(3, 3), new Rotation2d(0));

    
    /*
     * 
     * Idea: 
     * Split paths into 5 nodes, beg before-mid middle past-mid end
     * Goal is set at end
     * At past-mid start path gen for x-object
     * Do some math and set new goal to x-object preferiably before decel time
     * Reverse list and invert values on way back
     * 
     * Some problems
     * Still moving when calculating distance
     * 
     */