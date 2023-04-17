// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.TestAutos;

import java.util.LinkedList;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;
import frc.robot.commands.Move1;
import frc.robot.subsystems.Factory;
import frc.robot.subsystems.Swerve;

import frc.robot.subsystems.Vision;
import frc.robot.subsystems.WaypointActions;
import frc.robot.subsystems.Vision.ObjectToTarget;

public class DriveToAndAlign extends CommandBase {
  /** Creates a new DriveToAndAlign. */
  private final static double TRANSLATION_TOLERANCE = .02;
  private final static double THETA_TOLERANCE = Units.degreesToRadians(2);
  
  private static final TrapezoidProfile.Constraints XY_CONSTRAINTS = new TrapezoidProfile.Constraints(
    Constants.Swerve.maxSpeed,
    Constants.Swerve.maxSpeed*.5
  );

  private static final TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(
    Constants.Swerve.maxAngularVelocity, 
    Constants.Swerve.maxAngularVelocity*.3
    );

  private final ProfiledPIDController xController = new ProfiledPIDController(3, 0, 0, XY_CONSTRAINTS);
  private final ProfiledPIDController yController = new ProfiledPIDController(3, 0, 0, XY_CONSTRAINTS);
  private final ProfiledPIDController thetaController = new ProfiledPIDController(4, 0, 0, THETA_CONSTRAINTS);

  private final Swerve m_Swerve;
  private final Vision m_Vision;
  private final double XOffset;
  private final double YOffset;
  private final Supplier<Pose2d> m_providedPose;
  private final Pose2d m_goalPose;
  private boolean firstcycle = false;
  private final ObjectToTarget m_ObjectToTarget;
  public DriveToAndAlign(Swerve swerve, Vision vision, Supplier<Pose2d> pose, Pose2d goalPose, ObjectToTarget object, double XOffset, double YOffset) {
    this.m_Swerve = swerve;
    this.m_Vision = vision;
    this.m_providedPose = pose;
    this.m_goalPose = goalPose;
    this.XOffset = XOffset;
    this.YOffset = YOffset;
    this.m_ObjectToTarget = object;
    

    addRequirements(swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  resetController();
  if(m_ObjectToTarget == ObjectToTarget.NONE){
    xController.setGoal(m_goalPose.getX());
    yController.setGoal(m_goalPose.getY());
    thetaController.setGoal((m_goalPose.getRotation().getRadians()));
  } else {
    m_Vision.setObject(m_ObjectToTarget);
  }
  
  firstcycle = false;
  
  } 
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    var robotPose = m_providedPose.get();
    if(robotPose.getRotation().getDegrees() >= 90 || robotPose.getRotation().getDegrees() <= 90){
      boolean backwards = true;
    }
    if(firstcycle == false){
    if(m_ObjectToTarget != ObjectToTarget.NONE){
      if(m_Vision.generateDistanceXToObject() >= -5){
        xController.setGoal((m_Vision.generateDistanceXToObject())-XOffset);
        yController.setGoal((-m_Vision.generateDistanceYToObject())-YOffset);
        thetaController.setGoal((m_goalPose.getRotation().getRadians()));
        firstcycle = true;
        }
    }
  }

    var xSpeed = xController.calculate(robotPose.getX());
    var ySpeed = yController.calculate(robotPose.getY());
    var thetaSpeed  = thetaController.calculate(robotPose.getRotation().getRadians());

    m_Swerve.drive(new Translation2d(xSpeed, ySpeed), thetaSpeed, true, false);
    SmartDashboard.putNumber("xSpeed", xSpeed);
    SmartDashboard.putNumber("ySpeed", ySpeed);
    SmartDashboard.putNumber("thetaSpeed", thetaSpeed);
    SmartDashboard.putNumber("StartX", robotPose.getX());
  //  SmartDashboard.putNumber("XGoal", (m_Vision.generateDistanceXToObject(m_ObjectToTarget))-XOffset);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return goalReached();
  }

 
  private void resetController(){
    var providedPose = m_providedPose.get();
    xController.reset(providedPose.getX());
    yController.reset(providedPose.getY());
    thetaController.reset(providedPose.getRotation().getRadians());
  }

   boolean goalReached(){

    
    return (xController.atGoal() && yController.atGoal() && thetaController.atGoal());
  }

  public void TestMethod(){
    LinkedList<Pose2d> list = new LinkedList<>();
    list.add(m_goalPose);
    list.get(0);

    Pose2d test = new Pose2d(new Translation2d(3, 3), new Rotation2d(0));

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
    
    Factory testFactory = new Factory(test, test, new LinkedList<WaypointActions>(0, new Move1()));



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
    
  }


}
