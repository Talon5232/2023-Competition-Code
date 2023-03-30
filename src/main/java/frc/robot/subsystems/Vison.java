// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

enum ObjectToTarget{
  NONE,
  FLOOR_CONE,
  SUBSTATION_CONE,
  REFLECTIVE_TAPE,
  APRIL_TAG
}

public class Vison extends SubsystemBase {
  private double x_, y_, target_area_, pipeline_ = 0;
  private boolean has_target_;
  private ObjectToTarget object_to_target_ = ObjectToTarget.NONE;
  /** Creates a new Vison. */
  private final NetworkTable m_limelight = NetworkTableInstance.getDefault().getTable("limelight");

  @Override
  public void periodic() {
    updateVisionData();
    upadateSmartDashBoard();
    updatePipeline();
    // This method will be called once per scheduler run
  }

  //#region upadters
  public void updateVisionData() {
    this.x_ = this.m_limelight.getEntry("tx").getDouble(0);
    this.y_ = this.m_limelight.getEntry("ty").getDouble(0);
    this.target_area_ = this.m_limelight.getEntry("ta").getDouble(0);
    this.has_target_ = this.m_limelight.getEntry("tv").getBoolean(false);
  }

  public void upadateSmartDashBoard() {
    SmartDashboard.putNumber("X offset", this.x_);
    SmartDashboard.putNumber("Y Offset", this.y_);
    SmartDashboard.putBoolean("Has Target", this.has_target_);
    SmartDashboard.putNumber("Target Area", this.target_area_);
  }

  public void updatePipeline() {
    this.m_limelight.getEntry("pipeline").setNumber(this.pipeline_);
  }
//#endregionS

  // #region Get's
  public double getPipeline() {
    return this.pipeline_;
  }

  public double getX() {
    return this.x_;
  }

  public double getY() {
    return this.y_;
  }

  public double getArea() {
    return this.target_area_;
  }

  public boolean getTargets() {
    return this.has_target_;
  }
  // #endregion

  // #region Set's
  public void setPipeline(int pipe) {
    // if(pipe >= 0 && pipe <= 9){}
    this.pipeline_ = pipe;
    this.m_limelight.getEntry("pipeline").setNumber(pipe);
  }

  // #endregion

  //#region Builders/Generators
public void generateDistanceToObject(ObjectToTarget objectToTarget){
 switch(objectToTarget){
  case NONE:
  {}
  case FLOOR_CONE: {}
  case SUBSTATION_CONE: {}
  case APRIL_TAG: {}
  case REFLECTIVE_TAPE: {}
 } 
}


  //#endregion

}
