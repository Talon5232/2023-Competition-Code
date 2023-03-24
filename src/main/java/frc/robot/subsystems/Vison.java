// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vison extends SubsystemBase {
  private double x_, y_, angle_, target_area_;
  private boolean has_target_;
  /** Creates a new Vison. */
  private final NetworkTable m_limelight = NetworkTableInstance.getDefault().getTable("limelight");

  @Override
  public void periodic() {
    this.x_ = this.m_limelight.getEntry("tx").getDouble(0);
    this.y_ = this.m_limelight.getEntry("ty").getDouble(0);
    this.target_area_ = this.m_limelight.getEntry("ta").getDouble(0);
    this.has_target_ = this.m_limelight.getEntry("tv").getBoolean(false);

    SmartDashboard.putNumber("X offset", this.x_);
    SmartDashboard.putNumber("Y Offset", this.y_);
    SmartDashboard.putBoolean("Has Target", this.has_target_);
    SmartDashboard.putNumber("Target Area", this.target_area_);

    // This method will be called once per scheduler run
  }

  public double[] getVisionDoubles(){
    return new double[]{0};
  } 

}
