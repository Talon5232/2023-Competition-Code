// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
  public enum ObjectToTarget {
    NONE,
    FLOOR_CONE,
    SUBSTATION_CONE,
    REFLECTIVE_TAPE,
    APRIL_TAG;

  }

  private double x_, y_, target_area_, pipeline_,ConeX,ConeY = 0;
  private ObjectToTarget ObjectTarget = ObjectToTarget.FLOOR_CONE;

  private boolean has_target_;
  /** Creates a new Vision. */
  private final NetworkTable m_limelight = NetworkTableInstance.getDefault().getTable("limelight");

  @Override
  public void periodic() {
    updateVisionData();
    upadateSmartDashBoard();
    updatePipeline();
    generateDistanceXToObject();
    generateDistanceYToObject();
    // m_limelight.getEntry("CamMode").setNumber(0);

  }

  // #region updaters
  public void updateVisionData() {
    this.x_ = this.m_limelight.getEntry("tx").getDouble(0);
    this.y_ = this.m_limelight.getEntry("ty").getDouble(0);
    this.target_area_ = this.m_limelight.getEntry("ta").getDouble(0);
    this.has_target_ = this.m_limelight.getEntry("tv").getBoolean(false);

  }

  public void upadateSmartDashBoard() {
    SmartDashboard.putNumber("X offset", getX());
    SmartDashboard.putNumber("Y Offset", getY());
    SmartDashboard.putNumber("X Actual", this.x_);
    SmartDashboard.putBoolean("Has Target", this.has_target_);
    SmartDashboard.putNumber("Target Area", this.target_area_);
    SmartDashboard.putNumber("DistanceXToObject", generateDistanceXToObject());
    SmartDashboard.putNumber("DistnaceYToTarget", generateDistanceYToObject());
    SmartDashboard.putString("Object Tracking", getObject().toString());
    SmartDashboard.putNumber("Conex", ConeX);
    SmartDashboard.putNumber("ConeY", ConeY);

  }

  public void updatePipeline() {
    switch (this.ObjectTarget) {
      case APRIL_TAG: {
        this.m_limelight.getEntry("pipeline").setNumber(0);
      }
        break;
      case FLOOR_CONE: {
        this.m_limelight.getEntry("pipeline").setNumber(2);
      }
        break;
      case REFLECTIVE_TAPE: {
        this.m_limelight.getEntry("pipeline").setNumber(1);
      }
        break;
      case SUBSTATION_CONE: {
        this.m_limelight.getEntry("pipeline").setNumber(2);
      }
        break;
      default: {
        this.m_limelight.getEntry("pipeline").setNumber(0);
        System.out.println("Object is none in Vision.updatePipeline");
      }
        break;

    }
  }
  // #endregionS

  // #region Get's
  public ObjectToTarget getObject() {
    return this.ObjectTarget;
  }

  public double getPipeline() {
    return this.pipeline_;
  }

  public double getX() {
    switch (this.ObjectTarget) {
      case APRIL_TAG: {
        if (this.x_ >= 0) {
          return this.x_ + 2.725;
        } else {
          return this.x_ - 2.725;
        }
      }
      case REFLECTIVE_TAPE: {
        return Math.abs(this.x_) - 4;
      }
      case FLOOR_CONE: {
        return Math.abs(this.x_) - 4;
      }
      case SUBSTATION_CONE: {
        return Math.abs(this.x_) - 4;
      }
      default: {
        System.out.println("Defaulting in Vision.GetX");
        return (0);
      }
    }

  }

  public double getXRaw() {
    return this.x_;
  }

  public double getY() {
    switch (this.ObjectTarget) {
      case APRIL_TAG: {
        return Math.abs(this.y_);
      }
      case REFLECTIVE_TAPE: {
        return Math.abs(this.y_);
      }
      case FLOOR_CONE: {
        return Math.abs(this.y_) -2.5 /*1.3*/;
      }
      case SUBSTATION_CONE: {
        return Math.abs(this.y_) +1.3;
      }
      default: {
        System.out.println("Defaulting in Vision.Gety");
        return (0);
      }

    }

  }

  public double getArea() {
    return this.target_area_;
  }

  public double getConeX(){
    return this.ConeX;
  }
  public double getConeY(){
    return this.ConeY;
  }


  public boolean getTargets() {
    return this.has_target_;
  }

  private double getObjectHeight() {
    switch (this.ObjectTarget) {
      case APRIL_TAG: {
        return Constants.FieldConstants.LOW_APRIL_TAG;
      }
      case FLOOR_CONE: {
        return Constants.FieldConstants.FLOOR_CONE_HEIGHT;

      }
      case REFLECTIVE_TAPE: {
        return Constants.FieldConstants.LOW_REFLECTIVE_TAPE;
      }
      case SUBSTATION_CONE: {
        return Constants.FieldConstants.PLAYER_STATION_CONE_HEIGHT;
      }
      default: {
        System.out.println("Defaulting in Vision.getObjectHeight");
        return (0);
      }
    }
  };

  // #endregion

  // #region Set'
  public void setObject(ObjectToTarget object) {
    this.ObjectTarget = object;
    updatePipeline();
  }

  private void setPipeline(int pipe) {
    this.pipeline_ = pipe;
    this.m_limelight.getEntry("pipeline").setNumber(pipeline_);
  }

  public void setConeX(double coneX){
    this.ConeX = coneX; 
  }

  public void setConeY(double coneY){
   this.ConeY = coneY;
  }

  // #endregion

  // #region Builders/Generators
  public double generateDistanceXToObject() {
    if (this.ObjectTarget != ObjectToTarget.NONE) {
      SmartDashboard.putNumber("HeightOb", getObjectHeight());
      SmartDashboard.putNumber("kCameraHeight", Constants.VisionConstants.kCameraHeight);
      SmartDashboard.putNumber("CameraRoations", Constants.VisionConstants.kCameraRotation);
      SmartDashboard.putNumber("GetYCamera", getY());
      SmartDashboard.putNumber("tan(Y)", Math.tan(Units.degreesToRadians(getY())));

      return (Math.abs((getObjectHeight() - Constants.VisionConstants.kCameraHeight))
          / (Math.tan(Constants.VisionConstants.kCameraRotation + Units.degreesToRadians(getY()))));
    }

    else {
      return 0;
    }
  }

  public double generateDistanceYToObject() {
    if (this.ObjectTarget != ObjectToTarget.NONE) {
      return ((Math.tan(Units.degreesToRadians(getX()))) * generateDistanceXToObject());
    } else {
      return 0;
    }
  }

  /*
   * Low Cones, April-Tags, and Reflective Tape we can virtually be right next to
   * Substation cones our closest measurement may be up to 7 feet away. Does the
   * want to do something about this?
   * #TODO: Add offsets -- x for arm -- y for camera (This can also be done in
   * limelightlocal) -- and ANY OTHER offsets you may need.
   * #TODO: Adding get-set for objects might be easier for you to work with!
   * #TODO: Late night me thinks resetOdo may be your ticket to starting field
   * relative
   * #TODO: Usage in Teleop? <-- Driveteam question
   */
  public Translation2d generate2dPositionToObject() {
    switch (this.ObjectTarget) {
      case APRIL_TAG: {
        // Infront of the drop area
        return new Translation2d(generateDistanceXToObject(), generateDistanceYToObject());
      }
      case FLOOR_CONE: {

        // Grabbing floor cone drive
        return new Translation2d(generateDistanceXToObject(), generateDistanceYToObject());

      }
      case REFLECTIVE_TAPE: {

        return new Translation2d(generateDistanceXToObject(), generateDistanceYToObject());
      }
      case SUBSTATION_CONE: {

        return new Translation2d(generateDistanceXToObject(), generateDistanceYToObject());
      }
      default: {
        System.out.println("Defaulting in Vision.gen2DPos");
        return new Translation2d(0, 0);
      }
    }

  }

}
/*
 * IMO you and Noah should pull up onshape and build all the important field
 * elements in FieldConstants -- stuff you dont want to run into during auto
 * Putting offsets there
 * You should find exact measurements -- for red side should be mirroed just
 * 16.5-x for said red
 * Charge Station estimation blue (meters)
 * y 4.25
 * x 2.6m 5.2m
 * x: 2.6 -> 5.2 //////////////
 * y: 4.25 / /
 * / /
 * / /
 * / /
 * / /
 * / /
 * //////////////
 * y 1.25
 * 
 * 
 * If you dont want to use Swerve's drive method for movement and want to stick
 * with path planner for all auto check out
 * https://github.com/mjansen4857/pathplanner/wiki/PathPlannerLib:-Java-Usage
 * "On-the-fly-generation"
 * Otherwise Swerve.drive() or trajecGen like you originally used.
 * 
 * https://firstfrc.blob.core.windows.net/frc2023/FieldAssets/
 * 2023LayoutMarkingDiagram.pdf
 * https://firstfrc.blob.core.windows.net/frc2023/FieldAssets/2023FieldDrawings-
 * CHARGEDUPSpecific.pdf
 * https://www.chiefdelphi.com/t/dynamic-on-the-fly-path-generation-in-teleop-
 * video/423818 More of a summer project but A* is very much something that will
 * be used in Comp Sci
 * Id implement the A* star in something like python first to see how it works
 * then translate it to Java if you guys ever get to this point.
 * Never thought about it but using A* for auto is kinda genius -- but also
 * major sweat lord moments( in the case of custom implementation) and so many
 * things to improve before then
 * 
 */
// #endregion
