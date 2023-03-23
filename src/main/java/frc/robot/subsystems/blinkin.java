/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

    import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
    import edu.wpi.first.wpilibj2.command.SubsystemBase;
    import frc.robot.RobotContainer;

public class blinkin extends SubsystemBase {

	static Spark blinkin;

	public blinkin() {
		blinkin = new Spark(0);
	}
	public static void lightsNormal() {
        boolean isRed = DriverStation.getAlliance() == Alliance.Red;
        if (isRed == true){
          blinkin.set(-0.01);
          System.out.println("led RED");
        } else {
          blinkin.set(0.19);
          System.out.println("led BLUE");
	}
}
public void givecone() {
    blinkin.set(-.07); 
    
}
 /*   public static void lightsclose() {
        if (25>Ultrasonic){
            blinkin.set(0.77);
        }
    }
    public static void lightscloseish() {
        if (30>Ultrasonic>15){
            blinkin.set(0.69);
        }} */
}
