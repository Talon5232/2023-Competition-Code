package frc.robot.subsystems;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class blinkin extends SubsystemBase {

	static Spark blinkin;

	public blinkin() {
		blinkin = new Spark(0);
	}
	public static void lightsNormal() {
        boolean isRed = NetworkTableInstance.getDefault().getTable("FMSInfo").getEntry("IsRedAlliance").getBoolean(true);
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

}
