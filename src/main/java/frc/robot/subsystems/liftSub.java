package frc.robot.subsystems;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class liftSub extends SubsystemBase {
    double kP = .01;
    double kD = .01;
    double error;
    double lastError;
    double currentPosition;
    double correction;
    double derivitive;
   private CANSparkMax liftMotor1;
    private CANSparkMax liftMotor2;
   public liftSub(){
    setName("Lift");
        liftMotor1 = new CANSparkMax(55, MotorType.kBrushless);
        liftMotor2 = new CANSparkMax(56, MotorType.kBrushless);
     double liftencoder1 = liftMotor1.getEncoder().getPosition();
    double liftencoder2 = liftMotor2.getEncoder().getPosition();
    SmartDashboard.putNumber("liftencoder1", liftencoder1);
   }
  
   // liftEncoder = new 
    public void liftMovement(double setpoint){
        double liftencoder1 = liftMotor1.getEncoder().getPosition();
        double liftencoder2 = liftMotor2.getEncoder().getPosition();
        //SmartDashboard.putNumber("Encoder", liftencoder2);
        currentPosition = (liftencoder1 + liftencoder2)/2;
        //Porportional Math
        error = setpoint - currentPosition;
        //Derivitive Math
        derivitive = error - lastError;
        correction = (error * kP) + (derivitive * kD);
        lastError = error;
    }
    public void liftUp(){
        liftMotor1.set(-.2);
        liftMotor1.set(.2);
    }
    public void liftDown(){
        liftMotor1.set(.2);
        liftMotor1.set(-.2);
    }
    public void liftStop(){
        liftMotor1.set(0);
        liftMotor2.set(0);
    }
    @Override
public void periodic(){

}
    }
    
