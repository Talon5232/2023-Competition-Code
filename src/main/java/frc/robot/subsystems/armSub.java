package frc.robot.subsystems;
import java.util.ConcurrentModificationException;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class armSub extends SubsystemBase {
    double kP = .03;
    double kD = .03;
    double error;
    double lastError;
    double currentPosition;
    double correction;
    double setpoint;
    double test;
    private final CANSparkMax armMotor;
    private double armEncoder;
    double derivitive;
    public armSub(){
        setName("Arm");
        
        armMotor = new CANSparkMax(62, MotorType.kBrushless);
        armEncoder = armMotor.getEncoder().getPosition();

        

        //armMotor.set(correction);
    }
   
    
   // liftEncoder = new 
    public void armMovement(double setpoint){
        /*
        currentPosition = armEncoder;
        //Porportional Math
        error = setpoint - currentPosition;
        //Derivitive Math
        derivitive = error - lastError;
        correction = (error * kP) + (derivitive * kD);
        armMotor.set(correction);
        */
    }
    public void armUp(){
        setpoint = -8;
    }
    public void armMiddle(){
        setpoint = -5;
    }
   
    public void armDown(){
       setpoint = -2;
    }
   
@Override
public void periodic(){
    if(setpoint >= -1){
        setpoint = -1;
    }
    currentPosition = armEncoder;
    //Porportional Math
    error = setpoint - currentPosition;
    //Derivitive Math
    derivitive = error - lastError;
    lastError = error;
    correction = (error * kP) + (derivitive * kD);
    armMotor.set(correction);


    SmartDashboard.putNumber("Motor Speed", armMotor.get());
    SmartDashboard.putNumber("setpoint", setpoint);
    SmartDashboard.putNumber("correctoin", correction);
    armEncoder = armMotor.getEncoder().getPosition();
    SmartDashboard.putNumber("ArmEncoder", armEncoder);
    SmartDashboard.putNumber("test", test);
    SmartDashboard.updateValues();


}
}
