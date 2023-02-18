package frc.robot.subsystems;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class armSub extends SubsystemBase {
    double kP = .01;
    double kD = .01;
    double error;
    double lastError;
    double currentPosition;
    double setpoint;
    double correction;
    private final CANSparkMax armMotor;
    private double armEncoder;
    double derivitive;
    public armSub(){
        setName("Arm");
        armMotor = new CANSparkMax(62, MotorType.kBrushless);
        armEncoder = armMotor.getEncoder().getPosition();
        
        setpoint = setpoint + 1;
            

            
    }
   
    
   // liftEncoder = new 
    public void armMovement(double setpoint){
        
        currentPosition = armEncoder;
        //Porportional Math
        error = setpoint - currentPosition;
        //Derivitive Math
        derivitive = error - lastError;
        correction = (error * kP) + (derivitive * kD);
        armMotor.set(correction);
    }
    public void armUp(){
        armMotor.set(.5);
        setpoint = 5;
    }
    public void armDown(){
       armMotor.set(-.5);
       setpoint = -5;
    }
    public void armStop(){
        armMotor.set(0);
    }
@Override
public void periodic(){
    SmartDashboard.putNumber("Motor Speed", armMotor.get());
    SmartDashboard.putNumber("setpoint", setpoint);
    armEncoder = armMotor.getEncoder().getPosition();
    SmartDashboard.putNumber("ArmEncoder", armEncoder);
    SmartDashboard.updateValues();
}
}
