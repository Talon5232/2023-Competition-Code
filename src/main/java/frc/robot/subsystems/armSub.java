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
    double kP = .05;
    double kD = .035;
    double error;
    double lastError;
    double currentPosition;
    double correction;
    double setpoint;
    double test;
    boolean ManualUp;
    boolean ManualDown;
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
        setpoint = -14;
    }
    public void armMiddle(){
        setpoint = -12;
    }
    public void armDown(){
       setpoint = -7;
    }
    public void armAuto(){
        setpoint = -7;

    }
    public void armVeryDown(){
        setpoint = -4;
    }
    public void armUpManual(){
        ManualUp = true;
    }
    public void armDownManual(){
        ManualDown = true;
    }
    public void armUpManualStop(){
        ManualUp = false;
    }
    public void armDownManualStop(){
        ManualDown = false;
    }
   
@Override
public void periodic(){

    if(ManualDown == true){
        setpoint = setpoint + .04;
    }
    if(ManualUp == true){
        setpoint = setpoint - .04;
    }
    if(setpoint >= -.5){
        setpoint = -.5;
    }
    if(setpoint <= -30){
        setpoint = -30;
    }
    currentPosition = armEncoder;
    //Porportional Math
    error = setpoint - currentPosition;
    //Derivitive Math
    derivitive = error - lastError;
    lastError = error;
    correction = (error * kP) + (derivitive * kD);
    if(correction >= .4){
        correction = .4;
    }
    if(correction <= -.4){
        correction = -.4;
    }
    armMotor.set(correction);


   // SmartDashboard.putNumber("Motor Speed", armMotor.get());
    SmartDashboard.putNumber("setpoint", setpoint);
   // SmartDashboard.putNumber("correctoin", correction);
    armEncoder = armMotor.getEncoder().getPosition();
    SmartDashboard.putNumber("ArmEncoder", armEncoder);
    //SmartDashboard.putNumber("test", test);
    SmartDashboard.updateValues();


}
}
