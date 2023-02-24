package frc.robot.subsystems;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class liftSub extends SubsystemBase {
    double kP = .055;
    double kD = .035;
    double error;
    double lastError;
    double currentPosition;
    double correction;
    double derivitive;
    boolean ManualUp;
    boolean ManualDown;
    double setpoint;
    double liftencoder1;
    double liftencoder2;
   private CANSparkMax liftMotor1;
    private CANSparkMax liftMotor2;
   public liftSub(){
    setName("Lift");
    //right motor
        liftMotor1 = new CANSparkMax(55, MotorType.kBrushless);
        //left motor
        liftMotor2 = new CANSparkMax(56, MotorType.kBrushless);

   }
  
   // liftEncoder = new 
   
    public void liftUp(){
    //    liftMotor1.set(-.3);
       setpoint = 95;
      // liftMotor2.set(.3);
    }
    public void liftMiddle(){
        setpoint = 64;
    }
    public void liftDown(){
   //    liftMotor1.set(.3);
        setpoint = 1;
    //   liftMotor2.set(-.3);
    }
    public void liftUpManual(){
        ManualUp = true;
    }
    public void liftDownManual(){
        ManualDown = true;
    }
    public void liftUpManualStop(){
        ManualUp = false;
    }
    public void liftDownManualStop(){
        ManualDown = false;
    }

    public void liftStop(){
    //    liftMotor1.set(0);
        //liftMotor2.set(0);
    }
    @Override
public void periodic(){
    if(ManualUp == true){
        setpoint = setpoint + 0.2;
    }
    if(ManualDown == true){
        setpoint = setpoint - 0.2;
    }

   // liftMotor2.getEncoder().setPosition(0);
    double liftencoder2 = liftMotor2.getEncoder().getPosition();
    double liftencoder1 = liftMotor1.getEncoder().getPosition();
   // double liftencoder3 = liftMotor1

    SmartDashboard.putNumber("lf2", liftencoder2);
    SmartDashboard.putNumber("lf1", liftencoder1);
    SmartDashboard.putNumber("SetpointLift", setpoint);
   
    SmartDashboard.putNumber("correction", correction);
    SmartDashboard.putNumber("leftLiftMotor", liftMotor2.get());
    SmartDashboard.putNumber("rightLiftMotor", liftMotor1.get());

    
    currentPosition = liftencoder2;
    if(setpoint <= 1){
        setpoint = 1;
    }
    if(setpoint >= 95){
        setpoint = 95;
    }

        //Porportional Math
        error = setpoint - currentPosition;
        //Derivitive Math
        derivitive = error - lastError;
        correction = (error * kP) + (derivitive * kD);
        lastError = error;
        if(correction >= .4){
            correction = .4;
        }
        if(correction <= -.4){
            correction = -.4;
        }
  liftMotor1.set(-correction);
  liftMotor2.set(correction);
}
    }
    
