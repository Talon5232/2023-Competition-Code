/*package frc.robot.subsystems;
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
   private CANSparkMax liftMotor1 = new CANSparkMax(111, MotorType.kBrushless);
    private CANSparkMax liftMotor2 = new CANSparkMax(111, MotorType.kBrushless);
   private double liftencoder1 = liftMotor1.getEncoder().getPosition();
   private double liftencoder2 = liftMotor2.getEncoder().getPosition();

   // liftEncoder = new 
    public void liftMovement(double setpoint){
        setName("Lift");
        //SmartDashboard.putNumber("Encoder", liftencoder2);
        currentPosition = (liftencoder1 + liftencoder2)/2;
        //Porportional Math
        error = setpoint - currentPosition;
        //Derivitive Math
        derivitive = error - lastError;
        correction = (error * kP) + (derivitive * kD);
        lastError = error;
    }
   
    }
    */
