package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class intakeSub extends SubsystemBase {
    private int keepIntakeOn = 0;
    private int keepOutakeOn = 0; 
    private final TalonSRX intakeMotor;
    public intakeSub(){
        setName("Intake");
        intakeMotor = new TalonSRX(10);
            
    }
    


    public void intakein(){
        intakeMotor.set(TalonSRXControlMode.PercentOutput, -1);
        keepIntakeOn = 2;
        keepOutakeOn = 2;
    }
    public void intakeout(){
        intakeMotor.set(TalonSRXControlMode.PercentOutput, 1);
        keepIntakeOn = 2;
        keepOutakeOn = 2;
       
    }
    public void intakeStop(){
        intakeMotor.set(TalonSRXControlMode.PercentOutput, 0);
        keepIntakeOn = 2;
        keepOutakeOn = 2;

    }
    public void AutoIntakeIn(){
        keepIntakeOn = 1;
    }
    public void AutoIntakeOut(){
        keepOutakeOn = 3;
    }
    public void AutoIntakeOff(){
        keepIntakeOn = 0;
        keepOutakeOn = 0;
    }

@Override
public void periodic(){

    if(keepIntakeOn == 1)
    {
        intakeMotor.set(TalonSRXControlMode.PercentOutput, -1);
    }
    if(keepOutakeOn == 3){
        intakeMotor.set(TalonSRXControlMode.PercentOutput, 1);
    }
    if(keepIntakeOn == 0 && keepOutakeOn == 0){
        intakeMotor.set(TalonSRXControlMode.PercentOutput, 0);
    }
    if(keepIntakeOn == 2){

    }
}
}

