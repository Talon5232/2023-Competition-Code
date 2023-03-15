package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class intakeSub extends SubsystemBase {
    private boolean keepIntakeOn = false;
    private boolean keepOutakeOn = false; 
    private final TalonSRX intakeMotor;
    public intakeSub(){
        setName("Intake");
        intakeMotor = new TalonSRX(10);
            
    }
    


    public void intakein(){
        intakeMotor.set(TalonSRXControlMode.PercentOutput, 1);
        
    }
    public void intakeout(){
        intakeMotor.set(TalonSRXControlMode.PercentOutput, -1);
       
    }
    public void intakeStop(){
        intakeMotor.set(TalonSRXControlMode.PercentOutput, 0);

    }
    public void AutoIntakeIn(){
        keepIntakeOn = true;
    }
    public void AutoIntakeOut(){
        keepOutakeOn = true;
    }
    public void AutoIntakeOff(){
        keepIntakeOn = false;
        keepOutakeOn = false;
    }

@Override
public void periodic(){
    if(keepIntakeOn == true)
    {
        intakeMotor.set(TalonSRXControlMode.PercentOutput, 1);
    }
    if(keepOutakeOn == true){
        intakeMotor.set(TalonSRXControlMode.PercentOutput, -1);
    }
    if(keepIntakeOn == false){
        intakeMotor.set(TalonSRXControlMode.PercentOutput, 0);
    }
}
}

