package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class intakeSub extends SubsystemBase {

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
@Override
public void periodic(){
}
}
