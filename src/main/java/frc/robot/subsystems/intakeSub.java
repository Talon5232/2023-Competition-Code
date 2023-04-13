package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//#TODO: This stuff should get cleaned up, and LIMIT voltage/current/amps/etc going to the SRX
public class intakeSub extends SubsystemBase {
    private int keepIntakeOn = 0;
    private int keepOutakeOn = 0;
    private final TalonSRX intakeMotor;

    public intakeSub() {
        setName("Intake");
        intakeMotor = new TalonSRX(0);

    }

    public static final int intakeinContinuouscurrentlimit = 25;
    public static final int OutakeContinuousCurrentLimit = 25;

    public void intakein() {
        intakeMotor.set(TalonSRXControlMode.PercentOutput, -0.7);
        keepIntakeOn = 2;
        keepOutakeOn = 2;
    }

    public void intakeout() {
        intakeMotor.set(TalonSRXControlMode.PercentOutput, 0.7);
        keepIntakeOn = 2;
        keepOutakeOn = 2;

    }

    public void intakeStop() {
        intakeMotor.set(TalonSRXControlMode.PercentOutput, 0);
        keepIntakeOn = 2;
        keepOutakeOn = 2;

    }

    public void AutoIntakeIn() {
        keepIntakeOn = 1;
    }

    public void AutoIntakeOut() {
        keepOutakeOn = 1;
    }

    public void AutoIntakeOff() {
        keepIntakeOn = 0;
        keepOutakeOn = 0;
    }

    @Override
    public void periodic() {

        if (keepIntakeOn == 1) {
            intakeMotor.set(TalonSRXControlMode.PercentOutput, -0.7);
        }
        if (keepOutakeOn == 1) {
            intakeMotor.set(TalonSRXControlMode.PercentOutput, 0.7);
        }
        if (keepIntakeOn == 0 && keepOutakeOn == 0) {
            intakeMotor.set(TalonSRXControlMode.PercentOutput, 0.00);
        }
        if (keepIntakeOn == 2) {

        }
    }
}
