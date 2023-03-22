//***********************************************************************************
//
//  MoveArm Class - Move the robot arm to a predefined position.
//
//***********************************************************************************
package frc.robot.subsystems;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveArm extends CommandBase {

  private final ArmSubsystem m_armSubsystem;
  private final int m_pos;
  private final double m_setpoint;

  public void moveArm( int pos, ArmSubsystem subsystem) {

    m_armSubsystem = subsystem;   // arm subsystem
    m_pos = pos;                  // input position mode (0 = lower, 1 = middle, 2 = upper)

    switch (m_pos) {
      case 0:                     // lower position
        m_setpoint = -7.0;
	break;
      case 1:                     // midle position
        m_setpoint = -12.0;
	break;
      case 2:                     // upper position
        m_setpoint = -14.0;
	break;
      defalt:                     // invalid position mode
	return;
    }

  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    m_armSubsystem.set(m_setpoint);    // set new position
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    if (Math.abs(m_armSubsystem.getEncoder().getPosition()) - m_setpoint > 0.0) {
      return false;
    } else {
      return true;
    }
  }

}

