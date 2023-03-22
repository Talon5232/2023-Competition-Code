//***********************************************************************************
//
//  TestAuto Class - Run a sequence of commands
//
//***********************************************************************************
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TestAuto extends SequentialCommandGroup {

  /**
   * Run a squence of commands
   *
   * @param lift The lift subsystem this command will run on
   * @param arm  The arm  subsystem this command will run on
   */

  public runTestAuto (LiftSubsystem lift, ArmSubsystem arm) {
    addCommands( new moveLift(2, lift),
                 new moveArm(2, arm),
                 new moveArm(0, arm),
                 new moveLift(0, lift)
	       );
  }

}
