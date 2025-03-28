package frc.robot.commands.TestingCommands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.TestArmSubsystem;


public class MoveArmJoystick extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final TestArmSubsystem m_subsystem;
  private DoubleSupplier armSupplier;

  /**
   * Creates a new MovePulleyWithJoystick.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveArmJoystick(TestArmSubsystem subsystem, DoubleSupplier armSupplier) {
    m_subsystem = subsystem;
    this.armSupplier = armSupplier;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    double arm = armSupplier.getAsDouble() * Constants.TestArm.kSpeedDegressPerSecond * Constants.kRobotLoopTime;
    m_subsystem.move(arm);

  }

  @Override
  public void execute() {

  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return true;
 }
}
