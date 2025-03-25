package frc.robot.commands.TestingCommands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TestSubsystem;


public class ControlTestMotorWithJoystick extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final TestSubsystem m_subsystem;
  private DoubleSupplier joystickSupplier;

  /**
   * Creates a new MovePulleyWithJoystick.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ControlTestMotorWithJoystick(TestSubsystem subsystem, DoubleSupplier joystickSupplier) {
    m_subsystem = subsystem;
    this.joystickSupplier = joystickSupplier;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    m_subsystem.move(joystickSupplier.getAsDouble() * 3);
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
