package frc.robot.commands.TestingCommands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.TestElevatorSubsystem;


public class MoveElevatorJoystick extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final TestElevatorSubsystem m_subsystem;
  private DoubleSupplier elevatorSupplier;

  /**
   * Creates a new MovePulleyWithJoystick.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveElevatorJoystick(TestElevatorSubsystem subsystem, DoubleSupplier elevatorSupplier) {
    m_subsystem = subsystem;
    this.elevatorSupplier = elevatorSupplier;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    double elevator = elevatorSupplier.getAsDouble() * Constants.TestElevator.kSpeedMetersPerSecond * Constants.kRobotLoopTime;
    m_subsystem.move(elevator);

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
