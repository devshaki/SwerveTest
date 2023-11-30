package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Drive.DriveSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class DriveCommand extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DriveSubsystem m_driveSubsystem;
    private final DoubleSupplier m_xValue;
    private final DoubleSupplier m_yValue;
    private final Double m_xVelocity;
    private final Double m_yVelocity;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveCommand(DriveSubsystem driveSubsystem, DoubleSupplier xValue, DoubleSupplier yValue) {
    this.m_driveSubsystem = driveSubsystem;
    this.m_xValue = xValue;
    this.m_yValue = yValue;
    this.m_xVelocity = this.m_xValue.getAsDouble();
    this.m_yVelocity = this.m_yValue.getAsDouble();

    addRequirements(driveSubsystem);
  }

  /**
   * Eliminates the drift from the joystick input
   */
  public double correctJoystickDrift(final double input) {
    return (Math.abs(input) > Constants.OI.kXboxcontrollerDrift) ? input : 0;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() 
  {
      
  }

  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
