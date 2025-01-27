package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Drive.DriveSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class DriveCommand extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DriveSubsystem m_driveSubsystem;
    private final Double m_xVelocityMps;
    private final Double m_yVelocityMps;
    private final Double m_rotationVelocityRps;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveCommand(DriveSubsystem driveSubsystem, DoubleSupplier xValue, DoubleSupplier yValue, DoubleSupplier rotationValue) {
    this.m_driveSubsystem = driveSubsystem;
    this.m_xVelocityMps = xValue.getAsDouble();
    this.m_yVelocityMps = yValue.getAsDouble();
    this.m_rotationVelocityRps = rotationValue.getAsDouble();

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
    m_driveSubsystem.setModules(correctJoystickDrift(m_xVelocityMps), correctJoystickDrift(m_yVelocityMps), correctJoystickDrift(m_rotationVelocityRps));
  }

  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
