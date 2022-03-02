package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class AutonomousDriveCommand extends CommandBase {

    private final Drivetrain m_drivetrain;
    private final int m_direction;

    public AutonomousDriveCommand(Drivetrain drivetrain, int direction) {
        m_drivetrain = drivetrain;
        m_direction = direction;
        addRequirements(m_drivetrain);    
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_drivetrain.arcadeDrive(.3 * m_direction, 0);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}
