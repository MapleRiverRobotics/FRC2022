package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

/**
 *
 */
public class Drive extends CommandBase {

    private final Drivetrain m_drivetrain;

    public Drive(Drivetrain subsystem) {

        m_drivetrain = subsystem;
        addRequirements(m_drivetrain);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double throttleSpeed = RobotContainer.oi.getJoystickDriveThrottleSpeed();
        double forwardSpeed = RobotContainer.oi.getJoystickDriveForwardSpeed() * throttleSpeed;
        double rotation = RobotContainer.oi.getJoystickDriveRotation() * throttleSpeed;

        m_drivetrain.arcadeDrive(forwardSpeed, rotation);
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
