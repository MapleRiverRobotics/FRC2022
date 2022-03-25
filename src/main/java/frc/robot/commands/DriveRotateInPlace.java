package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

/**
 *
 */
public class DriveRotateInPlace extends CommandBase {

    private final Drivetrain m_drivetrain;
    private double m_targetHeading = 0;
    private final double m_degrees;

    public DriveRotateInPlace(Drivetrain subsystem, double degrees) {

        m_drivetrain = subsystem;
        m_degrees = degrees;
        addRequirements(m_drivetrain);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        double currentHeading = m_drivetrain.getHeading();
        m_targetHeading =  currentHeading + m_degrees;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double currentHeading = m_drivetrain.getHeading();
        double speed = (m_targetHeading - currentHeading) * .015;
        if (speed < .18) {
            speed = .18;
        } else if (speed > .35) {
            speed = .35;
        }
        SmartDashboard.putNumber("Gyro Heading", currentHeading);
        SmartDashboard.putNumber("Target Heading", m_targetHeading);
        SmartDashboard.putNumber("Spin speed", speed);
        //if (currentHeading < m_targetHeading * .9) {
            m_drivetrain.tankDrive(speed, -speed);
        //}
        //else if (currentHeading > m_targetHeading * .9) {
        //    m_drivetrain.tankDrive(-speed, speed);
        //} else {
        //    m_drivetrain.stop();
        //}

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drivetrain.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        double currentHeading = m_drivetrain.getHeading();
        return (currentHeading < m_targetHeading + 2  && currentHeading > m_targetHeading - 2);
    }
}
