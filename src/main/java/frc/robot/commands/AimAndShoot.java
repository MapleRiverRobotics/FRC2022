package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

import java.lang.Math;

public class AimAndShoot extends CommandBase {
    /**
     * Creates a new AimAndShoot.
     */
    double KpAim = -0.050f;
    // double KpDistance = 0.1f;
    // double min_aim_command = 0.05f;

    double minSpeed = 0.3;
    double minDegreeOffset = 2;

    double tx;
    double ty;
    double tv;
    private final Shooter m_shooter;
    private final Drivetrain m_drivetrain;
    private final Indexer m_indexer;
    private final NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

    public AimAndShoot(Shooter shooter, Drivetrain drivetrain, Indexer indexer) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);
        addRequirements(shooter);
        addRequirements(indexer);
        m_shooter = shooter;
        m_drivetrain = drivetrain;
        m_indexer = indexer;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        limelight.getEntry("ledMode").setNumber(3);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        SmartDashboard.putBoolean("AimAndShoot End Called with interrupted:", false);
        tx = limelight.getEntry("tx").getDouble(0);
        ty = limelight.getEntry("ty").getDouble(0);
        tv = limelight.getEntry("tv").getDouble(0);

        SmartDashboard.putNumber("tx", tx);
        SmartDashboard.putNumber("ty", ty);
        SmartDashboard.putNumber("tv", tv);

        double turnSpeed = KpAim * tx;

        SmartDashboard.putNumber("Caluclated Turnspeed", turnSpeed);

        if (0 < turnSpeed && turnSpeed < minSpeed) {
            turnSpeed = minSpeed;
        } else if (0 > turnSpeed && turnSpeed > -minSpeed) {
            turnSpeed = -minSpeed;
        }

        if (tx > minDegreeOffset || tx < -minDegreeOffset) {
            m_drivetrain.tankDrive(turnSpeed, 0);
            return;
        }

        m_drivetrain.tankDrive(0, 0);
        if ((tx > -minDegreeOffset && tx < minDegreeOffset) && tx != 0) {
            double rpm = GetShooterRpm();
            m_shooter.runShooterAtRpm(rpm);
            if (m_shooter.isWheelUpToSpeed(rpm)) {
                Timer.delay(0.5);
                m_indexer.Start(-1);
            }
        }
    }

    private double GetShooterRpm() {
        // d = (h2-h1) / tan(a1+a2)
        double limelightMountAngleDegrees = 18; // angle of camera
        double limelightHeightInches = 27.75; // height of camera
        double goalHeightInches = 103.0; // height of target

        double targetOffsetAngle_Vertical = ty;

        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        double distanceFromLimelightToGoalInches = (goalHeightInches - limelightHeightInches) / Math.tan(angleToGoalRadians);
 
        // rpm = 3000rpm at 9 feet. Add 93.75 for ever foot beyond 9 feet
        double rpm = ((distanceFromLimelightToGoalInches - 108) / 12 * 93.75) + 2600;

        SmartDashboard.putNumber("Caluclated Distance", distanceFromLimelightToGoalInches);
        SmartDashboard.putNumber("Calculated RPM", rpm);

        return rpm;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drivetrain.stop();;
        m_shooter.Stop();
        m_indexer.Stop();
        //limelight.getEntry("ledMode").setNumber(1);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
