// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class Shoot extends Command {
  /** Creates a new ShooterShoot. */
  private final Shooter m_shooter;
  private final Indexer m_indexer;
  public final double m_rpm;

  public Shoot(Shooter shooter, Indexer indexer, double rpm) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_rpm = rpm;
    m_shooter = shooter;
    m_indexer = indexer;
    addRequirements(m_shooter);
    addRequirements(m_indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double calculatedRpm = (m_rpm - 1500) / 100.0 * RobotContainer.oi.getJoystickDriveThrottleSpeed() * 100 + 1500;

    //double calculatedRpm = m_rpm * RobotContainer.oi.getJoystickDriveThrottleSpeed();

    m_shooter.runShooterAtRpm(calculatedRpm);
    if (m_shooter.isWheelUpToSpeed(calculatedRpm)) {
        Timer.delay(0.125);
        m_indexer.Start(-1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.Stop();
    m_indexer.Stop();
    ;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
