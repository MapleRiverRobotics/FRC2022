// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class Release extends CommandBase {
  /** Creates a new ShooterShoot. */

  private final Climber m_climber;
  private final int m_barNumber;

  public Release(Climber subsystem, int barnumber) {

    m_climber = subsystem;
    m_barNumber = barnumber;
    addRequirements(m_climber);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_barNumber == 1 || m_barNumber == 3) {
      m_climber.MediumTraverseRelease();
    } else if (m_barNumber == 2) {
      m_climber.HighRelease();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.Stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
