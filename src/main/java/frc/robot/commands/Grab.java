// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class Grab extends CommandBase {
  /** Creates a new ShooterShoot. */

  private final Climber m_climber;

  public Grab(Climber subsystem) {

    m_climber = subsystem;
    addRequirements(m_climber);

  }

  public void GrabBar(int barnumber) {
    if (barnumber == 1 || barnumber == 3) {
      m_climber.MediumTraverseGrab();
    } else if (barnumber == 2) {
      m_climber.HighGrab();
    } 

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    GrabBar(1);
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
