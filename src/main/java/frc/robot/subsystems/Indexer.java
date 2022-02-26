// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class Indexer extends SubsystemBase {
  private WPI_TalonSRX m_indexerMotor;

  public Indexer() {
    m_indexerMotor = new WPI_TalonSRX(IndexerConstants.IndexerMotorOneId);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void Start(int direction) {
    m_indexerMotor.set(80 * direction);
  }

  public void Stop() {
    m_indexerMotor.set(0);
  }
}
