// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

  private CANSparkMax m_intakeMotor;

  DoubleSolenoid intakeLift = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
      IntakeConstants.IntakeUpValve, IntakeConstants.IntakeDownValve);

  public Intake() {
    m_intakeMotor = new CANSparkMax(IntakeConstants.IntakeMotorOneId, MotorType.kBrushless);
    m_intakeMotor.restoreFactoryDefaults();
    m_intakeMotor.setIdleMode(IdleMode.kCoast);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void Start(int direction, double speed) {
    m_intakeMotor.set(speed * direction);
  }

  public void Stop() {
    m_intakeMotor.set(0);
  }

  public void up(){
    intakeLift.set(Value.kForward);
  }

  public void down(){
    intakeLift.set(Value.kReverse);
  }
}
