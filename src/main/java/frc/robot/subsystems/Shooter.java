// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */

  private CANSparkMax shooterMotorOne;
  private CANSparkMax shooterMotorTwo;
  private SparkMaxPIDController m_pidControllerOne;
  private RelativeEncoder m_encoderOne;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, kRPM;

  public void Start(double speed) {
    runShooterAtRpm(2000);
    // shooterMotorOne.set(speed / -100);
    // shooterMotorTwo.set(speed / 100 * -1);
  }

  public void Stop() {
    shooterMotorOne.set(0);
    shooterMotorTwo.set(0);
  }

  public Shooter() {
    shooterMotorOne = new CANSparkMax(ShooterConstants.ShooterMotorOneId, MotorType.kBrushless);
    shooterMotorOne.setInverted(true);
    shooterMotorTwo = new CANSparkMax(ShooterConstants.ShooterMotorTwoId, MotorType.kBrushless);
    // shooterMotorOne.restoreFactoryDefaults();
    // shooterMotorTwo.restoreFactoryDefaults();

    shooterMotorOne.setIdleMode(IdleMode.kCoast);
    shooterMotorTwo.setIdleMode(IdleMode.kCoast);
    shooterMotorTwo.follow(shooterMotorOne, true);

    m_pidControllerOne = shooterMotorOne.getPIDController();

    // Encoder object created to display position values
    m_encoderOne = shooterMotorOne.getEncoder();

    // PID coefficients
    kP = 0.00025;
    kI = 0.00000000;
    kD = 0;
    kIz = 0;
    kFF = 0.000200;
    kMaxOutput = 1;
    kMinOutput = -1;
    kRPM = 1000;

    // set PID coefficients
    m_pidControllerOne.setP(kP);
    m_pidControllerOne.setI(kI);
    m_pidControllerOne.setD(kD);
    m_pidControllerOne.setIZone(kIz);
    m_pidControllerOne.setFF(kFF);
    m_pidControllerOne.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Shooter RPM", kRPM);
  }

  @Override
  public void periodic() {
  }

  public void runShooterAtRpm(double rpm) {
    // This method will be called once per scheduler run
    // read PID coefficients from SmartDashboard
    // double p = SmartDashboard.getNumber("P Gain", 0);
    // double i = SmartDashboard.getNumber("I Gain", 0);
    // double d = SmartDashboard.getNumber("D Gain", 0);
    // double iz = SmartDashboard.getNumber("I Zone", 0);
    // double ff = SmartDashboard.getNumber("Feed Forward", 0);
    // double max = SmartDashboard.getNumber("Max Output", 0);
    // double min = SmartDashboard.getNumber("Min Output", 0);
    // double rpmFromDashboard = SmartDashboard.getNumber("Shooter RPM", 0);

    // // if PID coefficients on SmartDashboard have changed, write new values to
    // // controller
    // if ((p != kP)) {
    //   m_pidControllerOne.setP(p);
    //   kP = p;
    // }
    // if ((i != kI)) {
    //   m_pidControllerOne.setI(i);
    //   kI = i;
    // }
    // if ((d != kD)) {
    //   m_pidControllerOne.setD(d);
    //   kD = d;
    // }
    // if ((iz != kIz)) {
    //   m_pidControllerOne.setIZone(iz);
    //   kIz = iz;
    // }
    // if ((ff != kFF)) {
    //   m_pidControllerOne.setFF(ff);
    //   kFF = ff;
    // }
    // if ((max != kMaxOutput) || (min != kMinOutput)) {
    //   m_pidControllerOne.setOutputRange(min, max);
    //   kMinOutput = min;
    //   kMaxOutput = max;
    // }

    // if (rpmFromDashboard != rpm) {
    //   rpm = rpmFromDashboard;
    // }

    if (rpm < 500) {
      rpm = 500;
    }

    // Set the speed of the shooter wheel
    m_pidControllerOne.setReference(rpm, CANSparkMax.ControlType.kVelocity);

    SmartDashboard.putNumber("Shooter RPM", rpm);
    SmartDashboard.putNumber("ProcessVariable", m_encoderOne.getVelocity());
  }

  public boolean isWheelUpToSpeed(double desiredSpeed) {
    double rpm = m_encoderOne.getVelocity();
    if (rpm > (desiredSpeed * .90) && rpm < (desiredSpeed * 1.1)) {
        SmartDashboard.putBoolean("Shooter Wheel Up To Speed", true);
        return true;
    }
    SmartDashboard.putBoolean("Shooter Wheel Up To Speed", false);
    return false;
  }
}
