// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ClimberConstants.Arm;
import edu.wpi.first.wpilibj.DigitalInput;

public class Climber extends SubsystemBase {

  private final CANSparkMax m_motorLeft;
  private final CANSparkMax m_motorRight;
  private final SparkMaxPIDController PidControllerLeft;
  private final SparkMaxPIDController PidControllerRight;
  private final RelativeEncoder m_encoderLeft;
  // private RelativeEncoder m_encoderRight;
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  private double rotations;

  private final double gearboxGearRatio = 48.0;
  private final double chainGearRatio = 4.0;
  private final double gearRatio = gearboxGearRatio * chainGearRatio;

  private final DoubleSolenoid mediumTraverseValve = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
      ClimberConstants.MediumTraverseGrabId, ClimberConstants.MediumTraverseReleaseId);
  private final DoubleSolenoid highValve = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, ClimberConstants.HighValveGrabId,
      ClimberConstants.HighValveReleaseId);
  private final Servo brakeServoRight = new Servo(ClimberConstants.BreakServoOneId);
  private final Servo brakeServoLeft = new Servo(ClimberConstants.BreakServoTwoId);

  private final DigitalInput firstBarRightLimitSwitch, secondBarRightLimitSwitch, thirdBarRightLimitSwitch,
      firstBarLeftLimitSwitch, secondBarLeftLimitSwitch, thirdBarLeftLimitSwitch;

  public Climber() {
    m_motorLeft = new CANSparkMax(ClimberConstants.ClimberMotorOneId, MotorType.kBrushless);
    m_motorRight = new CANSparkMax(ClimberConstants.ClimberMotorTwoId, MotorType.kBrushless);

    m_motorLeft.restoreFactoryDefaults();
    m_motorRight.restoreFactoryDefaults();

    m_motorRight.setInverted(true);

    m_motorLeft.setIdleMode(IdleMode.kBrake);
    m_motorRight.setIdleMode(IdleMode.kBrake);
    /**
     * In order to use PID functionality for a controller, a SparkMaxPIDController
     * object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    PidControllerLeft = m_motorLeft.getPIDController();
    PidControllerRight = m_motorRight.getPIDController();

    // Encoder object created to display position values
    m_encoderLeft = m_motorLeft.getEncoder();
    // m_encoderRight = m_motorRight.getEncoder();

    // m_encoderLeft.setPositionConversionFactor(1.0 / gearRatio);
    // m_encoderRight.setPositionConversionFactor(1.0 / gearRatio);

    // PID coefficients
    kP = .05; // 3e-5;
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0.000015;
    kMaxOutput = 1;
    kMinOutput = -1;

    // set PID coefficients
    PidControllerLeft.setP(kP);
    PidControllerLeft.setI(kI);
    PidControllerLeft.setD(kD);
    PidControllerLeft.setIZone(kIz);
    PidControllerLeft.setFF(kFF);
    PidControllerLeft.setOutputRange(kMinOutput, kMaxOutput);

    // set PID coefficients
    PidControllerRight.setP(kP);
    PidControllerRight.setI(kI);
    PidControllerRight.setD(kD);
    PidControllerRight.setIZone(kIz);
    PidControllerRight.setFF(kFF);
    PidControllerRight.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);

    firstBarRightLimitSwitch = new DigitalInput(ClimberConstants.firstBarRightLimitSwitch);
    secondBarRightLimitSwitch = new DigitalInput(ClimberConstants.secondBarRightLimitSwitch);
    thirdBarRightLimitSwitch = new DigitalInput(ClimberConstants.thirdBarRightLimitSwitch);
    firstBarLeftLimitSwitch = new DigitalInput(ClimberConstants.firstBarLeftLimitSwitch);
    secondBarLeftLimitSwitch = new DigitalInput(ClimberConstants.secondBarLeftLimitSwitch);
    thirdBarLeftLimitSwitch = new DigitalInput(ClimberConstants.thirdBarLeftLimitSwitch);
  }

  public boolean IsBrakeEngaged() {
    double rightAngle = brakeServoRight.getAngle();
    double leftAngle = brakeServoLeft.getAngle();
    boolean isEngaged = false;
    if (rightAngle < 160 || leftAngle > 20) {
          isEngaged = true;
    }

    SmartDashboard.putBoolean("Brake engaged:", isEngaged);
    return isEngaged;
  }

  public void EngageBrake() {
    brakeServoRight.setAngle(90);
    SmartDashboard.putNumber("Brake Right Servo Angle", brakeServoRight.getAngle());
    brakeServoLeft.setAngle(90);
    SmartDashboard.putNumber("Brake Left Servo Angle", brakeServoLeft.getAngle());
  }

  public void DisengageBrake() {
    brakeServoRight.setAngle(180);
    SmartDashboard.putNumber("Brake Right Servo Angle", brakeServoRight.getAngle());
    brakeServoLeft.setAngle(0);
    SmartDashboard.putNumber("Brake Left Servo Angle", brakeServoLeft.getAngle());
  }

  public void MediumTraverseRelease() {
    mediumTraverseValve.set(Value.kReverse);
  }

  public void MediumTraverseGrab() {
    mediumTraverseValve.set(Value.kForward);
  }

  public void HighRelease() {
    highValve.set(Value.kReverse);
  }

  public void HighGrab() {
    highValve.set(Value.kForward);
  }

  public double GetArmAngle() {
    return m_encoderLeft.getPosition() / 360.0;
  }

  public void SetArmAngle(double angle, Arm arm) {
    rotations = (96.0 / 360.0) * angle;

    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to
    // controller
    if ((p != kP)) {
      PidControllerLeft.setP(p);
      PidControllerRight.setP(p);
      kP = p;
    }
    if ((i != kI)) {
      PidControllerLeft.setI(i);
      PidControllerRight.setI(i);
      kI = i;
    }
    if ((d != kD)) {
      PidControllerLeft.setD(d);
      PidControllerRight.setD(d);
      kD = d;
    }
    if ((iz != kIz)) {
      PidControllerLeft.setIZone(iz);
      PidControllerRight.setIZone(iz);
      kIz = iz;
    }
    if ((ff != kFF)) {
      PidControllerLeft.setFF(ff);
      PidControllerRight.setFF(ff);
      kFF = ff;
    }
    if ((max != kMaxOutput) || (min != kMinOutput)) {
      PidControllerLeft.setOutputRange(min, max);
      PidControllerRight.setOutputRange(min, max);
      kMinOutput = min;
      kMaxOutput = max;
    }

    /**
     * PIDController objects are commanded to a set point using the
     * SetReference() method.
     */

    if (arm == Arm.Left) {
      PidControllerLeft.setReference(rotations, CANSparkMax.ControlType.kPosition);
    } else {
      PidControllerRight.setReference(rotations, CANSparkMax.ControlType.kPosition);
    }

    SmartDashboard.putNumber("Rotations", rotations);
    SmartDashboard.putNumber("Arm Angle", m_encoderLeft.getPosition() / gearRatio * 360);
    SmartDashboard.putNumber("ProcessVariable", m_encoderLeft.getPosition());
  }

  public void Start(int direction, Arm arm) {
    double speed = .5 * direction;

    // Safety mechanism to ensure we don't strip the brakes while going the wrong direction
    if (direction > 0 && IsBrakeEngaged()) {
      DisengageBrake();
      Timer.delay(.5);
    }

    if (arm == Arm.Left) {
      m_motorLeft.set(speed);
    } else if (arm == Arm.Right) {
      m_motorRight.set(speed);
    } else {
      m_motorLeft.set(speed);
      m_motorRight.set(speed);
    }
  }

  public void Stop() {
    highValve.set(Value.kOff);
    mediumTraverseValve.set(Value.kOff);
  }

  public void Stop(Arm arm) {

    if (arm == Arm.Left) {
      m_motorLeft.set(0.0);
    } else if (arm == Arm.Right) {
      m_motorRight.set(0.0);
    } else {
      m_motorLeft.set(0.0);
      m_motorRight.set(0.0);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Limit switch methods
  public boolean IsFirstBarRightLimitSwitchPressed() {
    return firstBarRightLimitSwitch.get() == true;
  }
  public boolean IsFirstBarLeftLimitSwitchPressed() {
    return firstBarLeftLimitSwitch.get() == true;
  }
  public boolean IsSecondBarRightLimitSwitchPressed() {
    return secondBarRightLimitSwitch.get() == true;
  }
  public boolean IsSecondBarLeftLimitSwitchPressed() {
    return secondBarLeftLimitSwitch.get() == true;
  }
  public boolean IsThirdBarRightLimitSwitchPressed() {
    return thirdBarRightLimitSwitch.get() == true;
  }
  public boolean IsThirdBarLeftLimitSwitchPressed() {
    return thirdBarLeftLimitSwitch.get() == true;
  }
}
