// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

/**
 *
 */
public class Drivetrain extends SubsystemBase {

    private MotorControllerGroup leftMotors;
    private MotorControllerGroup rightMotors;
    private CANSparkMax leftMaster;
    private CANSparkMax leftSlave;
    private CANSparkMax rightSlave;
    private CANSparkMax rightMaster;
    private final DifferentialDrive differentialDrive;

    /**
    *
    */
    public Drivetrain() {

        rightMaster = new CANSparkMax(DriveConstants.RightMasterMotorId, MotorType.kBrushless);
        rightMaster.restoreFactoryDefaults();
        rightMaster.setInverted(false);
        rightMaster.setIdleMode(IdleMode.kBrake);
        rightMaster.burnFlash();

        rightSlave = new CANSparkMax(DriveConstants.RightSlaveMototId, MotorType.kBrushless);
        rightSlave.restoreFactoryDefaults();
        rightSlave.follow(rightMaster);
        rightSlave.setInverted(false);
        rightSlave.setIdleMode(IdleMode.kBrake);
        rightSlave.burnFlash();

        leftMaster = new CANSparkMax(DriveConstants.LeftMasterMotorId, MotorType.kBrushless);
        leftMaster.restoreFactoryDefaults();
        leftMaster.setInverted(true);
        leftMaster.setIdleMode(IdleMode.kBrake);
        leftMaster.burnFlash();

        leftSlave = new CANSparkMax(DriveConstants.LeftSlaveMotorId, MotorType.kBrushless);
        leftSlave.restoreFactoryDefaults();
        leftSlave.follow(leftMaster);
        leftSlave.setInverted(true);
        leftSlave.setIdleMode(IdleMode.kBrake);
        leftSlave.burnFlash();

        //leftMotors = new MotorControllerGroup(leftMaster, leftSlave);
        //rightMotors = new MotorControllerGroup(rightMaster, rightSlave);

        differentialDrive = new DifferentialDrive(leftMaster, rightMaster);
        // differentialDrive = new DifferentialDrive(leftMotors, rightMotors);

        WPI_TalonSRX talon = new WPI_TalonSRX(Constants.IndexerConstants.IndexerMotorOneId); 
        WPI_PigeonIMU gyro = new WPI_PigeonIMU(talon); // Pigeon uses the talon created above

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    public void arcadeDrive(final double speed, final double rotation) {
        differentialDrive.arcadeDrive(speed, rotation);
    }
    public void tankDrive(final double leftSpeed, final double rightSpeed) {
        differentialDrive.tankDrive(leftSpeed, rightSpeed);
    }

    public void stop() {
        differentialDrive.arcadeDrive(0, 0);
    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

}