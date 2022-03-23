// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be
 * declared globally (i.e. public static). Do not put anything functional in
 * this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // Drive system constants
    public static final class DriveConstants {
        public static final int RightMasterMotorId = 3;
        public static final int RightSlaveMototId = 4;
        public static final int LeftMasterMotorId = 1;
        public static final int LeftSlaveMotorId = 2;

    }

    // Shooter constants
    public static final class ShooterConstants {
        public static final int ShooterBaseRpm = 2800;
        public static final int ShooterMotorOneId = 6;
        public static final int ShooterMotorTwoId = 5;
    }

    // Operator Interface Constants
    public static final class OIConstants {
        public static final int DriverJoystickId = 0;
        public static final int OperatorJoystickId = 1;
    }

    // Climber Constants
    public static final class ClimberConstants {
        public static final int HighValveGrabId = 2;
        public static final int HighValveReleaseId = 3;
        public static final int MediumTraverseGrabId = 4;
        public static final int MediumTraverseReleaseId = 5;
        public static final int ClimberMotorOneId = 7;
        public static final int ClimberMotorTwoId = 8;
        public static final int firstBarRightLimitSwitch = 2;
        public static final int secondBarRightLimitSwitch = 1;
        public static final int thirdBarRightLimitSwitch = 3;
        public static final int firstBarLeftLimitSwitch = 5;
        public static final int secondBarLeftLimitSwitch = 4;
        public static final int thirdBarLeftLimitSwitch = 6;
        public static final int BrakeServoRightId = 0;
        public static final int BrakeServoLeftId = 1;

        public enum Arm {
            Left,
            Right,
            Both
        }
    }

    // Intake constants
    public static final class IntakeConstants {
        public static final int IntakeMotorOneId = 9;        
        public static final int IntakeDownValve = 0;
        public static final int IntakeUpValve = 1;
        public static final int IntakeUpButton = 8;
        public static final int IntakeDownButton = 7;
    }

    // Indexer constants
    public static final class IndexerConstants {
        public static final int IndexerMotorOneId = 10;
    }

    public static final class TrajectoryConstants {
        public static final double ksVolts = 0.21683; //0.22;
        public static final double kvVoltSecondsPerMeter = 2.6913; //1.98;
        public static final double kaVoltSecondsSquaredPerMeter = 0.15008; //0.2;
        public static final double kPDriveVel = 4.0239; //8.5;
        public static final double kTrackWidthMeters =  0.5588;
        public static final double kMaxSpeedMetersPerSecond = 1;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;
        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

        //public static final double kEncoderPulsesPerREvolution = 42;
        public static final double kGearRatio = 10.71;
        public static final double kWheelDiameterMeters = 0.1524;
        public static final double kWheelCircumfrence = Math.PI * kWheelDiameterMeters;
        public static final double kEncoderDistancePerPulse = kWheelCircumfrence / kGearRatio;

    }
}
