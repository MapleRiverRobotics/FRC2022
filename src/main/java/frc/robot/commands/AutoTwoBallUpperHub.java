package frc.robot.commands;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.TrajectoryHelper;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AutoTwoBallUpperHub  extends SequentialCommandGroup  {

    public AutoTwoBallUpperHub(Drivetrain drivetrain, Shooter shooter, Indexer indexer, Intake intake) {

        Trajectory t1 = TrajectoryHelper.loadTrajectoryFromFile("DriveToSideCargo");

        addCommands(
            new InstantCommand(() -> {
                drivetrain.resetOdometry(t1.getInitialPose());
              }),
              new IntakeLift(intake, 0).withTimeout(.7),
              new ParallelDeadlineGroup(
                TrajectoryHelper.getDriveStraightCommand(drivetrain, 40).withTimeout(3).withName("Forward"),
                new IntakeRun(intake, 1)
              ),
              new DriveRotateInPlace(drivetrain, 180).withTimeout(3).withName("Turn180"),
              TrajectoryHelper.getDriveStraightCommand(drivetrain, -40).withTimeout(3).withName("Reverse"),
              new AimAndShoot(shooter, drivetrain, indexer).withTimeout(6)
        );
    }
}
