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

public class AutoTwoBallLowerHub  extends SequentialCommandGroup  {

    public AutoTwoBallLowerHub(Drivetrain drivetrain, Shooter shooter, Indexer indexer, Intake intake) {

        Trajectory t1 = TrajectoryHelper.loadTrajectoryFromFile("DriveToSideCargo");

        addCommands(
            new InstantCommand(() -> {
                drivetrain.resetOdometry(t1.getInitialPose());
              }),
              new IntakeLift(intake, 0).withTimeout(1),
              new ParallelDeadlineGroup(
                TrajectoryHelper.getDriveStraightCommand(drivetrain, 36).withTimeout(5).withName("Forward"),
                new IntakeRun(intake, 1)
              ),
              TrajectoryHelper.getDriveStraightCommand(drivetrain, -36).withTimeout(5).withName("Reverse"),
              new DriveRotateInPlace(drivetrain, 180).withTimeout(4).withName("Turn180"),
              new Shoot(shooter, indexer, 1800).withTimeout(4),
              TrajectoryHelper.getDriveStraightCommand(drivetrain, -56).withTimeout(5).withName("Reverse")
        );
    }
}
