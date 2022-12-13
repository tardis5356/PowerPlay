package org.firstinspires.ftc.teamcode.commands.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.LiftToIntakePositionCommand;
import org.firstinspires.ftc.teamcode.commands.LiftToScoringPositionCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive_Barney;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive_R2V2;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.Junctions;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;

public class BarneyDeliverPreloadAutoCommand extends SequentialCommandGroup {
    private Gripper gripper;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    public BarneyDeliverPreloadAutoCommand(SampleMecanumDrive_R2V2 drive, Lift lift, Arm arm, Wrist wrist, Gripper gripper, int stackIndex) {
        this.gripper = gripper;

        addCommands(
                new ParallelCommandGroup(
                        new FollowTrajectoryCommand(drive, R2V2AutoTrajectories.blue_StartToPreloadPole)//,
//                        new SequentialCommandGroup(
//                                new WaitCommand(500),
//                                new LiftToScoringPositionCommand(lift, arm, gripper, wrist, Junctions.HIGH_JUNCTION),
//                                new WaitCommand(500)
//                        )
                ),
                new WaitCommand(1000),
//                new InstantCommand(gripper::open),
                new ParallelCommandGroup(
                        new FollowTrajectoryCommand(drive, R2V2AutoTrajectories.blue_PreloadPoleToStackWaypoint)//,
//                        new SequentialCommandGroup(
//                                new WaitCommand(250),
//                                new LiftToIntakePositionCommand(lift, arm, gripper, wrist, Junctions.INTAKE, stackIndex)
//                        ),
//                        new InstantCommand(gripper::open)
                )
        );
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
