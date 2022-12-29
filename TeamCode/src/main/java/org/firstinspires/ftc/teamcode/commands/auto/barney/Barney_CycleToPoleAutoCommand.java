package org.firstinspires.ftc.teamcode.commands.auto.barney;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.LiftToScoringPositionCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive_Barney;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.Junctions;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;

public class Barney_CycleToPoleAutoCommand extends SequentialCommandGroup {
    FtcDashboard dashboard = FtcDashboard.getInstance();

    private Gripper gripper;

    public Barney_CycleToPoleAutoCommand(SampleMecanumDrive_Barney drive, Lift lift, Arm arm, Wrist wrist, Gripper gripper) {
        this.gripper = gripper;

        addCommands(
//                new WaitCommand(1500),
//                new LiftToPositionCommand(lift, 400, 50),
                new WaitCommand(250),
                new ParallelCommandGroup(
                        new Barney_FollowTrajectoryCommand(drive, Barney_AutoTrajectories.blue_StackWaypointToMainPole),
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                new LiftToScoringPositionCommand(lift, arm, gripper, wrist, Junctions.HIGH_JUNCTION),
                                new WaitCommand(1500),
                                new InstantCommand(gripper::open)
                        )
                )
        );
    }

    @Override
    public boolean isFinished() {
//        return Gripper.getGripperPosition() == Gripper.OPEN_POSITION;
        return super.isFinished();
    }
}
