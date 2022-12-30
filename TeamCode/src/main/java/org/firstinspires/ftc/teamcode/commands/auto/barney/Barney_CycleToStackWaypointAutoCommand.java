package org.firstinspires.ftc.teamcode.commands.auto.barney;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.LiftToPositionCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive_Barney;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;

public class Barney_CycleToStackWaypointAutoCommand extends ParallelCommandGroup {
    FtcDashboard dashboard = FtcDashboard.getInstance();

    private Gripper gripper;

    public Barney_CycleToStackWaypointAutoCommand(SampleMecanumDrive_Barney drive, Lift lift, Arm arm, Wrist wrist, Gripper gripper, int stackIndex, boolean isBlue) {
        this.gripper = gripper;

        addCommands(
                new Barney_FollowTrajectoryCommand(drive, isBlue ? Barney_AutoTrajectories.blue_MainPoleToStackWaypoint : Barney_AutoTrajectories.red_MainPoleToStackWaypoint),
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                new LiftToPositionCommand(lift, 50, 25))

        );

    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
