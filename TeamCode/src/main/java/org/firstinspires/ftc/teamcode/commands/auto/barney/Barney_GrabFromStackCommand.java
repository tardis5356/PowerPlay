package org.firstinspires.ftc.teamcode.commands.auto.barney;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.LiftToIntakePositionCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive_Barney;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.Junctions;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;

public class Barney_GrabFromStackCommand extends SequentialCommandGroup {
    FtcDashboard dashboard = FtcDashboard.getInstance();

    private Gripper gripper;

    public Barney_GrabFromStackCommand(SampleMecanumDrive_Barney drive, Lift lift, Arm arm, Wrist wrist, Gripper gripper, int stackIndex, boolean isBlue) {
        this.gripper = gripper;

        addCommands(
                new LiftToIntakePositionCommand(lift, arm, gripper, wrist, Junctions.INTAKE, stackIndex),
                //new InstantCommand(gripper::open),
                new WaitCommand(250),
                new Barney_FollowTrajectoryCommand(drive, isBlue ? Barney_AutoTrajectories.blue_StackWaypointToStack : Barney_AutoTrajectories.red_StackWaypointToStack),
                new WaitCommand(250),
                new InstantCommand(() -> {
                    gripper.close();
                }),
               // new InstantCommand(gripper::close),
                new WaitCommand(250),
                new ParallelCommandGroup(
                        new Barney_FollowTrajectoryCommand(drive, isBlue ? Barney_AutoTrajectories.blue_StackToStackWaypoint : Barney_AutoTrajectories.red_StackToStackWaypoint),
                        new LiftToIntakePositionCommand(lift, arm, gripper, wrist, Junctions.INTAKE, 8)
                )
        );
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
