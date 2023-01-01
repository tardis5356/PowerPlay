package org.firstinspires.ftc.teamcode.commands.auto.barney;

import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.LIFT_INTAKE_Barney;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.RobotToStateCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive_Barney;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Coffin;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;

public class Barney_GrabFromStackCommand extends SequentialCommandGroup {
    FtcDashboard dashboard = FtcDashboard.getInstance();

    private Gripper gripper;

    public Barney_GrabFromStackCommand(SampleMecanumDrive_Barney drive, Lift lift, Arm arm, Wrist wrist, Gripper gripper, Coffin coffin, int stackIndex, boolean isBlue) {
        this.gripper = gripper;

        addCommands(
                new RobotToStateCommand(lift, arm, wrist, gripper, coffin, LIFT_INTAKE_Barney, stackIndex, "intake"),
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
                        new RobotToStateCommand(lift, arm, wrist, gripper, coffin, LIFT_INTAKE_Barney, 8, "intake")
                )
        );
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
