package org.firstinspires.ftc.teamcode.commands.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.LiftToIntakePositionCommand;
import org.firstinspires.ftc.teamcode.commands.LiftToScoringPositionCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive_Barney;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.Junctions;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;

public class BarneyCycleToPoleAutoCommand extends ParallelCommandGroup {

    private SampleMecanumDrive_Barney drive;
    private Lift Lift;
    private Arm Arm;
    private Wrist Wrist;
    private Gripper Gripper;

    private LiftToScoringPositionCommand liftRetractCommand, liftToGroundJunctionCommand, liftToLowJunctionCommand, liftToMediumJunctionCommand, liftToHighJunctionCommand;
    private LiftToIntakePositionCommand liftToIntakeCommand;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    public BarneyCycleToPoleAutoCommand(SampleMecanumDrive_Barney drive, Lift lift, Arm arm, Wrist wrist, Gripper gripper){
        addCommands(
                new FollowTrajectoryCommand(drive, BarneyAutoTrajectories.blue_StackToMainPole),
                new SequentialCommandGroup(
                        new WaitCommand(500),
                        new LiftToScoringPositionCommand(lift, arm, gripper, wrist, Junctions.HIGH_JUNCTION),
                        new WaitCommand(500)
//                        new InstantCommand(Gripper::open)
                )
        );
    }

    @Override
    public boolean isFinished() {
//        return Gripper.getGripperPosition() == Gripper.OPEN_POSITION;
        return super.isFinished();
    }
}
