package org.firstinspires.ftc.teamcode.commands.auto.R2V2;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive_R2V2;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class R2V2_FollowTrajectoryCommand extends CommandBase {
    private SampleMecanumDrive_R2V2 drive;
    private TrajectorySequence sequence;

    public R2V2_FollowTrajectoryCommand(SampleMecanumDrive_R2V2 drive, TrajectorySequence sequence){
        this.drive = drive;
        this.sequence = sequence;
    }

    @Override
    public void initialize(){
        drive.followTrajectorySequenceAsync(sequence);
    }

    @Override
    public void execute(){
        drive.update();
    }

    @Override
    public boolean isFinished(){
        return !drive.isBusy();
    }

}
