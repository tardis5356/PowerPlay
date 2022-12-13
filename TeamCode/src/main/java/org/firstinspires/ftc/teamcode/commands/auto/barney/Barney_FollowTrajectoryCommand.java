package org.firstinspires.ftc.teamcode.commands.auto.barney;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive_Barney;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive_R2V2;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class Barney_FollowTrajectoryCommand extends CommandBase {
    private SampleMecanumDrive_Barney drive;
    private TrajectorySequence sequence;

    public Barney_FollowTrajectoryCommand(SampleMecanumDrive_Barney drive, TrajectorySequence sequence){
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
