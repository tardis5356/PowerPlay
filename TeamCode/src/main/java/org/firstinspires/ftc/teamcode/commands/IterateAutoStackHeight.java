package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.BatWing;
import org.firstinspires.ftc.teamcode.subsystems.BotPositions;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.teleop.Gen2_TeleOp;

public class IterateAutoStackHeight extends CommandBase {
    private int activeStackHeight;
    private int iterateBy = 0;

    private Lift lift;

    public IterateAutoStackHeight(Lift lift, int activeStackHeight, int iterateBy) {
        this.lift = lift;
        this.activeStackHeight = activeStackHeight;
        this.iterateBy = iterateBy;
    }
    @Override
    public void initialize() { // runs once
        Gen2_TeleOp.activeStackHeight += iterateBy;
        if(Gen2_TeleOp.activeStackHeight > 4) Gen2_TeleOp.activeStackHeight = 1;
        if(Gen2_TeleOp.activeStackHeight < 1) Gen2_TeleOp.activeStackHeight = 4;
        lift.setTargetPosition(BotPositions.STACK_POSITIONS_R2V2[Gen2_TeleOp.activeStackHeight]);
    }

    @Override
    public boolean isFinished() { // returns true when finished
        if(lift.getLiftTargetPosition() == BotPositions.STACK_POSITIONS_R2V2[Gen2_TeleOp.activeStackHeight]) return true;
        return false;
    }
}