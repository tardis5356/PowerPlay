package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.BotPositions;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.teleop.Gen2_TeleOp_FC;

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
        Gen2_TeleOp_FC.activeStackHeight += iterateBy;
        if(Gen2_TeleOp_FC.activeStackHeight > 4) Gen2_TeleOp_FC.activeStackHeight = 1;
        if(Gen2_TeleOp_FC.activeStackHeight < 1) Gen2_TeleOp_FC.activeStackHeight = 4;
        lift.setTargetPosition(BotPositions.STACK_POSITIONS_R2V2[Gen2_TeleOp_FC.activeStackHeight]);
    }

    @Override
    public boolean isFinished() { // returns true when finished
        if(lift.getLiftTargetPosition() == BotPositions.STACK_POSITIONS_R2V2[Gen2_TeleOp_FC.activeStackHeight]) return true;
        return false;
    }
}