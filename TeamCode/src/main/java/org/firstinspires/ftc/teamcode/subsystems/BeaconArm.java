package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class BeaconArm extends SubsystemBase {
    private final Servo servo;

    private final double LOAD_POSITION = 0.99, SCORE_POSITION = 0.43;

    public BeaconArm(HardwareMap hardwareMap){
        servo = hardwareMap.get(Servo.class, "sBA");
        loadingPosition();
    }

    @Override
    public void periodic(){
        //happens every loop
    }

    public void loadingPosition(){
        servo.setPosition(LOAD_POSITION);
    }

    public void scoringPosition(){
        servo.setPosition(SCORE_POSITION);
    }

}