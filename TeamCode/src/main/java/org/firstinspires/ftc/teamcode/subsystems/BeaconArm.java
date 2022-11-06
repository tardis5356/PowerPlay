package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class BeaconArm extends SubsystemBase {
    private final Servo servo;

    private final double loadPosition = 0.99;
    private final double scoringPosition = 0.43;

    public boolean loading = true;

    public BeaconArm(HardwareMap hardwareMap){
        servo = hardwareMap.get(Servo.class, "sBeaconArm");
        loadingPosition();
    }

    @Override
    public void periodic(){
        //happens every loop
    }


    public void loadingPosition(){
        servo.setPosition(loadPosition);
        loading = true;
    }

    public void scoringPosition(){
        servo.setPosition(scoringPosition);
        loading = false;
    }

    public void setPosition(double position){
        servo.setPosition(position);
    }

}