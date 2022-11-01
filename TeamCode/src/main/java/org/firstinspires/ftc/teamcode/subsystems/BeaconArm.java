package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
public class BeaconArm extends SubsystemBase {
    private Servo servo;

    public static double LOAD_POSITION = 0, DELIVERY_POSITION = 0.5, STORAGE_POSITION = 1;

    public BeaconArm(HardwareMap hardwareMap){
        servo = hardwareMap.get(Servo.class, "sBA");
//        toLoadingPosition();
    }

    @Override
    public void periodic(){
        //happens every loop
    }

    public void toLoadingPosition(){
        servo.setPosition(LOAD_POSITION);
    }

    public void toDeliveryPosition(){
        servo.setPosition(DELIVERY_POSITION);
    }

    public void toStoragePosition(){
        servo.setPosition(STORAGE_POSITION);
    }

}