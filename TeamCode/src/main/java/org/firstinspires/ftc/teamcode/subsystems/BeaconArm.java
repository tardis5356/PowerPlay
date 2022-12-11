package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class BeaconArm extends SubsystemBase {
    private final Servo servo;

    private final double loadPosition = 0.99;
    private final double deliveryPosition = 0.43;
    private final double storagePosition = 0.5; //guess, not tested

    public boolean loading = true;

    public BeaconArm(HardwareMap hardwareMap){
        servo = hardwareMap.get(Servo.class, "sBA");
        toStoragePosition();
    }

    @Override
    public void periodic(){
        //happens every loop
    }


    public void toLoadingPosition(){
        servo.setPosition(loadPosition);
        loading = true;
    }

    public void toDeliveryPosition(){
        servo.setPosition(deliveryPosition);
        loading = false;
    }

    public void toStoragePosition(){
        servo.setPosition(storagePosition);
        loading = false;
    }

    public void setPosition(double position){
        servo.setPosition(position);
    }

    public double getBeaconArmPosition() {
        return servo.getPosition();
    }

}