package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.subsystems.BotPositions;

public class BeaconArm extends SubsystemBase {
    private final Servo servo;

//    private final double loadPosition = BotPositions.BEACON_ARM_LOAD, deliveryPosition = BotPositions.BEACON_ARM_DELIVERY, storagePosition = BotPositions.BEACON_ARM_STORAGE; //guess, not tested
    private final double loadPosition = BotPositions.BEACON_ARM_LOAD_R2V2, deliveryPosition = BotPositions.BEACON_ARM_DELIVERY_R2V2, storagePosition = BotPositions.BEACON_ARM_STORAGE_R2V2; //guess, not tested

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