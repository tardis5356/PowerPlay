package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.ARM_DELIVERY_V3PO;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.BEACON_ARM_DELIVERY_R2V2;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.BEACON_ARM_LOAD_V3PO;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.BEACON_ARM_LOAD_R2V2;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.BEACON_ARM_STORAGE_V3PO;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.BEACON_ARM_STORAGE_R2V2;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.isV3PO;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.subsystems.BotPositions;

@Config
public class BeaconArm extends SubsystemBase {
    //private final Servo servo;

    public boolean loading = true;

    public BeaconArm(HardwareMap hardwareMap){
        //servo = hardwareMap.get(Servo.class, "sBA");
        toStoragePosition();
    }

    @Override
    public void periodic(){
        //happens every loop
    }


    public void toLoadingPosition(){
        //servo.setPosition(isV3PO ? BEACON_ARM_LOAD_V3PO : BEACON_ARM_LOAD_R2V2);
        loading = true;
    }

    public void toDeliveryPosition(){
        //servo.setPosition(isV3PO ? ARM_DELIVERY_V3PO : BEACON_ARM_DELIVERY_R2V2);
        loading = false;
    }

    public void toStoragePosition(){
       // servo.setPosition(isV3PO ? BEACON_ARM_STORAGE_V3PO : BEACON_ARM_STORAGE_R2V2);
        loading = false;
    }

//    public void toTravelPosition(){
//        servo.setPosition(travelPosition);
//        loading = false;
//    }

    public void setPosition(double position){
       // servo.setPosition(position);
    }

    //public double getBeaconArmPosition()

        //return servo.getPosition();
    }

