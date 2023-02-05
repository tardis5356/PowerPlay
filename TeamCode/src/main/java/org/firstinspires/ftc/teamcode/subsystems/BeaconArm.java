package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.ARM_DELIVERY_Barney;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.BEACON_ARM_DELIVERY_R2V2;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.BEACON_ARM_LOAD_Barney;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.BEACON_ARM_LOAD_R2V2;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.BEACON_ARM_STORAGE_Barney;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.BEACON_ARM_STORAGE_R2V2;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.isBarney;

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
        //servo.setPosition(isBarney ? BEACON_ARM_LOAD_Barney : BEACON_ARM_LOAD_R2V2);
        loading = true;
    }

    public void toDeliveryPosition(){
        //servo.setPosition(isBarney ? ARM_DELIVERY_Barney : BEACON_ARM_DELIVERY_R2V2);
        loading = false;
    }

    public void toStoragePosition(){
       // servo.setPosition(isBarney ? BEACON_ARM_STORAGE_Barney : BEACON_ARM_STORAGE_R2V2);
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

