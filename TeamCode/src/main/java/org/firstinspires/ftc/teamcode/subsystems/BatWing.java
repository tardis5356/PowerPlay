package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.BATWING_DEPLOYED_Barney;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.BATWING_DEPLOYED_LOW_JUNCTION_Barney;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.BATWING_DEPLOYED_LOW_JUNCTION_R2V2;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.BATWING_DEPLOYED_R2V2;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.BATWING_RETRACTED_Barney;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.BATWING_RETRACTED_R2V2;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.BATWING_STORAGE_Barney;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.BATWING_STORAGE_R2V2;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.isBarney;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Config
public class BatWing extends SubsystemBase {
    private Servo servo;
    private NormalizedColorSensor colorSensor;

    public static String state = "STORAGE";

    public BatWing(HardwareMap hardwareMap) {
        if (!isBarney) {
            servo = hardwareMap.get(Servo.class, "sBW");
            colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorPole");
            storage();
        } else {
            servo = hardwareMap.get(Servo.class, "sPA");
            storage();
        }

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }


    public boolean atPole() {
        double distance = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
        boolean closed = false;
        if (distance < 4) {
            // servo.setPosition(isBarney ? GRIPPER_CLOSED_Barney : GRIPPER_CLOSED_R2V2);
            closed = true;
        } else {
            //servo.setPosition(isBarney ? GRIPPER_OPEN_Barney : GRIPPER_OPEN_R2V2);
            closed = false;
        }
        return closed;
    }

    public double getDistance() {
        if (!isBarney) return ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
        return 0;
    }

    public String getState() {
        if (!isBarney) return state;
        return "";
    }


    public void deployed() {
        if (isBarney) {
            servo.setPosition(BATWING_DEPLOYED_Barney);
        } else {
            servo.setPosition(BATWING_DEPLOYED_R2V2);
        }
        state = "DEPLOYED";
    } //this is deployed to align to a pole/junction --> parallel to the ground

    public void deployedLowJunction() {
        if (isBarney) {
            servo.setPosition(BATWING_DEPLOYED_LOW_JUNCTION_Barney);
        } else {
            servo.setPosition(BATWING_DEPLOYED_LOW_JUNCTION_R2V2);
        }
        state = "DEPLOYED";
    } //this is deployed to align to a low junction --> parallel to the ground

    public void retract() {
        if (!isBarney && state != "STORAGE") {
            if (isBarney) {
                servo.setPosition(BATWING_RETRACTED_Barney);
            } else {
                servo.setPosition(BATWING_RETRACTED_R2V2);
            }
            state = "RETRACT";
        }
    } //used after dropping a cone --> retracts DOWN

    public void storage() {
        if (isBarney) {
            servo.setPosition(BATWING_STORAGE_Barney);
        }else{
            servo.setPosition(BATWING_STORAGE_R2V2);
        }
        state = "STORAGE";
    } //used while driving around --> retracts UP

    public double getBatWingPosition() {
        return servo.getPosition();
    }
}