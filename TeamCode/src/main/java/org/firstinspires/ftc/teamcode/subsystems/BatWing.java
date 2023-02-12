package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.BATWING_DEPLOYED_LOW_JUNCTION_R2V2;
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

    public static double DEPLOYED_POSITION = BotPositions.BATWING_DEPLOYED_R2V2, DEPLOYED_LOW_JUNCTION_POSITION = BATWING_DEPLOYED_LOW_JUNCTION_R2V2, RETRACTED_POSITION = BotPositions.BATWING_RETRACTED_R2V2, STORAGE_POSITION = BotPositions.BATWING_STORAGE_R2V2;

    public BatWing(HardwareMap hardwareMap) {
        if (!isBarney) {
            servo = hardwareMap.get(Servo.class, "sBW");
            colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorPole");
            storage();
        }

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }


    public boolean atPole(){
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

    public double getDistance(){
        if (!isBarney) return ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
        return 0;
    }

    public void deployed() {
        if (!isBarney) servo.setPosition(DEPLOYED_POSITION);
    } //this is deployed to align to a pole/junction --> parallel to the ground
    public void deployedLowJunction() {
        if (!isBarney) servo.setPosition(BATWING_DEPLOYED_LOW_JUNCTION_R2V2);
    } //this is deployed to align to a low junction --> parallel to the ground

    public void retract() {
        if(!isBarney) servo.setPosition(RETRACTED_POSITION);
    } //used after dropping a cone --> retracts DOWN

    public void storage() {
        if(!isBarney) servo.setPosition(STORAGE_POSITION);
    } //used while driving around --> retracts UP

    public double getBatWingPosition() {
        return servo.getPosition();
    }
}