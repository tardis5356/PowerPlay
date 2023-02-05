package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.isBarney;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


@Config
public class BatWing extends SubsystemBase {
    private Servo servo;

    public static double DEPLOYED_POSITION = BotPositions.BATWING_DEPLOYED_R2V2, RETRACTED_POSITION = BotPositions.BATWING_RETRACTED_R2V2, STORAGE_POSITION = BotPositions.BATWING_STORAGE_R2V2;

    public BatWing(HardwareMap hardwareMap) {
        if (!isBarney) {
            servo = hardwareMap.get(Servo.class, "sBW");
            storage();
        }

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void deployed() {
        if (!isBarney) servo.setPosition(DEPLOYED_POSITION);
    } //this is deployed to align to a pole/junction --> parallel to the ground

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