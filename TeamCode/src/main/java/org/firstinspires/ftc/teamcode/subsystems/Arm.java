package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.ARM_AUTO_END_V3PO;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.ARM_AUTO_END_R2V2;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.ARM_AUTO_INTAKE_WAYPOINT_V3PO;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.ARM_AUTO_INTAKE_WAYPOINT_R2V2;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.ARM_DELIVERY_V3PO;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.ARM_DELIVERY_DROP_V3PO;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.ARM_DELIVERY_R2V2;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.ARM_INIT_V3PO;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.ARM_INIT_R2V2;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.ARM_INTAKE_V3PO;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.ARM_INTAKE_R2V2;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.ARM_TRAVEL_V3PO;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.ARM_TRAVEL_R2V2;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.isV3PO;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.BotPositions;

@Config
public class Arm extends SubsystemBase {
    private Servo servo;
    private Servo servo2;
    public double servoPosition;

    private double servoOffset = 1.02;

    public Arm(HardwareMap hardwareMap) {
        if(isV3PO) {
            servo = hardwareMap.get(Servo.class, "sAR");
            servo2 = hardwareMap.get(Servo.class, "sAL");
        }else{
            servo = hardwareMap.get(Servo.class, "sA");
        }
        toInitPosition();
    }

    @Override
    public void periodic() {

    }

    public void increasePosition() {
//        servoPosition += 0.005;
//        servoPosition = (Range.clip(servoPosition, 0, 1));
//        servo.setPosition(servoPosition);
    }

    public void decreasePosition() {
//        servoPosition -= 0.005;
//        servoPosition = (Range.clip(servoPosition, 0, 1));
//        servo.setPosition(servoPosition);
    }

    public void toInitPosition() {
        servoPosition = isV3PO ? ARM_INIT_V3PO : ARM_INIT_R2V2;
        if(isV3PO){
            servo.setPosition(ARM_INIT_V3PO);
            servo2.setPosition(servoOffset-ARM_INIT_V3PO);
        }else{
            servo.setPosition(ARM_INIT_R2V2);
        }
    }

    public void toIntakePosition() {
        servoPosition = isV3PO ? ARM_INTAKE_V3PO : ARM_INTAKE_R2V2;
        if(isV3PO){
            servo.setPosition(ARM_INTAKE_V3PO);
            servo2.setPosition(servoOffset-ARM_INTAKE_V3PO);
        }else{
            servo.setPosition(ARM_INTAKE_R2V2);
        }
    }

    public void toIntakeWaypointPosition() {
        servoPosition = isV3PO ? ARM_AUTO_INTAKE_WAYPOINT_V3PO : ARM_AUTO_INTAKE_WAYPOINT_R2V2;
        if(isV3PO){
            servo.setPosition(ARM_AUTO_INTAKE_WAYPOINT_V3PO);
            servo2.setPosition(servoOffset-ARM_AUTO_INTAKE_WAYPOINT_V3PO);
        }else{
            servo.setPosition(ARM_AUTO_INTAKE_WAYPOINT_R2V2);
        }
    }

    public void toDeliverPosition() {
        servoPosition = isV3PO ? ARM_DELIVERY_V3PO : ARM_DELIVERY_R2V2;
        if(isV3PO){
            servo.setPosition(ARM_DELIVERY_V3PO);
            servo2.setPosition(servoOffset-ARM_DELIVERY_V3PO);
        }else{
            servo.setPosition(ARM_DELIVERY_R2V2);
        }
    }
    public void toDeliverDropPosition() {
        servoPosition = isV3PO ? ARM_DELIVERY_DROP_V3PO : ARM_DELIVERY_R2V2;
        if(isV3PO){
            servo.setPosition(ARM_DELIVERY_DROP_V3PO);
            servo2.setPosition(servoOffset-ARM_DELIVERY_DROP_V3PO);
        }else{
            servo.setPosition(ARM_DELIVERY_R2V2);
        }
    }

    public void toTravelPosition() {
        servoPosition = isV3PO ? ARM_TRAVEL_V3PO : ARM_TRAVEL_R2V2;
        if(isV3PO){
            servo.setPosition(ARM_TRAVEL_V3PO);
            servo2.setPosition(servoOffset-ARM_TRAVEL_V3PO);
        }else{
            servo.setPosition(ARM_TRAVEL_R2V2);
        }
    }

    public void toAutoEndPosition() {
        servoPosition = isV3PO ? ARM_AUTO_END_V3PO : ARM_AUTO_END_R2V2;
        if(isV3PO){
            servo.setPosition(ARM_AUTO_END_V3PO);
            servo2.setPosition(servoOffset-ARM_AUTO_END_V3PO);
        }else{
            servo.setPosition(ARM_AUTO_END_R2V2);
        }
    }

    public double getArmPosition() {
        return servo.getPosition();
    }
}