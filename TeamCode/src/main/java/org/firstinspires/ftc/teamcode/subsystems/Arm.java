package org.firstinspires.ftc.teamcode.subsystems;

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

@Config
public class Arm extends SubsystemBase {
    private Servo servo;

    public static double  NEUTRAL_POSITION = 0.5, INTAKE_POSITION = 0.3, DELIVERY_POSITION = 0.6;

    public Arm(HardwareMap hardwareMap){
        servo = hardwareMap.get(Servo.class, "sA");
//        toNeutralPosition();
    }

    @Override
    public void periodic(){

    }

    public void toNeutralPosition(){
        servo.setPosition(NEUTRAL_POSITION);
    }

    public void toIntakePosition(){
        servo.setPosition(INTAKE_POSITION);
    }

    public void toDeliverPosition(){
        servo.setPosition(DELIVERY_POSITION);
    }

    public double getArmPosition() {
        return servo.getPosition();
    }
}