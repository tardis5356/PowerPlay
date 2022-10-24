package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Drivetrain extends SubsystemBase {
    private DcMotorEx mFR, mFL, mBR, mBL;



    public Drivetrain(HardwareMap hardwareMap) {
        // Motors and Other Stuff
        mFR = hardwareMap.get(DcMotorEx .class, "mFR");
        mFL = hardwareMap.get(DcMotorEx.class, "mFL ");
        mBR = hardwareMap.get(DcMotorEx.class, "mBR");
        mBL = hardwareMap.get(DcMotorEx.class, "mBL");
        // Behaviors
        mFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mFL.setDirection(DcMotorSimple.Direction.REVERSE);
        mBL.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void open(){
//        servo.setPosition(openPosition);
    }

//    public void close(){
//        servo.setPosition(closePosition);
//    }
}