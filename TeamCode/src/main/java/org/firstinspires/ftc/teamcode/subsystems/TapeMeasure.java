package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class TapeMeasure extends SubsystemBase {
    private DcMotor dcMotor;
    private CRServo servo;

    private boolean isGen1;

    public TapeMeasure(HardwareMap hardwareMap, boolean Gen1) {
         if (Gen1) {
             servo = hardwareMap.get(CRServo.class, "sTMT");
        } else{
             dcMotor = hardwareMap.get(DcMotor.class, "sTMT");
         }
        isGen1 = Gen1;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void extend() {
        if (isGen1) {
            servo.setPower(-1);
        } else {
            dcMotor.setPower(-1);
        }
    }

    public void retract() {
        if (isGen1) {
            servo.setPower(1);
        } else {

            dcMotor.setPower(1);
        }
    }

    public void stop() {
        if (isGen1) {
            servo.setPower(0);
        } else {

            dcMotor.setPower(0);
        }
    }

    public double getTapeMeasurePower() {
        if (isGen1) {
            return servo.getPower();
        } else {
            return dcMotor.getPower();
        }

    }
}