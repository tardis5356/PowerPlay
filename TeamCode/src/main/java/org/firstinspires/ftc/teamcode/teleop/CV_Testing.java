package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Disabled
@TeleOp(name = "CV_Testing")
public class CV_Testing extends CommandOpMode {
    private DcMotorEx m1;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        m1 = hardwareMap.get(DcMotorEx.class, "m1");

        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void run() {
        super.run();

//        telemetry.addData("right odometer", mFR.getCurrentPosition());
//        telemetry.addData("back odometer", mBR.getCurrentPosition());
        telemetry.addData("left odometer", m1.getCurrentPosition());
        telemetry.update();
    }
}