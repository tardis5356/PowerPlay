package org.firstinspires.ftc.teamcode.teleop;

import static java.lang.Math.abs;
import static java.lang.Math.round;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.text.DecimalFormat;

@Disabled
@TeleOp(name = "Gen2 DT Enhancements")
public class Gen2_DT_Enhancements extends CommandOpMode {
    private DcMotorEx mFR, mFL, mBR, mBL;
    private BNO055IMU imu;

    private static final DecimalFormat df = new DecimalFormat("0.00");

    private double offset = 0.0;
    private double CURRENT_POWER_MULTIPLIER = 1.0;
    private double CURRENT_BASE_POWER_MULTIPLIER = 1.0;
    private double ANTI_TIP_WEIGHTED_POWER_MULTIPLIER = 0;
    private double ADAPTIVE_SPEED_POWER_MULTIPLIER_OFFSET = 0;
    private double SLOW_POWER_MULTIPLIER = 0.3;
    private double FAST_POWER_MULTIPLIER = 0.8;
    private boolean manualModeOn = false;


    public float roll = 0.001f, rollOffset = 0, measuredMaxRoll = 0;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Object declarations
        GamepadEx driver = new GamepadEx(gamepad1);
        GamepadEx manipulator = new GamepadEx(gamepad2);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        // Motors and Other Stuff
        //defineComponents
        mFR = hardwareMap.get(DcMotorEx.class, "mFR");
        mFL = hardwareMap.get(DcMotorEx.class, "mFL ");
        mBR = hardwareMap.get(DcMotorEx.class, "mBR");
        mBL = hardwareMap.get(DcMotorEx.class, "mBL");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        // Behaviors
        mFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mFL.setDirection(DcMotorSimple.Direction.REVERSE);
        mBL.setDirection(DcMotorSimple.Direction.REVERSE);

        imu.initialize(parameters);

        telemetry.addData("Ready to start!", getRuntime());
        telemetry.update();


        // driver triggers
        //driver = gamepad 1

        // FAST SLOW TOGGLE
        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5)
                .whenActive(() -> {
                    CURRENT_BASE_POWER_MULTIPLIER = SLOW_POWER_MULTIPLIER;
                });
        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5)
                .whenActive(() -> {
                    CURRENT_BASE_POWER_MULTIPLIER = FAST_POWER_MULTIPLIER;
                });

        // ADAPTIVE SPEED
//        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5)
//                .toggleWhenActive(() -> {
//                    CURRENT_BASE_POWER_MULTIPLIER = SLOW_POWER_MULTIPLIER;
//                }, () -> {
//                    CURRENT_BASE_POWER_MULTIPLIER = FAST_POWER_MULTIPLIER;
//                });
//        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1)
//                .whenActive(() -> {
//                    ADAPTIVE_SPEED_POWER_MULTIPLIER_OFFSET = driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
//                });

        new Trigger(() -> driver.getButton(GamepadKeys.Button.A))
                .whenActive(() -> {
                    offset = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
                });
    }

    @Override
    public void run() {
        super.run();
        //FIELDCENTRIC
        Orientation botOrientationRadians = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        //Add the angle offset to be able to reset the 0 heading, and normalize it back to -pi to pi
        double heading = AngleUnit.normalizeRadians(botOrientationRadians.firstAngle - offset);

        double ly = -gamepad1.left_stick_y;
        double lx = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        // Rotate by the heading of the robot
        Vector2d vector = new Vector2d(lx, ly).rotated(-heading);
        lx = vector.getX();
        ly = vector.getY();

        double normalize = Math.max(abs(ly) + abs(lx) + abs(rx), 1.0);

        mFL.setPower((ly + lx + rx) / normalize * CURRENT_POWER_MULTIPLIER);
        mBL.setPower((ly - lx + rx) / normalize * CURRENT_POWER_MULTIPLIER);
        mFR.setPower((ly - lx - rx) / normalize * CURRENT_POWER_MULTIPLIER);
        mBR.setPower((ly + lx - rx) / normalize * CURRENT_POWER_MULTIPLIER);

        // ANTI-TIP
        // FORMULA (m−rmin/rmax−rmin)×(tmax−tmin)+tmin

        Orientation botOrientationDegs = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        if(rollOffset == 0) rollOffset = botOrientationDegs.thirdAngle;

        roll = Math.abs(botOrientationDegs.thirdAngle) - Math.abs(rollOffset);

        if(roll > measuredMaxRoll) measuredMaxRoll = roll;

        ANTI_TIP_WEIGHTED_POWER_MULTIPLIER = roll/measuredMaxRoll;

        if(Math.abs(roll) > 1) {
            CURRENT_POWER_MULTIPLIER = CURRENT_BASE_POWER_MULTIPLIER-ANTI_TIP_WEIGHTED_POWER_MULTIPLIER; //+ADAPTIVE_SPEED_POWER_MULTIPLIER_OFFSET;
        }
        else {
            CURRENT_POWER_MULTIPLIER = CURRENT_BASE_POWER_MULTIPLIER; //+ ADAPTIVE_SPEED_POWER_MULTIPLIER_OFFSET;
        }

        if (CURRENT_POWER_MULTIPLIER > 1) CURRENT_POWER_MULTIPLIER = 1;

//        telemetry.addData("first", botOrientationDegs.firstAngle);
//        telemetry.addData("second", botOrientationDegs.secondAngle);
//        telemetry.addData("third", botOrientationDegs.thirdAngle);

        telemetry.addData("heading", heading);
        telemetry.addData("roll", roll);
        telemetry.addData("rollOffset", rollOffset);
        telemetry.addData("measuredMaxRoll", measuredMaxRoll);

        telemetry.addData("CURRENT_POWER_MULTIPLIER", df.format(CURRENT_POWER_MULTIPLIER));
        telemetry.addData("CURRENT_BASE_POWER_MULTIPLIER", df.format(CURRENT_BASE_POWER_MULTIPLIER));
        telemetry.addData("ANTI_TIP_WEIGHTED_POWER_MULTIPLIER", df.format(ANTI_TIP_WEIGHTED_POWER_MULTIPLIER));
        telemetry.addData("ADAPTIVE_SPEED_POWER_MULTIPLIER_OFFSET", df.format(ADAPTIVE_SPEED_POWER_MULTIPLIER_OFFSET));
        telemetry.addData("ADAPTIVE TRIGGER", df.format(gamepad1.left_trigger));

        telemetry.update();
//    }
//
    }
}
