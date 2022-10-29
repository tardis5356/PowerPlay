package org.firstinspires.ftc.teamcode.teleop;

import static java.lang.Math.abs;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.bosch.BNO055IMU;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.commands.LiftToScoringPositionCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Junctions;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.BeaconArm;

@TeleOp(name = "Gen1 TeleOp")
public class Gen1_TeleOp extends CommandOpMode {
    private DcMotorEx mFR, mFL, mBR, mBL;
    private BNO055IMU imu;

    private double offset = 0.0;
    private double powerMultiplier = 1.0;
    private double POWER_MULTIPLIER = 1.0;

    private Drivetrain Drivetrain;
    private Lift Lift;
    private Arm Arm;
    private Gripper Gripper;
    private BeaconArm BeaconArm;

    //    private GrabStone m_grabCommand;
//    private ReleaseStone m_releaseCommand;
    private Button m_grabButton, m_releaseButton;
    private LiftToScoringPositionCommand liftRetractCommand, liftToIntakeCommand, liftToGroundJunctionCommand, liftToLowJunctionCommand, liftToMediumJunctionCommand, liftToHighJunctionCommand;

    FtcDashboard dashboard = FtcDashboard.getInstance();


    @Override
    public void initialize() {
//        schedule(new BulkCacheCommand(hardwareMap));
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Object declarations
        GamepadEx driver = new GamepadEx(gamepad1);
        GamepadEx manipulator = new GamepadEx(gamepad2);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        // Motors and Other Stuff
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

        telemetry.addData("powerMultiplier", powerMultiplier);
        telemetry.update();

//        Drivetrain = new Drivetrain(hardwareMap);
        Gripper = new Gripper(hardwareMap);
        Lift = new Lift(hardwareMap);
        Arm = new Arm(hardwareMap);
        BeaconArm = new BeaconArm(hardwareMap);

//        m_grabCommand = new GrabStone(m_gripper);
//        m_releaseCommand = new ReleaseStone(m_gripper);
        liftToIntakeCommand = new LiftToScoringPositionCommand(Lift, Arm, Gripper, Junctions.INTAKE);
        liftRetractCommand = new LiftToScoringPositionCommand(Lift, Arm, Gripper, Junctions.FULL_RETRACTION);
        liftToGroundJunctionCommand = new LiftToScoringPositionCommand(Lift, Arm, Gripper, Junctions.GROUND_JUNCTION);
        liftToLowJunctionCommand = new LiftToScoringPositionCommand(Lift, Arm, Gripper, Junctions.LOW_JUNCTION);
        liftToMediumJunctionCommand = new LiftToScoringPositionCommand(Lift, Arm, Gripper, Junctions.MEDIUM_JUNCTION);
        liftToHighJunctionCommand = new LiftToScoringPositionCommand(Lift, Arm, Gripper, Junctions.HIGH_JUNCTION);
        // Manipulator Triggers
        new Trigger(() -> manipulator.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5)
                //If the trigger is pressed and the scoring arm is not in the robot then open the bucket
                .whenActive(() -> {
//                    if (!scoringArm.loading) {
                    telemetry.addData("gripepr open", Gripper);
                    Gripper.open();
//                    }
                })
                .whenInactive(() -> {
//                    if (!scoringArm.loading) {
                    Gripper.close();
//                    }
                });

//        m_grabButton = (new GamepadButton(manipulator, GamepadKeys.Button.A))
//                .whenPressed(Gripper.open());
//        m_releaseButton = (new GamepadButton(m_driverOp, GamepadKeys.Button.B))
//                .whenPressed(m_releaseCommand);
        new Trigger(() -> manipulator.getButton(GamepadKeys.Button.X)) // extend and slow drive base
                .whenActive(liftToHighJunctionCommand)
                .whenActive(() -> powerMultiplier = 0.5);
        new Trigger(() -> manipulator.getButton(GamepadKeys.Button.Y)) // extend and slow drive base
                .whenActive(liftToMediumJunctionCommand)
                .whenActive(() -> powerMultiplier = 0.5);
        new Trigger(() -> manipulator.getButton(GamepadKeys.Button.B)) // extend and slow drive base
                .whenActive(liftToLowJunctionCommand)
                .whenActive(() -> powerMultiplier = 0.5);
        new Trigger(() -> manipulator.getButton(GamepadKeys.Button.A)) // extend and slow drive base
                .whenActive(liftRetractCommand)
                .whenActive(() -> powerMultiplier = 0.5);
//                .cancelWhenActive(liftRetractCommand);
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

        mFL.setPower((ly + lx + rx) / normalize * powerMultiplier);
        mBL.setPower((ly - lx + rx) / normalize * powerMultiplier);
        mFR.setPower((ly - lx - rx) / normalize * powerMultiplier);
        mBR.setPower((ly + lx - rx) / normalize * powerMultiplier);


        Lift.manualControl(-gamepad2.left_stick_y);

        //ANTI-TIP
//        (m−rmin/rmax−rmin)×(tmax−tmin)+tmin // FORMULA
//        if(rollOffset == 0) rollOffset = botOrientationDegs.thirdAngle;
//
//        roll = Math.abs(botOrientationDegs.thirdAngle) - Math.abs(rollOffset);
//
//        if(roll > measuredMaxRoll) measuredMaxRoll = roll;
//
//        float weightedPowerMultiplier = roll/measuredMaxRoll;
//
////        if(Math.abs(roll) > 5) {
//        powerMultiplier = POWER_MULTIPLIER-weightedPowerMultiplier;
////        }
////        else { powerMultiplier = POWER_MULTIPLIER; }
//
//        if (powerMultiplier > 1) powerMultiplier = 1;
//
//        telemetry.addData("heading", heading);
//        telemetry.addData("roll", roll);
//        telemetry.addData("rollOffset", rollOffset);
//        telemetry.addData("measuredMaxRoll", measuredMaxRoll);
//
//        telemetry.addData("weightedPowerMultiplier", weightedPowerMultiplier);
//        telemetry.addData("powerMultiplier", powerMultiplier);
        telemetry.addData("lift pos", Lift.getLiftPosition());
        telemetry.addData("lift power", Lift.getLiftPower());
        telemetry.addData("lift target", Lift.target);

        telemetry.addData("arm pos", Arm.getArmPosition());
        telemetry.addData("gripper pos", Gripper.getGripperPosition());

        telemetry.update();
    }

}