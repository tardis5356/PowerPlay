package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.Arrays;


public abstract class BaseClass_PP extends LinearOpMode {

    //Final variables (in inches)
    final static double wheelDiameter = 60 / 25.4; //black and white 60 mm omni wheel rev robotics
    //35/25.4; //using 35 mm wheel from rotacaster (blue and black)
    final static double wheelDistance = 16.25;
    // 17.625; //separation between y-axis odometer wheels (in)


    OpenCvCamera webcam;
   // TeamElementPositionTest.SkystoneDeterminationPipeline pipeline;

    //all instance field variables
    DcMotor mBL;//Back left
    DcMotor mBR;
    DcMotor mFL;
    DcMotor mFR;//Front right
    DcMotor mE;//arm motor extends
    DcMotor mA;
    DcMotor mR;
    DcMotor mL; //lift
    DcMotor mTME; //tape measure extend

    //connected to sparkMini controller don't have enough ports
    CRServo mArm;
    CRServo mTGIArm;
    CRServo sA; //controls arm
    CRServo sTMT; //tape measure


    Servo sV;//up-down wrist movement servo
    Servo sW;
    Servo sG;
    Servo sL;
    Servo sR;
    Servo sYL; //odometer Yleft servo
    Servo sYR; //odometer Yright servo
    Servo sX; //odometer X servo

    TouchSensor lAB; //Bottom arm limit
    AnalogInput armLimit; //limit switch for Primus
    AnalogInput potentiometer;

    BNO055IMU imuControl; //REV gyro - Control hub
    BNO055IMU imuExpansion;   // expansion hub gyro
    Orientation angles;

    ElapsedTime runtime;
    ElapsedTime runtimeTwo;
    Orientation lastAngle = new Orientation();
    Pose pose = new Pose(0, 0, 0);

    //Global sensor values
    float gyroZ = 0;        // initializing at zero just to be sure
    boolean isStartRecorded = false;

    int encoderXStart;
    int encoderYLeftStart;
    int encoderYRightStart;
    double leftDistance;// = rangeSensorLeft.getDistance(DistanceUnit.CM);
    double rightDistance; // = rangeSensorRight.getDistance(DistanceUnit.CM);
    double frontDistance; // = rangeSensorFront.getDistance(DistanceUnit.CM);
    double backDistance; // = rangeSensorBackRight.getDistance(DistanceUnit.CM);
    int sensorArrayLength = 5;
    double[] leftDistanceArray = new double[sensorArrayLength];
    double[] rightDistanceArray = new double[sensorArrayLength];
    double[] backDistanceArray = new double[sensorArrayLength];
    double[] frontDistanceArray = new double[sensorArrayLength];
    double leftDistanceFiltered = 0;
    double rightDistanceFiltered = 0;
    double frontDistanceFiltered = 0;


    //initialization step to fit bot in 18 inches


    public void scrunchUpBot() {
        drive(0, 0, 0);
    }



    public static double ticksToInches(int ticks) {
        //converts ticks to inches
        double circum = Math.PI * wheelDiameter;
        return (circum / 8000) * ticks;
    }

    public static double[] arcInfo(int deltaLeft, int deltaRight) {
        double inchesLeft = ticksToInches(deltaLeft);
        double inchesRight = ticksToInches(deltaRight);
        double deltaTheta = (inchesRight - inchesLeft) / wheelDistance;
        double arcLength = (inchesRight + inchesLeft) / 2;
        return new double[]{deltaTheta, arcLength};
    }


    ////////////////////////////defines all components of the bot, one for each robot this year//////////////

    public void defineComponentsCompBot() {
        mBL = hardwareMap.dcMotor.get("mBL");
        mBR = hardwareMap.dcMotor.get("mBR");
        mFL = hardwareMap.dcMotor.get("mFL");
        mFR = hardwareMap.dcMotor.get("mFR");

        mL = hardwareMap.dcMotor.get("mL");
        mTME = hardwareMap.dcMotor.get("mTME");

        sG = hardwareMap.servo.get("sG");
        sW = hardwareMap.servo.get("sW");
        sTMT = hardwareMap.crservo.get("sTMT");
        sA = hardwareMap.crservo.get("sA");

        mBR.setDirection(DcMotor.Direction.REVERSE);
        mFR.setDirection(DcMotor.Direction.REVERSE);

        mFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

    //Defines all components for init()
    public void defineComponentsTestBed() {

        mBL = hardwareMap.dcMotor.get("mBL");//Back left
        mBR = hardwareMap.dcMotor.get("mBR");
        mFL = hardwareMap.dcMotor.get("mFL");
        mFR = hardwareMap.dcMotor.get("mFR");//Front right

        mBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Set motors of robot backwards
        mBR.setDirection(DcMotor.Direction.REVERSE);
        mFR.setDirection(DcMotor.Direction.REVERSE);
//        mFL.setDirection(DcMotor.Direction.REVERSE);
//        mBL.setDirection(DcMotor.Direction.REVERSE);

        mFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        runtime = new ElapsedTime();
        runtimeTwo = new ElapsedTime();

        //0s out encoders
        encoderXStart = mFR.getCurrentPosition();
        encoderYLeftStart = mFL.getCurrentPosition();
        encoderYRightStart = mBL.getCurrentPosition();

    }

    public void defineComponentsOmni() {
        mBL = hardwareMap.dcMotor.get("mBL");//Back left
        mBR = hardwareMap.dcMotor.get("mBR");
        mFL = hardwareMap.dcMotor.get("mFL");
        mFR = hardwareMap.dcMotor.get("mFR");//Front right
        sG = hardwareMap.servo.get("sG");//Gripper

        mTGIArm = hardwareMap.crservo.get("mTGIArm");

        mBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mBL.setDirection(DcMotor.Direction.FORWARD);
        mBR.setDirection(DcMotor.Direction.REVERSE);
        mFR.setDirection(DcMotor.Direction.REVERSE);
        mFL.setDirection(DcMotor.Direction.FORWARD);

    }

    public void defineComponentsSmallz() {
        mBL = hardwareMap.dcMotor.get("mBL");
        mBR = hardwareMap.dcMotor.get("mBR");
        mFL = hardwareMap.dcMotor.get("mFL");
        mFR = hardwareMap.dcMotor.get("mFR");

        //hello world
        mBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mBL.setDirection(DcMotor.Direction.FORWARD);
        mBR.setDirection(DcMotor.Direction.REVERSE);
        mFR.setDirection(DcMotor.Direction.REVERSE);
        mFL.setDirection(DcMotor.Direction.FORWARD);


    }


    public void defineComponentsPrimus() {
        mBL = hardwareMap.dcMotor.get("mBL");//Back left
        mBR = hardwareMap.dcMotor.get("mBR");
        mFL = hardwareMap.dcMotor.get("mFL");
        mFR = hardwareMap.dcMotor.get("mFR");//Front right


        mBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mBL.setDirection(DcMotor.Direction.FORWARD);
        mBR.setDirection(DcMotor.Direction.REVERSE);
        mFR.setDirection(DcMotor.Direction.REVERSE);
        mFL.setDirection(DcMotor.Direction.REVERSE);

        sL = hardwareMap.servo.get("sL");
        sR = hardwareMap.servo.get("sR");
        mArm = hardwareMap.crservo.get("mArm");

//        armLimit = hardwareMap.get(AnalogInput.class, "armLimit");

    }

    public void defineComponentsOptimus() {

        mL = hardwareMap.dcMotor.get("mL");
        mR = hardwareMap.dcMotor.get("mR");
        mA = hardwareMap.dcMotor.get("mA");

        sG = hardwareMap.servo.get("sG");
        sW = hardwareMap.servo.get("sW");

        mL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        mL.setDirection(DcMotor.Direction.FORWARD);
        mR.setDirection(DcMotor.Direction.REVERSE);
    }

    //////////////////////////////////////////////////////odometry functions///////////////////////////////////////////////
    public Pose getPose() {
        return this.pose;
    }


    //////////////////////////////////////////////////drive functions///////////////////////////////////////////////////

    //Moves drive train in all possible directions
    public void drive(double forward, double strafe, double rotate) {
        mFL.setPower(forward + strafe + rotate);
        mFR.setPower(forward - strafe - rotate);
        mBL.setPower(forward - strafe + rotate);
        mBR.setPower(forward + strafe - rotate);
    }

    /////////////////////////////////////////////////////////sensor and odometry/////////////////////////////////////////

    public void gyroUpdate() {
        //updates gyro
        Orientation angles = imuControl.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float originalAngle = angles.firstAngle - lastAngle.firstAngle;

        if (originalAngle < -180) {
            originalAngle += 360;
        } else if (originalAngle > 180) {
            originalAngle -= 360;
        }

        lastAngle = angles;
        gyroZ += originalAngle;
        //gyroZ += (originalAngle + angle2 + angle3 + angle4 + angle5)/5;

    }


    /////////////////////////////////////////////////////////rotation///////////////////////////////////////////////////

    public void rotateClockwise(double power) {
        drive(0, 0, power);
    }

    public void rotateCounterclockwise(double power) {
        rotateClockwise(-power);
    }

    public void stopDriveTrain() {
        drive(0, 0, 0);
    }

    static double median(double[] values) {
        // get array length
        int totalElements = values.length;
        // make temporary array that gets the sorted/manipulated
        double[] newArray = new double[totalElements];
        // now make the actual copy
        for (int i = 0; i < totalElements; i++) {
            newArray[i] = values[i];
        }

        // sort array
        Arrays.sort(newArray);

        double median; // now get the median by finding the "halfway" element in the sorted array
        //      System.out.println("# elements is : " + totalElements);
        // check if total number of scores is even
        if (totalElements % 2 == 0) {
            double sumOfMiddleElements = newArray[totalElements / 2] +
                    newArray[totalElements / 2 - 1];
            // calculate average of middle elements
            median = ((double) sumOfMiddleElements) / 2;
        } else {
            // get the middle element
            median = (double) newArray[newArray.length / 2];
        }
        return median;
    }

    static double[] popValueIntoArray(double[] previousArray, double latestValue) {
        // add element to end of array, drop out first element in array;
        int totalElements = previousArray.length;
        double[] newArray = new double[totalElements];

        for (int i = 0; i < totalElements - 1; i++) {
            newArray[i] = previousArray[i + 1];
        }
        newArray[totalElements - 1] = latestValue; // append latest value to the end of the array

        // now placed updates in returned array
        for (int i = 0; i < totalElements; i++) {
            previousArray[i] = newArray[i];
        }

        return previousArray;
    }

    static void printArray(double[] values) {
        for (double i : values) {
            System.out.print(" " + i);
        }
        System.out.println("");
        int totalElements = values.length;
        System.out.println("# elements is : " + totalElements);
    }
}