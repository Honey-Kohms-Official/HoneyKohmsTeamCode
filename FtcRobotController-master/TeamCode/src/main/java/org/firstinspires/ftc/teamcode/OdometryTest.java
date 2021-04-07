/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.io.File;

import static com.company.robot.*;

import static org.firstinspires.ftc.teamcode.Auto16557.RingDeterminationPipeline.RingPosition.NONE;

@Autonomous(name="Autonomous With Camera Sensor Version 1.1", group ="Autonomous")


public class OdometryTest extends LinearOpMode
{
    OpenCvInternalCamera phoneCam;
    RingDeterminationPipeline pipeline;

    private DcMotor FrontRight, FrontLeft, BackRight, BackLeft, Launcher, IntakeMotor;
    private Servo WobbleGoalGrabber;
    private DcMotor WobbleGoalHeight;
    private Servo LauncherLoadUpper;
    private ColorSensor ColorSensor1;
    private ColorSensor ColorSensor2;

    private DcMotor LeftEncoder, MiddleEncoder, RightEncoder;
    BNO055IMU imu;
    ElapsedTime timer = new ElapsedTime();

    static final double calibrationSpeed = 0.5;
    static final double TICKS_PER_REV = 8192;
    static final double WHEEL_DIAMTER = 100/25.4;
    static final double GEAR_RATIO = 1;

    static final double TICKS_PER_INCH = WHEEL_DIAMTER * Math.PI * GEAR_RATIO / TICKS_PER_REV;

    File SideWheelSeperationFile = AppUtil.getInstance().getSettingsFile("SideWheelSeperationFile");
    File MiddleTickOffsetFile = AppUtil.getInstance().getSettingsFile("MiddleTickOffsetFile");

    public static Boolean NoRing = false;
    public static Boolean OneRing = false;
    public static Boolean FourRings = false;

    @Override
    public void runOpMode(){



        FrontRight = hardwareMap.dcMotor.get("FrontRight");
        FrontLeft = hardwareMap.dcMotor.get("FrontLeft");
        BackRight = hardwareMap.dcMotor.get("BackRight");
        BackLeft = hardwareMap.dcMotor.get("BackLeft");
        Launcher = hardwareMap.dcMotor.get("Launcher");
        IntakeMotor = hardwareMap.dcMotor.get("Intake");
        WobbleGoalGrabber = hardwareMap.servo.get("Goal Grabber");
        WobbleGoalHeight = hardwareMap.dcMotor.get("GoalHeight");
        LauncherLoadUpper = hardwareMap.servo.get("LaunchingServo");
        ColorSensor1 = hardwareMap.get(ColorSensor.class, "Color Sensor1");
        ColorSensor2 = hardwareMap.get(ColorSensor.class, "Color Sensor2");

        LeftEncoder = hardwareMap.dcMotor.get("LeftEncoder");
        RightEncoder = hardwareMap.dcMotor.get("RightEncoder");
        MiddleEncoder = hardwareMap.dcMotor.get("MiddleEncoder");

        FrontLeft.setDirection(DcMotor.Direction.FORWARD);
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setDirection(DcMotor.Direction.REVERSE);
        Launcher.setDirection(DcMotor.Direction.FORWARD);
        WobbleGoalHeight.setDirection((DcMotorSimple.Direction.FORWARD));
        IntakeMotor.setDirection(DcMotor.Direction.FORWARD);

        int ringNumber = -1;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new RingDeterminationPipeline();
        phoneCam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener(){
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });



        waitForStart();

        while(imu.getAngularOrientation().firstAngle < 90 && opModeIsActive()){
            BackRight.setPower(-calibrationSpeed);
            FrontLeft.setPower(calibrationSpeed);
            FrontRight.setPower(-calibrationSpeed);
            BackLeft.setPower(calibrationSpeed);

            if(imu.getAngularOrientation().firstAngle < 60){
                BackRight.setPower(-calibrationSpeed);
                FrontLeft.setPower(calibrationSpeed);
                FrontRight.setPower(-calibrationSpeed);
                BackLeft.setPower(calibrationSpeed);
            }
            else{
                BackRight.setPower(-calibrationSpeed/2);
                FrontLeft.setPower(calibrationSpeed/2);
                FrontRight.setPower(-calibrationSpeed/2);
                BackLeft.setPower(calibrationSpeed/2);
            }

        }
        BackRight.setPower(0);
        FrontLeft.setPower(0);
        FrontRight.setPower(-0);
        BackLeft.setPower(0);

        timer.reset();
        while(timer.seconds() < 1 &&opModeIsActive()){

        }

        

        while (opModeIsActive())
        { //put functions here for autonomous
            if (pipeline.position == RingDeterminationPipeline.RingPosition.NONE) {
                ringNumber = 0;
            } else if (pipeline.position == RingDeterminationPipeline.RingPosition.ONE) {
                ringNumber = 1;
            } else if (pipeline.position == RingDeterminationPipeline.RingPosition.FOUR) {
                ringNumber = 4;
            }

            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.update();

            if (ringNumber == 0) {
                Drive(1, 3310);
                Strafe(1, 600);
                BringDownGrabber();
                Release();
                BringUpGrabber();
                Strafe(-1, 970);
                TurnLeft(1, 535);
                Strafe(1, 2770);
                BringDownGrabber();
                Release();
                BringUpGrabber();
                Strafe(-1, 2650);
                TurnRight(1, 650);
                Strafe(1, 930);

            }

            else if (ringNumber == 1) {

                Strafe(1, 450);
                Drive(1, 4100);
                Strafe(-1, 1200);
                BringDownGrabber();
                Release();
                BringUpGrabber();
                Strafe(-1, 1300);
                TurnLeft(1, 90);
                Drive(-1, 3300);
                Strafe(1, 500);
                BringDownGrabber();
                Grab();
                BringUpGrabber();
                Drive(1, 3600);
                TurnRight(1, 90);
                Strafe(1, 1250);
                BringDownGrabber();
                Release();
                BringUpGrabber();
                Strafe(-1, 600);
                Drive(-1, 500);

            }

            else if (ringNumber == 4) {
                Strafe(1, 450);
                Drive(1, 5000);
                TurnRight(1, 300);
                BringDownGrabber();
                Release();
                BringUpGrabber();
                Strafe(-1, 3700);
                TurnLeft(1, 888);
                Strafe(1, 2250);
                BringDownGrabber();
                Grab();
                BringUpGrabber();
                Strafe(-1, 2500);
                Drive(1, 3400);
                TurnRight(1, 270);
                BringDownGrabber();
                Release();
                BringUpGrabber();
                Strafe(-1, 400);
                Drive(-1, 1500);
            }

            stop();
        }
    }

    public static class RingDeterminationPipeline extends OpenCvPipeline {
        /*
         * An enum to define the skystone position
         */
 /*       public enum RingPosition
        {
            FOUR,
            ONE,
            NONE
        }

        /*
         * Some color constants
         */
       // static final Scalar BLUE = new Scalar(0, 0, 255);
       // static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        /*static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(181,98);

        static final int REGION_WIDTH = 35;
        static final int REGION_HEIGHT = 25;

        final int FOUR_RING_THRESHOLD = 150;
        final int ONE_RING_THRESHOLD = 135;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
    /*    Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile RingPosition position = RingPosition.FOUR;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
   /*     void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = RingPosition.FOUR;
            // Record our analysis
            if(avg1 > FOUR_RING_THRESHOLD){
                position = RingPosition.FOUR;
            }else if (avg1 > ONE_RING_THRESHOLD){
                position = RingPosition.ONE;
            }else{
                //position = NONE;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }
    }

    public void Drive(double power, long time){

        BackRight.setPower(power);
        FrontLeft.setPower(power);
        FrontRight.setPower(power);
        BackLeft.setPower(power);
        sleep(time);
    }
    public void Strafe(double power, long time){

        BackRight.setPower(-power);
        FrontLeft.setPower(power);
        FrontRight.setPower(power);
        BackLeft.setPower(-power);
        sleep(time);
    }
    public void TurnLeft ( double power, long time)  {

        FrontRight.setPower(power);
        FrontLeft.setPower(power);
        BackRight.setPower(-power);
        BackLeft.setPower(-power);
        sleep(time);

    }
    public void TurnRight ( double power, long time) {

        FrontRight.setPower(-power);
        FrontLeft.setPower(power);
        BackRight.setPower(-power);
        BackLeft.setPower(power);

        sleep(time);
        Stop();

    }
    public void Stop(){
        BackLeft.setPower(0);
        BackRight.setPower(0);
        FrontLeft.setPower(0);
        FrontRight.setPower(0);

    }

    public void LaunchRing(double power){
        Launcher.setPower(power);
        sleep(100);
    }

    public void LoadUpLaucnher(){
        LauncherLoadUpper.setPosition(1);
        sleep(250);
        LauncherLoadUpper.setPosition(0);

    }

    public void Grab(){
        WobbleGoalGrabber.setPosition(1);

    }

    public void Release(){
        WobbleGoalGrabber.setPosition(0);

    }

    public void BringUpGrabber(){
        WobbleGoalHeight.setPower(1);
        sleep(250);
    }

    public void BringDownGrabber(){
        WobbleGoalHeight.setPower(-1);
        sleep(250);
    }
}*/