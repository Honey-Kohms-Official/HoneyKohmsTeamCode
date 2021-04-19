package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar	;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;


@Autonomous(name="Ching Cheng Hanji Autonomous Power Shots Version 1.0.4.0 (13.5 - 14 Volt Battery Version)", group ="Autonomous")


public class AutoPowerShots extends LinearOpMode
{


    private DcMotor FrontRight;
    private DcMotor FrontLeft;
    private DcMotor BackRight;
    private DcMotor BackLeft;
    private DcMotor Launcher;
    private DcMotor IntakeMotor;
    private Servo WobbleGoalGrabber;
    private DcMotor WobbleGoalHeight;
    private Servo LauncherLoadUpper;
    //   private VoltageSensor vs;
    //   private ColorSensor ColorSensor1;
    // private ColorSensor ColorSensor2;

    public static Boolean NoRing = false;
    public static Boolean OneRing = false;
    public static Boolean FourRings = false;

    OpenCvInternalCamera phoneCam;
    AutoPowerShots.RingDeterminationPipeline pipeline;


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
        //   vs = hardwareMap.voltageSensor.get("Voltage Sensor");
        // ColorSensor1 = hardwareMap.get(ColorSensor.class, "Color Sensor1");
        //   ColorSensor2 = hardwareMap.get(ColorSensor.class, "Color Sensor2");

        FrontLeft.setDirection(DcMotor.Direction.FORWARD);
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setDirection(DcMotor.Direction.REVERSE);
        Launcher.setDirection(DcMotor.Direction.FORWARD);
        WobbleGoalHeight.setDirection((DcMotorSimple.Direction.FORWARD));
        IntakeMotor.setDirection(DcMotor.Direction.FORWARD);

        int ringNumber = -1;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);
        pipeline = new  AutoPowerShots.RingDeterminationPipeline();
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

        //  double voltage = vs.getVoltage();

        telemetry.addData("Analysis", pipeline.getAnalysis());
        telemetry.addData("Position", pipeline.position);
        telemetry.update();



        waitForStart();

        while (opModeIsActive())
        { //put functions here for autonomous
            //     telemetry.addData("Voltage", voltage);

            Grab();

           /* Strafe(1, 900);
            LongStop(2000);*/

            if (pipeline.position == AutoPowerShots.RingDeterminationPipeline.RingPosition.NONE) {
                ringNumber = 0;
                sleep(500);
            } else if (pipeline.position == AutoPowerShots.RingDeterminationPipeline.RingPosition.ONE) {
                ringNumber = 1;
                sleep(500);
            } else if (pipeline.position == AutoPowerShots.RingDeterminationPipeline.RingPosition.FOUR) {
                ringNumber = 4;
                sleep(500);}

            Strafe(-1, 260);
            LongStop(1000);

            if (ringNumber==0){

                LongStop(500);
                Drive(1, 455);
                LongStop(1000);
                Release();
                LongStop(1000);
                BringUpGrabber();
                LongStop(1000);
                Drive(-1,175);
                LongStop(1000);
                LongStrafe(1, 925);          //STARFE
                Launcher.setPower(0.7395);
                LongStop(2000);
                LaunchFirstRing();
                Strafe(1,349);
                LongStop(1500);
                LaunchSecondRing();
                Strafe(1,361);
                LongStop(1500);
                LaunchThirdRing();
                Strafe(1,400);
                Drive(1, 190);
            }

            if(ringNumber==1){

                LongStop(500);
                Drive(1, 665);
                LongStop(1000);
                LongStrafe(1, 700);
                LongStop(2000);
                Release();
                LongStop(1000);
                BringUpGrabber();
                LongStop(1000);

                LongStrafe(1, 441);            //STARFE
                LongStop(1000);

                Launcher.setPower(0.7355);

                Drive(-1,300);
                LongStop(2000);

                LaunchFirstRingVar2();
                Strafe(1,305);
                LongStop(1500);
                LaunchSecondRingVar2();
                Strafe(1,359);
                LongStop(1500);
                LaunchRingVar2();
                Strafe(1,400);
                Drive(1, 150);


            }

            if (ringNumber == 4){
                LongStop(500);
                Drive(1, 900);
                LongStop(2000);
                Release();
                LongStop(1000);
                BringUpGrabber();
                LongStop(1000);

                LongStrafe(1, 915);           //STARFE
                LongStop(1000);

                Launcher.setPower(0.739);


                Drive(-1,450);
                LongStop(2000);

                LaunchFirstRingVar3();
                LongStrafe(1,400);
                LongStop(1500);
                LaunchSecondRingVar3();
                Strafe(1,395);
                LongStop(1500);
                LaunchRingVar3();
                Strafe(1,400);
                Drive(1, 150);

            }

            stop();
        }
    }

    public static class RingDeterminationPipeline extends OpenCvPipeline {
        //An enum to define the skystone position


        public enum RingPosition
        {
            FOUR,
            ONE,
            NONE
        }

        // Some color constants


        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        //The core values which define the location and size of the sample regions


        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(54,102);                    ///FIGURE OUT GOOD LOCATION FOR THE BOX. EXPERIMENT WITH THIS

        static final int REGION_WIDTH = 35;
        static final int REGION_HEIGHT = 25;

        final int FOUR_RING_THRESHOLD = 165;
        final int ONE_RING_THRESHOLD = 135;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        //Working variables


        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile AutoPowerShots.RingDeterminationPipeline.RingPosition position = AutoPowerShots.RingDeterminationPipeline.RingPosition.FOUR;

        // This function takes the RGB frame, converts AutoTesting.RingDeterminationPipeline.to YCrCb,
        // and extracts the Cb channel to the 'Cb' variable


        void inputToCb(Mat input)
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

            position = AutoPowerShots.RingDeterminationPipeline.RingPosition.FOUR;
            // Record our analysis
            if(avg1 > FOUR_RING_THRESHOLD){
                position = AutoPowerShots.RingDeterminationPipeline.RingPosition.FOUR;
            }else if (avg1 > ONE_RING_THRESHOLD){
                position = AutoPowerShots.RingDeterminationPipeline.RingPosition.ONE;
            }else{
                position = AutoPowerShots.RingDeterminationPipeline.RingPosition.NONE;
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

        BackRight.setPower(-power);
        FrontLeft.setPower(-power);
        FrontRight.setPower(-power);
        BackLeft.setPower(-power);
        sleep(time);
        Stop();
        sleep(50);
    }
    public void Strafe(double power, long time){  //STRAFES LEFT BY DEFAULT                                     ThIS CHANGES A LOT OF TIMES
        BackRight.setPower(power*0.5);
        FrontLeft.setPower(power*0.5);
        FrontRight.setPower(-power*0.7); //Changed from 0.6, it was uneven when testing
        BackLeft.setPower(-power*0.5);
        sleep(time);
        Stop();
        sleep(50);
    }

    public void LongStrafe(double power, long time){  //STRAFES LEFT BY DEFAULT                                  //FOR WHEN YOU WANT TO STRAFE FOR A LOOOOOOOOONG TIME
        BackRight.setPower(power*0.55);
        FrontLeft.setPower(power*0.55);
        FrontRight.setPower(-power*0.69); //Changed from 0.55, it was uneven when testing
        BackLeft.setPower(-power*0.55);
        sleep(time);
        Stop();
        sleep(50);
    }

    public void TurnLeft ( double power, long time)  {

        FrontRight.setPower(-power);
        FrontLeft.setPower(power);
        BackRight.setPower(-power);
        BackLeft.setPower(power);
        sleep(time);
        Stop();
        sleep(50);

    }
    public void TurnRight ( double power, long time) {

        FrontRight.setPower(power);
        FrontLeft.setPower(-power);
        BackRight.setPower(power);
        BackLeft.setPower(-power);
        sleep(time);
        Stop();
        sleep(50);

    }
    public void Stop(){
        BackLeft.setPower(0);
        BackRight.setPower(0);
        FrontLeft.setPower(0);
        FrontRight.setPower(0);

    }

    public void LongStop(long time){
        Stop();
        sleep(time);
    }

    public void LaunchThirdRing(){
        Launcher.setPower(0.7350);
        LauncherLoadUpper.setPosition(0.3);
        sleep(500);                                                         //LAUNCHER STUFF
        LauncherLoadUpper.setPosition(0);
    }
    public void LaunchFirstRing(){
        Launcher.setPower(.7365);
        LauncherLoadUpper.setPosition(0.3);
        sleep(500);
        LauncherLoadUpper.setPosition(0);
    }
    public void LaunchSecondRing(){
        Launcher.setPower(.7355);
        LauncherLoadUpper.setPosition(0.3);
        sleep(500);
        LauncherLoadUpper.setPosition(0);
    }


    public void LaunchRingVar2(){
        Launcher.setPower(0.7345);
        LauncherLoadUpper.setPosition(0.3);
        sleep(500);                                                         //LAUNCHER STUFF
        LauncherLoadUpper.setPosition(0);
    }
    public void LaunchFirstRingVar2(){
        Launcher.setPower(.7375);
        LauncherLoadUpper.setPosition(0.3);
        sleep(500);
        LauncherLoadUpper.setPosition(0);
    }

    public void LaunchSecondRingVar2(){
        Launcher.setPower(.7555);
        LauncherLoadUpper.setPosition(0.3);
        sleep(500);
        LauncherLoadUpper.setPosition(0);
    }
    //Test change
    public void LaunchRingVar3(){
        Launcher.setPower(0.734);
        LauncherLoadUpper.setPosition(0.3);
        sleep(500);                                                         //LAUNCHER STUFF
        LauncherLoadUpper.setPosition(0);
    }
    public void LaunchFirstRingVar3(){
        Launcher.setPower(.738);
        LauncherLoadUpper.setPosition(0.3);
        sleep(500);
        LauncherLoadUpper.setPosition(0);
    }
    public void LaunchSecondRingVar3(){
        Launcher.setPower(.7416);
        LauncherLoadUpper.setPosition(0.3);
        sleep(500);
        LauncherLoadUpper.setPosition(0);
    }


    public void Grab(){
        WobbleGoalGrabber.setPosition(0);

    }

    public void Release(){
        WobbleGoalGrabber.setPosition(1);

    }

    public void BringUpGrabber(){
        WobbleGoalHeight.setPower(0.5);
        sleep(750);
        WobbleGoalHeight.setPower(0);
        WobbleGoalHeight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void BringDownGrabber(){
        WobbleGoalHeight.setPower(-0.5);
        sleep(750);
        WobbleGoalHeight.setPower(0);
        WobbleGoalHeight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /*public void StopAtWhiteLine(){
        while(opModeIsActive()){
            Drive(100,10);
            if (ColorSensor1.red() == 255 && ColorSensor1.green() == 255 && ColorSensor1.blue() == 255 && ColorSensor2.red() == 255
                    && ColorSensor2.green() == 255 && ColorSensor2.blue() == 255) {
                Stop();
                break;
            }
        }
    }*/
}


