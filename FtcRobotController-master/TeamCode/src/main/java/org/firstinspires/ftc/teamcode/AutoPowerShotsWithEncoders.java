package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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


@Autonomous(name="IGNORE THIS CODE FOR NOW. WIP", group ="Autonomous")


public class AutoPowerShotsWithEncoders extends LinearOpMode
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

    public double DRIVING_VELOCITY_CONSTANT = 1000;                     //ChANGE THIS AS NECESSARY  ORIGINAL: 537.6

    public static Boolean NoRing = false;
    public static Boolean OneRing = false;
    public static Boolean FourRings = false;

    OpenCvInternalCamera phoneCam;
    AutoPowerShotsWithEncoders.RingDeterminationPipeline pipeline;


    @Override
    public void runOpMode(){

        FrontRight = hardwareMap.get(DcMotorEx.class, "FrontRight");
        FrontLeft = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        BackRight = hardwareMap.get(DcMotorEx.class, "BackRight");
        BackLeft = hardwareMap.get(DcMotorEx.class, "BackLeft");
        Launcher = hardwareMap.dcMotor.get("Launcher");
        IntakeMotor = hardwareMap.dcMotor.get("Intake");
        WobbleGoalGrabber = hardwareMap.servo.get("Goal Grabber");
        WobbleGoalHeight = hardwareMap.dcMotor.get("GoalHeight");
        LauncherLoadUpper = hardwareMap.servo.get("LaunchingServo");



        FrontLeft.setDirection(DcMotor.Direction.FORWARD);
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setDirection(DcMotor.Direction.REVERSE);
        Launcher.setDirection(DcMotor.Direction.FORWARD);
        WobbleGoalHeight.setDirection((DcMotorSimple.Direction.FORWARD));
        IntakeMotor.setDirection(DcMotor.Direction.FORWARD);
        WobbleGoalHeight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int ringNumber = -1;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);
        pipeline = new  AutoPowerShotsWithEncoders.RingDeterminationPipeline();
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

        while (opModeIsActive())
        {
            Grab();
            BringDownGrabber();
            Grab();


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
                BringUpGrabber();

                Drive(-1,175);
                LongStop(1000);
                LongStrafe(1, 925);          //STARFE
                Launcher.setPower(0.7435);
                LongStop(2000);
                LaunchFirstRing();
                Strafe(1,318);
                LongStop(1500);
                LaunchSecondRing();
                Strafe(1,328);
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

                LongStrafe(1, 440);            //STARFE
                LongStop(1000);

                Launcher.setPower(0.7405);

                Drive(-1,300);
                LongStop(2000);

                LaunchFirstRingVar2();
                Strafe(1,305);
                LongStop(1500);
                LaunchSecondRingVar2();
                Strafe(1,345);
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

                Launcher.setPower(0.7435);


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

        final int FOUR_RING_THRESHOLD = 150;
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

        BackRight.setPower(-DRIVING_VELOCITY_CONSTANT);
        FrontLeft.setPower(-DRIVING_VELOCITY_CONSTANT);
        FrontRight.setPower(-DRIVING_VELOCITY_CONSTANT);
        BackLeft.setPower(-DRIVING_VELOCITY_CONSTANT);
        sleep(time);
        Stop();
        sleep(50);
    }
    public void Strafe(double power, long time){  //STRAFES LEFT BY DEFAULT                                     ThIS CHANGES A LOT OF TIMES
        BackRight.setPower(DRIVING_VELOCITY_CONSTANT*0.5*power);
        FrontLeft.setPower(DRIVING_VELOCITY_CONSTANT*0.5*power);
        FrontRight.setPower(-DRIVING_VELOCITY_CONSTANT*0.7*power);
        BackLeft.setPower(-DRIVING_VELOCITY_CONSTANT*0.5*power);
        sleep(time);
        Stop();
        sleep(50);
    }

    public void LongStrafe(double power, long time){  //STRAFES LEFT BY DEFAULT                                  //FOR WHEN YOU WANT TO STRAFE FOR A LOOOOOOOOONG TIME
        BackRight.setPower(DRIVING_VELOCITY_CONSTANT*0.55*power);
        FrontLeft.setPower(DRIVING_VELOCITY_CONSTANT*0.55*power);
        FrontRight.setPower(-DRIVING_VELOCITY_CONSTANT*0.69*power);
        BackLeft.setPower(-DRIVING_VELOCITY_CONSTANT*0.55*power);
        sleep(time);
        Stop();
        sleep(50);
    }

    public void TurnLeft ( double power, long time)  {

        FrontRight.setPower(-DRIVING_VELOCITY_CONSTANT);
        FrontLeft.setPower(DRIVING_VELOCITY_CONSTANT);
        BackRight.setPower(-DRIVING_VELOCITY_CONSTANT);
        BackLeft.setPower(DRIVING_VELOCITY_CONSTANT);
        sleep(time);
        Stop();
        sleep(50);

    }
    public void TurnRight ( double power, long time) {

        FrontRight.setPower(DRIVING_VELOCITY_CONSTANT);
        FrontLeft.setPower(-DRIVING_VELOCITY_CONSTANT);
        BackRight.setPower(DRIVING_VELOCITY_CONSTANT);
        BackLeft.setPower(-DRIVING_VELOCITY_CONSTANT);
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
        Launcher.setPower(0.7415);
        LauncherLoadUpper.setPosition(0.3);
        sleep(500);                                                         //LAUNCHER STUFF
        LauncherLoadUpper.setPosition(0);
    }
    public void LaunchFirstRing(){
        Launcher.setPower(.7415);
        LauncherLoadUpper.setPosition(0.3);
        sleep(500);
        LauncherLoadUpper.setPosition(0);
    }
    public void LaunchSecondRing(){
        Launcher.setPower(.743);
        LauncherLoadUpper.setPosition(0.3);
        sleep(500);
        LauncherLoadUpper.setPosition(0);
    }


    public void LaunchRingVar2(){
        Launcher.setPower(0.7435);
        LauncherLoadUpper.setPosition(0.3);
        sleep(500);                                                         //LAUNCHER STUFF
        LauncherLoadUpper.setPosition(0);
    }
    public void LaunchFirstRingVar2(){
        Launcher.setPower(.7435);
        LauncherLoadUpper.setPosition(0.3);
        sleep(500);
        LauncherLoadUpper.setPosition(0);
    }

    public void LaunchSecondRingVar2(){
        Launcher.setPower(.7665);
        LauncherLoadUpper.setPosition(0.3);
        sleep(500);
        LauncherLoadUpper.setPosition(0);
    }

    public void LaunchRingVar3(){
        Launcher.setPower(0.743);
        LauncherLoadUpper.setPosition(0.3);
        sleep(500);                                                         //LAUNCHER STUFF
        LauncherLoadUpper.setPosition(0);
    }
    public void LaunchFirstRingVar3(){
        Launcher.setPower(.747);
        LauncherLoadUpper.setPosition(0.3);
        sleep(500);
        LauncherLoadUpper.setPosition(0);
    }
    public void LaunchSecondRingVar3(){
        Launcher.setPower(.7495);
        LauncherLoadUpper.setPosition(0.3);
        sleep(500);
        LauncherLoadUpper.setPosition(0);
    }

  /*  public void LaunchThirdRings(){
        Launcher.setPower(.75);
        LauncherLoadUpper.setPosition(0.3);
        sleep(500);
        LauncherLoadUpper.setPosition(0);
    }*/

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
    }

    public void BringDownGrabber(){
        WobbleGoalHeight.setPower(-0.5);
        sleep(750);
        WobbleGoalHeight.setPower(0);
    }

}



