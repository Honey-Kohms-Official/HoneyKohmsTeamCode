package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;

import java.util.Arrays;
import java.util.Timer;


@TeleOp(name="16557 TeleOp Version 1.5 (12 Volt Final Version)", group="Linear Opmode")
//@Disabled
public class TeleOp_16557_12_Volt extends LinearOpMode{

    private DcMotor FrontRight;
    private DcMotor FrontLeft;
    private DcMotor BackRight;
    private DcMotor BackLeft;
    private DcMotor Launcher;
    private DcMotor IntakeMotor;
    private Servo WobbleGoalGrabber;
    private DcMotor WobbleGoalHeight;
    private Servo LauncherLoadUpper;
 /*   private ColorSensor ColorSensor1;
    private ColorSensor ColorSensor2;*/

    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException{
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Status", "Runtime " + runtime.toString() );
        telemetry.update();

        FrontRight = hardwareMap.dcMotor.get("FrontRight");
        FrontLeft = hardwareMap.dcMotor.get("FrontLeft");
        BackRight = hardwareMap.dcMotor.get("BackRight");
        BackLeft = hardwareMap.dcMotor.get("BackLeft");
        Launcher = hardwareMap.dcMotor.get("Launcher");
        IntakeMotor = hardwareMap.dcMotor.get("Intake");
        WobbleGoalGrabber = hardwareMap.servo.get("Goal Grabber");
        WobbleGoalHeight = hardwareMap.dcMotor.get("GoalHeight");
        LauncherLoadUpper = hardwareMap.servo.get("LaunchingServo");
   /*     ColorSensor1 = hardwareMap.get(ColorSensor.class, "Color Sensor1");
        ColorSensor2 = hardwareMap.get(ColorSensor.class, "Color Sensor2");*/

        FrontLeft.setDirection(DcMotor.Direction.FORWARD);
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setDirection(DcMotor.Direction.REVERSE);
        Launcher.setDirection(DcMotor.Direction.FORWARD);
        IntakeMotor.setDirection(DcMotor.Direction.FORWARD);
        WobbleGoalHeight.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();
        runtime.reset();

        Boolean slow = false;
        Boolean GrabRelease = false;
        Boolean Launching = false;
        Boolean GoalUp = false;
        Boolean GoalDown = false;
        Boolean Intake = false;
        Boolean ReverseIntake = false;

    /*    boolean White = ColorSensor1.red() >= 235 && ColorSensor1.green() >= 235 && ColorSensor1.blue() >= 235 && ColorSensor2.red() >= 235
                && ColorSensor2.green() >= 235 && ColorSensor2.blue() >= 235;*/

        WobbleGoalHeight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double scale = 1;
/*
        ColorSensor1.enableLed(true);
        ColorSensor2.enableLed(true);*/

        while(opModeIsActive()){

            //CODE FOR CONTROLLING DIRECTION OF INTAKE
            if(gamepad2.b){  //toggle whether the intake system is running or not
                Intake = !Intake;
                ReverseIntake = false;
            }
            else if(gamepad2.a){
                ReverseIntake = !ReverseIntake;
                Intake = false;
            }
            else{

            }

            if (Intake){
                IntakeMotor.setPower(100);
            }
            else if(ReverseIntake){
                IntakeMotor.setPower(-100);
            }
            else {
                IntakeMotor.setPower(0);
            }
            /////////////////////////////////////////

            //CODE FOR CORRECTION SERVO (IN CASE WE GET A MESSED UP RING)
            if(gamepad2.y){
                LauncherLoadUpper.setPosition(1);
                sleep(500);
                LauncherLoadUpper.setPosition(0);
            }


            //////////////////////////////



            //CODE FOR GRABBING AND CHANGING HEIGHT OF THE WOBBLE GOAL
            if(gamepad1.x){  //grabs the wobbly goal
                GrabRelease = !GrabRelease;
            }
            if (GrabRelease){ //code for toggling grab and release of wobbly goal.
                WobbleGoalGrabber.setPosition(1);
            }
            else {
                WobbleGoalGrabber.setPosition(0);
            }


            if(gamepad1.a){
                WobbleGoalHeight.setPower(0.5);
                sleep(300);
            }
            else{
                WobbleGoalHeight.setPower(0);
            }
            if (gamepad1.b){
                WobbleGoalHeight.setPower(-0.5);
                sleep(300);
            }
            else{
                WobbleGoalHeight.setPower(0);
            }
            ///////////////////////////////////




            //CODE FOR LAUNCHING THE RINGS

            if(gamepad2.x){  //triggers the launcher
                LauncherLoadUpper.setPosition(0.3);
                sleep(500);
                LauncherLoadUpper.setPosition(0);
            }
            if(gamepad2.y){  //triggers the launcher
                Launching = !Launching;
            }
            if (Launching){ //code for toggling the motors spinning to launch the rings .
                Launcher.setPower(0.815);
            }
            else {
                Launcher.setPower(0);
            }

            double FrontLeftVal = gamepad1.left_stick_y - (gamepad1.left_stick_x) + -gamepad1.right_stick_x;
            double FrontRightVal = gamepad1.left_stick_y + (gamepad1.left_stick_x) - -gamepad1.right_stick_x;
            double BackLeftVal = gamepad1.left_stick_y + (gamepad1.left_stick_x) + -gamepad1.right_stick_x;
            double BackRightVal = gamepad1.left_stick_y - (gamepad1.left_stick_x) - -gamepad1.right_stick_x;

            //Move range to between 0 and +1, if not already
            double[] wheelPowers = {FrontRightVal, FrontLeftVal, BackLeftVal, BackRightVal};
            Arrays.sort(wheelPowers);
            if (wheelPowers[3] > 1) {
                FrontLeftVal /= wheelPowers[3];
                FrontRightVal /= wheelPowers[3];
                BackLeftVal /= wheelPowers[3];
                BackRightVal /= wheelPowers[3];
            }
            FrontLeft.setPower(FrontLeftVal);
            FrontRight.setPower(FrontRightVal);
            BackLeft.setPower(BackLeftVal);
            BackRight.setPower(BackRightVal);

     /*       if (White) {
                FrontLeft.setPower(0);
                FrontRight.setPower(0);
                BackLeft.setPower(0);
                BackRight.setPower(0);
                sleep(1000);
            }

*/



            telemetry.update();


        }



    }
}









