/*
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Tutorial TeleOp", group="Linear Opmode")
//@Disabled
public class TeleOpTutorial extends LinearOpMode{

    private DcMotor FrontRight;
    private DcMotor FrontLeft;
    private DcMotor BackRight;
    private DcMotor BackLeft;


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


        FrontLeft.setDirection(DcMotor.Direction.FORWARD);
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        runtime.reset();

        Boolean slow = false;



        double scale = 1;


        while(opModeIsActive()){
            if(slow){  //code for slow toggling slow mode
                scale = 0.15;}
            else{
                scale = 1;}

            if(gamepad1.left_stick_y != 0){
                FrontLeft.setPower(gamepad1.left_stick_y * scale  );
                FrontRight.setPower(gamepad1.left_stick_y * scale  );
                BackLeft.setPower(gamepad1.left_stick_y * scale );
                BackRight.setPower(gamepad1.left_stick_y * scale );

            }
            else if(gamepad1.right_stick_x != 0){
                FrontLeft.setPower(gamepad1.right_stick_x * scale);
                FrontRight.setPower(-gamepad1.right_stick_x * scale);
                BackLeft.setPower(gamepad1.right_stick_x * scale);
                BackRight.setPower(-gamepad1.right_stick_x * scale);
            }

            else {
                FrontLeft.setPower(0);
                FrontRight.setPower(0);
                BackLeft.setPower(0);
                BackRight.setPower(0);
            }
                //for strafing
            if(gamepad1.left_trigger != 0){
                FrontLeft.setPower(gamepad1.left_trigger * scale  );
                FrontRight.setPower(-gamepad1.left_trigger * scale  );
                BackLeft.setPower(-gamepad1.left_trigger * scale );
                BackRight.setPower(gamepad1.left_trigger * scale );
            }
            else if(gamepad1.right_trigger != 0){
                FrontLeft.setPower(-gamepad1.right_trigger * scale  );
                FrontRight.setPower(gamepad1.right_trigger * scale  );
                BackLeft.setPower(gamepad1.right_trigger * scale );
                BackRight.setPower(-gamepad1.right_trigger * scale );
            }
            else {
                FrontLeft.setPower(0);
                FrontRight.setPower(0);
                BackLeft.setPower(0);
                BackRight.setPower(0);
            }

            telemetry.update();

        }

    }
}


*/
