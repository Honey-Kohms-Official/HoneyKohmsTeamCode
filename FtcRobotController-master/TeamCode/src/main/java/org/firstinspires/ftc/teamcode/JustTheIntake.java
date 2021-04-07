package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Intake only", group="Linear Opmode")
//@Disabled
public class JustTheIntake extends LinearOpMode{

    private DcMotor IntakeMotor;


    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException{
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Status", "Runtime " + runtime.toString() );
        telemetry.update();


        IntakeMotor = hardwareMap.dcMotor.get("Intake");



        IntakeMotor.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();
        runtime.reset();


        Boolean Intake = false;


        while(opModeIsActive()){

            if (Intake){ //code for toggling the motors spinning to launch the rings .
                IntakeMotor.setPower(70);
            }
            else {
                IntakeMotor.setPower(0);
            }

            if(gamepad2.b){  //toggle whether the intake system is running or not
                Intake = !Intake;
            }
            telemetry.update();

        }

    }
}
