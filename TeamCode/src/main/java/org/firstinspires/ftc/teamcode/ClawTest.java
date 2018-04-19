package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "BAS Test", group = "Teleop")
//@Disabled
public class ClawTest extends LinearOpMode {

    public Servo discHold;

    public void runOpMode(){

        discHold = hardwareMap.servo.get("discHold");

        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.a){
                discHold.setPosition(0.3);
            }
            if(gamepad1.b){
                discHold.setPosition(0.6);
            }
            if(gamepad1.x){
                discHold.setPosition(0.0);
            }
            if(gamepad1.y){
                discHold.setPosition(0.9);
            }
            telemetry.addData("servo position", discHold.getPosition());
            idle();
        }
    }
}
