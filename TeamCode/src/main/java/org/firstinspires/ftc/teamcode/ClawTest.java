package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawTest extends LinearOpMode {

    public Servo leftGrab;
    public Servo rightGrab;

    public void runOpMode(){

        leftGrab = hardwareMap.servo.get("leftGrab");
        rightGrab = hardwareMap.servo.get("rightGrab");

        while(opModeIsActive()){

            if(gamepad1.a){
                leftGrab.setPosition(0);
                rightGrab.setPosition(0);
            }
            if(gamepad1.b){
                leftGrab.setPosition(0.5);
                rightGrab.setPosition(0.5);
            }
            idle();
        }
    }
}
