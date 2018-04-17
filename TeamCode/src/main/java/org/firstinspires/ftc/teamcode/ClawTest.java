package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "BAS Test", group = "Teleop")
//@Disabled
public class ClawTest extends LinearOpMode {

    public Servo rotateBox;

    public void runOpMode(){

        rotateBox = hardwareMap.servo.get("rotateBox");

        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.a){
                rotateBox.setPosition(0.1);
            }
            if(gamepad1.b){
                rotateBox.setPosition(0.15);
            }
            if(gamepad1.x){
                rotateBox.setPosition(0.18);
            }
            if(gamepad1.y){
                rotateBox.setPosition(0.13);
            }
            idle();
        }
    }
}
