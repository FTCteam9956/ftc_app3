package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

//3 motors and 1 servo
@TeleOp(name = "Baseball", group = "Teleop")
public class BaseballThrower extends LinearOpMode{

    private double SERVO_CLOSED = 0.0;
    private double SERVO_OPEN = 0.8;
    private int timer = 10;
    private double leftpower = 0;
    private double rightpower = 0;

    public void runOpMode(){

        DcMotor DriveLeft;
        DcMotor DriveRight;
        DcMotor shooter;
        Servo latch;
        DigitalChannel bottomLimit;

        DriveLeft = hardwareMap.dcMotor.get("DriveLeft");
        DriveRight = hardwareMap.dcMotor.get("DriveRight");
        shooter = hardwareMap.dcMotor.get("shooter");
        latch = hardwareMap.servo.get("latch");
        bottomLimit = hardwareMap.digitalChannel.get("bottomLimit");

        DriveRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DriveLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DriveLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DriveRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DriveRight.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()){

            leftpower = (-gamepad1.right_stick_y + gamepad1.right_stick_x);
            rightpower = (-gamepad1.right_stick_y - gamepad1.right_stick_x);
            DriveLeft.setPower(-leftpower);
            DriveRight.setPower(-rightpower);

//            if(gamepad1.a){
//                shooter.setPower(0.8);
//            }
//            if(bottomLimit.getState() == false){
//                latch.setPosition(SERVO_CLOSED);
//                if(timer == 0){
//                    shooter.setPower(0.0);
//                 }
//            }
//            if(bottomLimit.getState() == false) {
//                timer--;
//                if(timer < 0){
//                    timer = 0;
//                }
//            }
            if(gamepad1.a){
                latch.setPosition(SERVO_CLOSED);
            }
            if(gamepad1.b){
                latch.setPosition(SERVO_OPEN);
            }
            idle();
        }
    }
}
