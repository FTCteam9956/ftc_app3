package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Super Teleop", group = "Teleop")
//@Disabled
public class SuperTeleop extends LinearOpMode {
    public RRHardwarePresets robot = new RRHardwarePresets();

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    public int endGameMode = 0;
    public int relicClawMode = 0;
    public int relicTwistMode = 0;
    // declare motor speed variables
    double RF; double LF; double RR; double LR;
    // declare joystick position variables
    double X1; double Y1; double X2; double Y2;
    // operational constants
    double joyScale = 0.93;
    double motorMax = 0.7; // Limit motor power to this value for Andymark RUN_USING_ENCODER mode

    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        robot.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.left2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.right2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while(opModeIsActive()){
            if(gamepad1.start){
                endGameMode = 1;
            }else if(gamepad2.start){
                endGameMode = 0;
            }
            if(endGameMode == 0){
                // Reset speed variables
                LF = 0; RF = 0; LR = 0; RR = 0;

                // Get joystick values
                Y1 = gamepad1.right_stick_y * joyScale; // invert so up is positive
                X1 = gamepad1.right_stick_x * joyScale;
                // Y2 is not used at present
                X2 = -gamepad1.left_stick_x * joyScale;

                // Forward/back movement
                LF += Y1; RF += Y1; LR += Y1; RR += Y1;
                // Side to side movement
                LF += X1; RF -= X1; LR += X1; RR -= X1;
                //Rotation Movement
                LF += X2; RF -= X2; LR -= X2; RR += X2;

                // Clip motor power values to +-motorMax
                LF = Math.max(-motorMax, Math.min(LF, motorMax));
                RF = Math.max(-motorMax, Math.min(RF, motorMax));
                LR = Math.max(-motorMax, Math.min(LR, motorMax));
                RR = Math.max(-motorMax, Math.min(RR, motorMax));

                // Send values to the motors
                robot.left1.setPower(LF);
                robot.right1.setPower(RF);
                robot.left2.setPower(LR);
                robot.right2.setPower(RR);

                if (gamepad1.a) {
                    robot.intake.setPower(1.0);
                }
                if (gamepad1.b) {
                    robot.intake.setPower(-1.0);
                }
                if(gamepad1.y){
                    robot.intake.setPower(0.0);
                }
                if(gamepad1.right_bumper){
                    robot.moveServo(robot.rotateBox, 1.0, 250, 500);
                }
                else if(gamepad1.left_bumper){
                    robot.moveServo(robot.rotateBox, 0.0, 250, 500);
                }

                if(endGameMode == 1){
                    // Reset speed variables
                    LF = 0; RF = 0; LR = 0; RR = 0;

                    // Get joystick values
                    Y1 = -gamepad2.right_stick_y * joyScale; // invert so up is positive
                    X1 = gamepad2.right_stick_x * joyScale;
                    Y2 = -gamepad2.left_stick_y * joyScale; // Y2 is not used at present
                    X2 = gamepad2.left_stick_x * joyScale;

                    // Forward/back movement
                    LF += Y1; RF += Y1; LR += Y1; RR += Y1;
                    // Side to side movement
                    LF += X1; RF -= X1; LR += X1; RR -= X1;
                    //Rotation Movement
                    LF += X2; RF -= X2; LR -= X2; RR += X2;

                    // Clip motor power values to +-motorMax
                    LF = Math.max(-motorMax, Math.min(LF, motorMax));
                    RF = Math.max(-motorMax, Math.min(RF, motorMax));
                    LR = Math.max(-motorMax, Math.min(LR, motorMax));
                    RR = Math.max(-motorMax, Math.min(RR, motorMax));

                    // Send values to the motors
                    robot.left1.setPower(LF);
                    robot.right1.setPower(RF);
                    robot.left2.setPower(LR);
                    robot.right2.setPower(RR);

                    //Controls our linear slide

                    //Controls the claw on our linear slide
//                    if (gamepad2.left_bumper && relicClawMode == 0) {
//                        robot.pinch.setPosition(robot.RELIC_CLAW_CLOSED); //Closes claw
//                        sleep(500);
//                        relicClawMode++;
//                    } else if (gamepad2.left_bumper && relicClawMode == 1) {
//                        robot.pinch.setPosition(robot.RELIC_CLAW_OPENED); //Opens claw
//                        sleep(500);
//                        relicClawMode--;
//                    }
//                    //Slider Twisting Controls
//                    if (gamepad2.right_bumper && relicTwistMode == 0) {
//                        robot.twist.setPosition(robot.RELIC_TWIST_UP); //Twists the claw up
//                        sleep(500);
//                        relicTwistMode++;
//                    } else if (gamepad2.right_bumper && relicTwistMode == 1) {
//                        robot.twist.setPosition(robot.RELIC_TWIST_DOWN); // Twists the claw down
//                        sleep(500);
//                        relicTwistMode--;
//                    }
                }
            }
            // Send some useful parameters to the driver station
//            telemetry.addData("LF", "%.3f", LF);
//            telemetry.addData("RF", "%.3f", RF);
//            telemetry.addData("LR", "%.3f", LR);
//            telemetry.addData("RR", "%.3f", RR);
//            telemetry.addData("Left1 Speed", robot.left1.getPower());
//            telemetry.addData("Left2 Speed", robot.left2.getPower());
//            telemetry.addData("Right1 Speed", robot.right1.getPower());
//            telemetry.addData("Right2 Speed", robot.right2.getPower());
            telemetry.addData("Driver Mode", endGameMode);
        }
    }
}