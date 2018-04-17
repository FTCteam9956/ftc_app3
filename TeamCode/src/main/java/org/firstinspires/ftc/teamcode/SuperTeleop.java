package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "Super Teleop", group = "Teleop")
@Disabled
public class SuperTeleop extends LinearOpMode {
    public RRHardwarePresets robot = new RRHardwarePresets();

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    public int endGameMode = 0;
    public float leftPower;
    public float rightPower;
    public static int flipPosition = -70;
    public int relicClawMode = 0;
    public int relicTwistMode = 0;
    public int fingerMode = 0;
    // declare motor speed variables
    double RF; double LF; double RR; double LR;
    // declare joystick position variables
    double X1; double Y1; double X2; double Y2;
//    DcMotor FrontLeft, BackLeft, FrontRight, BackRight;
    // operational constants
    double joyScale = 1.0;
    double motorMax = 0.7; // Limit motor power to this value for Andymark RUN_USING_ENCODER mode

    @Override
    public void runOpMode(){
//        FrontLeft = hardwareMap.dcMotor.get("left1");
//        BackLeft = hardwareMap.dcMotor.get("left2");
//        FrontRight = hardwareMap.dcMotor.get("right1");
//        BackRight= hardwareMap.dcMotor.get("right2");
//        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.init(hardwareMap);
        robot.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.glyphFlip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.left2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.right2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.glyphFlip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.relicArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Status", "Initialized");
        BNO055IMU.Parameters parameters1 = new BNO055IMU.Parameters();
        parameters1.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters1.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters1.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters1.loggingEnabled = true;
        parameters1.loggingTag = "IMU";

        robot.imu = hardwareMap.get(BNO055IMU.class, "imu");
        robot.imu.initialize(parameters1);
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        robot.initServoPositions();

        // run until the end of the match (driver presses STOP)
        while(opModeIsActive()){
            if(endGameMode == 0) {
                if(gamepad1.start){
                    endGameMode = 1;
                    sleep(20);
                }
                 //Reset speed variables
                LF = 0;
                RF = 0;
                LR = 0;
                RR = 0;
//
//                // Get joystick values
                Y1 = gamepad1.left_stick_y * joyScale; // invert so up is positive
                X1 = gamepad1.left_stick_x * joyScale;
                // Y2 is not used at present
                X2 = -gamepad1.right_stick_x * joyScale;

                // Forward/back movement
                LF += Y1;
                RF += Y1;
                LR += Y1;
                RR += Y1;
                // Side to side movement
                LF += X1;
                RF -= X1;
                LR += X1;
                RR -= X1;
                //Rotation Movement
                LF += X2;
                RF += X2;
                LR -= X2;
                RR -= X2;

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

//                double x=-gamepad1.left_stick_x;
//                double turn=gamepad1.left_stick_y;
//                double y=gamepad1.right_stick_x;
//                left1.setPower(turn+y+x);
//                right1.setPower(y-turn-x);
//                left2.setPower(turn+y-x);
//                right2.setPower(y-turn+x);

//                double x=gamepad1.left_stick_x;
//                double y=gamepad1.left_stick_y;
//                double turn=-gamepad1.right_stick_x;
//                double heading;
//                if (x==0 && y==0) heading=0;
//                else if (x>=0) heading=Math.PI-Math.atan(y/x);
//                else heading=-Math.atan(y/x);
//                double pow=Math.sqrt(Math.pow(x,6)+Math.pow(y,6));
//                Orientation angles=robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
//                heading-=(angles.firstAngle+Math.PI/4.);
//                double pow1=Math.sqrt(2)*pow*Math.cos(heading);
//                double pow2=Math.sqrt(2)*pow*Math.sin(heading);
//                FrontLeft.setPower(turn+pow1);
//                BackLeft.setPower(turn+pow2);
//                FrontRight.setPower(pow1-turn);
//                BackRight.setPower(pow2-turn);


                // Intake Speed and Controls
                if (gamepad1.a) {
                    robot.intake.setPower(0.95);
                }
                if (gamepad1.b) {
                    robot.intake.setPower(-0.9);
                }
                if (gamepad1.y) {
                    robot.intake.setPower(0.0);
                }
                if (gamepad1.right_bumper) {
                    robot.bucketFinger.setPosition(robot.FINGER_OPEN);
                }
                else if (gamepad1.left_bumper) {
                    robot.rotateBox.setPosition(0.34);
                    //robot.intake.setPower(0.0);
                }

//                if(gamepad1.x){
//                    robot.glyphFlip.setPower(-0.9);
//                    sleep(650);
//                    robot.glyphFlip.setPower(0.9);
//                    sleep(600);
//                    robot.glyphFlip.setPower(0);
//                }
                if (gamepad1.dpad_right){
                    robot.moveServo(robot.lowerArm, 0.65, 100, 100);
                }
                if (gamepad1.dpad_left){
                    robot.moveServo(robot.lowerArm, 0.0, 100, 100);
                }

                if (gamepad1.x) {
//                    robot.glyphFlip.setPower(0.90);
//                    robot.glyphFlip.setTargetPosition(-50);
//                    sleep(500);
//                    robot.glyphFlip.setTargetPosition(-5);
//                    sleep(500);
//                    robot.glyphFlip.setPower(0.0);
                    robot.glyphFlip.setTargetPosition(-36);
                    sleep(50);
                    robot.glyphFlip.setPower(1);
                    sleep(50);
                    while(robot.glyphFlip.getCurrentPosition() > -35){
                        robot.glyphFlip.setPower(1);
                        telemetry.addData("GlyphFlip", robot.glyphFlip.getCurrentPosition());
                    }   robot.glyphFlip.setPower(0);
                }

            //This controls the winch and allows us to raise and lower it with limit switches to stop if from breaking itself.
             if (robot.upperLimit.getState() == false) { //Bottom Limit is pressed
                if (gamepad1.left_trigger > 0.5) {
                    robot.winch.setPower(-0.5);       //Raise winch for the time it is pressing the switch
                    robot.rotateBox.setPosition(0.21); //0.0
                    robot.bucketFinger.setPosition(robot.FINGER_OPEN);
                }else{
                    robot.winch.setPower(0.0);
                    robot.rotateBox.setPosition(0.21); //0.0
                    robot.bucketFinger.setPosition(robot.FINGER_OPEN);
                }
            }
            else if(robot.bottomLimit.getState() == false){//Top Switch is pressed
                robot.intake.setPower(0.0); //stops intake so we don't get a block stuck in
                if (gamepad1.right_trigger > 0.5){ //If the button is pressed we will lower the bucket
                    robot.winch.setPower(0.5);
//                    robot.bucketFinger.setPosition(robot.FINGER_OPEN);
                }else {
                    robot.winch.setPower(0.0);
//                    robot.bucketFinger.setPosition(robot.FINGER_OPEN);
                }
            }
            else{ //In the middle
                robot.intake.setPower(0.0); //Stop wheels to not get a block stuck
                if(gamepad1.right_trigger > 0.5){
                    robot.winch.setPower(0.5); //Lower winch
                    robot.rotateBox.setPosition(0.68);//Bucket at up postion
                } else if (gamepad1.left_trigger > 0.5) {
                    robot.winch.setPower(-0.5); //Raise winch
                    robot.rotateBox.setPosition(0.68);//Bucket at up postion
                }
                else{
                    robot.winch.setPower(0);
                    if(!gamepad1.right_bumper){
                        robot.bucketFinger.setPosition(robot.FINGER_CLOSED);
                    }
                }
            }
        }
            if(endGameMode == 1){
                if(gamepad1.start) {
                    endGameMode = 0;
                    sleep(20);
                }

//                double x=gamepad1.left_stick_x;
//                double y=gamepad1.left_stick_y;
//                double turn=-gamepad1.right_stick_x;
//                double heading;
//                if (x==0 && y==0) heading=0;
//                else if (x>=0) heading=Math.PI-Math.atan(y/x);
//                else heading=-Math.atan(y/x);
//                double pow=Math.sqrt(Math.pow(x,6)+Math.pow(y,6));
//                Orientation angles=robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
//                heading-=(angles.firstAngle+Math.PI/4.);
//                double pow1=Math.sqrt(2)*pow*Math.cos(heading);
//                double pow2=Math.sqrt(2)*pow*Math.sin(heading);
//                FrontLeft.setPower(turn+pow1);
//                BackLeft.setPower(turn+pow2);
//                FrontRight.setPower(pow1-turn);
//                BackRight.setPower(pow2-turn);

                    //Controls our linear slide
                robot.relicArm.setPower(-gamepad1.right_stick_y);

                    //Controls the claw on our linear slide
                    if (gamepad1.left_bumper && relicClawMode == 0) {
                        robot.pinch.setPosition(0); //Closes claw
                        sleep(500);
                        relicClawMode++;
                    } else if (gamepad1.left_bumper && relicClawMode == 1) {
                        robot.pinch.setPosition(1); //Opens claw
                        sleep(500);
                        relicClawMode--;
                    }
                    //Slider Twisting Controls
                    if (gamepad1.right_bumper && relicTwistMode == 0) {
                        robot.twist.setPosition(0.95); //Twists the claw down'
                        sleep(500);
                        relicTwistMode++;
                    } else if (gamepad1.right_bumper && relicTwistMode == 1) {
                        robot.pinch.setPosition(1);
                        robot.twist.setPosition(0.0); // Twists the claw up
                        sleep(500);
                        relicTwistMode--;
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
//            telemetry.addData("Left1 Encoder", robot.left1.getCurrentPosition());
//            telemetry.addData("Left2 Encoder", robot.left2.getCurrentPosition());
//            telemetry.addData("Right1 Encoder", robot.right1.getCurrentPosition());
//            telemetry.addData("Right2 Encoder", robot.right2.getCurrentPosition());
            telemetry.addData("endGameMode", endGameMode);
            telemetry.addData("pincher", robot.pinch.getPosition());
            telemetry.addData("Twister", robot.twist.getPosition());

//            telemetry.addData("lowerArm", robot.lowerArm.getPosition());
//            telemetry.addData("rotateArm", robot.rotateArm.getPosition());
//            telemetry.addData("upperLimit",robot.upperLimit.getState());
//            telemetry.addData("lowerLimit",robot.bottomLimit.getState());
//            telemetry.addData("Driver Mode", endGameMode);
//            telemetry.addData("Color Red", robot.jewelArm.red());
//            telemetry.addData("Color Blue", robot.jewelArm.blue());
//            telemetry.addData("GlyphFlip Enco", robot.glyphFlip.getCurrentPosition());
            telemetry.update();
        }
    }
}