package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Locale;
@Autonomous(name = "OneBlockBlueTurn", group = "Autonomous")
//@Disabled
public class SuperBlueTurn extends LinearOpMode{
    RRHardwarePresets robot = new RRHardwarePresets();
    VuforiaLocalizer vuforia;

    public static final double POWER = 1.15;
    int targetPosition = 0;

    public void runOpMode() {
        robot.init(hardwareMap);//Robot moves during init().
        robot.setRunMode("STOP_AND_RESET_ENCODER");
        robot.setRunMode("RUN_TO_POSITION");
        robot.winch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.winch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AU0kxmH/////AAAAGV4QPVzzlk6Hl969cSL2pmM4F6TuzhWZS/dKbY45MEzS31OYJxLbKewdt1CSFrmpvrpPnIYZyBJt3kFRJQCtEXet0LHd2KtBB5NsDTuBADfgIsQk+7TSWSTFDjSi8SpKaXtAjZPKePwGDaIKf5VK6mRBYaWxqTHpZFBlelejLHxib8qweOFrJjKTsbgsb2pwVNFhDeJabbI5aed8JSI8LxHs0368ezQfnCz3UK9u8pC1DkKgcwdgoJ0OXBKChXB4v2lEnIrQf7ROYcPtVuRJJ5/prBoyfR11pvp69iCA25Cttz9xVsdZ9VliuQJ4UO37Hzhz1dB2SPnxTQQmCJMDoDKqe3wpiCFu8ThQ4pmS05ka";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK; // Use FRONT Camera (Change to BACK if you want to use that one)
        parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES; // Display Axes

        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);

        //Initialize Gyro
        BNO055IMU.Parameters parameters1 = new BNO055IMU.Parameters();
        parameters1.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters1.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters1.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters1.loggingEnabled = true;
        parameters1.loggingTag = "IMU";
        parameters1.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        robot.imu = hardwareMap.get(BNO055IMU.class, "imu");
        robot.imu.initialize(parameters1);

        composeTelemetry();

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return robot.formatAngle(robot.angles.angleUnit, robot.angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override
                    public String value() {
                        return robot.formatAngle(robot.angles.angleUnit, robot.angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override
                    public String value() {
                        return robot.formatAngle(robot.angles.angleUnit, robot.angles.thirdAngle);
                    }
                });
        relicTrackables.activate();
        while (!opModeIsActive()) {
            telemetry.addData("Target Position", targetPosition);
            telemetry.update();
            targetPosition = lookForVuMark(relicTemplate);//1 - LEFT, 2 - RIGHT, 3 - CENTER, 0 - NOT VISIBLE, 4 - TIMEOUT}
        }

        waitForStart();
        robot.initServoPositions();
        robot.rotateBox.setPosition(robot.ROTATEBOX_MIDDLE);
        sleep(500);
        if (targetPosition == 0) {
            targetPosition = 3;
        }
        robot.moveServo(robot.lowerArm, robot.JEWEL_ARM_DOWN, 200, 300);

        sleep(300);

        int loopBreak = 0;
        while (loopBreak == 0) {
            sleep(300);
            if (robot.jewelArm.red() > robot.jewelArm.blue()) {
                knockOffBall(1); //go left
                telemetry.addData("Status", "Confirmed Red Ball!");
                sleep(500);
                loopBreak = 1;
            } else if (robot.jewelArm.red() < robot.jewelArm.blue()) {
                if (robot.jewelArm.blue() > 27) {
                    knockOffBall(0); //go right
                    telemetry.addData("Status", "Confirmed Blue Ball!");
                    sleep(500);
                    loopBreak = 1;
                } else {
                    telemetry.addData("Status", "Cannot determine color! Double Checking!");
                    robot.moveServo(robot.lowerArm, robot.JEWEL_ARM_UP, 200, 300);
                    sleep(300);
                    robot.rotateArm.setPosition(0.32);
                    robot.moveServo(robot.lowerArm, robot.JEWEL_ARM_DOWN, 200, 300);
                    sleep(300);
                    if (robot.jewelArm.red() > robot.jewelArm.blue()) {
                        knockOffBall(1); //Go left
                        telemetry.addData("Status", "Confirmed Red Ball!");
                        sleep(500);
                        loopBreak = 1;
                    } else if (robot.jewelArm.red() < robot.jewelArm.blue()) {
                        if (robot.jewelArm.blue() > 27) {
                            knockOffBall(0); //go right
                            telemetry.addData("Status", "Confirmed Blue Ball!");
                            sleep(500);
                            loopBreak = 1;
                        } else {
                            telemetry.addData("Status", "Cannot determine color! You screwed up!");
                            loopBreak = 1;
                        }
                    }
                }
            }
        }
        telemetry.update();
        sleep(300);
        robot.moveServo(robot.lowerArm, robot.JEWEL_ARM_UP, 200, 300);
        sleep(500);

        robot.left1.setTargetPosition(-850);
        robot.left2.setTargetPosition(-850);
        robot.right1.setTargetPosition(-850);
        robot.right2.setTargetPosition(-850);
        robot.left1.setPower(-0.25);
        robot.left2.setPower(-0.25);
        robot.right1.setPower(-0.25);
        robot.right2.setPower(-0.25);
        sleep(100);
        while(robot.left1.isBusy()){
            robot.glyphFlip.setTargetPosition(0);
            robot.glyphFlip.setPower(0.3);
        }
        sleep(300);

        robot.setRunMode("STOP_AND_RESET_ENCODER");
        sleep(50);
        robot.setRunMode("RUN_TO_POSITION");
        sleep(50);

        if(targetPosition == 1){
            robot.left1.setTargetPosition(-650);
            robot.left2.setTargetPosition(650);
            robot.right1.setTargetPosition(-650); //strafe left
            robot.right2.setTargetPosition(650);
            robot.left1.setPower(-0.2);
            robot.left2.setPower(0.2);
            robot.right1.setPower(-0.2);
            robot.right2.setPower(0.2);
            sleep(100);
            while (robot.left1.isBusy()){
                telemetry.addData("Left1 Encoder", robot.left1.getCurrentPosition());
                telemetry.addData("Left2 Encoder", robot.left2.getCurrentPosition());
                telemetry.addData("Right1 Encoder", robot.right1.getCurrentPosition());
                telemetry.addData("Right2 Encoder", robot.right2.getCurrentPosition());
                telemetry.update();
            }
            robot.left1.setPower(0);
            robot.left2.setPower(0);
            robot.right1.setPower(0);
            robot.right2.setPower(0);
            sleep(500);

            robot.setRunMode("RUN_USING_ENCODER");
            sleep(50);
            while (robot.angles.firstAngle > 35 || robot.angles.firstAngle < 22) { //30-17
                telemetry.update();
                if (robot.angles.firstAngle > 35) {
                    robot.left1.setPower(0.13);
                    robot.left2.setPower(0.13);
                    robot.right1.setPower(-0.13);
                    robot.right2.setPower(-0.13);
                } else if (robot.angles.firstAngle < 22) {
                    robot.left1.setPower(-0.13);
                    robot.left2.setPower(-0.13);
                    robot.right1.setPower(0.13);
                    robot.right2.setPower(0.13);
                }
            }
            robot.left1.setPower(0);
            robot.left2.setPower(0);
            robot.right1.setPower(0);
            robot.right2.setPower(0);
            sleep(500);

            robot.setRunMode("STOP_AND_RESET_ENCODER");
            sleep(50);
            robot.setRunMode("RUN_TO_POSITION");
            sleep(50);

//            robot.left1.setTargetPosition(-150);
//            robot.left2.setTargetPosition(-150);
//            robot.right1.setTargetPosition(-150);
//            robot.right2.setTargetPosition(-150);
//            robot.left1.setPower(-0.2);
//            robot.left2.setPower(-0.2);
//            robot.right1.setPower(-0.2);
//            robot.right2.setPower(-0.2);
//            sleep(100);
//            while(robot.left1.isBusy()){}

            sleep(500);
            robot.moveServo(robot.rotateBox, robot.ROTATEBOX_UP, 100,500);
            sleep(100);

            robot.setRunMode("STOP_AND_RESET_ENCODER");
            sleep(50);
            robot.setRunMode("RUN_TO_POSITION");
            sleep(50);
            robot.left1.setTargetPosition(-300);
            robot.left2.setTargetPosition(-300);
            robot.right1.setTargetPosition(-300);
            robot.right2.setTargetPosition(-300);
            robot.left1.setPower(-0.25);
            robot.left2.setPower(-0.25);
            robot.right1.setPower(-0.25);
            robot.right2.setPower(-0.25);

            sleep(500);
            robot.setRunMode("STOP_AND_RESET_ENCODER");
            sleep(50);
            robot.setRunMode("RUN_TO_POSITION");
            sleep(50);
            robot.left1.setTargetPosition(150);
            robot.left2.setTargetPosition(150);
            robot.right1.setTargetPosition(150);
            robot.right2.setTargetPosition(150);
            robot.left1.setPower(0.2);
            robot.left2.setPower(0.2);
            robot.right1.setPower(0.2);
            robot.right2.setPower(0.2);
            sleep(1000);

        }if (targetPosition == 2) {
            robot.left1.setTargetPosition(-1300);
            robot.left2.setTargetPosition(1300);
            robot.right1.setTargetPosition(-1300); //strafe left
            robot.right2.setTargetPosition(1300);
            robot.left1.setPower(-0.2);
            robot.left2.setPower(0.2);
            robot.right1.setPower(-0.2);
            robot.right2.setPower(0.2);
            sleep(100);
            while (robot.left1.isBusy()){
                telemetry.addData("Left1 Encoder", robot.left1.getCurrentPosition());
                telemetry.addData("Left2 Encoder", robot.left2.getCurrentPosition());
                telemetry.addData("Right1 Encoder", robot.right1.getCurrentPosition());
                telemetry.addData("Right2 Encoder", robot.right2.getCurrentPosition());
                telemetry.update();
            }
            robot.left1.setPower(0);
            robot.left2.setPower(0);
            robot.right1.setPower(0);
            robot.right2.setPower(0);
            sleep(500);

            robot.setRunMode("RUN_USING_ENCODER");
            sleep(50);
            while (robot.angles.firstAngle > 30 || robot.angles.firstAngle < 17) {
                telemetry.update();
                if (robot.angles.firstAngle > 30) {
                    robot.left1.setPower(0.13);
                    robot.left2.setPower(0.13);
                    robot.right1.setPower(-0.13);
                    robot.right2.setPower(-0.13);
                } else if (robot.angles.firstAngle < 17) {
                    robot.left1.setPower(-0.13);
                    robot.left2.setPower(-0.13);
                    robot.right1.setPower(0.13);
                    robot.right2.setPower(0.13);
                }
            }
            robot.left1.setPower(0);
            robot.left2.setPower(0);
            robot.right1.setPower(0);
            robot.right2.setPower(0);
            sleep(500);

            robot.setRunMode("STOP_AND_RESET_ENCODER");
            sleep(50);
            robot.setRunMode("RUN_TO_POSITION");
            sleep(50);

//            robot.left1.setTargetPosition(-150);
//            robot.left2.setTargetPosition(-150);
//            robot.right1.setTargetPosition(-150);
//            robot.right2.setTargetPosition(-150);
//            robot.left1.setPower(-0.2);
//            robot.left2.setPower(-0.2);
//            robot.right1.setPower(-0.2);
//            robot.right2.setPower(-0.2);
//            sleep(100);
//            while(robot.left1.isBusy()){}

            sleep(500);
            robot.moveServo(robot.rotateBox, robot.ROTATEBOX_UP, 100,500);
            sleep(100);

            robot.setRunMode("STOP_AND_RESET_ENCODER");
            sleep(50);
            robot.setRunMode("RUN_TO_POSITION");
            sleep(50);
            robot.left1.setTargetPosition(-300);
            robot.left2.setTargetPosition(-300);
            robot.right1.setTargetPosition(-300);
            robot.right2.setTargetPosition(-300);
            robot.left1.setPower(-0.25);
            robot.left2.setPower(-0.25);
            robot.right1.setPower(-0.25);
            robot.right2.setPower(-0.25);

            sleep(500);
            robot.setRunMode("STOP_AND_RESET_ENCODER");
            sleep(50);
            robot.setRunMode("RUN_TO_POSITION");
            sleep(50);
            robot.left1.setTargetPosition(150);
            robot.left2.setTargetPosition(150);
            robot.right1.setTargetPosition(150);
            robot.right2.setTargetPosition(150);
            robot.left1.setPower(0.2);
            robot.left2.setPower(0.2);
            robot.right1.setPower(0.2);
            robot.right2.setPower(0.2);
            sleep(1000);

        }if (targetPosition == 3) {
            robot.left1.setTargetPosition(-1000);
            robot.left2.setTargetPosition(1000);
            robot.right1.setTargetPosition(-1000); //strafe left
            robot.right2.setTargetPosition(1000);
            robot.left1.setPower(-0.2);
            robot.left2.setPower(0.2);
            robot.right1.setPower(-0.2);
            robot.right2.setPower(0.2);
            sleep(100);
            while (robot.left1.isBusy()){
                telemetry.addData("Left1 Encoder", robot.left1.getCurrentPosition());
                telemetry.addData("Left2 Encoder", robot.left2.getCurrentPosition());
                telemetry.addData("Right1 Encoder", robot.right1.getCurrentPosition());
                telemetry.addData("Right2 Encoder", robot.right2.getCurrentPosition());
                telemetry.update();
            }
            robot.left1.setPower(0);
            robot.left2.setPower(0);
            robot.right1.setPower(0);
            robot.right2.setPower(0);
            sleep(500);

            robot.setRunMode("RUN_USING_ENCODER");
            sleep(50);
            while (robot.angles.firstAngle > 30 || robot.angles.firstAngle < 17) {
                telemetry.update();
                if (robot.angles.firstAngle > 30) {
                    robot.left1.setPower(0.13);
                    robot.left2.setPower(0.13);
                    robot.right1.setPower(-0.13);
                    robot.right2.setPower(-0.13);
                } else if (robot.angles.firstAngle < 17) {
                    robot.left1.setPower(-0.13);
                    robot.left2.setPower(-0.13);
                    robot.right1.setPower(0.13);
                    robot.right2.setPower(0.13);
                }
            }
            robot.left1.setPower(0);
            robot.left2.setPower(0);
            robot.right1.setPower(0);
            robot.right2.setPower(0);
            sleep(500);

            robot.setRunMode("STOP_AND_RESET_ENCODER");
            sleep(50);
            robot.setRunMode("RUN_TO_POSITION");
            sleep(50);

//            robot.left1.setTargetPosition(-150);
//            robot.left2.setTargetPosition(-150);
//            robot.right1.setTargetPosition(-150);
//            robot.right2.setTargetPosition(-150);
//            robot.left1.setPower(-0.2);
//            robot.left2.setPower(-0.2);
//            robot.right1.setPower(-0.2);
//            robot.right2.setPower(-0.2);
//            sleep(100);
//            while(robot.left1.isBusy()){}

            sleep(500);
            robot.moveServo(robot.rotateBox, robot.ROTATEBOX_UP, 100,500);
            sleep(100);

            robot.setRunMode("STOP_AND_RESET_ENCODER");
            sleep(50);
            robot.setRunMode("RUN_TO_POSITION");
            sleep(50);
            robot.left1.setTargetPosition(-300);
            robot.left2.setTargetPosition(-300);
            robot.right1.setTargetPosition(-300);
            robot.right2.setTargetPosition(-300);
            robot.left1.setPower(-0.25);
            robot.left2.setPower(-0.25);
            robot.right1.setPower(-0.25);
            robot.right2.setPower(-0.25);

            sleep(500);
            robot.setRunMode("STOP_AND_RESET_ENCODER");
            sleep(50);
            robot.setRunMode("RUN_TO_POSITION");
            sleep(50);
            robot.left1.setTargetPosition(150);
            robot.left2.setTargetPosition(150);
            robot.right1.setTargetPosition(150);
            robot.right2.setTargetPosition(150);
            robot.left1.setPower(0.2);
            robot.left2.setPower(0.2);
            robot.right1.setPower(0.2);
            robot.right2.setPower(0.2);
            sleep(1000);
        }
        robot.glyphFlip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(50);
        robot.glyphFlip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(50);
        robot.glyphFlip.setTargetPosition(-36);
        sleep(50);
        robot.glyphFlip.setPower(1);
        sleep(50);
        while(robot.glyphFlip.getCurrentPosition() > -35){
            robot.glyphFlip.setPower(1);
            telemetry.addData("GlyphFlip", robot.glyphFlip.getCurrentPosition());
        }
        robot.glyphFlip.setPower(0);
        robot.rotateBox.setPosition(robot.ROTATEBOX_MIDDLE);
        sleep(500);
    }
    void composeTelemetry() {

        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                robot.gravity = robot.imu.getGravity();
            }
        });
        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override
                    public String value() {
                        return robot.imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override
                    public String value() {
                        return robot.imu.getCalibrationStatus().toString();
                    }
                });
        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return robot.formatAngle(robot.angles.angleUnit, robot.angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override
                    public String value() {
                        return robot.formatAngle(robot.angles.angleUnit, robot.angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override
                    public String value() {
                        return robot.formatAngle(robot.angles.angleUnit, robot.angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override
                    public String value() {
                        return robot.gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override
                    public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(robot.gravity.xAccel * robot.gravity.xAccel
                                        + robot.gravity.yAccel * robot.gravity.yAccel
                                        + robot.gravity.zAccel * robot.gravity.zAccel));
                    }
                });
    }

    public int lookForVuMark(VuforiaTrackable rTemplate){
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(rTemplate);
        int returnValue = 0;
        if(vuMark != RelicRecoveryVuMark.UNKNOWN){
            if(vuMark == RelicRecoveryVuMark.LEFT){ // Test to see if Image is the "LEFT" image and display value.
                telemetry.addData("VuMark is", "Left");
                returnValue = 1;
            }else if(vuMark == RelicRecoveryVuMark.RIGHT){ // Test to see if Image is the "RIGHT" image and display values.
                telemetry.addData("VuMark is", "Right");
                returnValue = 2;
            }else if(vuMark == RelicRecoveryVuMark.CENTER){ // Test to see if Image is the "CENTER" image and display values.
                telemetry.addData("VuMark is", "Center");
                returnValue = 3;
            }
        }
        telemetry.update();
        return(returnValue);
    }

    public void turnToAngle(String turnDirection, double power, int targetAngle){
        if(turnDirection == "CCW") {
            if(robot.angles.firstAngle < (targetAngle - 5)){
                robot.turn("CCW", power);
            }
            else if(robot.angles.firstAngle > (targetAngle + 5)){
                robot.turn("CW", power);
            }
            else{//(targetAngle - 5) < robot.angles.firstAngle < (targetAngle + 5)
                robot.setMotorPower(0.0);
            }
        }
        if(turnDirection == "CW"){
            if(robot.angles.firstAngle < (targetAngle - 5)){
                robot.turn("CW", power);
            }
            else if(robot.angles.firstAngle > (targetAngle + 5)){
                robot.turn("CCW", power);
            }
            else{//(targetAngle - 5) < robot.angles.firstAngle < (targetAngle + 5)
                robot.setMotorPower(0.0);
            }
        }
    }

    public void correctDriving(double power){
        if(robot.angles.firstAngle < 0.001){
            robot.left1.setPower(power);
            robot.left2.setPower(power);
            robot.right1.setPower(power * POWER);
            robot.right2.setPower(power * POWER);
        }
        else if(robot.angles.firstAngle > -0.001){
            robot.left1.setPower(power * POWER);
            robot.left2.setPower(power * POWER);
            robot.right1.setPower(power);
            robot.right2.setPower(power);
        }
        else{ // -0.001 < robot.angle.firstAngle < 0.001
            robot.left1.setPower(power);
            robot.left2.setPower(power);
            robot.right1.setPower(power);
            robot.right2.setPower(power);
        }
    }

    public void strafe(String direction, double power, int targetPosition){
        robot.setRunMode("RUN_TO_POSITION");
//        if(direction == "right") {
//            robot.left1.setTargetPosition(-targetPosition);
//            robot.left2.setTargetPosition(targetPosition);
//            robot.right1.setTargetPosition(targetPosition);
//            robot.right2.setTargetPosition(-targetPosition);
//            robot.left1.setPower(-power);
//            robot.left2.setPower(power);
//            robot.right1.setPower(power);
//            robot.right2.setPower(-power);
//        }
        if(direction == "left"){
            robot.left1.setTargetPosition(-targetPosition);
            robot.left2.setTargetPosition(targetPosition);
            robot.right1.setTargetPosition(-targetPosition);
            robot.right2.setTargetPosition(targetPosition);
            robot.left1.setPower(-power);
            robot.left2.setPower(power);
            robot.right1.setPower(-power);
            robot.right2.setPower(power);
        }
        robot.setRunMode("RUN_USING_ENCODER");
    }

    public void knockOffBall(int selection) {

        if (selection == 0) {
            robot.rotateArm.setPosition(robot.ROTATE_LEFT);
        }
        if (selection == 1) {
            robot.rotateArm.setPosition(robot.ROTATE_RIGHT);
        }
        sleep(500);
        robot.lowerArm.setPosition(robot.JEWEL_ARM_UP);
        sleep(500);
        robot.rotateArm.setPosition(robot.ROTATE_MID);
    }
}