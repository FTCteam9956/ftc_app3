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

@Autonomous(name = "SuperBlueStraight", group = "Autonomous")
//@Disabled
public class SuperBlueStraight extends LinearOpMode{
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
        robot.rotateBox.setPosition(0.3);
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

//        //1 - LEFT, 2 - RIGHT, 3 - CENTER, 0 - NOT VISIBLE, 4 - TIMEOUT
        if (targetPosition == 1) {
            robot.left1.setTargetPosition(-1125);
            robot.left2.setTargetPosition(-1125);
            robot.right1.setTargetPosition(-1125);
            robot.right2.setTargetPosition(-1125);
            robot.left1.setPower(-0.25);
            robot.left2.setPower(-0.25);
            robot.right1.setPower(-0.25);
            robot.right2.setPower(-0.25);
            sleep(100);
            while (robot.right1.isBusy()) {
                telemetry.addData("Left1 Encoder", robot.left1.getCurrentPosition());
                telemetry.addData("Left2 Encoder", robot.left2.getCurrentPosition());
                telemetry.addData("Right1 Encoder", robot.right1.getCurrentPosition());
                telemetry.addData("Right2 Encoder", robot.right2.getCurrentPosition());
                telemetry.update();
                if (robot.angles.firstAngle < 0.000001) {
                    robot.left1.setPower(-0.25);
                    robot.left2.setPower(-0.25);
                    robot.right1.setPower(-0.25 * POWER);
                    robot.right2.setPower(-0.25 * POWER);
                    telemetry.update();
                } else if (robot.angles.firstAngle > 0.000001) {
                    robot.left1.setPower(-0.25 * POWER);
                    robot.left2.setPower(-0.25 * POWER);
                    robot.right1.setPower(-0.25);
                    robot.right2.setPower(-0.25);
                    telemetry.update();
                } else {
                    robot.left1.setPower(-0.25);
                    robot.left2.setPower(-0.25);
                    robot.right1.setPower(-0.25);
                    robot.right2.setPower(-0.25);
                    telemetry.update();
                }
            }
            robot.left1.setPower(0);
            robot.left2.setPower(0);
            robot.right1.setPower(0);
            robot.right2.setPower(0);
            sleep(500);

            robot.setRunMode("RUN_USING_ENCODER");
            sleep(50);
            while (robot.angles.firstAngle > 118 || robot.angles.firstAngle < 101) {
                telemetry.update();
                if (robot.angles.firstAngle > 118) {
                    robot.left1.setPower(0.13);
                    robot.left2.setPower(0.13);
                    robot.right1.setPower(-0.13);
                    robot.right2.setPower(-0.13);
                } else if (robot.angles.firstAngle < 101) {
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
            robot.moveServo(robot.rotateBox, 0.63, 100,500);
            sleep(1000);

            robot.setRunMode("STOP_AND_RESET_ENCODER");
            sleep(50);
            robot.setRunMode("RUN_TO_POSITION");
            sleep(50);
            robot.left1.setTargetPosition(-200);
            robot.left2.setTargetPosition(-200);
            robot.right1.setTargetPosition(-200);
            robot.right2.setTargetPosition(-200);
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
            robot.left1.setPower(0.1);
            robot.left2.setPower(0.1);
            robot.right1.setPower(0.1);
            robot.right2.setPower(0.1);

        }
        if (targetPosition == 2) {
            robot.left1.setTargetPosition(-1150);
            robot.left2.setTargetPosition(-1150);
            robot.right1.setTargetPosition(-1150);
            robot.right2.setTargetPosition(-1150);
            robot.left1.setPower(-0.25);
            robot.left2.setPower(-0.25);
            robot.right1.setPower(-0.25);
            robot.right2.setPower(-0.25);
            sleep(100);
            while (robot.right1.isBusy()) {
                telemetry.addData("Left1 Encoder", robot.left1.getCurrentPosition());
                telemetry.addData("Left2 Encoder", robot.left2.getCurrentPosition());
                telemetry.addData("Right1 Encoder", robot.right1.getCurrentPosition());
                telemetry.addData("Right2 Encoder", robot.right2.getCurrentPosition());
                telemetry.update();
                if (robot.angles.firstAngle < 0.000001) {
                    robot.left1.setPower(-0.25);
                    robot.left2.setPower(-0.25);
                    robot.right1.setPower(-0.25 * POWER);
                    robot.right2.setPower(-0.25 * POWER);
                    telemetry.update();
                } else if (robot.angles.firstAngle > 0.000001) {
                    robot.left1.setPower(-0.25 * POWER);
                    robot.left2.setPower(-0.25 * POWER);
                    robot.right1.setPower(-0.25);
                    robot.right2.setPower(-0.25);
                    telemetry.update();
                } else {
                    robot.left1.setPower(-0.25);
                    robot.left2.setPower(-0.25);
                    robot.right1.setPower(-0.25);
                    robot.right2.setPower(-0.25);
                    telemetry.update();
                }
            }
            robot.left1.setPower(0);
            robot.left2.setPower(0);
            robot.right1.setPower(0);
            robot.right2.setPower(0);
            sleep(500);

            robot.setRunMode("RUN_USING_ENCODER");
            sleep(50);
            while (robot.angles.firstAngle > 63 || robot.angles.firstAngle < 50) {
                telemetry.update();
                if (robot.angles.firstAngle > 63) {
                    robot.left1.setPower(0.13);
                    robot.left2.setPower(0.13);
                    robot.right1.setPower(-0.13);
                    robot.right2.setPower(-0.13);
                } else if (robot.angles.firstAngle < 50) {
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
            robot.moveServo(robot.rotateBox, 0.63, 100,500);
            sleep(1000);

            robot.setRunMode("STOP_AND_RESET_ENCODER");
            sleep(50);
            robot.setRunMode("RUN_TO_POSITION");
            sleep(50);
            robot.left1.setTargetPosition(-200);
            robot.left2.setTargetPosition(-200);
            robot.right1.setTargetPosition(-200);
            robot.right2.setTargetPosition(-200);
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
            robot.left1.setPower(0.1);
            robot.left2.setPower(0.1);
            robot.right1.setPower(0.1);
            robot.right2.setPower(0.1);

        } else if (targetPosition == 3) {
            robot.left1.setTargetPosition(-1350);
            robot.left2.setTargetPosition(-1350);
            robot.right1.setTargetPosition(-1350);
            robot.right2.setTargetPosition(-1350);
            robot.left1.setPower(-0.25);
            robot.left2.setPower(-0.25);
            robot.right1.setPower(-0.25);
            robot.right2.setPower(-0.25);
            sleep(100);
            while (robot.right1.isBusy()) {
                telemetry.addData("Left1 Encoder", robot.left1.getCurrentPosition());
                telemetry.addData("Left2 Encoder", robot.left2.getCurrentPosition());
                telemetry.addData("Right1 Encoder", robot.right1.getCurrentPosition());
                telemetry.addData("Right2 Encoder", robot.right2.getCurrentPosition());
                telemetry.update();
                if (robot.angles.firstAngle < 0.000001) {
                    robot.left1.setPower(-0.25);
                    robot.left2.setPower(-0.25);
                    robot.right1.setPower(-0.25 * POWER);
                    robot.right2.setPower(-0.25 * POWER);
                    telemetry.update();
                } else if (robot.angles.firstAngle > 0.000001) {
                    robot.left1.setPower(-0.25 * POWER);
                    robot.left2.setPower(-0.25 * POWER);
                    robot.right1.setPower(-0.25);
                    robot.right2.setPower(-0.25);
                    telemetry.update();
                } else {
                    robot.left1.setPower(-0.25);
                    robot.left2.setPower(-0.25);
                    robot.right1.setPower(-0.25);
                    robot.right2.setPower(-0.25);
                    telemetry.update();
                }
            }
            robot.left1.setPower(0);
            robot.left2.setPower(0);
            robot.right1.setPower(0);
            robot.right2.setPower(0);
            sleep(500);

            robot.setRunMode("RUN_USING_ENCODER");
            sleep(50);
            while (robot.angles.firstAngle > 118 || robot.angles.firstAngle < 101) {
                telemetry.update();
                if (robot.angles.firstAngle > 118) {
                    robot.left1.setPower(0.13);
                    robot.left2.setPower(0.13);
                    robot.right1.setPower(-0.13);
                    robot.right2.setPower(-0.13);
                } else if (robot.angles.firstAngle < 101) {
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
            robot.moveServo(robot.rotateBox, 0.63, 100,500);
            sleep(1000);

            robot.setRunMode("STOP_AND_RESET_ENCODER");
            sleep(50);
            robot.setRunMode("RUN_TO_POSITION");
            sleep(50);
            robot.left1.setTargetPosition(-200);
            robot.left2.setTargetPosition(-200);
            robot.right1.setTargetPosition(-200);
            robot.right2.setTargetPosition(-200);
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
            robot.left1.setPower(0.1);
            robot.left2.setPower(0.1);
            robot.right1.setPower(0.1);
            robot.right2.setPower(0.1);
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
        robot.rotateBox.setPosition(0.34);
        sleep(500);
    }

    public int lookForVuMark(VuforiaTrackable rTemplate) {
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(rTemplate);
        int returnValue = 0;
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            if (vuMark == RelicRecoveryVuMark.LEFT) { // Test to see if Image is the "LEFT" image and display value.
                telemetry.addData("VuMark is", "Left");
                returnValue = 1;
            } else if (vuMark == RelicRecoveryVuMark.RIGHT) { // Test to see if Image is the "RIGHT" image and display values.
                telemetry.addData("VuMark is", "Right");
                returnValue = 2;
            } else if (vuMark == RelicRecoveryVuMark.CENTER) { // Test to see if Image is the "CENTER" image and display values.
                telemetry.addData("VuMark is", "Center");
                returnValue = 3;
            }
        } else {
            telemetry.addData("VuMark", "not visible");
            returnValue = 4;
        }
        telemetry.update();
        return (returnValue);
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
                        return robot.formatAngle(robot.angles.angleUnit, robot.angles.firstAngle); // removed negative sign
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

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public void strafe(String direction, double power) {
        if (direction == "right") {
            robot.left1.setPower(-power);
            robot.left2.setPower(power);
            robot.right1.setPower(power);
            robot.right2.setPower(-power);
        }
        if (direction == "left") {
            robot.left1.setPower(power);
            robot.left2.setPower(-power);
            robot.right1.setPower(-power);
            robot.right2.setPower(power);
        }
    }
}