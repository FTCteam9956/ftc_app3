package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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

@Autonomous(name = "SuperRedStraightTest", group = "Autonomous")
//@Disabled
public class SuperRedStraightTest extends LinearOpMode {
    RRHardwarePresets robot = new RRHardwarePresets();

    VuforiaLocalizer vuforia;

    int targetPosition = 0;
    private static double POWER = 1.15;

    public void runOpMode() {
        robot.init(hardwareMap);
        robot.setRunMode("STOP_AND_RESET_ENCODER");
        robot.setRunMode("RUN_USING_ENCODER");

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

        if (targetPosition == 0){
            targetPosition = 3;
        }
        telemetry.addData("TargetPosition", targetPosition);

        runAutonomous(targetPosition);
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

    public void runAutonomous(int targetPosition){
        if(targetPosition == 1){
            robot.driveForwardSetDistance(0.2, 1175);
            while(robot.anyMotorsBusy()){
                correctDriving(-0.2);
            }
            turnToAngle("CW",0.2, -90);
        }
        if(targetPosition == 2){
            robot.driveForwardSetDistance(0.2, 1150);
            while(robot.anyMotorsBusy()){
                correctDriving(-0.2);
            }
            turnToAngle("CW", 0.2, -90);
        }
        if(targetPosition == 3){
            robot.driveForwardSetDistance(0.2, 935);
            while(robot.anyMotorsBusy()){
                correctDriving(0.2);
            }
            turnToAngle("CW", 0.2, -95);
        }
    }

    public void strafe(String direction, double power){
        if(direction == "right"){
            robot.left1.setPower(-power);
            robot.left2.setPower(power);
            robot.right1.setPower(power);
            robot.right2.setPower(-power);
        }
        if(direction == "left"){
            robot.left1.setPower(power);
            robot.left2.setPower(-power);
            robot.right1.setPower(-power);
            robot.right2.setPower(power);
        }
    }
}