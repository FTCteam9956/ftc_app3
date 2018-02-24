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

@Autonomous(name = "SuperRedStraight", group = "Autonomous")
@Disabled
public class SuperRedStraight extends LinearOpMode{
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

        if (targetPosition == 1) {
            robot.driveForwardSetDistance(0.2, 1175);
            while (robot.left1.isBusy() & robot.right1.isBusy()){
                telemetry.update();
                if (robot.angles.firstAngle < 0.000001) {
                    robot.left1.setPower(-0.2);
                    robot.left2.setPower(-0.2);
                    robot.right1.setPower(-0.2 * POWER);
                    robot.right2.setPower(-0.2 * POWER);
                } else if (robot.angles.firstAngle > 0.000001) {
                    robot.left1.setPower(-0.2 * POWER);
                    robot.left2.setPower(-0.2 * POWER);
                    robot.right1.setPower(-0.2);
                    robot.right2.setPower(-0.2);
                } else {
                    robot.left1.setPower(-0.2);
                    robot.left2.setPower(-0.2);
                    robot.right1.setPower(-0.2);
                    robot.right2.setPower(-0.2);
                }
            }
            telemetry.update();
            while (opModeIsActive()) {
                telemetry.update();
                if (robot.angles.firstAngle < 84) {
                    robot.left1.setPower(0.05);
                    robot.left2.setPower(0.05);
                    robot.right1.setPower(-0.05);
                    robot.right2.setPower(-0.05);
                } else if (robot.angles.firstAngle > 96) {
                    robot.left1.setPower(-0.05);
                    robot.left2.setPower(-0.05);
                    robot.right1.setPower(0.05);
                    robot.right2.setPower(0.05);
                } else if (robot.angles.firstAngle > 84 && robot.angles.firstAngle < 96) {
                    robot.left1.setPower(0);
                    robot.left2.setPower(0);
                    robot.right1.setPower(0);
                    robot.right2.setPower(0);
                    sleep(30000);
                }
            }
        }
            if (targetPosition == 2) {
                robot.driveForwardSetDistance(0.2, 1150);
                while (robot.left1.isBusy() & robot.right1.isBusy()) {
                    //double firstAngle = Math.abs(robot.angles.firstAngle);
                    //double POWER = -1.03;
                    telemetry.update();
                    if (robot.angles.firstAngle < 0.000001) {
                        robot.left1.setPower(-0.2);
                        robot.left2.setPower(-0.2);
                        robot.right1.setPower(-0.2 * POWER);
                        robot.right2.setPower(-0.2 * POWER);
                    } else if (robot.angles.firstAngle > 0.000001) {
                        robot.left1.setPower(-0.2 * POWER);
                        robot.left2.setPower(-0.2 * POWER);
                        robot.right1.setPower(-0.2);
                        robot.right2.setPower(-0.2);
                    } else {
                        robot.left1.setPower(-0.2);
                        robot.left2.setPower(-0.2);
                        robot.right1.setPower(-0.2);
                        robot.right2.setPower(-0.2);
                    }
                }
                telemetry.update();
                while (opModeIsActive()){
                    telemetry.update();
                    if(robot.angles.firstAngle > -88){
                        robot.left1.setPower(0.2);
                        robot.left2.setPower(0.2);
                        robot.right1.setPower(-0.2);
                        robot.right2.setPower(-0.2);
                    }else if (robot.angles.firstAngle < -98){
                        robot.left1.setPower(-0.2);
                        robot.left2.setPower(-0.2);
                        robot.right1.setPower(0.2);
                        robot.right2.setPower(0.2);
                    } else if (robot.angles.firstAngle < -88 && robot.angles.firstAngle > -98) {
                        robot.left1.setPower(0);
                        robot.left2.setPower(0);
                        robot.right1.setPower(0);
                        robot.right2.setPower(0);
                        sleep(30000);
                    }
                }
            }
            if (targetPosition == 3) {
                robot.driveForwardSetDistance(0.2, 935);
                while (robot.left1.isBusy() & robot.right1.isBusy()) {
                    //double firstAngle = Math.abs(robot.angles.firstAngle);
                    //double POWER = -1.03;
                    telemetry.update();
                    if (robot.angles.firstAngle < 0.000001) {
                        robot.left1.setPower(-0.2);
                        robot.left2.setPower(-0.2);
                        robot.right1.setPower(-0.2 * POWER);
                        robot.right2.setPower(-0.2 * POWER);
                    } else if (robot.angles.firstAngle > 0.000001) {
                        robot.left1.setPower(-0.2 * POWER);
                        robot.left2.setPower(-0.2 * POWER);
                        robot.right1.setPower(-0.2);
                        robot.right2.setPower(-0.2);
                    } else {
                        robot.left1.setPower(-0.2);
                        robot.left2.setPower(-0.2);
                        robot.right1.setPower(-0.2);
                        robot.right2.setPower(-0.2);
                    }
                }
                telemetry.update();
                while(opModeIsActive()){
                    telemetry.update();
                    if(robot.angles.firstAngle < 84){
                        robot.left1.setPower(0.05);
                        robot.left2.setPower(0.05);
                        robot.right1.setPower(-0.05);
                        robot.right2.setPower(-0.05);
                    }else if (robot.angles.firstAngle > 96){
                        robot.left1.setPower(-0.05);
                        robot.left2.setPower(-0.05);
                        robot.right1.setPower(0.05);
                        robot.right2.setPower(0.05);
                    } else if (robot.angles.firstAngle > 84 && robot.angles.firstAngle < 96){
                        robot.left1.setPower(0);
                        robot.left2.setPower(0);
                        robot.right1.setPower(0);
                        robot.right2.setPower(0);
                        sleep(30000);
                    }
                }
            }
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
}
