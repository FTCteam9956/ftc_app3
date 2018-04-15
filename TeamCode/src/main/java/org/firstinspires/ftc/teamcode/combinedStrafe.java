package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "combinedStrafe", group = "Teleop")
public class combinedStrafe extends LinearOpMode {
    public RRHardwarePresets robot = new RRHardwarePresets();

    Orientation angles;

    public void runOpMode(){
        //Make mappings to give the phone something to look for or set how we wand hardware to run
        robot.init(hardwareMap);

        BNO055IMU.Parameters parameters1 = new BNO055IMU.Parameters();
        parameters1.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters1.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters1.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters1.loggingEnabled = true;
        parameters1.loggingTag = "IMU";
        parameters1.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        robot.imu = hardwareMap.get(BNO055IMU.class, "imu");
        robot.imu.initialize(parameters1);

        robot.setRunMode("RUN_TO_POSITION");
        robot.left1.setTargetPosition(1000);
        robot.left2.setTargetPosition(-1000);
        robot.right1.setTargetPosition(1000);
        robot.right2.setTargetPosition(-1000);
        //Start of code
        waitForStart();

        while (opModeIsActive()) {
//            if(gamepad1.a){
//                robot.setRunMode("STOP_AND_RESET_ENCODERS");
//                robot.setRunMode("RUN_TO_POSITION");
//                robot.left1.setPower(0.1 * 5);
//                robot.left2.setPower(-0.1 * 5);
//                robot.right1.setPower(0.1 * 5);
//                robot.right2.setPower(-0.1 * 5);
//            }
//            if(gamepad1.b){
//                robot.setRunMode("STOP_AND_RESET_ENCODERS");
//                robot.setRunMode("RUN_USING_ENCODER");
//                robot.left1.setPower(0.045 * 5);
//                robot.left2.setPower(-0.155 * 5);
//                robot.right1.setPower(0.155 * 5);
//                robot.right2.setPower(-0.045 * 5);
//            }
//            if(gamepad1.x){
//                robot.setRunMode("STOP_AND_RESET_ENCODERS");
//                robot.setRunMode("RUN_USING_ENCODER");
//                robot.left1.setPower(0.155 * 5);
//                robot.left2.setPower(-0.045 * 5);
//                robot.right1.setPower(0.045 * 5);
//                robot.right2.setPower(-0.155 * 5);
//            }
            if(gamepad1.a){
                double startingRange = robot.wallSensor.getDistance(DistanceUnit.CM);
                int targetAngle = 90;
                while (opModeIsActive()) {
                    double strafeCorrection = (startingRange - robot.wallSensor.getDistance(DistanceUnit.CM)) / startingRange;
                    double turnCorrection = (targetAngle - (angles.firstAngle + 90) / targetAngle);
                    double Ca = turnCorrection * 2.5;
                    double Cd = strafeCorrection * 0.4;
                    double powOffsetRight = ((Cd + Ca) / 2);
                    double powOffsetLeft = ((Cd - Ca) / 2);
                    //Move away from wall when get to close
                    robot.left1.setPower((0.1 + powOffsetLeft) * 3);
                    robot.left2.setPower((-0.1 + powOffsetLeft) * 3);
                    robot.right1.setPower((0.1 + powOffsetRight) * 3);
                    robot.right2.setPower((-0.1 + powOffsetRight) * 3);
                    telemetry.addData("Ca", Ca);
                    telemetry.addData("Cd", Cd);
                    telemetry.addData("powOffsetRight", powOffsetRight);
                    telemetry.addData("powOffsetLeft", powOffsetLeft);
                    telemetry.addData("left1 Power", robot.left1.getPower());
                    telemetry.addData("left2 Power", robot.left2.getPower());
                    telemetry.addData("right1 Power", robot.right1.getPower());
                    telemetry.addData("right2 Power", robot.right2.getPower());

                }
            }
        }
    }
}
