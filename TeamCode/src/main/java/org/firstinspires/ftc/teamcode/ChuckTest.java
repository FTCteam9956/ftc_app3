//package org.firstinspires.ftc.teamcode;
//
//
//import com.qualcomm.hardware.bosch.BNO055IMU;
//
//import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//
//import org.firstinspires.ftc.teamcode.hardware.subsystems.NavigationHardware;
//
//public class ChuckTest extends LinearOpMode {
//
//    public BNO055IMU imu;
//
//    public void runOpMode() {
//    public IMU(HardwareMap map, String name) {
//            super(map, name);
//            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//            parameters.loggingEnabled = true;
//            parameters.loggingTag = "IMU";
//            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//            imu = map.get(BNO055IMU.class, name);
//            imu.initialize(parameters);
//        }
//        @Override
//        public double getHeading () {
//            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//            return (angles.firstAngle + 360) % 360;
//        }
//
//        @Override
//        public double getError ( double targetAngle){
//            double angleError = 0;
//            angleError = (targetAngle - getHeading());
//            angleError -= (360 * Math.floor(0.5 + ((angleError) / 360.0)));
//            return angleError;
//        }
//    }
//}