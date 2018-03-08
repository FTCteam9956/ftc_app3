//RRHardwarePresets.java

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.util.Locale;

public class RRHardwarePresets {

    //Hardware Map.
    HardwareMap HwMap;

    //DcMotors
    public DcMotor left1; //Front Left Drive Motor
    public DcMotor left2; //Back Left Drive Motor
    public DcMotor right1;//Front Right Drive Motor
    public DcMotor right2;//Back Right Drive Motor
    public DcMotor intake;//Glyph Intake Motor
    public DcMotor winch;//Raises Glyph Export
    public DcMotor glyphFlip;
    public DcMotor relicArm; //Slides out Relic Arm

    //Servos
    public Servo rotateArm; //Jewel Arm
    public Servo lowerArm; //Jewel Arm
    public Servo rotateBox; //Glyph Intake
//    public Servo twist; //Relic Arm
//    public Servo pinch; //Relic Arm

    //Limits
    public DigitalChannel upperLimit;
    public DigitalChannel bottomLimit;

    //Sensors
    public ColorSensor jewelArm;
    public BNO055IMU imu;

    Orientation angles;
    Acceleration gravity;

    public static final double RELIC_CLAW_OPENED = 0;
    public static final double RELIC_CLAW_CLOSED = 0;
    public static final double RELIC_TWIST_UP = 0;
    public static final double RELIC_TWIST_DOWN = 0;
    public static final double JEWEL_ARM_UP = 0.65;
    public static final double JEWEL_ARM_DOWN = 0.01;
    public static final double ROTATE_RIGHT = 0.7;
    public static final double ROTATE_LEFT = 0.0;
    public static final double ROTATE_MID = 0.3;

    //Constructor
    public RRHardwarePresets() {
        System.out.println("Created new RRHardwarePresets Object!");
    }

    public void init(HardwareMap hwm){

        //Mappings.
        HwMap = hwm;
        left1 = HwMap.dcMotor.get("left1");
        left2 = HwMap.dcMotor.get("left2");
        right1 = HwMap.dcMotor.get("right1");
        right2 = HwMap.dcMotor.get("right2");
        intake = HwMap.dcMotor.get("intake");
        winch = HwMap.dcMotor.get("winch");
        relicArm = HwMap.dcMotor.get("relicArm");
        rotateArm = HwMap.servo.get("rotateArm");
        lowerArm = HwMap.servo.get("lowerArm");
        rotateBox = HwMap.servo.get("rotateBox");
        glyphFlip = HwMap.dcMotor.get("glyphFlip");
        upperLimit = HwMap.digitalChannel.get("upperLimit");
        bottomLimit = HwMap.digitalChannel.get("bottomLimit");
//        twist = HwMap.servo.get("twist");
//        pinch = HwMap.servo.get("pinch");

        jewelArm = HwMap.colorSensor.get("jewelArm");
        imu = HwMap.get(BNO055IMU.class, "imu");


        //DC Motor directions.
        left1.setDirection(DcMotorSimple.Direction.FORWARD);
        left2.setDirection(DcMotorSimple.Direction.FORWARD);
        right1.setDirection(DcMotorSimple.Direction.REVERSE);
        right2.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        winch.setDirection(DcMotorSimple.Direction.FORWARD);
//        relicArm.setDirection(DcMotorSimple.Direction.FORWARD);
        glyphFlip.setDirection(DcMotorSimple.Direction.FORWARD);

        //DC Motor stop behavior.
        left1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        winch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        relicArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        glyphFlip.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Sensor LED control.
        jewelArm.enableLed(true);
    }

    //---UNIVERSAL METHODS BELOW---

    //Servo we want to move, Position we want to move to, Number of servo movements we want, the time we want this movement to occur over in milliseconds.
    public void moveServo(Servo targetServo, double targetPosition, int steps, long timeInMilli) {
        //Total distance to travel.
        double distanceToTravel = Math.abs(targetServo.getPosition() - targetPosition);
        //Unit conversion to nanoseconds.
        long time = timeInMilli * 1000000;
        //Per Step values.
        long timePerStep = time / steps;
        //Loops number of steps.
        double distanceToTravelPerStep;
        if (targetPosition - targetServo.getPosition() >= 0) {
            distanceToTravelPerStep = (distanceToTravel / steps);
        } else {
            distanceToTravelPerStep = (distanceToTravel / steps) * -1;
        }
        for (int counter = 0; counter < steps; counter++) {
            double initialTime = System.nanoTime();
            double currentPosition = targetServo.getPosition(); //Gets current arm position.
            targetServo.setPosition(currentPosition + distanceToTravelPerStep);//Moves the arm.
            while ((System.nanoTime() - initialTime) < timePerStep) {
                //Wait.
            }
        }
    }

    //Takes power and distance to rotate and "CW" clockwise or "CCW" as directional input.
    public void turnDirection(double power, int distance, String direction) {
        //Resets encoders by setting to STOP_AND_RESET_ENCODER mode.
        setRunMode("STOP_AND_RESET_ENCODER");
        setRunMode("RUN_TO_POSITION");
        if (direction.equals("CCW")) {//Left
            this.left1.setTargetPosition(distance);
            this.left2.setTargetPosition(distance);
            this.right1.setTargetPosition(-distance);
            this.right2.setTargetPosition(-distance);
        } else if (direction.equals("CW")) {//Right
            this.left1.setTargetPosition(-distance);
            this.left2.setTargetPosition(-distance);
            this.right1.setTargetPosition(distance);
            this.right2.setTargetPosition(distance);
        }
        setMotorPower(power);
        //Waits while turning.
        while (anyMotorsBusy()) {
            //Spinning
            //Waiting while turning.
        }
        //Stop motors.
        setMotorPower(0.0);
        //Sets mode back to RUN_USING_ENCODER
        setRunMode("RUN_USING_ENCODER");
    }

    //Drives forward a certain distance at a certain speed. Only use if no intention to interrupt.
    public void driveForwardSetDistance(double power, int distance) {
        //Resets encoders by setting to STOP_AND_RESET_ENCODER mode.
        setRunMode("STOP_AND_RESET_ENCODER");
        //Sets to RUN_TO_POSITION mode
        setRunMode("RUN_TO_POSITION");
        //Sets target distance. Set to negative distance because motor was running backwards.
        setAllTargetPositions(-distance);
        //Sets power for DC Motors.
        setMotorPower(power);
        //Waits while driving to position.
        while (anyMotorsBusy()){

        }
        //Stops driving by setting power to 0.0.
        setMotorPower(0.0);
        //Sets back to RUN_USING_ENCODER mode.
        setRunMode("RUN_USING_ENCODER");
    }

    public void setRunMode(String input) {
        if (input.equals("STOP_AND_RESET_ENCODER")) {
            this.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.left2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.right2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if (input.equals("RUN_WITHOUT_ENCODER")) {
            this.left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            this.left2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            this.right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            this.right2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (input.equals("RUN_USING_ENCODER")) {
            this.left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.left2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.right2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (input.equals("RUN_TO_POSITION")) {
            this.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.left2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.right2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    //Returns TRUE if any drive motors are busy and FALSE if not.
    public boolean anyMotorsBusy() {
        if (this.left1.isBusy() || this.left2.isBusy() || this.right1.isBusy() || this.right2.isBusy()) {
            return (true);
        } else {
            return (false);
        }
    }

    //Sets all drive motor power.
    public void setMotorPower(double power) {
        this.left1.setPower(power);
        this.left2.setPower(power);
        this.right1.setPower(power);
        this.right2.setPower(power);
    }

    //Sets all motors target position.
    public void setAllTargetPositions(int distance) {
        left1.setTargetPosition(distance);
        left2.setTargetPosition(distance);
        right1.setTargetPosition(distance);
        right2.setTargetPosition(distance);
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    String formatAngle(AngleUnit angleUnit, double angle){
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    public void turn(String direction, double power){
        if(direction == "CW"){
            left1.setPower(power);
            left2.setPower(power);
            right1.setPower(-1 * power);
            right2.setPower(-1 * power);
        }
        if(direction == "CCW"){
            left1.setPower(-1 * power);
            left2.setPower(-1 * power);
            right1.setPower(power);
            right2.setPower(power);
        }
    }

    public void initServoPositions(){
        this.lowerArm.setPosition(this.JEWEL_ARM_UP);
        this.rotateArm.setPosition(this.ROTATE_MID);
    }
    public void oneEncoderDrive(double power, int distance){
        //Resets encoders by setting to STOP_AND_RESET_ENCODER mode.
        setRunMode("STOP_AND_RESET_ENCODER");
        //Sets to RUN_TO_POSITION mode
        setRunMode("RUN_TO_POSITION");
        //Sets target distance.
        this.left1.setTargetPosition(distance);
        //Sets power for DC Motors.
        setMotorPower(power);
        //Waits while driving to position.
        while (this.left1.getCurrentPosition() < distance){

        }
        //Stops driving by setting power to 0.0.
        setMotorPower(0.0);
        //Sets back to RUN_USING_ENCODER mode.
        setRunMode("RUN_USING_ENCODER");
    }
}
