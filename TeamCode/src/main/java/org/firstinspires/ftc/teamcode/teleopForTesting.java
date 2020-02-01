package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "teleop_two_remotes,Ytesting", group = "Linear Opmode")
//@Disabled
public class teleopForTesting extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();
    static final double COUNTS_PER_MOTOR_REV = 28;    // eg: pLAAAAACEHHHHOoOLLDDDEERRR Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 20.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_MM = 25;     // For figuring circumference
    static final double COUNTS_PER_MM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_MM * 3.1415);
    static final double LIFT_SPEED = 0.6;
    //math for slow

    double slow = 1;
    double armDown = 0.7;
    //arm math

    double armTime = 0;
    static final double armHeightInterval = 130;
    static final double armHeightUpperLimit = 8;
    static final double armHeightLowerLimit = 0;
    double armHeight = 0;
    private int armHeightCurrent = 0;
    //motors
    DcMotor leftBack;
    DcMotor rightBack;
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor liftPower;

    //servos
    Servo extend;
    Servo grab;
    Servo arm;
    Servo FRservo;
    Servo FLservo;
    Servo LED_strip;

    //sensors
    DistanceSensor sensorRange;

    //limit switches
    TouchSensor topLimit;
    TouchSensor bottomLimit;

    /**
     * CONSTANTS
     */

    //servos
    double extendOut = 0.75;
    double extendIn = 0;

    double groundArm = 0.05;
    double foundArm = 0.18;
    double foundSecure = 0.1;
    double highArm = 0.36;
    double highSecure = 0.29;
    double retractArm = 0.75;

    double fullGrab = 0.33;
    double releaseGrab = 1;

    double FLdown = 1;
    double FLup = 0.8;
    double FRdown = 0.3;
    double FRup = 0.6;

    //motors
    double liftReversal = 0.05;
    double liftStop = -0;


    @Override
    public void runOpMode() {
// Lift is defined earlier because its mode needs to be set,other motors are set below
        liftPower = hardwareMap.get(DcMotor.class, "lift_power");
        liftPower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftPower.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        liftPower.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftPower.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d ",
                liftPower.getCurrentPosition());

        telemetry.update();
        // motors
        leftBack = hardwareMap.get(DcMotor.class, "leftFront_drive");
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        rightBack = hardwareMap.get(DcMotor.class, "rightBack_drive");
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftFront = hardwareMap.get(DcMotor.class, "leftBack_drive");
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);

        rightFront = hardwareMap.get(DcMotor.class, "rightFront_drive");
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);

        //servos
        extend = hardwareMap.get(Servo.class, "extend");
        extend.setDirection(Servo.Direction.FORWARD);

        arm = hardwareMap.get(Servo.class, "arm");
        arm.setDirection(Servo.Direction.FORWARD);

        grab = hardwareMap.get(Servo.class, "grab");
        grab.setDirection(Servo.Direction.REVERSE);

        FLservo = hardwareMap.get(Servo.class, "FL_hook");
        FLservo.setDirection(Servo.Direction.REVERSE);

        FRservo = hardwareMap.get(Servo.class, "FR_hook");
        FRservo.setDirection(Servo.Direction.REVERSE);


        //LED lights
        LED_strip = hardwareMap.get(Servo.class, "LED_strip");

        //set hooks to correct up positions
        FLservo.setPosition(FLup);
        FRservo.setPosition(FRup);

        //sensors
        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");

        //limit switches
        topLimit = hardwareMap.get(TouchSensor.class, "topLimit");
        bottomLimit = hardwareMap.get(TouchSensor.class, "bottomLimit");

        //start
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            slow = gamepad1.right_trigger * 8;


//Driving - left stick y is forward and backwards and left stick x is turning.
//right stick x strafes.
            /** GAMEPAD 1 CONTROLS
             **/

            LED_strip.setPosition(0.93);
            //DRIVING
            //Driving: left stick y is forward and backwards and left stick x is turning.
            //right stick x strafes.
            leftBack.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) / slow);
            rightBack.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) / slow);
            leftFront.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) / slow);
            rightFront.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) / slow);


            //GRABBING
            //full grab
            if (gamepad1.right_bumper) {
                grab.setPosition(fullGrab);
            }
            //release grab
            if (gamepad1.left_bumper) {
                grab.setPosition(releaseGrab);
            }


            //FOUNDATION MOVEMENT
            //claws both go down
            if (gamepad1.dpad_up) {
                FLservo.setPosition(FLdown);
                FRservo.setPosition(FRdown);
            }
            //claws both retract
            if (gamepad1.dpad_down) {
                FLservo.setPosition(FLup);
                FRservo.setPosition(FRup);
            }


            /**

             GAMEPAD 2 CONTROLS
             **/

            //ARM EXTENSION
            //extend arm for grabbing on the playing field (not foundation)
            if (gamepad2.a) {
                arm.setPosition(groundArm);
            }
            //halfway hover with block (used for placing higher than first level)
            if (gamepad2.y) {
                arm.setPosition(highArm);
            }
            //secure the halfway placing
            if (gamepad2.left_bumper) {
                arm.setPosition(highSecure);
            }
            //foundation hover with block
            if (gamepad2.x) {
                arm.setPosition(foundArm);
            }
            //secure foundation placing
            if (gamepad2.b) {
                arm.setPosition(foundSecure);
            }
            //retract the arm
            if (gamepad2.right_bumper) {
                arm.setPosition(retractArm);
                sleep(500);
                grab.setPosition(fullGrab);
            }
            if (gamepad2.dpad_down && armTime < 1 && armHeight > armHeightLowerLimit) {
                liftDrive(LIFT_SPEED, armHeightInterval, 2.0);
                armTime = 50;
                armHeight = armHeight - 1;
            }
            if (gamepad2.dpad_up && armTime < 1 && armHeight < armHeightUpperLimit) {

                liftDrive(LIFT_SPEED, -armHeightInterval, 2.0);
                armTime = 50;
                armHeight = armHeight + 1;
            }

            //EXTEND CAM
            //extend all the way out
            if (gamepad2.dpad_right) {
                extend.setPosition(extendOut);
            }
            // pull extension back in
            if (gamepad2.dpad_left) {
                extend.setPosition(extendIn);
            }

            //GRABBING
            //full grab
            if (gamepad2.right_trigger == 1) {
                grab.setPosition(fullGrab);
            }
            //release grab
            if (gamepad2.left_trigger == 1) {
                grab.setPosition(releaseGrab);
            }


            //LIFT
            //top limit switch
            if (gamepad2.left_stick_y < 0 && topLimit.isPressed()) {
                liftPower.setPower(liftReversal);
            } else {
                //bottom limit switch
                if (gamepad2.left_stick_y > 0 && bottomLimit.isPressed()) {
                    liftPower.setPower(liftStop);
                } else {
                    liftPower.setPower(gamepad2.left_stick_y);
                }
            }

            //shows elapsed time and limit switch values
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Top Limit state", "State: " + topLimit.isPressed());
            telemetry.addData("Bottom Limit state", ": " + bottomLimit.isPressed());
            telemetry.addData("Gamepad 2 left stick y", ": " + gamepad2.left_stick_y);
            telemetry.update();
        }
    }




    public void liftDrive(double speed,
                          double liftMM,
                          double timeoutS) {

        int newLiftTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller

            newLiftTarget = liftPower.getCurrentPosition() + (int) (liftMM * COUNTS_PER_MM);

            liftPower.setTargetPosition(newLiftTarget);

            // Turn On RUN_TO_POSITION
            liftPower.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            liftPower.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (liftPower.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d", newLiftTarget);


                armHeightCurrent = liftPower.getCurrentPosition();
                telemetry.update();
            }

            // Stop all motion;

            liftPower.setPower(0.05);

            liftPower.setTargetPosition(armHeightCurrent);
            //  liftPower.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftPower.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //  sleep(250);   // optional pause after each move

        }
    }
}





