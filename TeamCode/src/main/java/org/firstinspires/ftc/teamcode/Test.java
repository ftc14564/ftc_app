package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;


@TeleOp (name = "Test")
public class Test extends LinearOpMode {

    DcMotor motorRightFront;
    DcMotor motorRightBack;
    DcMotor motorLeftFront;
    DcMotor motorLeftBack;
    DcMotor armBottom;
    DcMotor armTop;
    DcMotor lift;
    //DcMotor receiver;

    Servo grabServo;
    Servo grabBase;


    BNO055IMU imu, imu1;
    BNO055IMU.Parameters parameters;

    Orientation angles, angles1;


    float power = 0;
    float track = 0;
    boolean strafing;
    boolean initDone=false;
    double power_multiplier;

    double angleToTurn;


    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor





    public void initFn() {

        telemetry.addData("Init: start ","");

        motorRightFront = hardwareMap.dcMotor.get("right_front");
        motorRightBack = hardwareMap.dcMotor.get("right_back");
        motorLeftFront = hardwareMap.dcMotor.get("left_front");
        motorLeftBack = hardwareMap.dcMotor.get("left_back");
        motorLeftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLeftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        strafing = false;


        motorRightFront.setMode(RUN_WITHOUT_ENCODER);
        motorRightBack.setMode(RUN_WITHOUT_ENCODER);
        motorLeftFront.setMode(RUN_WITHOUT_ENCODER);
        motorLeftBack.setMode(RUN_WITHOUT_ENCODER);



        armBottom = hardwareMap.dcMotor.get("armBottom");
        armTop = hardwareMap.dcMotor.get("armTop");
        lift = hardwareMap.dcMotor.get("lift");
        //receiver = hardwareMap.dcMotor.get("receiver");

        armBottom.setDirection(DcMotorSimple.Direction.REVERSE);
        armTop.setDirection(DcMotorSimple.Direction.REVERSE);
        armBottom.setMode(STOP_AND_RESET_ENCODER);
        armTop.setMode(STOP_AND_RESET_ENCODER);

        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        //receiver.setDirection(DcMotorSimple.Direction.REVERSE);

        armTop.setMode(RUN_WITHOUT_ENCODER);
        armBottom.setMode(RUN_WITHOUT_ENCODER);
        lift.setMode(RUN_WITHOUT_ENCODER);
        //receiver.setMode(RUN_WITHOUT_ENCODER);

        grabServo = hardwareMap.servo.get("grab_servo");
        grabBase = hardwareMap.servo.get("grab_base");

        power_multiplier = 1;



        angleToTurn = 30;

        double driveSpeed = 0;
        motorLeftBack.setPower(driveSpeed);
        motorLeftFront.setPower(driveSpeed);
        motorRightBack.setPower(driveSpeed);
        motorRightFront.setPower(driveSpeed);

        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        parameters.mode = BNO055IMU.SensorMode.IMU;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

    }


    public void stopWheels() {
        motorRightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightBack.setPower(0);
        motorLeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeftBack.setPower(0);
        motorRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightFront.setPower(0);
        motorLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeftFront.setPower(0);
    }

    public String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    public String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    double P_TURN_COEFF =-0.05;
    double TURN_THRESHOLD = 2;

    public void gyroTurnREV(double speed, double angle){

        telemetry.addData("starting gyro turn","-----");
        telemetry.update();

        while(opModeIsActive() && !onTargetAngleREV(speed, angle, P_TURN_COEFF)){
            telemetry.update();
            idle();
            telemetry.addData("-->","inside while loop :-(");
            telemetry.update();
        }
        sleep(100);
        while(opModeIsActive() && !onTargetAngleREV(speed, angle, P_TURN_COEFF/3)){
            telemetry.update();
            idle();
            telemetry.addData("-->","inside while loop :-(");
            telemetry.update();
        }

        telemetry.addData("done with gyro turn","-----");
        telemetry.update();
    }
    boolean onTargetAngleREV(double speed, double angle, double PCoeff){
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        //determine turm power based on error
        error = getErrorREV(angle);

        if (Math.abs(error) <= TURN_THRESHOLD){

            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
            stopWheels();
        }
        else{

            steer = getSteerREV(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
            //leftSpeed = -5;
        }

        motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double weightConstant = 0.8;//this constant will depend on the robot. you need to test experimentally to see which is best

        while(Math.abs(weightConstant*leftSpeed)<0.2)
            weightConstant*=2;

        motorLeftFront.setPower(weightConstant*leftSpeed);
        motorRightFront.setPower(weightConstant*rightSpeed);
        motorLeftBack.setPower(weightConstant*leftSpeed);
        motorRightBack.setPower(weightConstant*rightSpeed);

        telemetry.addData("Target angle","%5.2f",angle);
        telemetry.addData("Error/Steer", "%5.2f/%5.2f", error, steer);
        telemetry.addData("speed", "%5.2f/%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }
    public double getErrorREV(double targetAngle){

        double robotError;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        robotError = targetAngle - angles.firstAngle;
        //telemetry.addData("Zvalue","%5.2f",gyro.getIntegratedZValue());
        //telemetry.update();

        while(robotError > 180) robotError -= 360;

        while(robotError <= -180) robotError += 360;

        telemetry.addData("Robot Error","%5.2f",robotError);
        telemetry.update();

        return robotError;

    }
    public double getSteerREV(double error , double PCoeff){
        return Range.clip(error * PCoeff, -1 , 1);
    }


    public void OLDrotate (double power, int direction, double angle) {
        if(direction == -1.0 ){
            // LEFT
            //Clockwise
            motorLeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
            motorLeftBack.setDirection(DcMotorSimple.Direction.REVERSE);
            motorRightFront.setDirection(DcMotorSimple.Direction.REVERSE);
            motorRightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        else{
            // RIGHT
            //Counter Clockwise
            motorLeftFront.setDirection(DcMotorSimple.Direction.FORWARD);
            motorLeftBack.setDirection(DcMotorSimple.Direction.FORWARD);
            motorRightFront.setDirection(DcMotorSimple.Direction.FORWARD);
            motorRightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        motorLeftBack.setMode(STOP_AND_RESET_ENCODER);
        motorLeftFront.setMode(STOP_AND_RESET_ENCODER);
        motorRightBack.setMode(STOP_AND_RESET_ENCODER);
        motorRightFront.setMode(STOP_AND_RESET_ENCODER);

        motorLeftBack.setMode(RUN_WITHOUT_ENCODER);
        motorLeftFront.setMode(RUN_WITHOUT_ENCODER);
        motorRightBack.setMode(RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(RUN_WITHOUT_ENCODER);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        telemetry.addData("Robot turning", "Yay!");
        telemetry.update();
        sleep(150);


        if(direction == 1)
        {
            // RIGHT
            telemetry.addData("Robot turning right", "Yay!");
            telemetry.update();
            sleep(150);
            while (Math.abs(((Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)))))  < angle )
            {
                /*if(System.currentTimeMillis()-startTime > 29500 ){
                    break;
                }*/
                telemetry.update();
                telemetry.addData("turning (imu degrees)", formatAngle(angles.angleUnit, angles.firstAngle));
                telemetry.addData("turning (imu1 degrees)", formatAngle(angles1.angleUnit, angles1.firstAngle));
                telemetry.update();


                double _power = 1.15*power*((angle-Math.abs(((Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle))))))/angle);
                motorLeftFront.setPower(_power);
                motorRightBack.setPower(_power);
                motorRightFront.setPower(_power);
                motorLeftBack.setPower(_power);

                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            }
            stopWheels();
        }
        else{
            // LEFT
            telemetry.addData("Robot turning left", "Yay!");
            telemetry.update();
            sleep(150);
            while (Math.abs(((Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)))))  < angle )
            {
                telemetry.update();
                telemetry.addData("turning (imu degrees)", formatAngle(angles.angleUnit, angles.firstAngle));
                telemetry.addData("turning (imu1 degrees)", formatAngle(angles1.angleUnit, angles1.firstAngle));
                telemetry.update();

                double _power = 1.15*power*((angle-Math.abs(((Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle))))))/angle);
                motorLeftFront.setPower(_power);
                motorRightBack.setPower(_power);
                motorRightFront.setPower(_power);
                motorLeftBack.setPower(_power);

                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            }
            stopWheels();
        }
        //stopRobot and change modes back to normal

        motorLeftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLeftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRightBack.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void straight (double power, int direction, double distance) throws InterruptedException {

        motorLeftBack.setMode(STOP_AND_RESET_ENCODER);
        motorLeftFront.setMode(STOP_AND_RESET_ENCODER);
        motorRightBack.setMode(STOP_AND_RESET_ENCODER);
        motorRightFront.setMode(STOP_AND_RESET_ENCODER);

/*
        while(motorLeftFront.getCurrentPosition() != 0) {
            waitOneFullHardwareCycle();
        }
        */

        telemetry.addData("Straight", "In straight()");
        telemetry.update();


        motorLeftBack.setMode(RUN_WITHOUT_ENCODER);
        motorLeftFront.setMode(RUN_WITHOUT_ENCODER);
        motorRightBack.setMode(RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(RUN_WITHOUT_ENCODER);


        motorLeftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLeftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        motorRightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLeftBack.setDirection(DcMotorSimple.Direction.FORWARD);


        telemetry.addData("Straight", "Still in straight()");
        telemetry.update();


        while (Math.abs(motorLeftBack.getCurrentPosition()) < Math.abs(distance)
                && Math.abs(motorLeftFront.getCurrentPosition()) < Math.abs(distance)
                && Math.abs(motorRightFront.getCurrentPosition()) < Math.abs(distance)
                && Math.abs(motorRightBack.getCurrentPosition()) < Math.abs(distance) ) {

            /*if(System.currentTimeMillis()-startTime > 29500 ){
                break;
            }*/
            telemetry.addData("Position", motorLeftFront.getCurrentPosition());
            telemetry.update();
            motorLeftFront.setPower(direction*.2*power);
            motorRightBack.setPower(direction*.2*power);
            motorRightFront.setPower(direction*.2*power);
            motorLeftBack.setPower(direction*.2*power);
            if (Math.abs(motorLeftFront.getCurrentPosition()) < .05 * Math.abs(distance)){
                motorLeftFront.setPower(direction*power);
                motorRightBack.setPower(direction*power);
                motorRightFront.setPower(direction*power);
                motorLeftBack.setPower(direction*power);
            }
            else if (Math.abs(motorLeftFront.getCurrentPosition()) < .3 * Math.abs(distance)) {
                motorLeftFront.setPower(direction*.6*power);
                motorRightBack.setPower(direction*.6*power);
                motorRightFront.setPower(direction*.6*power);
                motorLeftBack.setPower(direction*.6*power);
            }
            else if (Math.abs(motorLeftFront.getCurrentPosition()) < .6 * Math.abs(distance)){
                motorLeftFront.setPower(direction*.55*power);
                motorRightBack.setPower(direction*.55*power);
                motorRightFront.setPower(direction*.55*power);
                motorLeftBack.setPower(direction*.55*power);
            }
            else if (Math.abs(motorLeftFront.getCurrentPosition()) < .7 * Math.abs(distance)){
                motorLeftFront.setPower(direction*.5*power);
                motorRightBack.setPower(direction*.5*power);
                motorRightFront.setPower(direction*.5*power);
                motorLeftBack.setPower(direction*.5*power);
            }
            else{
                motorLeftFront.setPower(direction*.2*power);
                motorRightBack.setPower(direction*.2*power);
                motorRightFront.setPower(direction*.2*power);
                motorLeftBack.setPower(direction*.2*power);
            }
        }
        stopWheels();

        //stopRobot and change modes back to normal
        motorLeftBack.setMode(STOP_AND_RESET_ENCODER);
        motorLeftFront.setMode(STOP_AND_RESET_ENCODER);
        motorRightBack.setMode(STOP_AND_RESET_ENCODER);
        motorRightFront.setMode(STOP_AND_RESET_ENCODER);
        //while (motorLeftFront.getCurrentPosition() != 0) {
        //waitOneFullHardwareCycle();
        //}
        sleep(200);


    }

    /* direction : +1 is right , -1 is left
       distance: in ticks
     */
    public void strafe (double power, int direction, double distance) throws InterruptedException {

        motorLeftBack.setMode(STOP_AND_RESET_ENCODER);
        motorLeftFront.setMode(STOP_AND_RESET_ENCODER);
        motorRightBack.setMode(STOP_AND_RESET_ENCODER);
        motorRightFront.setMode(STOP_AND_RESET_ENCODER);



        telemetry.addData("Strafe", "strafe");
        telemetry.update();


        motorLeftBack.setMode(RUN_WITHOUT_ENCODER);
        motorLeftFront.setMode(RUN_WITHOUT_ENCODER);
        motorRightBack.setMode(RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(RUN_WITHOUT_ENCODER);


        motorLeftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        motorRightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLeftBack.setDirection(DcMotorSimple.Direction.REVERSE);





        while (Math.abs(motorLeftBack.getCurrentPosition()) < Math.abs(distance)
                && Math.abs(motorLeftFront.getCurrentPosition()) < Math.abs(distance)
                && Math.abs(motorRightFront.getCurrentPosition()) < Math.abs(distance)
                && Math.abs(motorRightBack.getCurrentPosition()) < Math.abs(distance) ) {

            /*if(System.currentTimeMillis()-startTime > 29500 ){
                break;
            }*/
            telemetry.addData("Position", motorLeftFront.getCurrentPosition());
            telemetry.update();
            motorLeftFront.setPower(direction*.3*power);
            motorRightBack.setPower(direction*.3*power);
            motorRightFront.setPower(direction*.3*power);
            motorLeftBack.setPower(direction*.3*power);
            if (Math.abs(motorLeftFront.getCurrentPosition()) < .05 * Math.abs(distance)){
                motorLeftFront.setPower(direction*power);
                motorRightBack.setPower(direction*power);
                motorRightFront.setPower(direction*power);
                motorLeftBack.setPower(direction*power);
            }
            else if (Math.abs(motorLeftFront.getCurrentPosition()) < .5 * Math.abs(distance)) {
                motorLeftFront.setPower(direction*.8*power);
                motorRightBack.setPower(direction*.8*power);
                motorRightFront.setPower(direction*.8*power);
                motorLeftBack.setPower(direction*.8*power);
            }
            else if (Math.abs(motorLeftFront.getCurrentPosition()) < .6 * Math.abs(distance)){
                motorLeftFront.setPower(direction*.65*power);
                motorRightBack.setPower(direction*.65*power);
                motorRightFront.setPower(direction*.65*power);
                motorLeftBack.setPower(direction*.65*power);
            }
            else if (Math.abs(motorLeftFront.getCurrentPosition()) < .8 * Math.abs(distance)){
                motorLeftFront.setPower(direction*.6*power);
                motorRightBack.setPower(direction*.6*power);
                motorRightFront.setPower(direction*.6*power);
                motorLeftBack.setPower(direction*.6*power);
            }
            else{
                motorLeftFront.setPower(direction*.4*power);
                motorRightBack.setPower(direction*.4*power);
                motorRightFront.setPower(direction*.4*power);
                motorLeftBack.setPower(direction*.4*power);
            }
        }
        stopWheels();

        //stopRobot and change modes back to normal
        motorLeftBack.setMode(STOP_AND_RESET_ENCODER);
        motorLeftFront.setMode(STOP_AND_RESET_ENCODER);
        motorRightBack.setMode(STOP_AND_RESET_ENCODER);
        motorRightFront.setMode(STOP_AND_RESET_ENCODER);
        //while (motorLeftFront.getCurrentPosition() != 0) {
        //waitOneFullHardwareCycle();
        //}

        //back to non strafing convention
        motorRightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLeftBack.setDirection(DcMotorSimple.Direction.FORWARD);

        sleep(200);

    }


    @Override
    public void runOpMode() {


        initFn();

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            float forward = gamepad1.right_stick_y;
            float sideways = gamepad1.left_stick_x;


            if (Math.abs(sideways) > 0.1) {
                //strafe
                motorLeftFront.setDirection(DcMotorSimple.Direction.FORWARD);  //default
                motorLeftBack.setDirection(DcMotorSimple.Direction.REVERSE);  //changed for strafe
                motorRightBack.setDirection(DcMotorSimple.Direction.REVERSE);  //default
                motorRightFront.setDirection(DcMotorSimple.Direction.FORWARD); //changed for strafe

                motorRightFront.setPower(-1*sideways*power_multiplier);
                motorRightBack.setPower(-1*sideways*power_multiplier);
                motorLeftFront.setPower(-1*sideways*power_multiplier);
                motorLeftBack.setPower(-1*sideways*power_multiplier);
            }
            else
            if (gamepad1.left_bumper && Math.abs(forward) > 0.1) {
                //right turn
                motorLeftFront.setDirection(DcMotorSimple.Direction.REVERSE);  //chnaged
                motorLeftBack.setDirection(DcMotorSimple.Direction.REVERSE);   //changed
                motorRightFront.setDirection(DcMotorSimple.Direction.REVERSE); //default
                motorRightBack.setDirection(DcMotorSimple.Direction.REVERSE);  //default
                motorRightFront.setPower(forward*power_multiplier);
                motorRightBack.setPower(forward*power_multiplier);
                motorLeftFront.setPower(forward*power_multiplier);
                motorLeftBack.setPower(forward*power_multiplier);
            }
            else
            if (gamepad1.right_bumper && Math.abs(forward) > 0.1) {
                //left turn
                motorLeftFront.setDirection(DcMotorSimple.Direction.FORWARD);  //default
                motorLeftBack.setDirection(DcMotorSimple.Direction.FORWARD);  //default
                motorRightFront.setDirection(DcMotorSimple.Direction.FORWARD); //changed
                motorRightBack.setDirection(DcMotorSimple.Direction.FORWARD);  //changed
                motorRightFront.setPower(forward*power_multiplier);
                motorRightBack.setPower(forward*power_multiplier);
                motorLeftFront.setPower(forward*power_multiplier);
                motorLeftBack.setPower(forward*power_multiplier);
            }
            else
            if (Math.abs(forward) > 0.1) {
                //forward
                motorLeftBack.setDirection(DcMotorSimple.Direction.FORWARD);
                motorLeftFront.setDirection(DcMotorSimple.Direction.FORWARD);
                motorRightFront.setDirection(DcMotorSimple.Direction.REVERSE);
                motorRightBack.setDirection(DcMotorSimple.Direction.REVERSE);
                motorRightFront.setPower(forward*power_multiplier);
                motorRightBack.setPower(forward*power_multiplier);
                motorLeftFront.setPower(forward*power_multiplier);
                motorLeftBack.setPower(forward*power_multiplier);
            }
            else
            {
                motorLeftBack.setDirection(DcMotorSimple.Direction.FORWARD);
                motorLeftFront.setDirection(DcMotorSimple.Direction.FORWARD);
                motorRightFront.setDirection(DcMotorSimple.Direction.REVERSE);
                motorRightBack.setDirection(DcMotorSimple.Direction.REVERSE);
                motorRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motorRightFront.setPower(0);
                motorRightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motorRightBack.setPower(0);
                motorLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motorLeftFront.setPower(0);
                motorLeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motorLeftBack.setPower(0);
            }

            if (gamepad2.right_bumper && gamepad2.right_trigger > 0.1) {
                armTop.setDirection(DcMotorSimple.Direction.FORWARD);
                armTop.setMode(RUN_WITHOUT_ENCODER);

                armTop.setPower(gamepad2.right_trigger/2);
            } else if (gamepad2.right_trigger > 0.05) {
                armTop.setDirection(DcMotorSimple.Direction.REVERSE);
                armTop.setMode(RUN_WITHOUT_ENCODER);

                armTop.setPower(gamepad2.right_trigger/2);
            } else {
                armBottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                armTop.setPower(0);
            }


            if (gamepad2.left_bumper && gamepad2.left_trigger > 0.1) {
                armBottom.setDirection(DcMotorSimple.Direction.REVERSE);
                armBottom.setMode(RUN_WITHOUT_ENCODER);
                armBottom.setPower(gamepad2.left_trigger/2);
            } else if (gamepad2.left_trigger > 0.05) {
                armBottom.setDirection(DcMotorSimple.Direction.FORWARD);
                armBottom.setMode(RUN_WITHOUT_ENCODER);
                armBottom.setPower(gamepad2.left_trigger/2);
            } else {
                armBottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                armBottom.setPower(0);

            }

            if(gamepad2.left_stick_y>0)
            {
                lift.setPower(1);
            }
            else if(gamepad2.left_stick_y<0){
                lift.setPower(-1);
            }
            else{
                lift.setPower(0);
            }

//            if (gamepad1.dpad_right) {
//                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                angleToTurn = 30 + Math.abs(((Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)))));
//                OLDrotate(0.8, 1, angleToTurn);
//            }
//            if (gamepad1.dpad_left) {
//                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                angleToTurn = Math.abs(((Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle))))) + 30;
//                OLDrotate(0.8, -1, angleToTurn);
//            }

            if (gamepad1.dpad_right) {
                gyroTurnREV(1, 90);
            }
            if (gamepad1.dpad_left) {
                gyroTurnREV(1, -90);

            }


            if (gamepad2.y) {
                telemetry.addData("grab position y", grabServo.getPosition() );

                grabServo.setPosition(1);
            }
            if (gamepad2.x) {
                telemetry.addData("grab position x", grabServo.getPosition() );
                grabServo.setPosition(0.5);
            }
            if (gamepad2.a) {
                telemetry.addData("grab position a", grabServo.getPosition() );
                grabServo.setPosition(0);
            }

            if(gamepad2.right_stick_y > 0) {
                grabBase.setPosition(1-gamepad2.right_stick_y);
            }
            else
                grabBase.setPosition(0.95);

            if (gamepad1.b) {
                if (power_multiplier == 1)
                    power_multiplier = 0.5;
                else if (power_multiplier == 0.5)
                    power_multiplier = 1;
                sleep(200);
            }


            telemetry.update();

//            if (gamepad2.dpad_right) {
//                try {
//                    turnTopArm(0.8, 1, 3*1440 * 70 / 360);
//                    turnBottomArm(0.5, -1, 3 * 1440 * 70 / (360));
//                    turnTopArm(0.4, 1, 3*1440 * 40 / 360);
//                    turnBottomArm(0.3, -1, 3 * 1440 * 20 / (360));
//                }
//                catch (Exception e){
//                    e.printStackTrace();
//                }
//
//            }
//            if (gamepad2.dpad_up) {
//                try {
//                    turnTopArm(0.8, 1, 3*1440 * 20 / 360);
//                    turnBottomArm(0.8, 1, 3 * 1440 * 95 / (360));
//                    turnTopArm(0.6, -1, 3*1440 * 10 / 360);
//
//                }
//                catch (Exception e){
//                    e.printStackTrace();
//                }
//            }
//            if (gamepad2.dpad_down) {
//                try {
//                    turnTopArm(0.4, -1, 3*1440 * 40 / 360);
//                    turnBottomArm(0.4, -1, 3 * 1440 * 50 / (360));
//                }
//                catch (Exception e){
//                    e.printStackTrace();
//                }
//            }
//            if (gamepad2.dpad_left) {
//                try {
//                    turnTopArm(0.3, -1, 3*1440 * 50 / 360);
//                    turnBottomArm(0.3, 1, 3 * 1440 * 50 / (360));
//                }
//                catch (Exception e){
//                    e.printStackTrace();
//                }
//            }

//            if (gamepad1.right_trigger > 0.05) {
//                try {
//                    turnBothArms(gamepad1.right_trigger, 1, 100);
//                }
//                catch (Exception e){
//                    e.printStackTrace();
//                }
//            }
//            if (gamepad1.left_trigger > 0.05) {
//                try {
//                    turnBothArms(gamepad1.left_trigger, -1, 100);
//                }
//                catch (Exception e){
//                    e.printStackTrace();
//                }
//            }




//            if (gamepad1.dpad_right) {
//                try {
//                    turnTopArm_E(1,3*1440 * 70 / 360);
//                    turnBottomArm_E(-1,3 * 1440 * 70 / (360));
//                    turnTopArm_E(1,3*1440 * 40 / 360);
//                    turnBottomArm_E(-1,3 * 1440 * 20 / (360));
//                }
//                catch (Exception e){
//                    e.printStackTrace();
//                }
//
//            }
//            if (gamepad1.dpad_up) {
//                try {
//                    turnTopArm_E(1, 3*1440 * 20 / 360);
//                    turnBottomArm_E(1, 3 * 1440 * 95 / (360));
//                    turnTopArm_E( 1, 3*1440 * 20 / 360);
//
//                }
//                catch (Exception e){
//                    e.printStackTrace();
//                }
//
//            }





//                telemetry.addData("Right Front Power", motorRightFront.getPower());
//                telemetry.addData("Right Back Power", motorRightBack.getPower());
//                telemetry.addData("Left Front Power", motorLeftFront.getPower());
//                telemetry.addData("Left Back Power", motorLeftBack.getPower());

//            telemetry.addData("top %d", armTop.getCurrentPosition() );
//            telemetry.addData("bottom %d", armBottom.getCurrentPosition() );
            telemetry.addData("Encoder ticks: ", lift.getCurrentPosition());




            telemetry.update();
        }
        //stop all motors
        grabServo.setPosition(0.5);
        armBottom.setPower(0);
        armTop.setPower(0);
        motorLeftBack.setPower(0);
        motorLeftFront.setPower(0);
        motorRightBack.setPower(0);
        motorRightFront.setPower(0);
        lift.setPower(0);






    }


    public void turnTopArm_E(int direction, int ticks){

        armTop.setDirection(DcMotorSimple.Direction.REVERSE);
        armTop.setMode(RUN_USING_ENCODER);
        armTop.setMode(STOP_AND_RESET_ENCODER);
        armTop.setMode(RUN_TO_POSITION);
        armTop.setTargetPosition(ticks);
        armTop.setPower(direction * 0.3);
        //while (armTop.getCurrentPosition()<ticks)
        sleep(1);
        armTop.setMode(RUN_WITHOUT_ENCODER);


    }

    public void turnBottomArm_E(int direction, int ticks){
        armTop.setDirection(DcMotorSimple.Direction.REVERSE);
        armBottom.setMode(RUN_USING_ENCODER);
        armBottom.setMode(STOP_AND_RESET_ENCODER);
        armBottom.setMode(RUN_TO_POSITION);
        armBottom.setTargetPosition(ticks);
        armBottom.setPower(direction* 0.3);
        //while(armBottom.getCurrentPosition()<ticks)
        sleep(1);
        armBottom.setMode(RUN_WITHOUT_ENCODER);
    }


    public void turnBottomArm (double power, int direction, double distance) throws InterruptedException {

        armBottom.setMode(STOP_AND_RESET_ENCODER);


        telemetry.addData("turnBottomArm", "In straight()");
        telemetry.update();


        armBottom.setMode(RUN_WITHOUT_ENCODER);
        armBottom.setDirection(DcMotorSimple.Direction.REVERSE);

        int count = 0;
        while (Math.abs(armBottom.getCurrentPosition()) < Math.abs(distance)
                && count++ <200) {

            telemetry.addData("armBottom Position", armBottom.getCurrentPosition());
            telemetry.update();
            armBottom.setPower(direction*.4*power);

            if (Math.abs(armBottom.getCurrentPosition()) < .05 * Math.abs(distance)){
                armBottom.setPower(direction*power);
            }
            else if (Math.abs(armBottom.getCurrentPosition()) < .5 * Math.abs(distance)) {
                armBottom.setPower(direction*.8*power);

            }
            else if (Math.abs(armBottom.getCurrentPosition()) < .6 * Math.abs(distance)){
                armBottom.setPower(direction*.6*power);
            }
            else if (Math.abs(armBottom.getCurrentPosition()) < .9 * Math.abs(distance)){
                armBottom.setPower(direction*.5*power);

            }
            else{
                armBottom.setPower(direction*.2*power);
            }
        }
        armBottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armBottom.setPower(0);

        //stopRobot and change modes back to normal
        armBottom.setMode(STOP_AND_RESET_ENCODER);

       // sleep(100);


    }
    public void turnTopArm (double power, int direction, double distance) throws InterruptedException {

        armTop.setMode(STOP_AND_RESET_ENCODER);


        telemetry.addData("armTop", "In straight()");
        telemetry.update();


        armTop.setMode(RUN_WITHOUT_ENCODER);
        armTop.setDirection(DcMotorSimple.Direction.REVERSE);

        int count =0;
        while (Math.abs(armTop.getCurrentPosition()) < Math.abs(distance)
                && count++ < 200) {

            telemetry.addData("armTop Position", armTop.getCurrentPosition());
            telemetry.update();
            armTop.setPower(direction*.4*power);

            if (Math.abs(armTop.getCurrentPosition()) < .4 * Math.abs(distance)){
                armTop.setPower(direction*power);
            }
            else if (Math.abs(armTop.getCurrentPosition()) < .8 * Math.abs(distance)) {
                armTop.setPower(direction*.9*power);

            }
            else if (Math.abs(armTop.getCurrentPosition()) < .85 * Math.abs(distance)){
                armTop.setPower(direction*.8*power);
            }
            else if (Math.abs(armTop.getCurrentPosition()) < .9 * Math.abs(distance)){
                armTop.setPower(direction*.7*power);

            }
            else{
                armTop.setPower(direction*.6*power);
            }
        }
        armTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armTop.setPower(0);

        //stopRobot and change modes back to normal
        armTop.setMode(STOP_AND_RESET_ENCODER);

    //    sleep(100);


    }
    public void turnBothArms (double power, int direction, double distance) throws InterruptedException {

        armBottom.setMode(STOP_AND_RESET_ENCODER);
        armTop.setMode(STOP_AND_RESET_ENCODER);


        telemetry.addData("turnBothArms", "In straight()");
        telemetry.update();

        double distanceTop = 4 * distance / 3;
        double distanceBottom = distance;
        armBottom.setMode(RUN_WITHOUT_ENCODER);
        armBottom.setDirection(DcMotorSimple.Direction.REVERSE);
        armTop.setMode(RUN_WITHOUT_ENCODER);
        armTop.setDirection(DcMotorSimple.Direction.REVERSE);

        int count = 0;
        while (Math.abs(armBottom.getCurrentPosition()) < Math.abs(distanceBottom)
                && count++ <50) {

            armBottom.setPower(-direction*power);

            armTop.setPower(direction*power);

        }


    }
}


