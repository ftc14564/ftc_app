package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.os.Process;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;


@Autonomous(name = "Auto_Crater")
public class MainAutonomous_Crater extends LinearOpMode {

    DcMotor motorRightFront;
    DcMotor motorRightBack;
    DcMotor motorLeftFront;
    DcMotor motorLeftBack;
    DcMotor armBottom;
    DcMotor armTop;
    DcMotor lift;

    Servo grabServo;
    Servo grabBase;

    double grabpos;

    private DistanceSensor sensorRange_rf;
    private DistanceSensor sensorRange_rb;
    private DistanceSensor sensorRange_lf;
    private DistanceSensor sensorRange_lb;

    double distanceToWall;

    Rev2mDistanceSensor distanceSensor_rf;
    Rev2mDistanceSensor distanceSensor_rb;
    Rev2mDistanceSensor distanceSensor_lf;
    Rev2mDistanceSensor distanceSensor_lb;

    ColorSensor sensorColor;
    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F, 0F, 0F};

    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;

    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    final double SCALE_FACTOR = 255;




    int pixyCounter;
    boolean isPixyObjectSeen;
    boolean opModeActive;





    float power = 0;
    float track = 0;
    boolean strafing;
    boolean initDone=false;


    I2cDeviceSynch pixy;

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity, gravity1;
    BNO055IMU.Parameters parameters;

    private static final String VUFORIA_KEY = "AYpOJ0H/////AAABGeEbm+5m+k5BrTnPlF3X9R177NGoUFUGl1kpgLa7MBwlsRdnD3IdxY7LmZ41NTQMASZ1MbCWaEpM4Sag7tDfQsJjqVvCwZr3qJm5y33J8rnMWz1ViOwwzZgnsSZqeGRY9+uPGa6cTMO/cxs+YF+4OqsD+iu4exeMCsxyAPYhXQrEIaW6h7zYVrdi9b5WsgNGUfP60Qz8U3szKTfVmaHmMFvc+iuJ1qmAM5AjlsBlc8MMHzLAL/3sf3UiCDe4tgo4mmYEsdl499QhqhhImEiKS8rTkap/53B8Hm89z3m5HuBoH4EKVUc65k2aCBg5c5jXVoZan8DkQFqSPnArwQnCHpaL/d1y79BRE44nJXj54E6V";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;
    private List<VuforiaTrackable> allTrackables;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia = null;
    VuforiaLocalizer.Parameters Vu_parameters;


    class InitThread_Depot implements Runnable{
        @Override
        public void run() {
            try {

                telemetry.addData("Init: Thread start ","");

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

                sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color");


                initDone=true;

            } catch (Exception e) {
                e.printStackTrace();
            }

            pixy = hardwareMap.i2cDeviceSynch.get("pixy");


            //setting Pixy's I2C Address
            pixy.setI2cAddress(I2cAddr.create7bit(0x54));

            I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow (1, 26,
                    I2cDeviceSynch.ReadMode.REPEAT);
            pixy.setReadWindow(readWindow);

            //required to "turn on" the device
            pixy.engage();


            //initDone = true;
            telemetry.addData("Init: Thread done ","");

            while(opModeActive){
                           /*
Bytes    16-bit word    Description
        ----------------------------------------------------------------
        0, 1     y              sync: 0xaa55=normal object, 0xaa56=color code object
        2, 3     y              checksum (sum of all 16-bit words 2-6, that is, bytes 4-13)
        4, 5     y              signature number
        6, 7     y              x center of object
        8, 9     y              y center of object
        10, 11   y              width of object
        12, 13   y              height of object
        */

                //  int pixy_x;

                //  pixy_x = (int) pixy.read8(6);
                //  pixy_x = pixy_x << 8;
                //  pixy_x = (pixy_x & (0xff00) )| (int) pixy.read8(7);

                // int pixy_x = ((pixy.read8(4) & 0xff) << 8) | (pixy.read8(5) & 0xff);
                // telemetry.addData("Pixy_x", pixy_x);
                // https://docs.pixycam.com/wiki/doku.php?id=wiki:v2:protocol_reference
                //getBlocks

//                pixy.write8(0,173);
//                pixy.write8(1,193);
//                pixy.write8(2,32);
//                pixy.write8(3,2);
//                pixy.write8(4,1);
//                pixy.write8(5,1);

                byte b = pixy.read8(0);
//                telemetry.addData("Byte 0", b);
                b = pixy.read8(1);
//                telemetry.addData("Byte 1", b);
                b = pixy.read8(2);
//                telemetry.addData("Byte 2",b );
                b = pixy.read8(3);
//                telemetry.addData("Byte 3", b);
                b = pixy.read8(4);
//                telemetry.addData("Byte 4", b);
                b = pixy.read8(5);
//                telemetry.addData("Byte 5", b);
                b = pixy.read8(6);
//                telemetry.addData("Byte 6", b);

                if (b!=0){
                    if(pixyCounter < 5)
                        pixyCounter+=2;
                }
                else{
                    if(pixyCounter>1)
                        pixyCounter--;
                }

                if(pixyCounter >1){
                    isPixyObjectSeen = true;
                }
                else {
                    isPixyObjectSeen = false;
                }


                b = pixy.read8(7);
//                telemetry.addData("Byte 7", pixy.read8(7));
                b = pixy.read8(8);
//                telemetry.addData("Byte 8", pixy.read8(8));
                b = pixy.read8(9);
//                telemetry.addData("Byte 9", pixy.read8(9));
                b = pixy.read8(10);
//                telemetry.addData("Byte 10", pixy.read8(10));
                b = pixy.read8(11);
//                telemetry.addData("Byte 11", pixy.read8(11));
                b = pixy.read8(12);
//                telemetry.addData("Byte 12", pixy.read8(12));
                b = pixy.read8(13);
//                telemetry.addData("Byte 13", pixy.read8(13));
                //  telemetry.addData("Byte 14", pixy.read8(14));
                //  telemetry.addData("Byte 15", pixy.read8(15));
                //  telemetry.addData("Byte 16", pixy.read8(16));
                //  telemetry.addData("Byte 17", pixy.read8(17));
                //telemetry.addData("Byte 18", pixy.read8(18));
                //telemetry.addData("Byte 19", pixy.read8(19));
                //telemetry.addData("Byte 20", pixy.read8(20));
                //telemetry.addData("Byte 21", pixy.read8(21));
                // telemetry.addData("pixyCounter", pixyCounter);
                // telemetry.update();
                sleep(20);



//                Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
//                        (int) (sensorColor.green() * SCALE_FACTOR),
//                        (int) (sensorColor.blue() * SCALE_FACTOR),
//                        hsvValues);
//
//                // send the info back to driver station using telemetry function.
//                telemetry.addData("Alpha", sensorColor.alpha());
//                telemetry.addData("Red  ", sensorColor.red());
//                telemetry.addData("Green", sensorColor.green());
//                telemetry.addData("Blue ", sensorColor.blue());
//                telemetry.addData("Hue", hsvValues[0]);
//                telemetry.update();
//
//                if(hsvValues[0] < 90)
//                    isPixyObjectSeen = true;
//                else
//                    isPixyObjectSeen = false;
            }

        }

    };



    public void initFn() {

        telemetry.addData("Init: start ","");

        motorRightFront = hardwareMap.dcMotor.get("right_front");
        motorRightBack = hardwareMap.dcMotor.get("right_back");
        motorLeftFront = hardwareMap.dcMotor.get("left_front");
        motorLeftBack = hardwareMap.dcMotor.get("left_back");
        motorRightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        strafing = false;

        motorRightFront.setMode(RUN_WITHOUT_ENCODER);
        motorRightBack.setMode(RUN_WITHOUT_ENCODER);
        motorLeftFront.setMode(RUN_WITHOUT_ENCODER);
        motorLeftBack.setMode(RUN_WITHOUT_ENCODER);

        lift = hardwareMap.dcMotor.get("lift");
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        lift.setMode(RUN_WITHOUT_ENCODER);


        armBottom = hardwareMap.dcMotor.get("armBottom");
        armTop = hardwareMap.dcMotor.get("armTop");
        armBottom.setDirection(DcMotorSimple.Direction.REVERSE);
        armTop.setDirection(DcMotorSimple.Direction.REVERSE);
        armTop.setMode(RUN_WITHOUT_ENCODER);
        armBottom.setMode(RUN_WITHOUT_ENCODER);
        grabServo = hardwareMap.servo.get("grab_servo");
        grabBase = hardwareMap.servo.get("grab_base");



        grabpos = 0.5;


        double driveSpeed = 0;
        motorLeftBack.setPower(driveSpeed);
        motorLeftFront.setPower(driveSpeed);
        motorRightBack.setPower(driveSpeed);
        motorRightFront.setPower(driveSpeed);

        grabServo.setPosition(grabpos);

        new Thread(new InitThread_Depot()).start();

        //leftServo.setPosition(leftpos);
        //rightServo.setPosition(rightServoPos);

        sensorRange_rf = hardwareMap.get(DistanceSensor.class, "2m_rf");
        distanceSensor_rf = (Rev2mDistanceSensor)sensorRange_rf;
        sensorRange_rb = hardwareMap.get(DistanceSensor.class, "2m_rb");
        distanceSensor_rb = (Rev2mDistanceSensor)sensorRange_rb;
        sensorRange_lf = hardwareMap.get(DistanceSensor.class, "2m_lf");
        distanceSensor_lf = (Rev2mDistanceSensor)sensorRange_lf;
        sensorRange_lb = hardwareMap.get(DistanceSensor.class, "2m_lb");
        distanceSensor_lb = (Rev2mDistanceSensor)sensorRange_lb;
//        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
//        colorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");


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

    public void OLDrotate (double power, int direction, double angle) {

        //angle -=angle*.35;
        power /= 3;

        imu.initialize(parameters);
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
        //sleep(150);

        int counter = 0;

        if(direction == 1)
        {

            // RIGHT
            telemetry.addData("Robot turning right: ", angle);
            telemetry.update();
            //sleep(150);
            while ((Math.abs(((Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)))))  < angle ) )
            {
                //counter++;
                /*if(System.currentTimeMillis()-startTime > 29500 ){
                    break;
                }*/
                telemetry.update();
                telemetry.addData("turning (imu degrees)", formatAngle(angles.angleUnit, angles.firstAngle));
                telemetry.update();


                double _power = 1.5*power*((angle-Math.abs(((Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle))))))/angle);

                if(_power < 0.3) _power = 0.3;

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
            telemetry.addData("Robot turning left: ", angle);
            telemetry.update();
            //sleep(150);

            while ((((Math.abs(Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)))))  < angle ) )
            {
                telemetry.addData("turning (imu degrees)", formatAngle(angles.angleUnit, angles.firstAngle));
                telemetry.update();

                double _power = 1.5*power*((angle-Math.abs(((Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle))))))/angle);
                if(_power < 0.3) _power = 0.3;
                motorLeftFront.setPower(_power);
                motorRightBack.setPower(_power);
                motorRightFront.setPower(_power);
                motorLeftBack.setPower(_power);

                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            }
            stopWheels();

        }
        //stopRobot and change modes back to normal
        telemetry.addData("turned (imu degrees)", formatAngle(angles.angleUnit, angles.firstAngle));
        telemetry.update();

        motorLeftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLeftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRightBack.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void straight (double power, int direction, double distance) throws InterruptedException {

        distance /= 2.25;
        power /= 2;

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
            if (Math.abs(motorLeftFront.getCurrentPosition()) < .1 * Math.abs(distance)) {

                motorLeftFront.setPower(direction * power);
                motorRightBack.setPower(direction * power);
                motorRightFront.setPower(direction * power);
                motorLeftBack.setPower(direction * power);
            }
            else if (Math.abs(motorLeftFront.getCurrentPosition()) < .2 * Math.abs(distance)){
                motorLeftFront.setPower(direction* .7 * power);
                motorRightBack.setPower(direction* .7 *power);
                motorRightFront.setPower(direction* .7 *power);
                motorLeftBack.setPower(direction* .7 *power);
            }
            else if (Math.abs(motorLeftFront.getCurrentPosition()) < .7 * Math.abs(distance)) {
                motorLeftFront.setPower(direction*power);
                motorRightBack.setPower(direction*power);
                motorRightFront.setPower(direction*power);
                motorLeftBack.setPower(direction*power);
            }
            else if (Math.abs(motorLeftFront.getCurrentPosition()) < .8 * Math.abs(distance)){
                motorLeftFront.setPower(direction*.7*power);
                motorRightBack.setPower(direction*.7*power);
                motorRightFront.setPower(direction*.7*power);
                motorLeftBack.setPower(direction*.7*power);
            }
            else if (Math.abs(motorLeftFront.getCurrentPosition()) < .9 * Math.abs(distance)){
                motorLeftFront.setPower(direction*.5*power);
                motorRightBack.setPower(direction*.5*power);
                motorRightFront.setPower(direction*.5*power);
                motorLeftBack.setPower(direction*.5*power);
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
        sleep(200);


    }

    /* direction : +1 is right , -1 is left
       distance: in ticks
     */
    public void strafe (double power, int direction, double distance) throws InterruptedException {

        distance /= 2.25;
        power /= 1.5;

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
            if (Math.abs(motorLeftFront.getCurrentPosition()) < .1 * Math.abs(distance)) {

                motorLeftFront.setPower(direction * power);
                motorRightBack.setPower(direction * power);
                motorRightFront.setPower(direction * power);
                motorLeftBack.setPower(direction * power);
            }
            else if (Math.abs(motorLeftFront.getCurrentPosition()) < .8 * Math.abs(distance)){
                motorLeftFront.setPower(direction*power);
                motorRightBack.setPower(direction*power);
                motorRightFront.setPower(direction*power);
                motorLeftBack.setPower(direction*power);
            }
            else if (Math.abs(motorLeftFront.getCurrentPosition()) < .85 * Math.abs(distance)) {
                motorLeftFront.setPower(direction*.8*power);
                motorRightBack.setPower(direction*.8*power);
                motorRightFront.setPower(direction*.8*power);
                motorLeftBack.setPower(direction*.8*power);
            }
            else if (Math.abs(motorLeftFront.getCurrentPosition()) < .9 * Math.abs(distance)){
                motorLeftFront.setPower(direction*.7*power);
                motorRightBack.setPower(direction*.7*power);
                motorRightFront.setPower(direction*.7*power);
                motorLeftBack.setPower(direction*.7*power);
            }
            else if (Math.abs(motorLeftFront.getCurrentPosition()) < .95 * Math.abs(distance)){
                motorLeftFront.setPower(direction*.6*power);
                motorRightBack.setPower(direction*.6*power);
                motorRightFront.setPower(direction*.6*power);
                motorLeftBack.setPower(direction*.6*power);
            }
            else{
                motorLeftFront.setPower(direction*.5*power);
                motorRightBack.setPower(direction*.5*power);
                motorRightFront.setPower(direction*.5*power);
                motorLeftBack.setPower(direction*.5*power);
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

    public void makeParallel()
    {
        if(distanceSensor_lb.getDistance(DistanceUnit.CM) < (distanceSensor_lf.getDistance(DistanceUnit.CM)) ){
            double theta;
            //telemetry.addData(" test ", 1);

            double diff1 = (distanceSensor_lf.getDistance(DistanceUnit.CM)) - (distanceSensor_lb.getDistance(DistanceUnit.CM));
            double diff2 = (distanceSensor_lf.getDistance(DistanceUnit.CM)) - (distanceSensor_lb.getDistance(DistanceUnit.CM));
            double diff3 = (distanceSensor_lf.getDistance(DistanceUnit.CM)) - (distanceSensor_lb.getDistance(DistanceUnit.CM));
            double diff4 = (distanceSensor_lf.getDistance(DistanceUnit.CM)) - (distanceSensor_lb.getDistance(DistanceUnit.CM));
            double diff5 = (distanceSensor_lf.getDistance(DistanceUnit.CM)) - (distanceSensor_lb.getDistance(DistanceUnit.CM));
            double diff6 = (distanceSensor_lf.getDistance(DistanceUnit.CM)) - (distanceSensor_lb.getDistance(DistanceUnit.CM));
            double diff = (diff1+diff2+diff3 + diff4 + diff5 + diff6)/6;
            double temp = diff/23.5;
            //telemetry.addData(" test ", 2);
            //telemetry.update();
            theta = Math.asin(temp) *180/3.141592;
            telemetry.addData(" lb", distanceSensor_lb.getDistance(DistanceUnit.CM));
            telemetry.addData(" lf", distanceSensor_lf.getDistance(DistanceUnit.CM));
            telemetry.addData(" theta", theta);
            telemetry.update();
            if(theta > 5){
                OLDrotate(.7,1,theta-5);
            }

        }
        else{
            double teta;
            double diff1 = (distanceSensor_lb.getDistance(DistanceUnit.CM)) - (distanceSensor_lf.getDistance(DistanceUnit.CM));
            double diff2 = (distanceSensor_lb.getDistance(DistanceUnit.CM)) - (distanceSensor_lf.getDistance(DistanceUnit.CM));
            double diff3 = (distanceSensor_lb.getDistance(DistanceUnit.CM)) - (distanceSensor_lf.getDistance(DistanceUnit.CM));
            double diff = (diff1+diff2+diff3)/3;
            double temp = diff/23;
            teta = Math.asin(temp) *180/3.141592;
            if(teta > 5){
                OLDrotate(.7,-1,teta-5);
            }
            telemetry.addData(" lb_", distanceSensor_lb.getDistance(DistanceUnit.CM));
            telemetry.addData(" lf_", distanceSensor_lf.getDistance(DistanceUnit.CM));
            telemetry.addData(" theta_", teta);
            telemetry.update();
        }
    }


    @Override
    public void runOpMode() {

        pixyCounter = 0;
        isPixyObjectSeen = false;
        opModeActive = true;
        initFn();

        waitForStart();



//        int counter=0;
//        while(counter++<100){
//            OLDrotate(1, -1, 45);
//            sleep(1000);
//            OLDrotate(1, -1, 45);
//            sleep(1000);
//
//            OLDrotate(1, -1, 45);
//            sleep(1000);
//
//            OLDrotate(1, -1, 45);
//            sleep(1000);
//
//        }


        try {





            lift.setMode(STOP_AND_RESET_ENCODER);
            lift.setMode(RUN_WITHOUT_ENCODER);
            lift.setDirection(DcMotorSimple.Direction.FORWARD);
            while (Math.abs(lift.getCurrentPosition()) < Math.abs(11*1120))
            {
                lift.setPower(-1.0);
            }
            lift.setPower(0);


            grabBase.setPosition(0.95);

            straight(1,-1,266);   // 2 inch fwd
            strafe(1,1,1596);    // 8 inch left
            straight(1,1,665);   // 5 inch back

            OLDrotate(1, -1, 80);

            //         strafe(1, -1, 750);  //16 inch = 133 * 16 (3/2)
            //left 2 in
            straight(1,1,798); // 6 inch fwd
            telemetry.addData("Debug", "0");
            //
            telemetry.addData("Debug", "1");
            int wallStrafe=0;
//            turnTopArm(0.8, 1, 3*1440 * 70 / 360);
//            turnBottomArm(0.5, -1, 3 * 1440 * 70 / (360));
//            turnTopArm(0.4, 1, 3*1440 * 40 / 360);
//            turnBottomArm(0.3, -1, 3 * 1440 * 20 / (360));
//            grabBase.setPosition(0);
//            sleep(500);
//            grabBase.setPosition(0.95);



            sleep(300);
            if (isPixyObjectSeen) {
                straight(1, 1, 1596); // 13 inch
                straight(1,1,1200);
                telemetry.addData("Debug", "object seen");
                wallStrafe = 8578; // 43 inch
            } else {
                straight(1, -1, 266);
                strafe(1, 1, 2693);//13.5
                straight(1, 1, 266);


                sleep(300);
                telemetry.addData("Debug", "2");
                if (isPixyObjectSeen) {

                    straight(1, 1, 1596); // 8 inch
                    straight(1, 1, 1200);

                    wallStrafe = 5885; // 29.5 inch
                }
                else {
                    straight(1, -1, 266);

                    strafe(1, -1, 4987); //25
                    straight(1, 1, 266);

                    sleep(300);
                    if (isPixyObjectSeen) {
                        straight(1, 1, 1596); // 6 inch = 133*6*(3/2)
                        straight(1, 1, 1200);
                        telemetry.addData("Debug", "3");
                    }

                    wallStrafe = 10885; // 54.5 inch
                }
            }
//            strafe(1, 1, wallStrafe );
//            OLDrotate(1, 1, 40);
//            makeParallel();
//            double dis1 = distanceSensor_lb.getDistance(DistanceUnit.CM);
//            double dis2 = distanceSensor_lf.getDistance(DistanceUnit.CM);
//            double dis = (dis1 + dis2)/2;
//            if(dis > 8){
//                strafe(1,1,((dis-7)/2.54)*200);
//            }
//
//            straight(1,-1,3100);


        } catch (Exception e) {
            e.printStackTrace();
        }



        telemetry.update();

        opModeActive = false;
        //stop all motors
        grabServo.setPosition(0.5);
        armBottom.setPower(0);
        armTop.setPower(0);
        stopWheels();
        lift.setPower(0);

    }
    public void turnTopArm_E(int degrees){

        int ticks = 1440 * degrees / (360*4);
        armTop.setMode(RUN_USING_ENCODER);
        armTop.setMode(STOP_AND_RESET_ENCODER);
        armTop.setMode(RUN_TO_POSITION);
        armTop.setTargetPosition(10);
        armTop.setPower(0.6);
        //while (armTop.getCurrentPosition()<ticks)
        sleep(1);
        armTop.setMode(RUN_WITHOUT_ENCODER);


    }

    public void turnBottomArm_E(int degrees){

        int ticks = 3*1440*degrees/(360*4);
        armBottom.setMode(RUN_USING_ENCODER);
        armBottom.setMode(STOP_AND_RESET_ENCODER);
        armBottom.setMode(RUN_TO_POSITION);
        armBottom.setTargetPosition(10);
        armBottom.setPower(0.6);
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
                && count++ <50) {

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
                && count++ < 30) {

            telemetry.addData("armTop Position", armTop.getCurrentPosition());
            telemetry.update();
            armTop.setPower(direction*.4*power);

            if (Math.abs(armTop.getCurrentPosition()) < .4 * Math.abs(distance)){
                armTop.setPower(direction*power);
            }
            else if (Math.abs(armTop.getCurrentPosition()) < .8 * Math.abs(distance)) {
                armTop.setPower(direction*.8*power);

            }
            else if (Math.abs(armTop.getCurrentPosition()) < .85 * Math.abs(distance)){
                armTop.setPower(direction*.7*power);
            }
            else if (Math.abs(armTop.getCurrentPosition()) < .9 * Math.abs(distance)){
                armTop.setPower(direction*.5*power);

            }
            else{
                armTop.setPower(direction*.4*power);
            }
        }
        armTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armTop.setPower(0.2);

        //stopRobot and change modes back to normal
        armTop.setMode(STOP_AND_RESET_ENCODER);

        //    sleep(100);


    }

}


