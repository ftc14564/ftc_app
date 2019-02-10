package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.CameraDevice;
import com.vuforia.ObjectTracker;
import com.vuforia.Tracker;
import com.vuforia.TrackerManager;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.Locale;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


@Autonomous(name = "Auto_Crater_cam")
public class MainAutonomous_Crater_cam extends LinearOpMode {

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

    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F, 0F, 0F};

    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;

    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    final double SCALE_FACTOR = 255;




    int pixyCounter;
    boolean isPixyObjectSeen;
    boolean pixyContinue = true;


    float power = 0;
    float track = 0;
    boolean strafing;
    boolean initDone=false;
    boolean vuInitDone=false;


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


    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";


    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;


    class InitThread_Depot implements Runnable{
        @Override
        public void run() {
            try {

                telemetry.addData("Init: Thread start ","");

                parameters = new BNO055IMU.Parameters();
                parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
                parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
                parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
                parameters.loggingEnabled = false;
                parameters.loggingTag = "IMU";
                parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
                parameters.mode = BNO055IMU.SensorMode.IMU;

                // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
                // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
                // and named "imu".
                imu = hardwareMap.get(BNO055IMU.class, "imu");
                imu.initialize(parameters);

                // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
                // first.
                initVuforia();

                if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
                    initTfod();
                } else {
                    telemetry.addData("Sorry!", "This device is not compatible with TFOD");
                }
                vuInitDone = true;
            } catch (Exception e) {
                e.printStackTrace();
            }

            initDone=true;
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

            while (pixyContinue) {
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

                byte b0 = pixy.read8(0);
                telemetry.addData("Byte 0", b0);
                byte b1 = pixy.read8(1);
                telemetry.addData("Byte 1", b1);
                byte b2 = pixy.read8(2);
                telemetry.addData("Byte 2", b2 );
                byte b3 = pixy.read8(3);
                telemetry.addData("Byte 3", b3);
                byte b4 = pixy.read8(4);
                telemetry.addData("Byte 4", b4);
                byte b5 = pixy.read8(5);
                telemetry.addData("Byte 5", b5);
                byte b6 = pixy.read8(6);
                telemetry.addData("Byte 6", b6);
                telemetry.update();
                if (b0!=0){
                    if(pixyCounter < 10)
                        pixyCounter+=4;

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


                byte b = pixy.read8(7);
//                telemetry.addData("Byte 7", b);
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

        sensorRange_rf = hardwareMap.get(DistanceSensor.class, "2m_rf");
        distanceSensor_rf = (Rev2mDistanceSensor)sensorRange_rf;
        sensorRange_rb = hardwareMap.get(DistanceSensor.class, "2m_rb");
        distanceSensor_rb = (Rev2mDistanceSensor)sensorRange_rb;
        sensorRange_lf = hardwareMap.get(DistanceSensor.class, "2m_lf");
        distanceSensor_lf = (Rev2mDistanceSensor)sensorRange_lf;
        sensorRange_lb = hardwareMap.get(DistanceSensor.class, "2m_lb");
        distanceSensor_lb = (Rev2mDistanceSensor)sensorRange_lb;

        new Thread(new InitThread_Depot()).start();
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
    double TURN_THRESHOLD = 1;

    public void gyroTurnREV(double speed, double angle){

        telemetry.addData("starting gyro turn","-----");
        telemetry.update();

        while(opModeIsActive() && !onTargetAngleREV(speed, angle, P_TURN_COEFF, 3)){
            telemetry.update();
            idle();
            telemetry.addData("-->","inside while loop :-(");
            telemetry.update();
        }
        //sleep(100);
        while(opModeIsActive() && !onTargetAngleREV(speed, angle, P_TURN_COEFF/3, 1)){
            telemetry.update();
            idle();
            telemetry.addData("-->","inside while loop :-(");
            telemetry.update();
        }

        telemetry.addData("done with gyro turn","-----");
        telemetry.update();
    }
    boolean onTargetAngleREV(double speed, double angle, double PCoeff, double turnThreshold){
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        //determine turm power based on error
        error = getErrorREV(angle);

        if (Math.abs(error) <= turnThreshold){

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
            weightConstant*=1.5;

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

    public void Rotate (double power, int direction, double angle) {

        //angle -=angle*.35;
        power /= 1.5;

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
            while (opModeIsActive() && (Math.abs(((Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)))))  < angle ) )
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

            while (opModeIsActive() && (((Math.abs(Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)))))  < angle ) )
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
    public void SlowerRotate (double power, int direction, double angle) {

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
            while (opModeIsActive() && (Math.abs(((Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)))))  < angle ) &&
                    counter++<50)
            {
                //counter++;
                /*if(System.currentTimeMillis()-startTime > 29500 ){
                    break;
                }*/
                telemetry.update();
                telemetry.addData("turning (imu degrees)", formatAngle(angles.angleUnit, angles.firstAngle));
                telemetry.update();


                double _power = 1.5*power*((angle-Math.abs(((Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle))))))/angle);

               // if(_power < 0.3) _power = 0.3;

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

            while (opModeIsActive() && (((Math.abs(Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)))))  < angle ) &&
                    counter++ < 50)
            {
                telemetry.addData("turning (imu degrees)", formatAngle(angles.angleUnit, angles.firstAngle));
                telemetry.update();

                double _power = 1.5*power*((angle-Math.abs(((Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle))))))/angle);
              //  if(_power < 0.3) _power = 0.3;
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

    public void straight (double power, int direction, double distance) {

        distance /= 2.25;
        power /= 1;

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


        while ( opModeIsActive() && (Math.abs(motorLeftBack.getCurrentPosition()) < Math.abs(distance)
                && Math.abs(motorLeftFront.getCurrentPosition()) < Math.abs(distance)
                && Math.abs(motorRightFront.getCurrentPosition()) < Math.abs(distance)
                && Math.abs(motorRightBack.getCurrentPosition()) < Math.abs(distance) )) {

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
//        sleep(200);


    }

    /* direction : +1 is right , -1 is left
       distance: in ticks
     */
    public void strafe (double power, int direction, double distance)  {

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





        while (opModeIsActive() && (Math.abs(motorLeftBack.getCurrentPosition()) < Math.abs(distance)
                && Math.abs(motorLeftFront.getCurrentPosition()) < Math.abs(distance)
                && Math.abs(motorRightFront.getCurrentPosition()) < Math.abs(distance)
                && Math.abs(motorRightBack.getCurrentPosition()) < Math.abs(distance) )) {

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

       // sleep(200);

    }
    public void depositMarker(){
        try{
//            turnBottomArm(0.9, -1, 600);
//            turnTopArm(0.9, 1, 600);
//
//            grabBase.setPosition(0.2);
//            sleep(1000);
//            grabBase.setPosition(0.7);
//            turnTopArm(0.6, -1, 500);
//            turnBottomArm(0.6, 1, 500);

            grabServo.setPosition(1);
            sleep(1000);
            grabServo.setPosition(0.5);

        } catch (Exception e) {
            e.printStackTrace();
        }
    }
    public void makeParallelLeft()  {
        double sensor_gap = 24;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if(distanceSensor_lb.getDistance(DistanceUnit.CM) < (distanceSensor_lf.getDistance(DistanceUnit.CM)) ){
            double theta;
            //telemetry.addData(" test ", 1);

            double diff1 = (distanceSensor_lf.getDistance(DistanceUnit.CM)) - (distanceSensor_lb.getDistance(DistanceUnit.CM));
            double diff2 = (distanceSensor_lf.getDistance(DistanceUnit.CM)) - (distanceSensor_lb.getDistance(DistanceUnit.CM));
            double diff3 = (distanceSensor_lf.getDistance(DistanceUnit.CM)) - (distanceSensor_lb.getDistance(DistanceUnit.CM));
            double diff = (diff1+diff2+diff3 )/3;
            double temp = diff/sensor_gap;
            //telemetry.addData(" test ", 2);
            //telemetry.update();
            theta = Math.asin(temp) *180/3.141592;
            telemetry.addData(" lb", distanceSensor_lb.getDistance(DistanceUnit.CM));
            telemetry.addData(" lf", distanceSensor_lf.getDistance(DistanceUnit.CM));
            telemetry.addData(" theta", theta);
            telemetry.update();
            if(theta > 1){
                gyroTurnREV(1, angles.firstAngle + theta);
            }

        }
        else{
            double teta;
            double diff1 = (distanceSensor_lb.getDistance(DistanceUnit.CM)) - (distanceSensor_lf.getDistance(DistanceUnit.CM));
            double diff2 = (distanceSensor_lb.getDistance(DistanceUnit.CM)) - (distanceSensor_lf.getDistance(DistanceUnit.CM));
            double diff3 = (distanceSensor_lb.getDistance(DistanceUnit.CM)) - (distanceSensor_lf.getDistance(DistanceUnit.CM));
            double diff = (diff1+diff2+diff3)/3;
            double temp = diff/sensor_gap;
            teta = Math.asin(temp) *180/3.141592;
            if(teta > 1){
                gyroTurnREV(1, angles.firstAngle - teta);
            }
            telemetry.addData(" lb_", distanceSensor_lb.getDistance(DistanceUnit.CM));
            telemetry.addData(" lf_", distanceSensor_lf.getDistance(DistanceUnit.CM));
            telemetry.addData(" theta_", teta);
            telemetry.update();
        }

        double dis = distanceSensor_lb.getDistance(DistanceUnit.CM);
        if(dis < 9){
            strafe(0.7,-1,200);
        }
        else if(dis > 12){
            strafe(0.7,1,((dis-11)/2.54)*200);
        }
    }
    public void makeParallelRight()
    {

        double sensor_gap = 26;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        if(distanceSensor_rb.getDistance(DistanceUnit.CM) < (distanceSensor_rf.getDistance(DistanceUnit.CM)) ){
            double theta;
            //telemetry.addData(" test ", 1);

            double diff1 = (distanceSensor_rf.getDistance(DistanceUnit.CM)) - (distanceSensor_rb.getDistance(DistanceUnit.CM));
            double diff2 = (distanceSensor_rf.getDistance(DistanceUnit.CM)) - (distanceSensor_rb.getDistance(DistanceUnit.CM));
            double diff3 = (distanceSensor_rf.getDistance(DistanceUnit.CM)) - (distanceSensor_rb.getDistance(DistanceUnit.CM));
            double diff4 = (distanceSensor_rf.getDistance(DistanceUnit.CM)) - (distanceSensor_rb.getDistance(DistanceUnit.CM));
            double diff5 = (distanceSensor_rf.getDistance(DistanceUnit.CM)) - (distanceSensor_rb.getDistance(DistanceUnit.CM));
            double diff6 = (distanceSensor_rf.getDistance(DistanceUnit.CM)) - (distanceSensor_rb.getDistance(DistanceUnit.CM));
            double diff = (diff1+diff2+diff3 + diff4 + diff5 + diff6)/6;
            double temp = diff/sensor_gap;
            //telemetry.addData(" test ", 2);
            //telemetry.update();
            theta = Math.asin(temp) *180/3.141592;
            telemetry.addData(" lb", distanceSensor_lb.getDistance(DistanceUnit.CM));
            telemetry.addData(" lf", distanceSensor_lf.getDistance(DistanceUnit.CM));
            telemetry.addData(" theta", theta);
            telemetry.update();
            if(theta > 1){

                gyroTurnREV(1, angles.firstAngle - theta);
            }

        }
        else{
            double teta;
            double diff1 = (distanceSensor_rb.getDistance(DistanceUnit.CM)) - (distanceSensor_rf.getDistance(DistanceUnit.CM));
            double diff2 = (distanceSensor_rb.getDistance(DistanceUnit.CM)) - (distanceSensor_rf.getDistance(DistanceUnit.CM));
            double diff3 = (distanceSensor_rb.getDistance(DistanceUnit.CM)) - (distanceSensor_rf.getDistance(DistanceUnit.CM));
            double diff = (diff1+diff2+diff3)/3;
            double temp = diff/sensor_gap;
            teta = Math.asin(temp) *180/3.141592;
            if(teta > 1){
                gyroTurnREV(1, angles.firstAngle + teta);
            }
            telemetry.addData(" rb_", distanceSensor_rb.getDistance(DistanceUnit.CM));
            telemetry.addData(" rf_", distanceSensor_rf.getDistance(DistanceUnit.CM));
            telemetry.addData(" theta_", teta);
            telemetry.update();
        }

        double dis = distanceSensor_rb.getDistance(DistanceUnit.CM);
        if(dis < 13){
            strafe(0.7,1,200);
        }
        else if(dis > 16){
            strafe(0.7,-1,((dis-15)/2.54)*200);
        }
    }


    @Override
    public void runOpMode() {




        pixyCounter = 0;
        initFn();

        waitForStart();

        while(!initDone && opModeIsActive()) sleep(100);


//            int x = 0;
//            x++;
//            while (x == 1 && opModeIsActive()) {
//                telemetry.addData("PixyObjSeen ", isPixyObjectSeen);
//
//                sleep(100);
//            }

//        int x=0;
//        //makeParallelLeft();
//        makeParallelRight();
//        x++;
//        if(x==1)
//            return;

//        int counter=0;
//        while(counter++<100){
//            makeParallelLeftLeftLeft();
//            sleep(1000);
//
//        }


            try {

                boolean right = false;
                boolean center = false;
                boolean left = false;


//            lift.setMode(STOP_AND_RESET_ENCODER);
//            lift.setMode(RUN_WITHOUT_ENCODER);
//            lift.setDirection(DcMotorSimple.Direction.FORWARD);
//            while (Math.abs(lift.getCurrentPosition()) < Math.abs(11.25*1120))
//            {
//                lift.setPower(-1.0);
//            }
//            lift.setPower(0);
//            //    unhooking


                if (vuInitDone && tfod != null) {
                    tfod.activate();


                    sleep(1000);
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() >= 0) {
                            int goldMineralX = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                    if (goldMineralX < 250) {
                                        left = true;
                                        telemetry.addData("From Cam : Left", "");
                                    } else if (goldMineralX > 350) {
                                        right = true;
                                        telemetry.addData("From Cam : right", "");
                                    } else {
                                        center = true;
                                        telemetry.addData("From Cam : center", "");

                                    }
                                }

                            }

                        }
                    }
                    stopVuforia();
                }
                telemetry.update();

                if (left == true) {
                    ProceedWithCam(0);

                } else if (center == true) {
                    ProceedWithCam(1);

                } else if (right == true) {
                    ProceedWithCam(2);

                } else {
                    ProccedWithPixy();
                }


            } catch (Exception e) {
                e.printStackTrace();
            }

        //stop all motors
        grabServo.setPosition(0.5);
        armBottom.setPower(0);
        armTop.setPower(0);
        stopWheels();
        lift.setPower(0);
        pixyContinue = false;

    }
    void ProceedWithCam(int position){

        telemetry.addData("ProceedWithCam", "");
        telemetry.update();

        try {


            straight(1, -1, 332);   // 2.5 inch fwd
            strafe(0.8, 1, 1800);    // 9 inch left
            straight(1, 1, 450);   // 4.25 inch back

            if (position == 0){
                gyroTurnREV(1, -60);
                straight(0.8, 1, 2926); // 22 inch
                straight(0.8,-1,1863); // 14 inch back
                gyroTurnREV(1, 180);

                straight(1,-1,4500); // 36 inch fwd

                gyroTurnREV(1, -135);
                makeParallelRight();
                straight(1,-1,5300); //  inch fwd

                depositMarker();
                strafe(1,1, 400);
                gyroTurnREV(1, 45);
                makeParallelLeft();
                straight(1, -1, 6000); //  inch fwd
                makeParallelLeft();
                straight(0.6, -1, 3500); //  inch fwd

            }
            else if (position == 1){
                gyroTurnREV(1, -90);
                straight(0.8, 1, 2394); // 18 inch
                straight(0.8,-1,1600); // 12 inch back
                gyroTurnREV(1, 180);


                straight(1,-1,5000); // 32 inch fwd
                gyroTurnREV(1, -135);
                makeParallelRight();
                straight(1,-1,5500); //  inch fwd


                depositMarker();
                strafe(1,1, 400);
                gyroTurnREV(1, 45);
                makeParallelLeft();
                straight(1, -1, 6000); //  inch fwd
                makeParallelLeft();
                straight(0.6, -1, 3500); //  inch fwd


            }
            else if (position == 2) {
                gyroTurnREV(1, -120);
                straight(0.8, 1, 2926); // 22 inch
                straight(0.8,-1,1863); // 14 inch back
                gyroTurnREV(1, 180);

                straight(1,-1,6400); // 50 inch fwd
                gyroTurnREV(1, -135);
                makeParallelRight();
                straight(1,-1,5200); //  inch fwd

                depositMarker();
                strafe(1,1, 400);
                gyroTurnREV(1, 45);
                makeParallelLeft();
                straight(1, -1, 6000); //  inch fwd
                makeParallelLeft();
                straight(0.6, -1, 3500); //  inch fwd

            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
    void ProccedWithPixy() {

        pixyCounter = 0;
        isPixyObjectSeen = false;

        telemetry.addData("ProceedWithPixy", "");
        telemetry.update();

        try {

            straight(1, -1, 332);   // 2.5 inch fwd
            strafe(0.8, 1, 2400);    // 12 inch left
            straight(0.8, 1, 450);   // 4.25 inch back

            //check middle
            gyroTurnREV(1, -55);
            sleep(1000);

            telemetry.addData("PixyObjSeen ", isPixyObjectSeen);
            telemetry.update();
            if(isPixyObjectSeen)
                System.out.println("pixyObjSeen A 1 ");
            else
                System.out.println("pixyObjSeen A 0 ");

            if (isPixyObjectSeen) {
                pixyContinue = false;
                straight(0.8, 1, 2400); // 18 inch
                straight(0.8, -1, 1330); // 10 inch back
                gyroTurnREV(1, 180);
                straight(1, -1, 4500); // 42 inch fwd

                gyroTurnREV(1, -135);
                makeParallelRight();
                straight(1, -1, 5200); //  inch fwd

                depositMarker();
                strafe(1, 1, 400);
                gyroTurnREV(1, 45);
                makeParallelLeft();
                straight(1, -1, 6000); //  inch fwd
                makeParallelLeft();
                straight(0.6, -1, 3500); //  inch fwd


            } else {

                gyroTurnREV(1, -90);

                sleep(1000);
                if(isPixyObjectSeen)
                    System.out.println("pixyObjSeen B 1 ");
                else
                    System.out.println("pixyObjSeen B 0 ");

                telemetry.addData("PixyObjSeen ", isPixyObjectSeen);
                telemetry.update();
                telemetry.addData("Debug", "2");
                if (isPixyObjectSeen) {
                    pixyContinue = false;


                    straight(0.8, 1, 1596); // 11 inch
                    straight(0.8, -1, 1330); // 10 inch back
                    gyroTurnREV(1, 180);

                    straight(1, -1, 4800); // 38 inch fwd
                    gyroTurnREV(1, -135);
                    makeParallelRight();
                    straight(1, -1, 5200); //  inch fwd


                    depositMarker();
                    strafe(1, 1, 400);
                    gyroTurnREV(1, 45);
                    makeParallelLeft();
                    straight(1, -1, 6000); //  inch fwd
                    makeParallelLeft();
                    straight(0.6, -1, 3500); //  inch fwd


                } else {
                    pixyContinue = false;

                    gyroTurnREV(1, -125);

                    sleep(1000);
                    if(isPixyObjectSeen)
                        System.out.println("pixyObjSeen C 1 ");
                    else
                        System.out.println("pixyObjSeen C 0 ");

                    telemetry.addData("PixyObjSeen ", isPixyObjectSeen);
                    telemetry.update();
                    if (isPixyObjectSeen) {
                        pixyContinue = false;
                        straight(0.8, 1, 2400); // 18 inch
                        straight(0.8, -1, 1330); // 10 inch back
                        gyroTurnREV(1, 180);
                        straight(1, -1, 5000); // 50 inch fwd

                        gyroTurnREV(1, -135);
                        makeParallelRight();
                        straight(1, -1, 5200); //  inch fwd

                        depositMarker();
                        strafe(1, 1, 400);
                        gyroTurnREV(1, 45);
                        makeParallelLeft();
                        straight(1, -1, 6000); //  inch fwd
                        makeParallelLeft();
                        straight(0.6, -1, 3500); //  inch fwd

                    }
                    else {
                        pixyContinue = false;
                        gyroTurnREV(1, 180);
                        straight(1, -1, 5000); // 50 inch fwd

                        gyroTurnREV(1, -135);
                        makeParallelRight();
                        straight(1, -1, 5200); //  inch fwd

                        depositMarker();
                        strafe(1, 1, 400);
                        gyroTurnREV(1, 45);
                        makeParallelLeft();
                        straight(1, -1, 6000); //  inch fwd
                        makeParallelLeft();
                        straight(0.6, -1, 3500); //  inch fwd

                    }

                }
            }


        } catch (Exception e) {
            e.printStackTrace();
        }

        telemetry.update();

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

    public void turnBottomArm (double power, int direction, double distance)  {

        armBottom.setMode(STOP_AND_RESET_ENCODER);


        telemetry.addData("turnBottomArm", "In straight()");
        telemetry.update();


        armBottom.setMode(RUN_WITHOUT_ENCODER);
        armBottom.setDirection(DcMotorSimple.Direction.REVERSE);

        int count = 0;
        while (Math.abs(armBottom.getCurrentPosition()) < Math.abs(distance)
                && count++ <50 && opModeIsActive()) {

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
    public void turnTopArm (double power, int direction, double distance)  {

        armTop.setMode(STOP_AND_RESET_ENCODER);


        telemetry.addData("armTop", "In straight()");
        telemetry.update();


        armTop.setMode(RUN_WITHOUT_ENCODER);
        armTop.setDirection(DcMotorSimple.Direction.REVERSE);

        int count =0;
        while (Math.abs(armTop.getCurrentPosition()) < Math.abs(distance)
                && count++ < 30 && opModeIsActive()) {

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


    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    private void stopVuforia() {
        Tracker objectTracker = TrackerManager.getInstance().getTracker(ObjectTracker.getClassType());

        if (objectTracker != null) {
            objectTracker.stop();
        }
        tfod.deactivate();
        tfod.shutdown();
        vuforia.getCamera().close();
    }
}


