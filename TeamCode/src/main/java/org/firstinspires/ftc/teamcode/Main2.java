package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static android.os.SystemClock.sleep;
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


@TeleOp (name = "TeleOp Drive Main2")
public class Main2 extends OpMode {

    DcMotor motorRightFront;
    DcMotor motorRightBack;
    DcMotor motorLeftFront;
    DcMotor motorLeftBack;
    DcMotor armBottom;
    DcMotor armTop;

    Servo grabServo;
    double grabpos;




    float power = 0;
    float track = 0;
    boolean strafing;
    boolean initDone=false;

    double angleToTurn;

    I2cDeviceSynch pixy;

    BNO055IMU imu, imu1;
    Orientation angles, angles1;
    Acceleration gravity, gravity1;

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


    class InitThread implements Runnable{
        @Override
        public void run() {
            try {

                telemetry.addData("Init: Thread start ","");

                BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
                parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
                parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
                parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
                parameters.loggingEnabled = true;
                parameters.loggingTag = "IMU";
                parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

                // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
                // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
                // and named "imu".
                imu = hardwareMap.get(BNO055IMU.class, "imu");
                imu.initialize(parameters);
                imu1 = hardwareMap.get(BNO055IMU.class, "imu_1");
                imu1.initialize(parameters);


                /*
                 * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
                 * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
                 * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
                 */
                int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
                Vu_parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

                // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

                Vu_parameters.vuforiaLicenseKey = VUFORIA_KEY ;
                Vu_parameters.cameraDirection   = CAMERA_CHOICE;

                //  Instantiate the Vuforia engine
                if(vuforia == null)
                    vuforia = ClassFactory.getInstance().createVuforia(Vu_parameters);

                // Load the data sets that for the trackable objects. These particular data
                // sets are stored in the 'assets' part of our application.
                VuforiaTrackables targetsRoverRuckus = vuforia.loadTrackablesFromAsset("RoverRuckus");
                VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
                blueRover.setName("Blue-Rover");
                VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
                redFootprint.setName("Red-Footprint");
                VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
                frontCraters.setName("Front-Craters");
                VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
                backSpace.setName("Back-Space");

                // For convenience, gather together all the trackable objects in one easily-iterable collection */
                allTrackables = new ArrayList<VuforiaTrackable>();
                allTrackables.addAll(targetsRoverRuckus);

                /**
                 * In order for localization to work, we need to tell the system where each target is on the field, and
                 * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
                 * Transformation matrices are a central, important concept in the math here involved in localization.
                 * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
                 * for detailed information. Commonly, you'll encounter transformation matrices as instances
                 * of the {@link OpenGLMatrix} class.
                 *
                 * If you are standing in the Red Alliance Station looking towards the center of the field,
                 *     - The X axis runs from your left to the right. (positive from the center to the right)
                 *     - The Y axis runs from the Red Alliance Station towards the other side of the field
                 *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
                 *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
                 *
                 * This Rover Ruckus sample places a specific target in the middle of each perimeter wall.
                 *
                 * Before being transformed, each target image is conceptually located at the origin of the field's
                 *  coordinate system (the center of the field), facing up.
                 */

                /**
                 * To place the BlueRover target in the middle of the blue perimeter wall:
                 * - First we rotate it 90 around the field's X axis to flip it upright.
                 * - Then, we translate it along the Y axis to the blue perimeter wall.
                 */
                OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                        .translation(0, mmFTCFieldWidth, mmTargetHeight)
                        .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
                blueRover.setLocation(blueRoverLocationOnField);

                /**
                 * To place the RedFootprint target in the middle of the red perimeter wall:
                 * - First we rotate it 90 around the field's X axis to flip it upright.
                 * - Second, we rotate it 180 around the field's Z axis so the image is flat against the red perimeter wall
                 *   and facing inwards to the center of the field.
                 * - Then, we translate it along the negative Y axis to the red perimeter wall.
                 */
                OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                        .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                        .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
                redFootprint.setLocation(redFootprintLocationOnField);

                /**
                 * To place the FrontCraters target in the middle of the front perimeter wall:
                 * - First we rotate it 90 around the field's X axis to flip it upright.
                 * - Second, we rotate it 90 around the field's Z axis so the image is flat against the front wall
                 *   and facing inwards to the center of the field.
                 * - Then, we translate it along the negative X axis to the front perimeter wall.
                 */
                OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                        .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                        .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90));
                frontCraters.setLocation(frontCratersLocationOnField);

                /**
                 * To place the BackSpace target in the middle of the back perimeter wall:
                 * - First we rotate it 90 around the field's X axis to flip it upright.
                 * - Second, we rotate it -90 around the field's Z axis so the image is flat against the back wall
                 *   and facing inwards to the center of the field.
                 * - Then, we translate it along the X axis to the back perimeter wall.
                 */
                OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                        .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                        .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
                backSpace.setLocation(backSpaceLocationOnField);

                /**
                 * Create a transformation matrix describing where the phone is on the robot.
                 *
                 * The coordinate frame for the robot looks the same as the field.
                 * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
                 * Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
                 *
                 * The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
                 * pointing to the LEFT side of the Robot.  It's very important when you test this code that the top of the
                 * camera is pointing to the left side of the  robot.  The rotation angles don't work if you flip the phone.
                 *
                 * If using the rear (High Res) camera:
                 * We need to rotate the camera around it's long axis to bring the rear camera forward.
                 * This requires a negative 90 degree rotation on the Y axis
                 *
                 * If using the Front (Low Res) camera
                 * We need to rotate the camera around it's long axis to bring the FRONT camera forward.
                 * This requires a Positive 90 degree rotation on the Y axis
                 *
                 * Next, translate the camera lens to where it is on the robot.
                 * In this example, it is centered (left to right), but 110 mm forward of the middle of the robot, and 200 mm above ground level.
                 */

                final int CAMERA_FORWARD_DISPLACEMENT  = 110;   // eg: Camera is 110 mm in front of robot center
                final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
                final int CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

                OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                        .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                        .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                                CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));

                /**  Let all the trackable listeners know where the phone is.  */
                for (VuforiaTrackable trackable : allTrackables)
                {
                    ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, Vu_parameters.cameraDirection);
                }

                targetsRoverRuckus.activate();



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


            initDone = true;
            telemetry.addData("Init: Thread done ","");

        }

    };


    @Override
    public void init() {

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



        armBottom = hardwareMap.dcMotor.get("armBottom");
        armTop = hardwareMap.dcMotor.get("armTop");
        armBottom.setDirection(DcMotorSimple.Direction.REVERSE);
        armTop.setDirection(DcMotorSimple.Direction.REVERSE);
        armTop.setMode(RUN_WITHOUT_ENCODER);
        armBottom.setMode(RUN_WITHOUT_ENCODER);
        grabServo = hardwareMap.servo.get("grab_servo");



        grabpos = 0.55;

        angleToTurn = 30;

        double driveSpeed = 0;
        motorLeftBack.setPower(driveSpeed);
        motorLeftFront.setPower(driveSpeed);
        motorRightBack.setPower(driveSpeed);
        motorRightFront.setPower(driveSpeed);

        grabServo.setPosition(grabpos);

        new Thread(new InitThread()).start();

        //leftServo.setPosition(leftpos);
        //rightServo.setPosition(rightServoPos);

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
        if(direction == -1.0 ){
            // LEFT
            motorLeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
            motorLeftBack.setDirection(DcMotorSimple.Direction.REVERSE);
            motorRightFront.setDirection(DcMotorSimple.Direction.REVERSE);
            motorRightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        else{
            // RIGHT
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
        angles1 = imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

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
                telemetry.addData("sum of gyros / 2", (((Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle))))  + Double.parseDouble(formatAngle(angles1.angleUnit, angles1.firstAngle)) / 2));
                telemetry.update();


                double _power = 1.15*power*((angle-Math.abs(((Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle))))))/angle);
                motorLeftFront.setPower(_power);
                motorRightBack.setPower(_power);
                motorRightFront.setPower(_power);
                motorLeftBack.setPower(_power);

                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                angles1 = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

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
                angles1 = imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
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


    @Override
    public void loop() {

        float right_x = gamepad1.right_stick_x;
        float left_Y = gamepad1.left_stick_y;



        if(gamepad1.dpad_right == true) {
            if (!strafing) {
                motorRightFront.setDirection(DcMotorSimple.Direction.FORWARD);
                motorLeftBack.setDirection(DcMotorSimple.Direction.REVERSE);
                strafing = true;
            }
        }
        else {
            if(strafing) {
                motorRightFront.setDirection(DcMotorSimple.Direction.REVERSE);
                motorLeftBack.setDirection(DcMotorSimple.Direction.FORWARD);
                strafing = false;
            }
        }



        if (right_x < 0) {
            power = 1+right_x;
            motorRightFront.setPower(left_Y);
            motorRightBack.setPower(left_Y);
            motorLeftFront.setPower(-left_Y);
            motorLeftBack.setPower(-left_Y );
        }

        else if (right_x > 0) {
            power = 1 - right_x;
            motorRightFront.setPower(-left_Y);
            motorRightBack.setPower(-left_Y);
            motorLeftFront.setPower(left_Y);
            motorLeftBack.setPower(left_Y);
        }

        else{
            motorRightFront.setPower(left_Y);
            motorRightBack.setPower(left_Y );
            motorLeftFront.setPower(left_Y);
            motorLeftBack.setPower(left_Y);
        }


        if (gamepad1.right_bumper && gamepad1.right_trigger > 0.1) {
            //armBottom.setPower(-1 * gamepad1.right_trigger);
            try {
                turnTopArm(1, 1, 100);
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
        else if(gamepad1.right_trigger >0.1) {
            //armBottom.setPower(gamepad1.right_trigger);
            try {
                turnTopArm(0.8, -1, 100);
            } catch (Exception e) {
            e.printStackTrace();
        }

        }
//        else
//            armTop.setPower(0);

        if (gamepad1.left_bumper && gamepad1.left_trigger > 0.1) {
            //armTop.setPower(-1 * gamepad1.right_trigger);
            try {

                turnBottomArm(0.8, 1, 300);
            }
            catch(Exception e){
                e.printStackTrace();
            }
        }
        else if(gamepad1.left_trigger >0.1) {
            //armTop.setPower(gamepad1.right_trigger);
            try {

                turnBottomArm(0.8, -1, 300);
            }
            catch(Exception e){
                e.printStackTrace();
            }
        }
        else
            armBottom.setPower(0);

        if(gamepad1.dpad_up) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            angleToTurn = 30 + Math.abs(((Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)))));
            OLDrotate(0.8, 1, angleToTurn);
        }
        if(gamepad1.dpad_down) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            angleToTurn =  Math.abs(((Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle))))) + 30;
            OLDrotate(0.8, -1, angleToTurn);
        }

        if(gamepad1.y) {
            try {
                straight(1, 1, 1600);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        if(gamepad1.x) {
            try {
                straight(1, -1, 1600);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
//        if(gamepad1.y){
//            turnBottomArm_E(5);
//        }
//        if(gamepad1.b){
//            turnTopArm_E(5);
//        }

/*
        if (gamepad1.left_bumper && gamepad1.left_trigger > 0.1) {
            rightServo.setPower(-1 * gamepad1.left_trigger);
        }
        else if(gamepad1.left_trigger >0.1) {
            rightServo.setPower(gamepad1.left_trigger);
        }
        else
            rightServo.setPower(0);
*/

/*
        if (gamepad2.left_bumper && gamepad2.left_trigger > 0.1) {
            leftpos = leftServo.getPosition();
            if(leftpos<170) {
                leftpos += 0.005;
                leftServo.setPosition(leftpos);
            }
        }
        else if(gamepad2.left_trigger >0.1) {
            leftpos = leftServo.getPosition();
            if(leftpos>=0.005) {
                leftpos -= 0.005;
                leftServo.setPosition(leftpos);
            }
        }
*/
        telemetry.addData("grab pos", grabServo.getPosition());

        if (gamepad2.left_trigger != 0) {
           // leftpos-=0.005;

         //   if(leftpos<=0.3)
                grabpos=0.35;
            grabServo.setPosition(grabpos);
            telemetry.addData("grab pos", grabpos);
        }
        if(gamepad2.left_bumper)
        {
         //   leftpos+=0.005;
         //   if(leftpos>=0.6)
                grabpos=0.7;
            grabServo.setPosition(grabpos);
            telemetry.addData("grab pos", grabpos);
        }



        if(initDone) {


            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            } else {
                telemetry.addData("Visible Target", "none");
            }


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


        telemetry.addData("Byte 0", pixy.read8(0));
        telemetry.addData("Byte 1", pixy.read8(1));
        telemetry.addData("Byte 2", pixy.read8(2));
        telemetry.addData("Byte 3", pixy.read8(3));
        telemetry.addData("Byte 4", pixy.read8(4));
        telemetry.addData("Byte 5", pixy.read8(5));
        telemetry.addData("Byte 6", pixy.read8(6));
        telemetry.addData("Byte 7", pixy.read8(7));
        telemetry.addData("Byte 8", pixy.read8(8));
        telemetry.addData("Byte 9", pixy.read8(9));
        telemetry.addData("Byte 10", pixy.read8(10));
        telemetry.addData("Byte 11", pixy.read8(11));
        telemetry.addData("Byte 12", pixy.read8(12));
        telemetry.addData("Byte 13", pixy.read8(13));

        telemetry.addData("Right Front Power", motorRightFront.getPower());
        telemetry.addData("Right Back Power", motorRightBack.getPower());
        telemetry.addData("Left Front Power", motorLeftFront.getPower());
        telemetry.addData("Left Back Power", motorLeftBack.getPower());

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
            armBottom.setPower(direction*.2*power);

            if (Math.abs(armBottom.getCurrentPosition()) < .05 * Math.abs(distance)){
                armBottom.setPower(direction*power);
            }
            else if (Math.abs(armBottom.getCurrentPosition()) < .3 * Math.abs(distance)) {
                armBottom.setPower(direction*.6*power);

            }
            else if (Math.abs(armBottom.getCurrentPosition()) < .6 * Math.abs(distance)){
                armBottom.setPower(direction*.55*power);
            }
            else if (Math.abs(armBottom.getCurrentPosition()) < .7 * Math.abs(distance)){
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
            armTop.setPower(direction*.2*power);

            if (Math.abs(armTop.getCurrentPosition()) < .1 * Math.abs(distance)){
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
                armTop.setPower(direction*.3*power);
            }
        }
        armTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armTop.setPower(0.2);

        //stopRobot and change modes back to normal
        armTop.setMode(STOP_AND_RESET_ENCODER);

    //    sleep(100);


    }

}


