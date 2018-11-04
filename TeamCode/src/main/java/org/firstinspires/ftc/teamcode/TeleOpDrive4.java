package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp (name = "TeleOp Drive 4")
public class TeleOpDrive4 extends OpMode {

    DcMotor right_front;
    DcMotor right_back;
    DcMotor left_front;
    DcMotor left_back;

    BNO055IMU imu, imu1;

    double bottom_pos ;
    double top_pos;
    double pull_pos;
    double incrementTop;
    double incrementBottom;
    int bottomTarget;



    @Override
    public void init() {

        right_front = hardwareMap.dcMotor.get("right_front");
        right_back = hardwareMap.dcMotor.get("right_back");
        left_front = hardwareMap.dcMotor.get("left_front");
        left_back = hardwareMap.dcMotor.get("left_back");

        right_front.setDirection(DcMotorSimple.Direction.REVERSE);
        right_back.setDirection(DcMotorSimple.Direction.REVERSE);
        //top.setDirection(Servo.Direction.REVERSE);



    }

    @Override
    public void loop() {

        float left_X = gamepad1.left_stick_x;
        float right_Y = gamepad1.right_stick_y;
        float left_power = right_Y;
        float right_power = right_Y;




        telemetry.addData("Right Front Power", right_front.getPower());
        telemetry.addData("Right Back Power", right_back.getPower());
        telemetry.addData("Left Front Power", left_front.getPower());
        telemetry.addData("Left Back Power", left_back.getPower());

    }
}
