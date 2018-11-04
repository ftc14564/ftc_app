package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import java.lang.Math;

@TeleOp (name = "TeleOp Drive 1")
public class TeleOpDrive1 extends OpMode {

    DcMotor right_front;
    DcMotor right_back;
    DcMotor left_front;
    DcMotor left_back;
    Servo bottom;
    Servo top;
    Servo pull;
    double bottom_pos ;
    double top_pos;


    @Override
    public void init() {

        right_front = hardwareMap.dcMotor.get("right_front");
        right_back = hardwareMap.dcMotor.get("right_back");
        left_front = hardwareMap.dcMotor.get("left_front");
        left_back = hardwareMap.dcMotor.get("left_back");
        bottom = hardwareMap.servo.get("bottom");
        top = hardwareMap.servo.get("top");
        pull = hardwareMap.servo.get("pull");
        right_front.setDirection(DcMotorSimple.Direction.REVERSE);
        right_back.setDirection(DcMotorSimple.Direction.REVERSE);
        //top.setDirection(Servo.Direction.REVERSE);

        top.setPosition(1);
        bottom.setPosition(1);
        pull.setPosition(0);

    }

    @Override
    public void loop() {

        float left_Y = gamepad1.left_stick_y;
        float right_Y = gamepad1.right_stick_y;
        bottom_pos = bottom.getPosition();
        top_pos=top.getPosition();

        if (Math.abs(left_Y) > 0.05) {
            left_front.setPower(gamepad1.left_stick_y);
            left_back.setPower(gamepad1.left_stick_y);

        }
        else{
            left_front.setPower(0);
            left_back.setPower(0);
        }

        if (Math.abs(right_Y) > 0.05) {

            right_front.setPower(gamepad1.right_stick_y);
            right_back.setPower(gamepad1.right_stick_y);
        }
        else {
            right_front.setPower(0);
            right_back.setPower(0);

        }

        if (gamepad1.left_trigger != 0) {
            bottom_pos-=0.005;

            if(bottom_pos<=0)
                bottom_pos=0;
            bottom.setPosition(bottom_pos);
        }
        if(gamepad1.left_bumper)
        {
            bottom_pos+=0.005;
            if(bottom_pos>=1)
                bottom_pos=1;
            bottom.setPosition(bottom_pos);
        }

        if (gamepad1.right_bumper) {

            top_pos-=0.005;
            if(top_pos<=0)
                top_pos=0;

            top.setPosition(top_pos);
        }
        if (gamepad1.right_trigger !=0)
        {
            top_pos+=0.005;
            if(top_pos>=1)
               top_pos=1;
            top.setPosition(top_pos);
        }

        if (gamepad1.dpad_down)
        {
            pull.setPosition(0);
        }

        if (gamepad1.dpad_up)
        {
            pull.setPosition(1);
        }

        telemetry.addData("Right Front Power", right_front.getPower());
        telemetry.addData("Right Back Power", right_back.getPower());
        telemetry.addData("Left Front Power", left_front.getPower());
        telemetry.addData("Left Back Power", left_back.getPower());

    }
}
