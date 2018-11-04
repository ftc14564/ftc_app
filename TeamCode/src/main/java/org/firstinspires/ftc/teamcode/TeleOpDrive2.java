package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "TeleOp Drive 2")
public class TeleOpDrive2 extends OpMode {

    DcMotor right_front;
    DcMotor right_back;
    DcMotor left_front;
    DcMotor left_back;
    Servo bottom;
    Servo top;
    Servo pull;
    double bottom_pos ;
    double top_pos;
    double pull_pos;
    double incrementTop;
    double incrementBottom;



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
        //pull.setPosition(0);

        incrementTop = 0.005;
        incrementBottom = 0.001;
        pull_pos=0;
    }

    @Override
    public void loop() {

        float left_X = gamepad1.left_stick_x;
        float right_Y = gamepad1.right_stick_y;
        float left_power = right_Y;
        float right_power = right_Y;
        bottom_pos = bottom.getPosition();
        top_pos=top.getPosition();
        pull_pos=pull.getPosition();


        if (Math.abs(left_X) > 0.05) {
            left_power -= left_X;
            right_power += left_X;

        }


        if (Math.abs(gamepad1.right_stick_y) > 0.05) {

            right_front.setPower(right_power);
            right_back.setPower(right_power);
            left_front.setPower(left_power);
            left_back.setPower(left_power);
        }
        else {
            right_front.setPower(0);
            right_back.setPower(0);
            left_front.setPower(0);
            left_back.setPower(0);
        }




        if (gamepad1.left_trigger != 0) {
            bottom_pos-=incrementBottom;
            if (incrementBottom > 0.001)
  //              incrementBottom -= 0.001;

            if(bottom_pos<=0)
                bottom_pos=0;
            bottom.setPosition(bottom_pos);
        }
        else if(gamepad1.left_bumper)
        {
            bottom_pos+=incrementBottom;
            if (incrementBottom > 0.001)
                //incrementBottom -= 0.001;
            if(bottom_pos>=1)
                bottom_pos=1;
            bottom.setPosition(bottom_pos);
        }
        else
            incrementBottom = 0.001;

        if (gamepad1.right_bumper) {

            top_pos-=incrementTop;
            if (incrementTop > 0.001)
                incrementTop-=0.001;

            if(top_pos<=0)
                top_pos=0;

            top.setPosition(top_pos);
        }
        else if (gamepad1.right_trigger !=0)
        {
            top_pos+=incrementTop;
            if (incrementTop > 0.001)
                incrementTop-=0.001;

            if(top_pos>=1)
               top_pos=1;
            top.setPosition(top_pos);
        }
        else
            incrementTop = 0.005;

        if (gamepad1.dpad_down)
        {
            pull.setPosition(0.1);
        }

        if (gamepad1.dpad_up)
        {
            pull.setPosition(0.6);
        }

        telemetry.addData("Right Front Power", right_front.getPower());
        telemetry.addData("Right Back Power", right_back.getPower());
        telemetry.addData("Left Front Power", left_front.getPower());
        telemetry.addData("Left Back Power", left_back.getPower());

    }
}
