package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

    @TeleOp (name = "TeleOp Drive Strafe")
    public class Strafing extends OpMode {

        DcMotor right_front;
        DcMotor right_back;
        DcMotor left_front;
        DcMotor left_back;
        DcMotor lift;
        float power = 0;
        float track = 0;
        boolean strafing;


        @Override
        public void init() {

            right_front = hardwareMap.dcMotor.get("right_front");
            right_back = hardwareMap.dcMotor.get("right_back");
            left_front = hardwareMap.dcMotor.get("left_front");
            left_back = hardwareMap.dcMotor.get("left_back");
            right_front.setDirection(DcMotorSimple.Direction.REVERSE);
            right_back.setDirection(DcMotorSimple.Direction.REVERSE);
            strafing = false;

            lift = hardwareMap.dcMotor.get("lift");

        }

        @Override
        public void loop() {

            float right_x = gamepad1.right_stick_x;
            float left_Y = gamepad1.left_stick_y;

            if(gamepad1.dpad_right == true) {
                if (!strafing) {
                    right_front.setDirection(DcMotorSimple.Direction.FORWARD);
                    left_back.setDirection(DcMotorSimple.Direction.REVERSE);
                    strafing = true;
                }
            }
            else {
                if(strafing) {
                    right_front.setDirection(DcMotorSimple.Direction.REVERSE);
                    left_back.setDirection(DcMotorSimple.Direction.FORWARD);
                    strafing = false;
                }
            }


            if (right_x < 0) {
                power = 1+right_x;
                right_front.setPower(left_Y);
                right_back.setPower(left_Y);
                left_front.setPower(-left_Y);
                left_back.setPower(-left_Y );
            }

            else if (right_x > 0) {
                power = 1 - right_x;
                right_front.setPower(-left_Y);
                right_back.setPower(-left_Y);
                left_front.setPower(left_Y);
                left_back.setPower(left_Y);
            }

            else{
                right_front.setPower(left_Y);
                right_back.setPower(left_Y );
                left_front.setPower(left_Y);
                left_back.setPower(left_Y);
            }

            if(gamepad1.right_trigger >0.1) {
                lift.setPower(gamepad1.right_trigger);
            }
            else if (gamepad1.left_trigger > 0.1) {
                lift.setPower(-1 * gamepad1.left_trigger);
            }
            else
                lift.setPower(0);



            telemetry.addData("Right Front Power", right_front.getPower());
            telemetry.addData("Right Back Power", right_back.getPower());
            telemetry.addData("Left Front Power", left_front.getPower());
            telemetry.addData("Left Back Power", left_back.getPower());

        }
    }


