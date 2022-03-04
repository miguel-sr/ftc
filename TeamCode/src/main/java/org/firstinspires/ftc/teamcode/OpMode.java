/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Timer;

@TeleOp(name="OpMode", group="Linear Opmode")

public class OpMode extends LinearOpMode {

    private DcMotor leftFront, leftRear, rightFront, rightRear;
    private DcMotor arm;
    private CRServo intake;

    ElapsedTime time = new ElapsedTime();
    private double lastError = 0;

    @Override
    public void runOpMode() {

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        arm = hardwareMap.get(DcMotor.class, "arm");
        intake = hardwareMap.get(CRServo.class, "intakeServo");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        Gamepad driverStick = gamepad1;
        Gamepad armStick = gamepad2;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double max;

            double leftStickY = -driverStick.left_stick_y;
            double leftStickX = driverStick.left_stick_x;
            double rightStickX = driverStick.right_stick_x;

            double theta = Math.atan2(leftStickY, leftStickX);
            double power = Math.hypot(leftStickX, leftStickY);

            double sin = Math.sin(theta - Math.PI/4);
            double cos = Math.cos(theta - Math.PI/4);
            max = Math.max(Math.abs(sin), Math.abs(cos));

            double leftFrontPower = power + cos/max + rightStickX;
            double rightFrontPower = power - sin/max - rightStickX;
            double leftBackPower = power - sin/max + rightStickX;
            double rightBackPower = power + cos/max - rightStickX;

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            /*
            leftFrontPower  = gamepad1.a ? 1.0 : 0.0;
            leftBackPower   = gamepad1.b ? 1.0 : 0.0;
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;
            rightBackPower  = gamepad1.x ? 1.0 : 0.0;
            */

            leftFront.setPower(leftFrontPower);
            rightFront.setPower(rightFrontPower);
            leftRear.setPower(leftBackPower);
            rightRear.setPower(rightBackPower);

            armPID(0);

            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();
        }
    }

    private void armPID(int target) {
        double kP = 0.5;
        double kI = 0.5;
        double kD = 0.1;
        double integralSum = 0;

        double armPosition = arm.getCurrentPosition();

        double error = target - armPosition;
        integralSum += error * time.seconds();
        double derivative = (error - lastError) * time.seconds();
        lastError = error;

        time.reset();

        double output = (error * kP) + (integralSum * kI) + (derivative * kD);
        arm.setTargetPosition(target);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setPower(output);
    }
}