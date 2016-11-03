/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "Omnidirectional Drive TestMotors", group = "Linear Opmode")
// @Autonomous(...) is the other common choice
//@Disabled
public class Omnidirectional_Drive_TestMotors extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();


    DcMotor frontLeftMotor = null;
    DcMotor frontRightMotor = null;
    DcMotor backLeftMotor = null;
    DcMotor backRightMotor = null;

    Impulse i = new Impulse();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();



        //variable setup
        double buffer = 0.25;            //how far the joystick must move before moving the motors
        String direction = "stop";  //the direction the robot will be heading
        double motorSpeed = 0.25;          //the power the motors will be set to

        //motor setup
        frontLeftMotor = hardwareMap.dcMotor.get("front left");
        frontRightMotor = hardwareMap.dcMotor.get("front right");
        backLeftMotor = hardwareMap.dcMotor.get("back left");
        backRightMotor = hardwareMap.dcMotor.get("back right");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);


        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Direction", "Direction: " + direction);
            telemetry.addData("Motor Speed", "Motor Speed: " + motorSpeed);
            telemetry.update();

            direction = "";


        //move each motor individually
        if (gamepad1.dpad_left) {
            backLeftMotor.setPower(motorSpeed);
            direction = "back left ";
        } else {
            backLeftMotor.setPower(0);
        }

        if (gamepad1.dpad_down) {
            backRightMotor.setPower(motorSpeed);
            direction = "back right ";
        } else {
            backRightMotor.setPower(0);
        }

        if (gamepad1.dpad_up) {
            frontLeftMotor.setPower(motorSpeed);
            direction = "front left ";
        } else {
            frontLeftMotor.setPower(0);
        }

        if (gamepad1.dpad_right) {
            frontRightMotor.setPower(motorSpeed);
            direction = "front right ";
        } else {
            frontRightMotor.setPower(0);
        }


            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }


        //when stopping
    }
}