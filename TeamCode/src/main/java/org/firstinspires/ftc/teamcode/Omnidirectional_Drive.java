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


import android.media.MediaPlayer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "Omnidirectional Drive", group = "Linear Opmode")
// @Autonomous(...) is the other common choice
//@Disabled
public class Omnidirectional_Drive extends LinearOpMode {

    MediaPlayer crow1;
    MediaPlayer crow2;
    MediaPlayer crow3;

    DcMotor frontLeftMotor = null;
    DcMotor frontRightMotor = null;
    DcMotor backLeftMotor = null;
    DcMotor backRightMotor = null;
    Servo leftFlipper = null;
    Servo rightFlipper = null;
    Servo loader = null;
    DcMotor shootMotor = null;
    ColorSensor leftColor;  // Hardware Device Object
    ColorSensor rightColor;
    TouchSensor leftTouch;
    TouchSensor rightTouch;
    OpticalDistanceSensor lineReader;
    Impulse i = new Impulse();
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        crow1 = MediaPlayer.create(hardwareMap.appContext, R.raw.crow1);
        crow2 = MediaPlayer.create(hardwareMap.appContext, R.raw.crow2);
        crow3 = MediaPlayer.create(hardwareMap.appContext, R.raw.crow3);
        double crowRandomnessMultiplier = 1000000;

        //variable setup
        double buffer = 0.25;               //how far the joystick must move before moving the motors
        String direction = "stop";          //the direction the robot will be heading
        double motorSpeed = 0.25;           //the power the motors will be set to
        int maxSpeed = 2800;
        double spinSpeed = 0.25;            //the power the motors will be set to while spinning
        double ticksPerRev = 1120;          //the amount of encoder ticks per revolution of the wheel

        int shootTimer = 0;                 //how long the gun has been shooting for
        double shootSpeed = 1;              //how fast the gun shoots at
        int shootMax = 600;
        int shootMin = 0;
        int loadTimer = 0;
        int loadTime = 2000;

        boolean colorSensorLEDOn = false;    //if the color sensor LED is on or not
        boolean y2Pressed = false;           //if y has been pressed on gamepad 2

        double leftFlipperBack = 0.25;      //position of the left flipper when retracted
        double leftFlipperForward = 0.0;    //position of the left flipper when extended
        boolean leftFlipperOut = false;     //if the left flipper is out or not

        double rightFlipperBack = 0.75;     //position of the right flipper when retracted
        double rightFlipperForward = 1.0;   //position of the right flipper when extended
        boolean rightFlipperOut = false;    //if the right flipper is out or not

        double loaderDown = 0.5;            //position of the loader when retracted
        double loaderUp = 0.0;              //position of the loader when hitting the ball
        boolean loaderOut = false;          //if the loader is out or not

        //motor setup
        frontLeftMotor = hardwareMap.dcMotor.get("front left");
        frontRightMotor = hardwareMap.dcMotor.get("front right");
        backLeftMotor = hardwareMap.dcMotor.get("back left");
        backRightMotor = hardwareMap.dcMotor.get("back right");

        leftFlipper = hardwareMap.servo.get("left flipper");
        rightFlipper = hardwareMap.servo.get("right flipper");
        loader = hardwareMap.servo.get("loader");

        shootMotor = hardwareMap.dcMotor.get("shoot");

        shootMotor.setDirection(DcMotor.Direction.REVERSE);
        shootMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shootMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Crow crow = new Crow(telemetry, frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor, leftFlipper, rightFlipper, shootMotor, leftColor, rightColor);
        //i.setCrow(crow);

        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftMotor.setMaxSpeed(maxSpeed);
        frontRightMotor.setMaxSpeed(maxSpeed);
        backLeftMotor.setMaxSpeed(maxSpeed);
        backRightMotor.setMaxSpeed(maxSpeed);

        //sensor setup
        leftColor = hardwareMap.colorSensor.get("left color");
        rightColor = hardwareMap.colorSensor.get("right color");
        rightColor.setI2cAddress(I2cAddr.create8bit(0x4c)); //this makes it so that we can have 2 color sensors
        leftTouch = hardwareMap.touchSensor.get("left touch");
        rightTouch = hardwareMap.touchSensor.get("right touch");
        lineReader = hardwareMap.opticalDistanceSensor.get("ods");

        leftColor.enableLed(colorSensorLEDOn);  //turn on the left color sensor light when init
        rightColor.enableLed(colorSensorLEDOn); //turn off the right color sensor light when init
        telemetry.addData("Status", "Initialized");//tell that everything is started
        telemetry.update();

        //intialize servo positions
        leftFlipper.setPosition(leftFlipperBack);
        rightFlipper.setPosition(rightFlipperBack);
        loader.setPosition(loaderDown);

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Direction :", direction);
            telemetry.addData("Motor Speed: ", motorSpeed);
            telemetry.addData("Spin Speed: ", spinSpeed);
            telemetry.addData("L color - ", "Red: " + leftColor.red() + "Green: " + leftColor.green() + "Blue: " + leftColor.blue());
            telemetry.addData("R color - ", "Red: " + rightColor.red() + "Green: " + rightColor.green() + "Blue: " + rightColor.blue());
            telemetry.addData("Flippers - ", "Right " + rightFlipperOut + " Left " + leftFlipperOut);
            telemetry.addData("ColorSensor LED: ", colorSensorLEDOn);
            telemetry.addData("Touch - ", "left: " + leftTouch.isPressed() + " right: " + rightTouch.isPressed());
            telemetry.addData("ODS: ", lineReader.getRawLightDetected());
            telemetry.addData("Encoders: ", "FL " + frontLeftMotor.getCurrentPosition() + " FR " + frontRightMotor.getCurrentPosition() + " BL " + backLeftMotor.getCurrentPosition() + " BR " + backRightMotor.getCurrentPosition());
            telemetry.addData("Shooting ", "Timer: " + shootTimer + " Pos: " + shootMotor.getCurrentPosition());
            telemetry.addData("Triggers: ", "L2: " + gamepad2.left_trigger + " R2: " + gamepad2.right_trigger);
            telemetry.update();

            if(Math.rint(Math.random()*crowRandomnessMultiplier) == 1){
                crow1.start();
            } else if(Math.rint(Math.random()*crowRandomnessMultiplier) == 2){
                crow2.start();
            } else if(Math.rint(Math.random()*crowRandomnessMultiplier) == 3){
                crow3.start();
            }

            //auto shooting
            /*
            if (gamepad2.x && shootTimer == 0 && false) {
                shootTimer = 1;
            } else if (shootTimer == 1) {
                shootMotor.setPower(shootSpeed);
                if (shootMotor.getCurrentPosition() > shootMax) {
                    shootTimer = 2;
                }
            } else if (shootTimer == 2) {
                shootMotor.setPower(-shootSpeed * 0.25);
                if (shootMotor.getCurrentPosition() < shootMin) {
                    shootTimer = 3;
                }
            } else if (shootTimer == 3) {
                loaderOut = true;
                shootMotor.setPower(0);
                if (loadTimer > loadTime) {
                    shootTimer = 4;
                } else {
                    loadTimer++;
                }
            } else if (shootTimer == 4) {
                shootMotor.setPower(shootSpeed);
                if (shootMotor.getCurrentPosition() > shootMax) {
                    shootTimer = 5;
                }
            } else if (shootTimer == 5) {
                shootMotor.setPower(-shootSpeed * 0.25);
                if (shootMotor.getCurrentPosition() < shootMin) {
                    shootTimer = 6;
                }
            } else {
                shootTimer = 0;
                loadTimer = 0;
                loaderOut = false;
            }
            */

            //shooting manually
            if (shootTimer == 0) {
                if (gamepad2.right_trigger > 0.25) {
                    shootMotor.setPower(shootSpeed);
                } else if (gamepad2.left_trigger > 0.25) {
                    shootMotor.setPower(-shootSpeed / 10);
                } else {
                    shootMotor.setPower(0);
                }
            }

            //controlling the flippers
            if(gamepad1.dpad_up || gamepad2.dpad_up){
                leftFlipperOut = true;
                rightFlipperOut = true;
            } else {
                leftFlipperOut = gamepad1.dpad_left || gamepad2.dpad_left;
                rightFlipperOut = gamepad1.dpad_right || gamepad2.dpad_right;
            }



            if (leftFlipperOut) {//sets the position of the flippers
                leftFlipper.setPosition(leftFlipperForward);
            } else {
                leftFlipper.setPosition(leftFlipperBack);
            }

            if (rightFlipperOut) {
                rightFlipper.setPosition(rightFlipperForward);
            } else {
                rightFlipper.setPosition(rightFlipperBack);
            }

            //controlling the loader
            if (shootTimer == 0) {
                loaderOut = gamepad2.a; //maps loader position to A on gamepad2
            }
            if (loaderOut) {
                loader.setPosition(loaderUp);
            } else {
                loader.setPosition(loaderDown);
            }


            //turning on and off the light on the color sensor
            if (gamepad2.y && !y2Pressed) {
                y2Pressed = true;
                colorSensorLEDOn = !colorSensorLEDOn;
            } else if (!gamepad2.y) {
                y2Pressed = false;
            }

            leftColor.enableLed(colorSensorLEDOn);
            rightColor.enableLed(colorSensorLEDOn);

            ///------------------Movement code below------------------\\\

            //modify motor speed based off of how far the joystick is being pushed
            motorSpeed = Math.max(0, Math.min(1, ((Math.sqrt(Math.pow(gamepad1.left_stick_x, 2) + Math.pow(gamepad1.left_stick_y, 2))) - 0.25) / 0.75));
            spinSpeed = Math.max(0, Math.min(1, (Math.abs(gamepad1.right_stick_x) - buffer) / 0.75));

            //set movement direction based off of stick
            if (gamepad1.left_stick_x < -buffer) { //buffer is how far the joystick needs to go before the robot starts moving
                if (gamepad1.left_stick_y < -buffer) {
                    direction = "left forwards";
                } else if (gamepad1.left_stick_y > buffer) {
                    direction = "left backwards";
                } else {
                    direction = "left";
                }


            } else if (gamepad1.left_stick_x > buffer) {
                if (gamepad1.left_stick_y < -buffer) {
                    direction = "right forwards";
                } else if (gamepad1.left_stick_y > buffer) {
                    direction = "right backwards";
                } else {
                    direction = "right";
                }

            } else {
                if (gamepad1.left_stick_y < -buffer) {
                    direction = "forwards";
                } else if (gamepad1.left_stick_y > buffer) {
                    direction = "backwards";
                } else {
                    direction = "stop";
                }

            }

            //making the robot spin
            if (gamepad1.right_stick_x > buffer) {
                direction = "clockwise";
            } else if (gamepad1.right_stick_x < -buffer) {
                direction = "counter clockwise";
            }


            switch (direction) {//set the motors at different speeds based off of the directions
                case "left forwards":

                    frontLeftMotor.setPower(0);
                    frontRightMotor.setPower(motorSpeed);
                    backLeftMotor.setPower(motorSpeed);
                    backRightMotor.setPower(0);

                    break;

                case "left backwards":

                    frontLeftMotor.setPower(-motorSpeed);
                    frontRightMotor.setPower(0);
                    backLeftMotor.setPower(0);
                    backRightMotor.setPower(-motorSpeed);

                    break;

                case "left":

                    frontLeftMotor.setPower(-motorSpeed);
                    frontRightMotor.setPower(motorSpeed);
                    backLeftMotor.setPower(motorSpeed);
                    backRightMotor.setPower(-motorSpeed);

                    break;

                case "right forwards":

                    frontLeftMotor.setPower(motorSpeed);
                    frontRightMotor.setPower(0);
                    backLeftMotor.setPower(0);
                    backRightMotor.setPower(motorSpeed);

                    break;

                case "right backwards":

                    frontLeftMotor.setPower(0);
                    frontRightMotor.setPower(-motorSpeed);
                    backLeftMotor.setPower(-motorSpeed);
                    backRightMotor.setPower(0);

                    break;

                case "right":

                    frontLeftMotor.setPower(motorSpeed);
                    frontRightMotor.setPower(-motorSpeed);
                    backLeftMotor.setPower(-motorSpeed);
                    backRightMotor.setPower(motorSpeed);

                    break;

                case "forwards":

                    frontLeftMotor.setPower(motorSpeed);
                    frontRightMotor.setPower(motorSpeed);
                    backLeftMotor.setPower(motorSpeed);
                    backRightMotor.setPower(motorSpeed);

                    break;

                case "backwards":

                    frontLeftMotor.setPower(-motorSpeed);
                    frontRightMotor.setPower(-motorSpeed);
                    backLeftMotor.setPower(-motorSpeed);
                    backRightMotor.setPower(-motorSpeed);

                    break;

                case "stop":

                    frontLeftMotor.setPower(0);
                    frontRightMotor.setPower(0);
                    backLeftMotor.setPower(0);
                    backRightMotor.setPower(0);

                    break;

                case "clockwise":

                    frontLeftMotor.setPower(spinSpeed);
                    frontRightMotor.setPower(-spinSpeed);
                    backLeftMotor.setPower(spinSpeed);
                    backRightMotor.setPower(-spinSpeed);

                    break;

                case "counter clockwise":

                    frontLeftMotor.setPower(-spinSpeed);
                    frontRightMotor.setPower(spinSpeed);
                    backLeftMotor.setPower(-spinSpeed);
                    backRightMotor.setPower(spinSpeed);

                    break;

                default:

                    frontLeftMotor.setPower(0);
                    frontRightMotor.setPower(0);
                    backLeftMotor.setPower(0);
                    backRightMotor.setPower(0);

                    break;

            }


            ///---------------End of Movement code--------------\\\
            //sleep(100);
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }

        //this code will be executed when the robot stops
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);


    }




}



