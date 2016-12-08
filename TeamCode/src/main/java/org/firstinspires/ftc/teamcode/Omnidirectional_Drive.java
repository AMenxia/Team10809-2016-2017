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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "Omnidirectional Drive", group = "Linear Opmode")
// @Autonomous(...) is the other common choice
//@Disabled
public class Omnidirectional_Drive extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor frontLeftMotor = null;
    DcMotor frontRightMotor = null;
    DcMotor backLeftMotor = null;
    DcMotor backRightMotor = null;
    Servo leftFlipper = null;
    Servo rightFlipper = null;
    Servo leftArm = null;

    DcMotor shootMotor = null;

    ColorSensor leftColor;  // Hardware Device Object
    ColorSensor rightColor;
    TouchSensor leftTouch;
    TouchSensor rightTouch;
    OpticalDistanceSensor lineReader;

    Impulse i = new Impulse();

    @Override
    public void runOpMode() throws InterruptedException {

        //variable setup
        double buffer = 0.25;               //how far the joystick must move before moving the motors
        String direction = "stop";          //the direction the robot will be heading
        double motorSpeed = 0.25;           //the power the motors will be set to
        double spinSpeed = 0.25;            //the power the motors will be set to while spinning
        double ticksPerRev = 1120;          //the amount of encoder ticks per revolution of the wheel

        int shootTimer = 0;                 //how long the gun has been shooting for
        double shootSpeed = 1;              //how fast the gun shoots at
        int shootMax = 50;                  //the maximum position of the launcher
        int shootMin = 10;                  //the minimum position of the launcher

        boolean colorSensorLEDOn = true;    //if the color sensor LED is on or not
        boolean buttonPressed1 = false;     //if a,b,x, or y is pressed on gamepad1

        double leftFlipperBack = 0.25;      //position of the left flipper when retracted
        double leftFlipperForward = 0.0;    //position of the left flipper when extended
        boolean leftFlipperOut = false;     //if the left flipper is out or not

        double rightFlipperBack = 0.75;     //position of the right flipper when retracted
        double rightFlipperForward = 1.0;   //position of the right flipper when extended
        boolean rightFlipperOut = false;    //if the right flipper is out or not

        double leftArmUp = 1;
        double leftArmDown = 0;
        boolean leftArmOut = false;

        //motor setup
        frontLeftMotor = hardwareMap.dcMotor.get("front left");
        frontRightMotor = hardwareMap.dcMotor.get("front right");
        backLeftMotor = hardwareMap.dcMotor.get("back left");
        backRightMotor = hardwareMap.dcMotor.get("back right");

        leftFlipper = hardwareMap.servo.get("left flipper");
        rightFlipper = hardwareMap.servo.get("right flipper");

        leftArm = hardwareMap.servo.get("left arm");

        shootMotor = hardwareMap.dcMotor.get("shoot");

        //Crow crow = new Crow(telemetry, frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor, leftFlipper, rightFlipper, shootMotor, colorSensor);
        //Impulse i = new Impulse();
        //i.setCrow(crow);

        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shootMotor.setDirection(DcMotor.Direction.FORWARD);
        shootMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shootMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //sensor setup
        leftColor = hardwareMap.colorSensor.get("left color");
        rightColor = hardwareMap.colorSensor.get("rightColor");
        leftTouch = hardwareMap.touchSensor.get("left touch");
        rightTouch = hardwareMap.touchSensor.get("right touch");
        lineReader = hardwareMap.opticalDistanceSensor.get("ods");

        leftColor.enableLed(colorSensorLEDOn);//turn on the color sensor light when init
        telemetry.addData("Status", "Initialized");//tell that everything is started
        telemetry.update();


        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Direction :", direction);
            telemetry.addData("Motor Speed: ", motorSpeed);
            //telemetry.addData("Spin Speed: ", spinSpeed);
            telemetry.addData("Colors - ", "Red: " + leftColor.red() + "Green: " + leftColor.green() + "Blue: " + leftColor.blue());
            //telemetry.addData("Flippers - ", "Right " + rightFlipperOut + " Left " + leftFlipperOut);
            //telemetry.addData("ColorSensor LED: ", colorSensorLEDOn);
            //telemetry.addData("Touch - ", "left: " + leftTouch + " right: " + rightTouch);
            telemetry.addData("ODS: ", lineReader.getRawLightDetected());
            telemetry.addData("Encoders: ", "FL " + frontLeftMotor.getCurrentPosition() + " FR " + frontRightMotor.getCurrentPosition() + " BL " + backLeftMotor.getCurrentPosition() + " BR " + backRightMotor.getCurrentPosition());
            telemetry.addData("Shoot Motor: ", shootMotor.getCurrentPosition());
            telemetry.update();

            /*
            if (gamepad2.left_stick_y > buffer){
               shootMotor.setPower(shootSpeed);
            } else if(gamepad2.left_stick_y < -buffer){
               shootMotor.setPower(-shootSpeed);
            } else {
               shootMotor.setPower(0);
            }
            */

            if ((gamepad2.right_trigger > 0.25 || gamepad2.left_trigger > 0.25) && shootTimer == 0){
                shootTimer = 1;
            }

            if(shootTimer == 1){
                shootMotor.setPower(shootSpeed);
                if(shootMotor.getCurrentPosition() > shootMax){
                    shootTimer = 2;
                }
            } else if(shootTimer == 2) {
                shootMotor.setPower(-shootSpeed/10);
                if(shootMotor.getCurrentPosition() < shootMin){
                    shootTimer = 0;
                }
            } else {
                shootMotor.setPower(0);
            }



            //controlling the flippers
            if (gamepad2.dpad_left) {
                leftFlipperOut = true;
                leftFlipper.setPosition(leftFlipperForward);
            } else {
                leftFlipperOut = false;
                leftFlipper.setPosition(leftFlipperBack);
            }

            if (gamepad2.dpad_right) {
                rightFlipperOut = true;
                rightFlipper.setPosition(rightFlipperForward);
            } else {
                rightFlipperOut = false;
                rightFlipper.setPosition(rightFlipperBack);
            }

            //controlling the arms
            if(gamepad2.x){
                if(leftArmOut){
                    leftArmOut = false;
                    leftArm.setPosition(leftArmUp);
                } else {
                    leftArmOut = true;
                    leftArm.setPosition(leftArmDown);
                }
            }


            //turning on and off the light on the color sensor
            if (gamepad1.x && !buttonPressed1) {
                buttonPressed1 = true;
                colorSensorLEDOn = !colorSensorLEDOn;
            } else if (!gamepad1.x) {
                buttonPressed1 = false;
            }




            ///------------------Movement code below------------------\\\

            //modify motor speed based off of how far the joystick is being pushed
            // (working) motorSpeed = Math.min(1,(Math.sqrt(Math.max(0, gamepad1.left_stick_x - 0.25) * Math.max(0, gamepad1.left_stick_x - 0.25) + Math.max(0, gamepad1.left_stick_y - 0.25) * Math.max(0, gamepad1.left_stick_y - 0.25)))/0.75);
            motorSpeed = Math.min(1,  ((Math.sqrt(Math.pow(gamepad1.left_stick_x, 2) + Math.pow(gamepad1.left_stick_y, 2)))-0.25)/0.75);
            //(old) motorSpeed = Math.sqrt(gamepad1.left_stick_x*gamepad1.left_stick_x + gamepad1.left_stick_y*gamepad1.left_stick_y) - buffer;
            spinSpeed = Math.max(0, Math.abs(gamepad1.right_stick_x) - buffer);

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

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }

        //this code will be executed when the robot stops
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);


    }


}



