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

    DcMotor shootMotor = null;

    ColorSensor colorSensor;  // Hardware Device Object

    Impulse i = new Impulse();

    @Override
    public void runOpMode() throws InterruptedException {

        //variable setup
        double buffer = 0.25;           //how far the joystick must move before moving the motors
        String direction = "stop";      //the direction the robot will be heading
        double motorSpeed = 0.25;       //the power the motors will be set to
        boolean shooting = false;       //if the robot is in the process of shooting
        int shootTimer = 0;             //how long the gun has been shooting for
        double shootSpeed = 0.25;       //how fast the gun shoots at
        int pullBackTime = 100;         //how long the gun pulls back for
        boolean colorSensorLEDOn = true;//if the color sensor LED is on or not
        boolean buttonPressed1 = false; //if a,b,x, or y is pressed on gamepad1

        i.setTelemetry(telemetry);

        //motor setup
        frontLeftMotor = hardwareMap.dcMotor.get("front left");
        frontRightMotor = hardwareMap.dcMotor.get("front right");
        backLeftMotor = hardwareMap.dcMotor.get("back left");
        backRightMotor = hardwareMap.dcMotor.get("back right");

        shootMotor = hardwareMap.dcMotor.get("shoot");


        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        shootMotor.setDirection(DcMotor.Direction.FORWARD);

        //sensor setup
        colorSensor = hardwareMap.colorSensor.get("color sensor");





        colorSensor.enableLed(colorSensorLEDOn);//turn on the color sensor light when init
        telemetry.addData("Status", "Initialized");//tell that everything is started
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Direction :",  direction);
            telemetry.addData( "Motor Speed: ", motorSpeed);
            telemetry.addData("Colors ", "Red: " + colorSensor.red() + "Green: " + colorSensor.green() + "Blue: " + colorSensor.blue());
            telemetry.update();



            //code for shooting the projectile (WIP)
            if(gamepad1.a && !shooting){
                shooting = true;
                shootTimer = 0;
            }

            if(shooting){
                shootTimer++;
                if (shootTimer < pullBackTime){
                    shootMotor.setPower(shootSpeed);
                } else {
                    shootMotor.setPower(0);
                    //release?
                    //idfk
                }

            }


            //turning on and off the light on the color sensor
            if(gamepad1.x && !buttonPressed1){
                buttonPressed1 = true;
                colorSensorLEDOn = !colorSensorLEDOn;
            } else if(!gamepad1.x){
                buttonPressed1 = false;
            }




            

            ///------------------Movement code below------------------\\\

            //modify motor speed based off of how far the joystick is being pushed
            motorSpeed = Math.sqrt(gamepad1.left_stick_x*gamepad1.left_stick_x + gamepad1.left_stick_y*gamepad1.left_stick_y) - 0.25;

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

                    frontLeftMotor.setPower(motorSpeed);
                    frontRightMotor.setPower(-motorSpeed);
                    backLeftMotor.setPower(motorSpeed);
                    backRightMotor.setPower(-motorSpeed);

                    break;

                case "counter clockwise":

                    frontLeftMotor.setPower(-motorSpeed);
                    frontRightMotor.setPower(motorSpeed);
                    backLeftMotor.setPower(-motorSpeed);
                    backRightMotor.setPower(motorSpeed);

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
    }


}



