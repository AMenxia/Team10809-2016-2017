package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by alex on 11/9/16.
 */

public class Crow {

    public Telemetry telemetry;
    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backLeftMotor;
    public DcMotor backRightMotor;
    public Servo leftFlipper;
    public Servo rightFlipper;
    public DcMotor shootMotor;
    public ColorSensor colorSensor;

    public Crow(Telemetry telemetry, DcMotor frontLeftMotor, DcMotor frontRightMotor, DcMotor backLeftMotor, DcMotor backRightMotor, Servo leftFlipper, Servo rightFlipper, DcMotor shootMotor, ColorSensor colorSensor) {
        this.telemetry = telemetry;
        this.frontLeftMotor = frontLeftMotor;
        this.frontRightMotor = frontRightMotor;
        this.backLeftMotor = backLeftMotor;
        this.backRightMotor = backRightMotor;
        this.leftFlipper = leftFlipper;
        this.rightFlipper = rightFlipper;
        this.shootMotor = shootMotor;
        this.colorSensor = colorSensor;
    }
}
