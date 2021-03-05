package org.firstinspires.ftc.teamcode.Robot.Drivetrain.Odometry;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Hardware
{
    public DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    public DcMotor leftShooter, rightShooter, wobbleCoreMotor, frontIntake;

    public Servo angleServo, wobbleServo, ringServo;
    public CRServo stackServo, uptakeServo;

    public DigitalChannel touchSensor;
    public AnalogInput potentiometer;

    public HardwareMap hardwareMap;

    public void setPowerAll(double lf, double rf, double rb, double lb)
    {
        frontLeftDrive.setPower(lf);
        frontRightDrive.setPower(rf);
        backLeftDrive.setPower(rb);
        backRightDrive.setPower(lb);
    }

    public void init()
    {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");

        leftShooter = hardwareMap.get(DcMotor.class, "leftShooter");
        rightShooter = hardwareMap.get(DcMotor.class, "rightShooter");
        wobbleCoreMotor = hardwareMap.get(DcMotor.class, "wobbleCoreMotor");
        frontIntake = hardwareMap.get(DcMotor.class, "frontIntake");

        angleServo = hardwareMap.get(Servo.class, "angleServo");
        wobbleServo = hardwareMap.get(Servo.class, "wobbleServo");
        ringServo = hardwareMap.get(Servo.class, "ringServo");

        stackServo = hardwareMap.get(CRServo.class, "stackServo");
        uptakeServo = hardwareMap.get(CRServo.class, "uptakeServo");

        touchSensor = hardwareMap.get(DigitalChannel.class, "touchSensor");
        potentiometer = hardwareMap.get(AnalogInput.class, "potentiometer");

        touchSensor.setMode(DigitalChannel.Mode.INPUT);

        rightShooter.setDirection(DcMotor.Direction.REVERSE);
        leftShooter.setDirection(DcMotor.Direction.REVERSE);

        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        setPowerAll(0.0, 0.0, 0.0, 0.0);

        leftShooter.setPower(0.0);
        rightShooter.setPower(0.0);

        wobbleCoreMotor.setPower(0.0);
        frontIntake.setPower(0.0);

        uptakeServo.setPower(0.0);
        stackServo.setPower(0.0);

    }
}