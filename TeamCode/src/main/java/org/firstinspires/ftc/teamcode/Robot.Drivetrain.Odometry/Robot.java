package org.firstinspires.ftc.teamcode.Robot.Drivetrain.Odometry;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Robot
{
    public DcMotor frontLeftDrive = null, frontRightDrive = null, backLeftDrive = null, backRightDrive = null;
    public DcMotor leftShooter = null, rightShooter = null, wobbleCoreMotor = null, frontIntake = null;

    public Servo angleServo = null, wobbleServo = null, ringServo = null;
    public CRServo stackServo = null, uptakeServo = null;

    public DigitalChannel touchSensor = null;
    public AnalogInput potentiometer = null;

    public HardwareMap hardwareMap = null;
    public ElapsedTime elapsedTime = null;

    public boolean uptakeOn = false;

    public final double COUNTS_PER_INCH = 735.92113;
    public final double DRIVE_SPEED = 0.65;

    public final double SERVO_LOCK = 0.5;

    public final double RING_UP = 1.0;
    public final double RING_DOWN = 0.0;

    public final double WOBBLE_CLOSED = 0.0;
    public final double WOBBLE_OPEN = 1.0;

    public final double MAX_POWER = 1.0;
    public final double MIN_POWER = 0.0;

    public final double SHOOTER_POWER = 0.8;

    public boolean debounceOK()
    {
        return (elapsedTime.milliseconds() > 100);
    }

    public void setPowerAll(double lf, double rf, double rb, double lb)
    {
        frontLeftDrive.setPower(lf);
        frontRightDrive.setPower(rf);
        backLeftDrive.setPower(rb);
        backRightDrive.setPower(lb);
    }

    public void init(HardwareMap hwMap)
    {
        hardwareMap = hwMap;

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

        setPowerAll(MIN_POWER, MIN_POWER, MIN_POWER, MIN_POWER);

        leftShooter.setPower(0.0);
        rightShooter.setPower(0.0);

        wobbleCoreMotor.setPower(0.0);
        frontIntake.setPower(0.0);

        uptakeServo.setPower(0.0);
        stackServo.setPower(0.0);
    }

    public Robot(HardwareMap hardwareMap)
    {
        init(hardwareMap);
    }
}