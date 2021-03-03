package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.Device;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
@TeleOp(name="Mecanum Drive", group="TeleOP")

public class MecanumDrive extends OpMode
{
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
   
    private DcMotor leftShooter = null;
    private DcMotor rightShooter = null;
    
    private DcMotor wobbleCoreMotor = null;
    private DcMotor frontIntake = null;
    
    private CRServo stackServo = null;
    private CRServo uptakeServo = null;
    
    private Servo angleServo = null;
    private Servo wobbleServo = null;
    private Servo ringServo = null;
   
    private AnalogInput potentiometer = null;
    private DigitalChannel touchSensor = null;
   
    private boolean uptakeOn = false;
    
    @Override
    public void init()
    {
        telemetry.addData("Status: ", "Initialized");
        
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");
        
        leftShooter = hardwareMap.get(DcMotor.class, "leftShooter");
        rightShooter = hardwareMap.get(DcMotor.class, "rightShooter");
        
        wobbleCoreMotor = hardwareMap.get(DcMotor.class, "wobbleCoreMotor");
        frontIntake = hardwareMap.get(DcMotor.class, "frontIntake");
        
        uptakeServo = hardwareMap.get(CRServo.class, "uptakeServo");
        stackServo = hardwareMap.get(CRServo.class, "stackServo");
        
        wobbleServo = hardwareMap.get(Servo.class, "wobbleServo");
        angleServo = hardwareMap.get(Servo.class, "angleServo");
        ringServo = hardwareMap.get(Servo.class, "ringServo");
        
        potentiometer = hardwareMap.get(AnalogInput.class, "potentiometer");
        //touchSensor = hardwareMap.touchSensor.get("touchSensor");
        touchSensor = hardwareMap.get(DigitalChannel.class, "touchSensor");
    
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        
        rightShooter.setDirection(DcMotor.Direction.REVERSE);
        
        frontLeftDrive.setPower(0.0);
        frontRightDrive.setPower(0.0);
        backLeftDrive.setPower(0.0);
        

     
        touchSensor.setMode(DigitalChannel.Mode.INPUT);
        
        telemetry.addData("Status", "Initialized");
    }
    
    @Override
    public void init_loop()
    {
    
    }

    @Override
    public void start()
    {
       // ringServo.setPosition(0.6);
    }
    
    @Override
    public void loop()
    {
        double magnitude = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        
        final double fld = magnitude * Math.cos(robotAngle) + rightX;
        final double frd = magnitude * Math.sin(robotAngle) - rightX;
        final double bld = magnitude * Math.sin(robotAngle) + rightX;
        final double brd = magnitude * Math.cos(robotAngle) - rightX;
        
        frontLeftDrive.setPower(fld/0.707);
        frontRightDrive.setPower(frd/0.707);
        backLeftDrive.setPower(bld/0.707);
        backRightDrive.setPower(brd/0.707);
        
        leftShooter.setPower(gamepad2.right_trigger*0.8);
        rightShooter.setPower(gamepad2.right_trigger*0.8);
        
        leftShooter.setPower(gamepad2.left_trigger*0.8);
        rightShooter.setPower(gamepad2.left_trigger*0.8);

        
        stackServo.setPower(gamepad2.left_stick_y);
        //angleServo.setPosition(gamepad2.left_stick_x);
        
        wobbleCoreMotor.setPower(-gamepad2.right_stick_y);
        frontIntake.setPower(gamepad1.left_trigger);
        frontIntake.setPower(-gamepad1.right_trigger);
        
        if (gamepad1.dpad_left) { wobbleServo.setPosition(0); }
        if (gamepad1.dpad_right) { wobbleServo.setPosition(1); }
        
        if (uptakeOn) { uptakeServo.setPower(1.0); }
        else { uptakeServo.setPower(0.0); }
        
        if (gamepad2.a) { uptakeOn = !uptakeOn; }
        
        if (gamepad2.y) { ringServo.setPosition(1.0); }   
        if (gamepad2.x) { ringServo.setPosition(0.5); }

        //if (gamepad2.dpad_left) { angleServo.setPosition(0.0); }
        //if (gamepad2.dpad_right) { angleServo.setPosition(0.5); }
        //if (gamepad2.dpad_up) { angleServo.setPosition(1.0); }
        
        if (touchSensor.getState() == false)
        {
            telemetry.addData("Poten = ", potentiometer.getVoltage()); 
            telemetry.update(); 
        }
    }
    
    @Override
    public void stop()
    {
    
    }

}