package org.firstinspires.ftc.teamcode.Robot.Drivetrain.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Robot.Drivetrain.Odometry.OdometryGlobalCoordinatePosition;


//changelog:
//v1 - work at school and first draft of strafing function
//v2 - second draft of strafing function and concept of turning
//v3 - revisions to turning function and allowing for backwards/left
//v4 - scratch rewrite with optimizations and ease of use improvements
//
//testing for tomorrow:
//flip necessary negatives
//make sure driving in reverse and left works properly
//make sure robot is pivoting around middle point in  errorCorrectedTurn
//verify the validity of the strafing code at all - may not work at all


@TeleOp(name = "My Odometry OpMode")
public class MyOdometryOpmode_Copy extends LinearOpMode
{
    //Drive motors
    DcMotor right_front, right_back, left_front, left_back;
    //Odometry Wheels
    DcMotor verticalLeft, verticalRight, horizontal;

    DcMotor leftShooter = null, rightShooter = null, wobbleCoreMotor = null;
    CRServo stackServo = null, uptakeServo = null;

    Servo wobbleServo = null, ringServo = null;
    
    DigitalChannel touchSensor = null;
    AnalogInput potentiometer = null;

    double potentiometerReading = 0.0;

    final double COUNTS_PER_INCH = 735.92113;

    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
    String rfName = "frontRightDrive", rbName = "backRightDrive", lfName = "frontLeftDrive", lbName = "backLeftDrive";
    String verticalLeftEncoderName = rfName, verticalRightEncoderName = lfName, horizontalEncoderName = lbName;
    OdometryGlobalCoordinatePosition globalPositionUpdate;
 
    public void errorCorrectedDriveXlr(double power, double distance)
    {
        left_front.setPower(-power);
        right_front.setPower(power);
        left_back.setPower(power);
        right_back.setPower(-power);
 
        sleep(100);
    
        double px = (globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
        double o = globalPositionUpdate.returnOrientation();
 
        double fx, t, dt;
 
        do
        {
            fx = (globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            t = globalPositionUpdate.returnOrientation();
 
            dt = (t - o);
 
            if (dt >= 0.5)
            {
                right_front.setPower(power + 0.2);
                //right_back.setPower(power - 0.1);
            }
 
            if (dt <= -0.5)
            {
                left_front.setPower(-power - 0.2);
                //left_back.setPower(-power + 0.1);
            }
 
            if (dt > -0.5 && dt < 0.5)
            {
                left_front.setPower(-power);
                right_front.setPower(power);
                left_back.setPower(power);
                right_back.setPower(-power);     
            }
        } while (Math.abs((distance + px) - fx) >= 1.0);
 
        left_front.setPower(0.0);
        right_front.setPower(0.0);
        left_back.setPower(0.0);
        right_back.setPower(0.0); 
    }
    
    public void errorCorrectedDriveXfb(double power, double distance)
    {
        left_front.setPower(-power);
        right_front.setPower(-power);
        left_back.setPower(-power);
        right_back.setPower(-power);
 
        sleep(100);
    
        double px = (globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
        double o = globalPositionUpdate.returnOrientation();
    
        double fx, t, dt;
 
        do
        {
            fx = (globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            t = globalPositionUpdate.returnOrientation();
 
            dt = (t - o);
 
            if (dt >= 0.5)
            {
                left_front.setPower(-power + 0.2);
                left_back.setPower(-power + 0.2);
            }
 
            if (dt <= -0.5)
            {
                right_front.setPower(-power + 0.2);
                right_back.setPower(-power + 0.2);
            }
 
            if (dt > -0.5 && dt < 0.5)
            {
                left_front.setPower(-power);
                right_front.setPower(-power);
                left_back.setPower(-power);
                right_back.setPower(-power); 
            }
            

        } while (Math.abs((distance + px) - fx) >= 2.55);
 
        left_front.setPower(0.0);
        right_front.setPower(0.0);
        left_back.setPower(0.0);
        right_back.setPower(0.0); 
    }
 
    public void errorCorrectedDriveYfb(double power, double distance)
    {
        left_front.setPower(-power);
        right_front.setPower(-power);
        left_back.setPower(-power);
        right_back.setPower(-power);
 
        sleep(100);
    
        double py = (globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
        double o = globalPositionUpdate.returnOrientation();
    
        double fy, t, dt;
 
        do
        {
            fy = (globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            t = globalPositionUpdate.returnOrientation();
 
            dt = (t - o);
 
            if (dt >= 0.5)
            {
                left_front.setPower(-power + 0.2);
                left_back.setPower(-power + 0.2);
            }
 
            if (dt <= -0.5)
            {
                right_front.setPower(-power + 0.2);
                right_back.setPower(-power + 0.2);
            }
 
            if (dt > -0.5 && dt < 0.5)
            {
                left_front.setPower(-power);
                right_front.setPower(-power);
                left_back.setPower(-power);
                right_back.setPower(-power); 
            }
            

        } while (Math.abs((distance + py) - fy) >= 2.55);
 
        left_front.setPower(0.0);
        right_front.setPower(0.0);
        left_back.setPower(0.0);
        right_back.setPower(0.0); 
    }
    
    public void errorCorrectedDriveYlr(double power, double distance)
    {
        left_front.setPower(-power);
        right_front.setPower(power);
        left_back.setPower(power);
        right_back.setPower(-power);
 
        sleep(100);
    
        double py = (globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
        double o = globalPositionUpdate.returnOrientation();
 
        double fy, t, dt;
 
        do
        {
            fy = (globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            t = globalPositionUpdate.returnOrientation();
 
            dt = (t - o);
 
            if (dt >= 0.5)
            {
                right_front.setPower(power + 0.2);
                //right_back.setPower(power - 0.1);
            }
 
            if (dt <= -0.5)
            {
                left_front.setPower(-power - 0.2);
                //left_back.setPower(-power + 0.1);
            }
 
            if (dt > -0.5 && dt < 0.5)
            {
                left_front.setPower(-power);
                right_front.setPower(power);
                left_back.setPower(power);
                right_back.setPower(-power);     
            }
        } while (Math.abs((distance + py) - fy) >= 1.0);
 
        left_front.setPower(0.0);
        right_front.setPower(0.0);
        left_back.setPower(0.0);
        right_back.setPower(0.0); 
    }
    
 
    public void errorCorrectedTurn(double power, double angle)
    {
        if (angle == 0.0) return;
 
        double o = globalPositionUpdate.returnOrientation();
    
        double t;
 
        do
        {
            t = globalPositionUpdate.returnOrientation();
            
            left_front.setPower(-power);
            left_back.setPower(-power);
 
            right_front.setPower(power);
            right_back.setPower(power);
            
        } while (Math.abs((angle + o) - t) >= 5.0);
        
        left_front.setPower(0.0);
        right_front.setPower(0.0);
        left_back.setPower(0.0);
        right_back.setPower(0.0);
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        leftShooter = hardwareMap.get(DcMotor.class, "leftShooter");
        rightShooter = hardwareMap.get(DcMotor.class, "rightShooter");
        
        stackServo = hardwareMap.get(CRServo.class, "stackServo");
        uptakeServo = hardwareMap.get(CRServo.class, "uptakeServo");

        wobbleServo = hardwareMap.get(Servo.class, "wobbleServo");
        ringServo = hardwareMap.get(Servo.class, "ringServo");
    
        touchSensor = hardwareMap.get(DigitalChannel.class, "touchSensor");
        potentiometer = hardwareMap.get(AnalogInput.class, "potentiometer");
        
        touchSensor.setMode(DigitalChannel.Mode.INPUT);
        
        wobbleCoreMotor = hardwareMap.get(DcMotor.class, "wobbleCoreMotor");
        
        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        initDriveHardwareMap(rfName, rbName, lfName, lbName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);

        //right_front.setDirection(DcMotor.Direction.REVERSE);
        //right_back.setDirection(DcMotor.Direction.REVERSE);
        //left_front.setDirection(DcMotor.Direction.REVERSE);
        //left_back.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseRightEncoder();
        globalPositionUpdate.reverseNormalEncoder();
        
        //code for the autonomous
        ringServo.setPosition(1.0);
        wobbleServo.setPosition(1.0);
        
        errorCorrectedDriveYfb(0.65, 60.0);
        errorCorrectedTurn(0.65, 90);
        errorCorrectedDriveXfb(0.65, 19.5);
        sleep(100);
        errorCorrectedTurn(-0.65, -86.0);
        
        //bring rings up
        stackServo.setPower(1.0);
        uptakeServo.setPower(1.0);
        leftShooter.setPower(1.0);
        rightShooter.setPower(-1.0);
        sleep(10000);
      
        //turn off shooter and stacker
        leftShooter.setPower(0.0);
        rightShooter.setPower(0.0);
        stackServo.setPower(0.0);
        uptakeServo.setPower(0.0);

        errorCorrectedTurn(-0.65, -179.0);
        errorCorrectedDriveYfb(0.65, -6.5);
        
        ringServo.setPosition(0.0);
        sleep(1500);
      
        if (touchSensor.getState() == false)
        {
            potentiometerReading = potentiometer.getVoltage();
            telemetry.addData("Potentiometer: ", potentiometerReading); 
        }
      
        //bring down ring touch
        //get poteniometer data
        sleep(500);
        ringServo.setPosition(1.0);
        
      
        if (potentiometerReading <= 2.1999)
        {
            errorCorrectedDriveYfb(-0.65, 65.0);
        
            //bring down wobble
            wobbleCoreMotor.setPower(-0.5);
            sleep(1300);
            wobbleCoreMotor.setPower(0.0);
        
            //release wobble
            wobbleServo.setPosition(0);
            sleep(500);
        
            errorCorrectedDriveYfb(0.65, -40.0);
      }
      
      else if (potentiometerReading >= 2.2 && potentiometerReading <= 2.4999)
      { 
        // 1 ring
        //bring ring servo up
        ringServo.setPosition(1);
        
        //drive forward
        //backLeftDrive.setPower(0.7); 
        //backRightDrive.setPower(0.7); 
        //frontRightDrive.setPower(0.7); 
        //frontLeftDrive.setPower(0.7); 
        sleep(1000);
        
        //stop drive
        //backLeftDrive.setPower(0.0);
        //backRightDrive.setPower(0.0);
        //frontRightDrive.setPower(0.0);
        //frontLeftDrive.setPower(0.0);
        sleep(500);
        
        //turn left 
        //backLeftDrive.setPower(-0.7);
        //backRightDrive.setPower(0.7);
       // frontRightDrive.setPower(0.0);
        //frontLeftDrive.setPower(0.0);
        sleep(1500);
        
        //bring down wobble
        sleep(1000);
        
        //release wobble 
        wobbleCoreMotor.setPower(-0.5);
        sleep(1250);
        wobbleCoreMotor.setPower(0.0);
        
        //turn right
        //backLeftDrive.setPower(-0.7);
        //backRightDrive.setPower(0.7);
        //frontRightDrive.setPower(0.0);
        //frontLeftDrive.setPower(0.0);
        sleep(1500);
        
        //park
        //backLeftDrive.setPower(-0.7);
        //backRightDrive.setPower(-0.7);
        //frontRightDrive.setPower(-0.7);
        //frontLeftDrive.setPower(-0.7);
        sleep(750);
      }
      
      else if (potentiometerReading >= 2.5)
      { 
        // 0 rings
        // bring up ring servo
        ringServo.setPosition(1);
        
        // drive forward
        //backLeftDrive.setPower(0.7); 
        //backRightDrive.setPower(0.7); 
        //frontRightDrive.setPower(0.7); 
        //frontLeftDrive.setPower(0.7); 
        sleep(750);
        
        //stop drive
        //backLeftDrive.setPower(0.0);
        //backRightDrive.setPower(0.0);
        //frontRightDrive.setPower(0.0);
        //frontLeftDrive.setPower(0.0);
        sleep(500);
        
        //bring down wobble
        wobbleCoreMotor.setPower(-0.5);
        sleep(1250);
        wobbleCoreMotor.setPower(0.0);
        
        //release wobble
        wobbleServo.setPosition(0);
        sleep(500);
        
        //park
        //backLeftDrive.setPower(-0.7);
        //backRightDrive.setPower(-0.7);
        //frontRightDrive.setPower(-0.7);
        //frontLeftDrive.setPower(-0.7);
        sleep(250);
      }

        while(opModeIsActive()){
            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());
            telemetry.addData("Change In Left:", globalPositionUpdate.globalLeftChange());
            telemetry.addData("Change In Right:", globalPositionUpdate.globalRightChange());
            telemetry.addData("Change in Orientation", globalPositionUpdate.globalChangeInOrientation());
            telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
            telemetry.addData("horizontal encoder position", horizontal.getCurrentPosition());

            telemetry.addData("Thread Active", positionThread.isAlive());
            int direction = 0;
            
            if ((globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH)<= 14 && (globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH)>= -14 ) 
            {
                setPowerAll(-0.5*direction,-0.5*direction,-0.5*direction,-0.5*direction); 
            }
            else
            {
                setPowerAll(0,0,0,0);
            }
            
            telemetry.update();
        }

        //Stop the thread
        globalPositionUpdate.stop();

    }

    private void initDriveHardwareMap(String rfName, String rbName, String lfName, String lbName, String vlEncoderName, String vrEncoderName, String hEncoderName){
        right_front = hardwareMap.dcMotor.get(rfName);
        right_back = hardwareMap.dcMotor.get(rbName);
        left_front = hardwareMap.dcMotor.get(lfName);
        left_back = hardwareMap.dcMotor.get(lbName);

        verticalLeft = hardwareMap.dcMotor.get(vlEncoderName);
        verticalRight = hardwareMap.dcMotor.get(vrEncoderName);
        horizontal = hardwareMap.dcMotor.get(hEncoderName);

        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_front.setDirection(DcMotorSimple.Direction.REVERSE);
      //  right_front.setDirection(DcMotorSimple.Direction.REVERSE);
        //right_back.setDirection(DcMotorSimple.Direction.REVERSE);
        left_back.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();
    }

    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }
    private void setPowerAll(double rf, double rb, double lf, double lb){
        right_front.setPower(rf);
        right_back.setPower(rb);
        left_front.setPower(lf);
        left_back.setPower(lb);
    }
}
