package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Motors (Blocks to Java)", group = "motors")
public class Motors extends LinearOpMode
{
  private DcMotor backLeftDrive = null, backRightDrive = null, frontRightDrive = null, frontLeftDrive = null;
  
  private DcMotor leftShooter = null, rightShooter = null;
  
  private DcMotor wobbleCoreMotor = null, frontIntake = null;
  
  private CRServo uptakeServo = null, stackServo = null;
  
  private Servo ringServo = null, wobbleServo = null;

  private AnalogInput potentiometer = null;
  private DigitalChannel touchSensor = null;
  
  private double potentiometerReading = 0.0;
  
  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode()
  {
    backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
    backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");
    frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
    frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
    
    leftShooter = hardwareMap.get(DcMotor.class, "leftShooter");
    rightShooter = hardwareMap.get(DcMotor.class, "rightShooter");
    
    wobbleCoreMotor = hardwareMap.get(DcMotor.class, "wobbleCoreMotor");
    frontIntake = hardwareMap.get(DcMotor.class, "frontIntake");
    
    uptakeServo = hardwareMap.get(CRServo.class, "uptakeServo");
    stackServo = hardwareMap.get(CRServo.class, "stackServo");
    ringServo = hardwareMap.get(Servo.class, "ringServo");
    wobbleServo = hardwareMap.get(Servo.class, "wobbleServo");
    
    potentiometer = hardwareMap.get(AnalogInput.class, "potentiometer");
    touchSensor = hardwareMap.get(DigitalChannel.class, "touchSensor");
    
    backLeftDrive.setPower(0.0);
    backRightDrive.setPower(0.0);
    frontLeftDrive.setPower(0.0);
    frontRightDrive.setPower(0.0);
    
    leftShooter.setPower(0.0);
    rightShooter.setPower(0.0);
    
    wobbleCoreMotor.setPower(0.0);
    frontIntake.setPower(0.0);
    
    uptakeServo.setPower(0.0);
    stackServo.setPower(0.0);
    
    touchSensor.setMode(DigitalChannel.Mode.INPUT);
    
    rightShooter.setDirection(DcMotor.Direction.REVERSE);
    leftShooter.setDirection(DcMotor.Direction.REVERSE);
    backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
    frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

    waitForStart();
    
    if (opModeIsActive())
    {
      wobbleServo.setPosition(1);
      ringServo.setPosition(1);
      backLeftDrive.setPower(-0.7);
      backRightDrive.setPower(-0.7);
      frontRightDrive.setPower(-0.7);
      frontLeftDrive.setPower(-0.7);
      sleep(1900);
      
      //drive forward
      backLeftDrive.setPower(0.0);
      backRightDrive.setPower(0.0);
      frontRightDrive.setPower(0.0);
      frontLeftDrive.setPower(0.0);
      sleep(500);
      
      //stopdrive
      backLeftDrive.setPower(-0.7);
      backRightDrive.setPower(0.7);
      frontRightDrive.setPower(0.0);
      frontLeftDrive.setPower(0.0);
      sleep(1500);
      
      //turnleft
      backLeftDrive.setPower(-1.0);
      backRightDrive.setPower(-1.0);
      frontRightDrive.setPower(-1.0);
      frontLeftDrive.setPower(-1.0);
      sleep(550);
      
      //driveforward
      backLeftDrive.setPower(0.7);
      backRightDrive.setPower(-0.7);
      frontRightDrive.setPower(0.0);
      frontLeftDrive.setPower(0.0);
      sleep(1400);
      
      //turnright
      backLeftDrive.setPower(0.0);
      backRightDrive.setPower(0.0);
      frontRightDrive.setPower(0.0);
      frontLeftDrive.setPower(0.0);
      sleep(500);
      
      //stopdrive
      stackServo.setPower(1.0);
      uptakeServo.setPower(1.0);
      sleep(1000);
      
      //bring rings up
      leftShooter.setPower(-1.0);
      rightShooter.setPower(1.0);
      sleep(5000);
      
      //turn on shooter
      leftShooter.setPower(0.0);
      rightShooter.setPower(0.0);
      stackServo.setPower(0.0);
      uptakeServo.setPower(0.0);
      
      //drive forward
      backLeftDrive.setPower(-0.7);
      backRightDrive.setPower(-0.7);
      frontRightDrive.setPower(-0.7);
      frontLeftDrive.setPower(-0.7);
      sleep(225);
      
      //stop shooting
      backLeftDrive.setPower(0.7);
      backRightDrive.setPower(-0.7);
      frontRightDrive.setPower(0.0);
      frontLeftDrive.setPower(0.0);
      sleep(3250);
      
      //turnaround
      backLeftDrive.setPower(0.0);
      backRightDrive.setPower(0.0);
      frontRightDrive.setPower(0.0);
      frontLeftDrive.setPower(0.0);
      sleep(500);
      
      //stopdrive
      ringServo.setPosition(0);
      sleep(1500);
      
      if (touchSensor.getState() == false)
      {
        potentiometerReading = potentiometer.getVoltage();
        telemetry.addData("Potentiometer: ", potentiometerReading); 
      }
      
      //bring down ring touch
      //get poteniometer data
      sleep(500);
      
      if (potentiometerReading <= 2.1999)
      {
        // 4 rings
        // bring up ring servo
        ringServo.setPosition(1);
        
        // drive forward
        backLeftDrive.setPower(0.7); 
        backRightDrive.setPower(0.7); 
        frontRightDrive.setPower(0.7); 
        frontLeftDrive.setPower(0.7); 
        sleep(1750);
        
        //stop drive
        backLeftDrive.setPower(0.0);
        backRightDrive.setPower(0.0);
        frontRightDrive.setPower(0.0);
        frontLeftDrive.setPower(0.0);
        sleep(500);
        
        //bring down wobble
        wobbleCoreMotor.setPower(-0.5);
        sleep(1250);
        wobbleCoreMotor.setPower(0.0);
        
        //release wobble
        wobbleServo.setPosition(0);
        sleep(500);
        
        //park
        backLeftDrive.setPower(-0.7);
        backRightDrive.setPower(-0.7);
        frontRightDrive.setPower(-0.7);
        frontLeftDrive.setPower(-0.7);
        sleep(1500);
      }
      
      else if (potentiometerReading >= 2.2 && potentiometerReading <= 2.4999)
      { 
        // 1 ring
        //bring ring servo up
        ringServo.setPosition(1);
        
        //drive forward
        backLeftDrive.setPower(0.7); 
        backRightDrive.setPower(0.7); 
        frontRightDrive.setPower(0.7); 
        frontLeftDrive.setPower(0.7); 
        sleep(1000);
        
        //stop drive
        backLeftDrive.setPower(0.0);
        backRightDrive.setPower(0.0);
        frontRightDrive.setPower(0.0);
        frontLeftDrive.setPower(0.0);
        sleep(500);
        
        //turn left 
        backLeftDrive.setPower(-0.7);
        backRightDrive.setPower(0.7);
        frontRightDrive.setPower(0.0);
        frontLeftDrive.setPower(0.0);
        sleep(1500);
        
        //bring down wobble
        sleep(1000);
        
        //release wobble 
        wobbleCoreMotor.setPower(-0.5);
        sleep(1250);
        wobbleCoreMotor.setPower(0.0);
        
        //turn right
        backLeftDrive.setPower(-0.7);
        backRightDrive.setPower(0.7);
        frontRightDrive.setPower(0.0);
        frontLeftDrive.setPower(0.0);
        sleep(1500);
        
        //park
        backLeftDrive.setPower(-0.7);
        backRightDrive.setPower(-0.7);
        frontRightDrive.setPower(-0.7);
        frontLeftDrive.setPower(-0.7);
        sleep(750);
      }
      
      else if (potentiometerReading >= 2.5)
      { 
        // 0 rings
        // bring up ring servo
        ringServo.setPosition(1);
        
        // drive forward
        backLeftDrive.setPower(0.7); 
        backRightDrive.setPower(0.7); 
        frontRightDrive.setPower(0.7); 
        frontLeftDrive.setPower(0.7); 
        sleep(750);
        
        //stop drive
        backLeftDrive.setPower(0.0);
        backRightDrive.setPower(0.0);
        frontRightDrive.setPower(0.0);
        frontLeftDrive.setPower(0.0);
        sleep(500);
        
        //bring down wobble
        wobbleCoreMotor.setPower(-0.5);
        sleep(1250);
        wobbleCoreMotor.setPower(0.0);
        
        //release wobble
        wobbleServo.setPosition(0);
        sleep(500);
        
        //park
        backLeftDrive.setPower(-0.7);
        backRightDrive.setPower(-0.7);
        frontRightDrive.setPower(-0.7);
        frontLeftDrive.setPower(-0.7);
        sleep(250);
      }
    } 
  }
}
