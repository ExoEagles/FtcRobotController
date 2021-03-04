package org.firstinspires.ftc.teamcode.Robot.Drivetrain.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Robot.Drivetrain.Odometry.OdometryGlobalCoordinatePosition;

@Autonomous(name = "OdometryAutonomous for Super Qualifier", group = "Odometric Corrected Drive")
public class OdometryAutonomous extends LinearOpMode
{
    DcMotor right_front, right_back, left_front, left_back;
    DcMotor verticalLeft, verticalRight, horizontal;

    DcMotor leftShooter = null, rightShooter = null, wobbleCoreMotor = null;
    CRServo stackServo = null, uptakeServo = null;

    Servo wobbleServo = null, ringServo = null;
    BNO055IMU imu;
    DigitalChannel touchSensor = null;
    AnalogInput potentiometer = null;

    double potentiometerReading = 0.0;

    final double COUNTS_PER_INCH = 735.92113;
    final double DRIVE_SPEED = 0.65;


    String rfName = "frontRightDrive", rbName = "backRightDrive", lfName = "frontLeftDrive", lbName = "backLeftDrive";
    String verticalLeftEncoderName = rfName, verticalRightEncoderName = lfName, horizontalEncoderName = lbName;
    OdometryGlobalCoordinatePosition globalPositionUpdate;
    private double getXAngle(){
        return (-imu.getAngularOrientation().firstAngle);
    }

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

    public void errorCorrectedDriveFB(double power, double distance)
    {
        double t = globalPositionUpdate.returnOrientation();
        double o = (t % 360.0);

        if (o <= 45.0 && o > -45.0)
        {
            double correctedDistance = (distance * Math.cos(Math.toRadians(o)));
            errorCorrectedDriveYfb(power, correctedDistance);
        }

        else if (o <= 135.0 && o > 45.0)
        {
            o -= 90.0;

            double correctedDistance = (distance * Math.cos(Math.toRadians(o)));
            errorCorrectedDriveXfb(power, -correctedDistance);
        }

        else if (o <= 225.0 && o > 135.0)
        {
            o -= 180.0;

            double correctedDistance = (distance * Math.cos(Math.toRadians(o)));
            errorCorrectedDriveYfb(power, -correctedDistance);
        }

        else if (o <= 315.0 && o > 225.0)
        {
            o -= 270.0;

            double correctedDistance = (distance * Math.cos(Math.toRadians(o)));
            errorCorrectedDriveXfb(power, correctedDistance);
        }

        else if (o < 360.0 && o > 315.0)
        {
            o = (360.0 - o);

            double correctedDistance = (distance * Math.cos(Math.toRadians(o)));
            errorCorrectedDriveYfb(power, correctedDistance);
        }

        else if (o <= -45.0 && o > -135.0)
        {
            o += 90.0;

            double correctedDistance = (distance * Math.cos(Math.toRadians(o)));
            errorCorrectedDriveXfb(power, correctedDistance);
        }

        else if (o <= -135.0 && o > -225.0)
        {
            o += 180.0;

            double correctedDistance = (distance * Math.cos(Math.toRadians(o)));
            errorCorrectedDriveYfb(-power, -correctedDistance);
        }

        else if (o <= -225.0 && o > -315.0)
        {
            o += 270;

            double correctedDistance = (distance * Math.cos(Math.toRadians(o)));
            errorCorrectedDriveXfb(-power, correctedDistance);
        }

        else if (o <= -315.0 && o > -360.0)
        {
            o = (360.0 + o);

            double correctedDistance = (distance * Math.cos(Math.toRadians(o)));
            errorCorrectedDriveYfb(power, correctedDistance);
        }
    }

    public void errorCorrectedDriveLR(double power, double distance)
    {
    }

    public void gotoXY(double x, double y, double theta)
    {
        double xc = globalPositionUpdate.returnXCoordinate();
        double yc = globalPositionUpdate.returnYCoordinate();

        if (x == xc) return;
        if (y == yc) return;

        double dx = (x - xc);
        double dy = (y - yc);

        double distance = Math.sqrt((dx * dx) + (dy * dy));

        double nx = (x - xc);
        double ny = (y - yc);

        double angle = Math.toDegrees(Math.atan2(ny, nx));

        errorCorrectedTurn((Math.signum(angle) * DRIVE_SPEED), angle);
        errorCorrectedDriveFB(DRIVE_SPEED, distance);

        double o = (globalPositionUpdate.returnOrientation() % 360);
        double t = (theta % 360);

        double dt = (t - o);

        errorCorrectedTurn((Math.signum(dt) * DRIVE_SPEED), dt);
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


        initDriveHardwareMap(rfName, rbName, lfName, lbName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        //Initialize IMU parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
        telemetry.addData("Odometry System Calibration Status", "IMU Init Complete");
        telemetry.clear();

        //Odometry System Calibration Init Complete
        telemetry.addData("Odometry System Calibration Status", "Init Complete");
        telemetry.addData("IMU Angle", getXAngle());
        telemetry.addData("Status", "Init Complete");
        telemetry.update();

        waitForStart();

        if (opModeIsActive())
        {
            //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
            globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
            Thread positionThread = new Thread(globalPositionUpdate);
            positionThread.start();

            globalPositionUpdate.reverseRightEncoder();
            globalPositionUpdate.reverseNormalEncoder();


            /* CODE FOR THE AUTONOMOUS DRIVE */

            //move servos to starting positions
            ringServo.setPosition(1.0);
            wobbleServo.setPosition(1.0);
            sleep(500);

            //align robot with goal
            errorCorrectedDriveYfb(0.65, 60.0);
            sleep(50);
            errorCorrectedTurn(0.65, 90);
            sleep(50);
            errorCorrectedDriveXfb(0.65, 19.5);
            sleep(100);
            errorCorrectedTurn(-0.65, -85.0);

            //bring rings up
            stackServo.setPower(1.0);
            uptakeServo.setPower(1.0);
            leftShooter.setPower(0.5);
            rightShooter.setPower(-0.5);
            sleep(5000);
            uptakeServo.setPower(-1.0);
            sleep(500);
            uptakeServo.setPower(1.0);
            sleep(2000);

            //turn off shooter and stacker
            leftShooter.setPower(0.0);
            rightShooter.setPower(0.0);
            stackServo.setPower(0.0);
            uptakeServo.setPower(0.0);

            //turn around and detect rings
            errorCorrectedTurn(-0.65, -170);
            sleep(50);
            errorCorrectedDriveYfb(0.65, -8.0);
            ringServo.setPosition(0.0);
            sleep(1500);

            //read touch sensor and potentiometer
            if (touchSensor.getState() == false)
            {
                potentiometerReading = potentiometer.getVoltage();
                telemetry.addData("Potentiometer: ", potentiometerReading);
                sleep(500);
                ringServo.setPosition(1.0);
            }

            //if 4 rings
            if (potentiometerReading <= 2.1999)
            {
                //drive backwards towards wobble drop zone
                errorCorrectedTurn(-0.65, -8.0);
                errorCorrectedDriveYfb(-0.65, 61.0);

                //lower wobble arm
                wobbleCoreMotor.setPower(-0.5);
                sleep(1300);
                wobbleCoreMotor.setPower(0.0);

                //release wobble goal
                wobbleServo.setPosition(0.0);
                sleep(500);
                errorCorrectedDriveYfb(0.65, -1.0);
                sleep(500);
                wobbleCoreMotor.setPower(1);
                sleep(1000);
                wobbleCoreMotor.setPower(0.0);

                //drive forward to park
                errorCorrectedDriveYfb(0.65, -35.5);
            }

            //if 1 ring
            else if (potentiometerReading >= 2.2 && potentiometerReading <= 2.539)
            {
                //turn a bit right to line up with wobble drop zone and drive backwards
                //CAN DELETE OR MODIFY TURN IF NEEDED
                errorCorrectedTurn(-0.65, -5.0);
                sleep(50);
                errorCorrectedDriveYfb(-0.65, 25.0);
                sleep(50);
                errorCorrectedTurn(0.65, 90);
                sleep(50);
                errorCorrectedDriveYfb(0.65, 0.25);
                sleep(50);
                //lower wobble arm
                wobbleCoreMotor.setPower(-0.5);
                sleep(1300);
                wobbleCoreMotor.setPower(0.0);

                //release wobble goal
                wobbleServo.setPosition(0.0);
                sleep(1000);
                errorCorrectedDriveYfb(0.65, -1.0);
                sleep(500);
                wobbleCoreMotor.setPower(1);
                sleep(1000);
                wobbleCoreMotor.setPower(0.0);

                //strafe right
                //errorCorrectedDriveYlr(0.65, -10.0);
                right_front.setPower(0.8);
                left_front.setPower(-0.8);
                right_back.setPower(-0.8);
                left_back.setPower(0.8);

                sleep(400);

                right_front.setPower(0.0);
                left_front.setPower(0.0);
                right_back.setPower(0.0);
                left_back.setPower(0.0);
            }

            //if 0 rings
            else if (potentiometerReading >= 2.5)
            {
                //turn a bit right to line up with wobble drop zone and drive backwards
                //CAN DELETE OR MODIFY TURN IF NEEDED
                errorCorrectedTurn(-0.65, -15.0);
                sleep(50);
                errorCorrectedDriveYfb(-0.65, 18.5);

                //lower wobble arm
                wobbleCoreMotor.setPower(-0.5);
                sleep(1300);
                wobbleCoreMotor.setPower(0.0);

                //release wobble goal
                wobbleServo.setPosition(0.0);
                sleep(800);
                errorCorrectedDriveYfb(0.65, -1.0);
                sleep(500);
                wobbleCoreMotor.setPower(1);
                sleep(1000);
                wobbleCoreMotor.setPower(0.0);

                //drive forward to park
                errorCorrectedDriveYfb(0.65, -1.0);
            }
        }

        //Stop the thread
        globalPositionUpdate.stop();

    }

    private void initDriveHardwareMap(String rfName, String rbName, String lfName, String lbName, String vlEncoderName, String vrEncoderName, String hEncoderName)
    {
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
        left_back.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();
    }

    private double calculateX(double desiredAngle, double speed)
    {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    private double calculateY(double desiredAngle, double speed)
    {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }

    private void setPowerAll(double rf, double rb, double lf, double lb)
    {
        right_front.setPower(rf);
        right_back.setPower(rb);
        left_front.setPower(lf);
        left_back.setPower(lb);
    }
}
