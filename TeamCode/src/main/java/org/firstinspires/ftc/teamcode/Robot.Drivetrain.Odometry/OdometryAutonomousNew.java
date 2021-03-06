package org.firstinspires.ftc.teamcode.Robot.Drivetrain.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Robot.Drivetrain.Odometry.OdometryGlobalCoordinatePosition;

@Autonomous(name = "OdometryAutonomousNew", group = "Autonomous")
public class OdometryAutonomousNew extends LinearOpMode
{
    public Robot robot = null;
    public OdometryGlobalCoordinatePosition globalPositionUpdate = null;

    public void errorCorrectedDriveXlr(double power, double distance)
    {
        robot.setPowerAll(-power, power, -power, power);

        sleep(100);

        double px = (globalPositionUpdate.returnXCoordinate() / robot.COUNTS_PER_INCH);
        double o = globalPositionUpdate.returnOrientation();

        double fx, t, dt;

        do
        {
            fx = (globalPositionUpdate.returnXCoordinate() / robot.COUNTS_PER_INCH);
            t = globalPositionUpdate.returnOrientation();

            dt = (t - o);

            if (dt >= 0.5)
            {
                robot.frontRightDrive.setPower(power + 0.2);
                //right_back.setPower(power - 0.1);
            }

            if (dt <= -0.5)
            {
                robot.frontLeftDrive.setPower(-power - 0.2);
                //left_back.setPower(-power + 0.1);
            }

            if (dt > -0.5 && dt < 0.5)
            {
                robot.setPowerAll(-power, power, -power, power);
            }
        } while (Math.abs((distance + px) - fx) >= 1.0 && opModeIsActive());

        robot.setPowerAll(robot.MIN_POWER, robot.MIN_POWER, robot.MIN_POWER, robot.MIN_POWER);
    }

    public void errorCorrectedDriveXfb(double power, double distance)
    {
        robot.setPowerAll(-power, -power, -power, -power);

        sleep(100);

        double px = (globalPositionUpdate.returnXCoordinate() / robot.COUNTS_PER_INCH);
        double o = globalPositionUpdate.returnOrientation();

        double fx, t, dt;

        do
        {
            fx = (globalPositionUpdate.returnXCoordinate() / robot.COUNTS_PER_INCH);
            t = globalPositionUpdate.returnOrientation();

            dt = (t - o);

            if (dt >= 0.5)
            {
                robot.frontLeftDrive.setPower(-power + 0.2);
                robot.backLeftDrive.setPower(-power + 0.2);
            }

            if (dt <= -0.5)
            {
                robot.frontRightDrive.setPower(-power + 0.2);
                robot.backRightDrive.setPower(-power + 0.2);
            }

            if (dt > -0.5 && dt < 0.5)
            {
                robot.setPowerAll(-power, -power, -power, -power);
            }
        } while (Math.abs((distance + px) - fx) >= 2.55 && opModeIsActive());

        robot.setPowerAll(robot.MIN_POWER, robot.MIN_POWER, robot.MIN_POWER, robot.MIN_POWER);
    }

    public void errorCorrectedDriveYfb(double power, double distance)
    {
        robot.setPowerAll(-power, -power, -power, -power);

        sleep(100);

        double py = (globalPositionUpdate.returnYCoordinate() / robot.COUNTS_PER_INCH);
        double o = globalPositionUpdate.returnOrientation();

        double fy, t, dt;

        do
        {
            fy = (globalPositionUpdate.returnYCoordinate() / robot.COUNTS_PER_INCH);
            t = globalPositionUpdate.returnOrientation();

            dt = (t - o);

            if (dt >= 0.5)
            {
                robot.frontLeftDrive.setPower(-power + 0.2);
                robot.backLeftDrive.setPower(-power + 0.2);
            }

            if (dt <= -0.5)
            {
                robot.frontRightDrive.setPower(-power + 0.2);
                robot.backRightDrive.setPower(-power + 0.2);
            }

            if (dt > -0.5 && dt < 0.5)
            {
                robot.setPowerAll(-power, -power, -power, -power);
            }
        } while (Math.abs((distance + py) - fy) >= 2.55 && opModeIsActive());

        robot.setPowerAll(robot.MIN_POWER, robot.MIN_POWER, robot.MIN_POWER, robot.MIN_POWER);
    }

    public void errorCorrectedDriveYlr(double power, double distance)
    {
        robot.setPowerAll(-power, -power, -power, -power);

        sleep(100);

        double py = (globalPositionUpdate.returnYCoordinate() / robot.COUNTS_PER_INCH);
        double o = globalPositionUpdate.returnOrientation();

        double fy, t, dt;

        do
        {
            fy = (globalPositionUpdate.returnYCoordinate() / robot.COUNTS_PER_INCH);
            t = globalPositionUpdate.returnOrientation();

            dt = (t - o);

            if (dt >= 0.5)
            {
                robot.frontRightDrive.setPower(power + 0.2);
                //right_back.setPower(power - 0.1);
            }

            if (dt <= -0.5)
            {
                robot.frontLeftDrive.setPower(-power - 0.2);
                //left_back.setPower(-power + 0.1);
            }

            if (dt > -0.5 && dt < 0.5)
            {
                robot.setPowerAll(-power, power, -power, power);
            }
        } while (Math.abs((distance + py) - fy) >= 1.0 && opModeIsActive());

        robot.setPowerAll(robot.MIN_POWER, robot.MIN_POWER, robot.MIN_POWER, robot.MIN_POWER);
    }

    public void errorCorrectedTurn(double power, double angle)
    {
        if (angle == 0.0) return;

        double o = globalPositionUpdate.returnOrientation();
        double np = (power * Math.signum(angle));
        double t;

        do
        {
            t = globalPositionUpdate.returnOrientation();

            robot.setPowerAll(-np, np, np, -np);
        } while (Math.abs((angle + o) - t) >= 5.0 && opModeIsActive());

        robot.setPowerAll(robot.MIN_POWER, robot.MIN_POWER, robot.MIN_POWER, robot.MIN_POWER);
    }

    public void errorCorrectedDriveFB(double power, double distance)
    {
        double t = globalPositionUpdate.returnOrientation(), o;

        if (power > 0) o = (t % 360.0);
        else o = (180 + (t % 360));

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
            errorCorrectedDriveYfb(power, -correctedDistance);
        }

        else if (o <= -225.0 && o > -315.0)
        {
            o += 270;

            double correctedDistance = (distance * Math.cos(Math.toRadians(o)));
            errorCorrectedDriveXfb(power, -correctedDistance);
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

        errorCorrectedTurn((Math.signum(angle) * robot.DRIVE_SPEED), angle);
        errorCorrectedDriveFB(robot.DRIVE_SPEED, distance);

        double o = (globalPositionUpdate.returnOrientation() % 360);
        double t = (theta % 360);

        double dt = (t - o);

        errorCorrectedTurn((Math.signum(dt) * robot.DRIVE_SPEED), dt);
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot = new Robot(hardwareMap);

        robot.frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        if (opModeIsActive())
        {
            //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
            globalPositionUpdate = new OdometryGlobalCoordinatePosition(robot.verticalLeft, robot.verticalRight, robot.horizontal, robot.COUNTS_PER_INCH, 75);
            Thread positionThread = new Thread(globalPositionUpdate);
            positionThread.start();

            globalPositionUpdate.reverseRightEncoder();
            globalPositionUpdate.reverseNormalEncoder();


            /* CODE FOR THE AUTONOMOUS DRIVE */

            //move servos to starting positions
            robot.ringServo.setPosition(robot.RING_UP);
            robot.wobbleServo.setPosition(robot.WOBBLE_CLOSED);
            sleep(500);

            //align robot with goal
            errorCorrectedDriveFB(robot.DRIVE_SPEED, 60.0); sleep(50);
            errorCorrectedTurn(robot.DRIVE_SPEED, 90);
            sleep(50);
            errorCorrectedDriveFB(robot.DRIVE_SPEED, 19.5);
            sleep(50);
            errorCorrectedTurn(robot.DRIVE_SPEED, -85.0);

            //bring rings up
            robot.stackServo.setPower(robot.MAX_POWER);
            robot.uptakeServo.setPower(robot.MAX_POWER);
            robot.leftShooter.setPower(robot.HALF_POWER);
            robot.rightShooter.setPower(robot.HALF_POWER);
            sleep(5000);
            robot.uptakeServo.setPower(-robot.MAX_POWER);
            sleep(500);
            robot.uptakeServo.setPower(robot.MAX_POWER);
            sleep(2000);

            //turn off shooter and stacker
            robot.leftShooter.setPower(robot.MIN_POWER);
            robot.rightShooter.setPower(robot.MIN_POWER);
            robot.stackServo.setPower(robot.MIN_POWER);
            robot.uptakeServo.setPower(robot.MIN_POWER);

            //turn around and detect rings
            errorCorrectedTurn(robot.DRIVE_SPEED, -170);
            sleep(50);
            errorCorrectedDriveFB(robot.DRIVE_SPEED, 8.0);
            robot.ringServo.setPosition(robot.RING_DOWN);
            sleep(1500);

            //read touch sensor and potentiometer
            if (robot.touchSensor.getState() == false)
            {
                robot.potentiometerReading = robot.potentiometer.getVoltage();
                telemetry.addData("Potentiometer: ", robot.potentiometerReading);
                sleep(500);
                robot.ringServo.setPosition(robot.RING_UP);
                sleep(500);
                robot.ringServo.setPosition(robot.SERVO_LOCK);
            }

            //if 4 rings
            if (robot.potentiometerReading <= 2.1999)
            {
                //drive backwards towards wobble drop zone
                errorCorrectedTurn(robot.DRIVE_SPEED, -8.0);
                errorCorrectedDriveFB(-robot.DRIVE_SPEED, 61.0);

                //lower wobble arm
                robot.wobbleCoreMotor.setPower(-robot.HALF_POWER);
                sleep(1300);
                robot.wobbleCoreMotor.setPower(robot.MIN_POWER);

                //release wobble goal
                robot.wobbleServo.setPosition(robot.WOBBLE_OPEN);
                sleep(500);
                errorCorrectedDriveFB(robot.DRIVE_SPEED, 1.0);
                sleep(500);
                robot.wobbleCoreMotor.setPower(robot.MAX_POWER);
                sleep(1000);
                robot.wobbleCoreMotor.setPower(robot.MIN_POWER);

                //drive forward to park
                errorCorrectedDriveFB(robot.DRIVE_SPEED, 35.5);
            }

            //if 1 ring
            else if (robot.potentiometerReading >= 2.2 && robot.potentiometerReading <= 2.539)
            {
                //turn a bit right to line up with wobble drop zone and drive backwards
                errorCorrectedTurn(robot.DRIVE_SPEED, -5.0);
                sleep(50);
                errorCorrectedDriveFB(-robot.DRIVE_SPEED, 25.0);
                sleep(50);
                errorCorrectedTurn(robot.DRIVE_SPEED, 90);
                sleep(50);
                errorCorrectedDriveFB(robot.DRIVE_SPEED, 0.25);
                sleep(50);

                //lower wobble arm
                robot.wobbleCoreMotor.setPower(-robot.HALF_POWER);
                sleep(1300);
                robot.wobbleCoreMotor.setPower(robot.MIN_POWER);

                //release wobble goal
                robot.wobbleServo.setPosition(robot.WOBBLE_OPEN);
                sleep(500);
                errorCorrectedDriveFB(robot.DRIVE_SPEED, 1.0);
                sleep(500);
                robot.wobbleCoreMotor.setPower(robot.MAX_POWER);
                sleep(1000);
                robot.wobbleCoreMotor.setPower(robot.MIN_POWER);

                //strafe right the last little bit to touch the launch line
                robot.setPowerAll(robot.MAX_POWER, -robot.MAX_POWER, -robot.MAX_POWER, robot.MAX_POWER);
                sleep(400);
                robot.setPowerAll(robot.MAX_POWER, robot.MAX_POWER, robot.MAX_POWER, robot.MAX_POWER);
            }

            //if 0 rings
            else if (robot.potentiometerReading >= 2.5)
            {
                //turn a bit right to line up with wobble drop zone and drive backwards
                errorCorrectedTurn(0.65, -15.0);
                sleep(50);
                errorCorrectedDriveFB(-0.65, 18.5);

                //lower wobble arm
                robot.wobbleCoreMotor.setPower(-robot.HALF_POWER);
                sleep(1300);
                robot.wobbleCoreMotor.setPower(robot.MIN_POWER);

                //release wobble goal
                robot.wobbleServo.setPosition(robot.WOBBLE_OPEN);
                sleep(500);
                errorCorrectedDriveFB(robot.DRIVE_SPEED, 1.0);
                sleep(500);
                robot.wobbleCoreMotor.setPower(robot.MAX_POWER);
                sleep(1000);
                robot.wobbleCoreMotor.setPower(robot.MIN_POWER);

                //drive forward to park
                errorCorrectedDriveFB(0.65, 1.0);
            }
        }

        //Stop the thread
        globalPositionUpdate.stop();
    }
}
