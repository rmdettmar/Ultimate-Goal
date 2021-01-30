// Simple autonomous program that drives bot forward until end of period
// or touch sensor is hit. If touched, backs up a bit and turns 90 degrees
// right and keeps going. Demonstrates obstacle avoidance and use of the
// REV Hub's built in IMU in place of a gyro. Also uses gamepad1 buttons to
// simulate touch sensor press and supports left as well as right turn.
//
// Also uses IMU to drive in a straight line when not avoiding an obstacle.

package org.firstinspires.ftc.teamcode.Autonomous;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Date;
import java.util.Timer;
import java.util.TimerTask;

@TeleOp(name="Drive Avoid Imu 3", group="Exercises")
//@Disabled
public class DriveAvoidIMU3 extends LinearOpMode
{
    String                  TAG = "DriveAvoidIMU3";
    DcMotor                 leftMotorFront, rightMotorFront, leftMotorBack, rightMotorBack;
//    TouchSensor             touch;
    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, _power = .30, correction;
    boolean                 aButton, bButton, touched, isStraight = true;

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException
    {
        leftMotorFront = hardwareMap.dcMotor.get("left_drive_1");
        rightMotorFront = hardwareMap.dcMotor.get("right_drive_1");
        leftMotorBack = hardwareMap.dcMotor.get("left_drive_2");
        rightMotorBack = hardwareMap.dcMotor.get("right_drive_2");


        // leftMotorFront.setDirection(DcMotor.Direction.REVERSE);

        // leftMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // rightMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // get a reference to touch sensor.
//        touch = hardwareMap.touchSensor.get("touch_sensor");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        // wait for start button.

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        // sleep(1000);

        // drive until end of period.

        double targetAngle = 0;

        while (opModeIsActive())
        {
            new Timer().schedule(new TimerTask() {
                public void run() {
                    Log.i(TAG,"Task performed on: " + new Date() + "n" +
                            "Thread's name: " + Thread.currentThread().getName());
                    isStraight = false;
                }}, 3000);

            // Use gyro to drive in a straight line.
            while (isStraight == true && opModeIsActive()) {
                correction = getCorrectionFactor(targetAngle);

                leftMotorFront.setPower(maxMotorPower(.35, _power - correction));
                rightMotorFront.setPower(maxMotorPower(.35, _power + correction));
                leftMotorBack.setPower(maxMotorPower(.35, _power - correction));
                rightMotorBack.setPower(maxMotorPower(.35, _power + correction));

                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", Math.round(correction * 10) / 10);
                telemetry.addData("4 target angle", targetAngle);
                telemetry.addData("5 left motor power front", maxMotorPower(.35, _power - correction));
                telemetry.addData("6 right motor power front", maxMotorPower(.35, _power + correction));
                telemetry.addData("7 left motor power back", maxMotorPower(.35, _power - correction));
                telemetry.addData("8 right motor power back", maxMotorPower(.35, _power + correction));
                telemetry.update();
            }

            targetAngle += 180;
            targetAngle %= 360;
            rotate(180, .27, targetAngle);
            isStraight = true;

            // We record the sensor values because we will test them in more than
            // one place with time passing between those places. See the lesson on
            // Timing Considerations to know why.

/*          aButton = gamepad1.a;
            bButton = gamepad1.b;
            touched = touch.isPressed();

            if (touched || aButton || bButton)
            {
                // backup.
                leftMotor.setPower(power);
                rightMotor.setPower(power);

                sleep(500);

                // stop.
                leftMotor.setPower(0);
                rightMotor.setPower(0);

                // turn 90 degrees right.
                if (touched || aButton) rotate(-90, power);

                // turn 90 degrees left.
                if (bButton) rotate(90, power);
            }
*/
        }

        // turn the motors off.
        rightMotorFront.setPower(0);
        leftMotorFront.setPower(0);
        rightMotorBack.setPower(0);
        leftMotorBack.setPower(0);
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle(double targetAngle)
    {
        //lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        lastAngles = new Orientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES, (float)targetAngle, 0, 0, 0);

        globalAngle = targetAngle;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double getCorrectionFactor(double targetAngle)
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == targetAngle)
            correction = 0;             // no adjustment.
        else
            correction = targetAngle - angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double turnPower, double targetAngle)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle(targetAngle - degrees);

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = _power + turnPower;
            rightPower = _power - turnPower;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = _power - turnPower;
            rightPower = _power + turnPower;
        }
        else return;

        // set power to rotate.
        leftMotorFront.setPower(leftPower);
        rightMotorFront.setPower(rightPower);
        leftMotorBack.setPower(leftPower);
        rightMotorBack.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > targetAngle) {}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < targetAngle) {}

        // turn the motors off.
        rightMotorFront.setPower(_power);
        leftMotorFront.setPower(_power);
        rightMotorBack.setPower(_power);
        leftMotorBack.setPower(_power);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle(targetAngle);
    }

    private double maxMotorPower(double limit, double power)
    {
        if (Math.abs(power) > limit)
        {
            double limitedPower;
            if(power > 0) limitedPower = limit;
            else limitedPower = -limit;
            return limitedPower;
        }
        else return power;
    }

}