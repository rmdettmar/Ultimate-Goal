package org.firstinspires.ftc.teamcode.Shared;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

public class Drive {
    DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive = null;
    OpMode opMode;
    Telemetry telemetry;
    public HardwareMap hardwareMap; // will be set in OpModeManager.runActiveOpMode
    public Drive(OpMode _opMode){
        opMode = _opMode;
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;
    }

    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        //robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void vroom_vroom (double magRight, double thetaRight, double magLeft, double thetaLeft) {
        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        double frontRightPowerFactor, frontLeftPowerFactor, backRightPowerFactor, backLeftPowerFactor;
        double pi = Math.PI;


        if(thetaRight > 0 && thetaRight < pi/2){
            frontRightPowerFactor = -Math.cos(2 * thetaRight);
        }else if(thetaRight >= -pi && thetaRight < -pi/2){
            frontRightPowerFactor = Math.cos(2 * thetaRight);
        }else if(thetaRight >= pi/2 && thetaRight <= pi){
            frontRightPowerFactor = 1;
        }else{
            frontRightPowerFactor = -1;
        }

        if(thetaLeft > 0 && thetaLeft < pi/2) {
            backLeftPowerFactor = -Math.cos(2 * thetaLeft);
        }else if(thetaLeft >= -pi && thetaLeft < -pi/2){
            backLeftPowerFactor = Math.cos(2 * thetaLeft);
        }else if(thetaLeft >= pi/2 && thetaLeft <= pi){
            backLeftPowerFactor = 1;
        }else{
            backLeftPowerFactor = -1;
        }

        if(thetaRight > -pi/2 && thetaRight < 0) {
            backRightPowerFactor = Math.cos(2 * thetaRight);
        }else if(thetaRight > pi/2 && thetaRight < pi){
            backRightPowerFactor = -Math.cos(2 * thetaRight);
        }else if(thetaRight >= 0 && thetaRight <= pi/2){
            backRightPowerFactor = 1;
        }else{
            backRightPowerFactor = -1;
        }

        if(thetaLeft > -pi/2 && thetaLeft < 0) {
            frontLeftPowerFactor = Math.cos(2 * thetaLeft);
        }else if(thetaLeft > pi/2 && thetaLeft < pi){
            frontLeftPowerFactor = -Math.cos(2 * thetaLeft);
        }else if(thetaLeft >= 0 && thetaLeft <= pi/2){
            frontLeftPowerFactor = 1;
        }else{
            frontLeftPowerFactor = -1;
        }


        leftFrontDrive.setPower(frontLeftPowerFactor * magLeft);
        rightFrontDrive.setPower(frontRightPowerFactor * magRight);
        leftBackDrive.setPower(backLeftPowerFactor * magLeft);
        rightBackDrive.setPower(backRightPowerFactor * magRight);


        telemetry.addData("front right power ", ((float)Math.round(rightFrontDrive.getPower()*100))/100);
        telemetry.addData("front left power ", ((float)Math.round(leftFrontDrive.getPower() *100))/100);
        telemetry.addData("back right power ", ((float)Math.round(rightBackDrive.getPower()*100))/100);
        telemetry.addData("back left power ", ((float)Math.round(leftBackDrive.getPower()*100))/100);
        telemetry.addData("magnitude left ", ((float)Math.round(magLeft*100))/100);
        telemetry.addData("thetaLeft ", ((float)Math.round(thetaLeft/pi*100))/100);

        telemetry.update();
    }
}
