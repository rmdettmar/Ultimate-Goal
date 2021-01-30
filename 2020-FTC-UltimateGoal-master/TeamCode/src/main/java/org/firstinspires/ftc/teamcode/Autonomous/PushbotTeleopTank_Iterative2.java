/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Pushbot: Teleop Tank Continuous Sinewave", group="Pushbot")
//@Disabled
public class PushbotTeleopTank_Iterative2 extends OpMode{

    /* Declare OpMode members. */
    //HardwarePushbot robot       = new HardwarePushbot(); // use the class created to define a Pushbot's hardware
    DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive = null;
    double          clawOffset  = 0.0 ;                  // Servo mid position
    final double    CLAW_SPEED  = 0.02 ;                 // sets rate to move servo

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
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

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        double frontRightPowerFactor, frontLeftPowerFactor, backRightPowerFactor, backLeftPowerFactor;
        double magRight = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
        double thetaRight = Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x);
        double magLeft = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double thetaLeft = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x);
        double pi = Math.PI;

//        if(thetaRight > 0 && thetaRight < pi/2){
//            frontRightPowerFactor = -Math.cos(2 * thetaRight);
//        }else if(thetaRight >= -pi && thetaRight < -pi/2){
//            frontRightPowerFactor = Math.cos(2 * thetaRight);
//        }else if(thetaRight >= pi/2 && thetaRight <= pi){
//            frontRightPowerFactor = 1;
//        }else{
//            frontRightPowerFactor = -1;
//        }

        frontRightPowerFactor = Math.sin(thetaRight - pi/4);

//        if(thetaLeft > 0 && thetaLeft < pi/2) {
//            backLeftPowerFactor = -Math.cos(2 * thetaLeft);
//        }else if(thetaLeft >= -pi && thetaLeft < -pi/2){
//            backLeftPowerFactor = Math.cos(2 * thetaLeft);
//        }else if(thetaLeft >= pi/2 && thetaLeft <= pi){
//            backLeftPowerFactor = 1;
//        }else{
//            backLeftPowerFactor = -1;
//        }

        backLeftPowerFactor = Math.sin(thetaLeft - pi/4);

//        if(thetaRight > -pi/2 && thetaRight < 0) {
//            backRightPowerFactor = Math.cos(2 * thetaRight);
//        }else if(thetaRight > pi/2 && thetaRight < pi){
//            backRightPowerFactor = -Math.cos(2 * thetaRight);
//        }else if(thetaRight >= 0 && thetaRight <= pi/2){
//            backRightPowerFactor = 1;
//        }else{
//            backRightPowerFactor = -1;
//        }

        backRightPowerFactor = Math.sin(thetaRight + pi/4);

//        if(thetaLeft > -pi/2 && thetaLeft < 0) {
//            frontLeftPowerFactor = Math.cos(2 * thetaLeft);
//        }else if(thetaLeft > pi/2 && thetaLeft < pi){
//            frontLeftPowerFactor = -Math.cos(2 * thetaLeft);
//        }else if(thetaLeft >= 0 && thetaLeft <= pi/2){
//            frontLeftPowerFactor = 1;
//        }else{
//            frontLeftPowerFactor = -1;
//        }

        frontLeftPowerFactor = Math.sin(thetaLeft + pi/4);

        if(gamepad1.a){
            leftFrontDrive.setPower(0.5);
            leftBackDrive.setPower(0.5);
            rightFrontDrive.setPower(0.5);
            rightBackDrive.setPower(0.5);
        }else if(gamepad1.b){
            leftFrontDrive.setPower(-0.5);
            leftBackDrive.setPower(-0.5);
            rightFrontDrive.setPower(-0.5);
            rightBackDrive.setPower(-0.5);
        }else{
            leftFrontDrive.setPower(frontLeftPowerFactor * magLeft);
            rightFrontDrive.setPower(frontRightPowerFactor * magRight);
            leftBackDrive.setPower(backLeftPowerFactor * magLeft);
            rightBackDrive.setPower(backRightPowerFactor * magRight);
        }


        telemetry.addData("front right power ", ((float)Math.round(rightFrontDrive.getPower()*100))/100);
        telemetry.addData("front left power ", ((float)Math.round(leftFrontDrive.getPower() *100))/100);
        telemetry.addData("back right power ", ((float)Math.round(rightBackDrive.getPower()*100))/100);
        telemetry.addData("back left power ", ((float)Math.round(leftBackDrive.getPower()*100))/100);
        telemetry.addData("left joystick x", ((float)Math.round(gamepad1.left_stick_x * 100))/100);
        telemetry.addData("left joystick y", ((float)Math.round(-gamepad1.left_stick_y * 100))/100);
        telemetry.addData("magnitude left", ((float)Math.round(magLeft*100))/100);
        telemetry.addData("thetaLeft", ((float)Math.round(thetaLeft/pi*100))/100);

        telemetry.update();

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
