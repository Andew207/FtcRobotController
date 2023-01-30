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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Camden's Dumb Layout", group="Linear Opmode")
public class CamdensDumbLayout extends LinearOpMode {

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor lift = null;
    private DcMotor turnMech = null;
    private CRServo servo1 = null;
    private TouchSensor frontLimit = null;
    private TouchSensor backLimit = null;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left");
        backLeftDrive  = hardwareMap.get(DcMotor.class, "back_left");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right");
        turnMech = hardwareMap.get(DcMotor.class, "turnMech");
        lift = hardwareMap.get(DcMotor.class, "lift");
        servo1 = hardwareMap.crservo.get("servo1");
        frontLimit = hardwareMap.touchSensor.get("frontLimit");
        backLimit = hardwareMap.touchSensor.get("backLimit");
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        int slow = 1;




        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        double dpadleft = 0;
        double dpadright = 0;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        boolean changed = false; //Outside of loop()
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double frontLeftPower;
            double frontRightPower;
            double backLeftPower;
            double backRightPower;
            double grabber;
            double turnMechPower;



            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn  =  gamepad1.right_stick_x/2;

            if(gamepad1.b && !changed) {
                if(slow == 1) slow = 2;
                else slow = 1;
                changed = true;
            } else if(!gamepad1.b) changed = false;
            double robotest = Range.clip(gamepad2.left_trigger - gamepad2.right_trigger, -1, 1);

            frontLeftPower    = Range.clip((drive + strafe + turn)/slow, -0.75, 0.75);
            frontRightPower   = Range.clip((drive - strafe - turn)/slow, -0.75, 0.75);
            backLeftPower    = Range.clip((drive - strafe + turn)/slow, -0.75, 0.75);
            backRightPower   = Range.clip((drive + strafe - turn)/slow, -0.75, 0.75);


            if (gamepad2.dpad_left == true){
                dpadleft = .4;
            }
            else{dpadleft = 0;}

            if (gamepad2.dpad_right == true){
                dpadright = .4;
            }
            else{dpadright = 0;}

            if (backLimit.isPressed() == false && ((gamepad2.dpad_right == false && gamepad2.dpad_left == true) || (gamepad2.right_stick_x < 0))){
                turnMechPower = 0.5;
            }
            else if (frontLimit.isPressed() == false && ((gamepad2.dpad_right == true && gamepad2.dpad_left == false) || (gamepad2.right_stick_x > 0))){
                turnMechPower = -0.5;
            }
            else{turnMechPower = 0;}

            // Send calculated power to wheels
            frontLeftDrive.setPower(frontLeftPower);
            backLeftDrive.setPower(backLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backRightDrive.setPower(backRightPower);
            lift.setPower(robotest);
            if (gamepad2.right_bumper){
                grabber = 1;
            }
            else if (gamepad2.left_bumper){
                grabber = -1;
            }
            else{grabber = 0;}
            servo1.setPower(grabber);
            turnMech.setPower(turnMechPower);




            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", frontLeftPower, frontRightPower, backLeftPower, backRightPower);
            telemetry.addData("FL Encoder", frontLeftDrive.getCurrentPosition());
            telemetry.addData("FR Encoder", frontRightDrive.getCurrentPosition());
            telemetry.addData("BL Encoder", backLeftDrive.getCurrentPosition());
            telemetry.addData("BR Encoder", backRightDrive.getCurrentPosition());
            telemetry.addData("lift encoder", lift.getCurrentPosition());
            telemetry.addData("Lift power", robotest);

            telemetry.addData("FL Power", frontLeftPower);
            telemetry.addData("FR Power", frontRightPower);
            telemetry.addData("BL Power", backLeftPower);
            telemetry.addData("BR Power", backRightPower);
            telemetry.addData("Bumper", grabber);
            telemetry.addData("Bumper Pressed?", gamepad2.left_bumper);
            telemetry.addData("left dpad", gamepad2.dpad_left);
            telemetry.addData("right dpad", gamepad2.dpad_right);
            telemetry.addData("ldpad", dpadleft);
            telemetry.addData("rdpad", dpadright);
            telemetry.update();
        }
    }
}
