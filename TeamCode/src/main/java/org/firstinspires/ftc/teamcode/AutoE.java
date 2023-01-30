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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "Basic: Linear Opcode", group = "Linear Opcode")
public class AutoE extends LinearOpMode {

    //BNO055IMU imu;
    //double angles;

    // PID Variables
    //double Kp = 5;
    //double Ki = 0;
    //double Kd = 0.2;



    //double integralSum = 0;

    //double lastError = 0;

    ElapsedTime timer = new ElapsedTime();


    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;

    public void setMotorMode(DcMotor.RunMode runMode){
        frontLeftDrive.setMode(runMode);
        backLeftDrive.setMode(runMode);
        frontRightDrive.setMode(runMode);
        backRightDrive.setMode(runMode);
    }

    public void setAllPower(double power){
        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
        backLeftDrive.setPower(power);
        backRightDrive.setPower(power);
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right");
/*
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
*/
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)


        //String allPowerOut = null;
        
        //int targetpos = 1000;
        //int reference = targetpos;

        while (opModeIsActive()) {

            int targetpos = 30;

            while ((frontRightDrive.getCurrentPosition()+frontLeftDrive.getCurrentPosition()+backLeftDrive.getCurrentPosition()+ backRightDrive.getCurrentPosition())/4 < targetpos){
                setAllPower(0.25);
            }


            //setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
            /*
            while ((frontLeftDrive.getCurrentPosition()+frontRightDrive.getCurrentPosition()+backRightDrive.getCurrentPosition()+backLeftDrive.getCurrentPosition())/4 < targetpos) {
                // obtain the encoder position
                double encoderPosition = (frontLeftDrive.getCurrentPosition() + frontRightDrive.getCurrentPosition() + backRightDrive.getCurrentPosition() + backLeftDrive.getCurrentPosition()) / 4;

                // calculate the error
                double error = reference - encoderPosition;


                // rate of change of the error
                double derivative = (error - lastError) / timer.seconds();

                // sum of all error over time
                integralSum = integralSum + (error * timer.seconds());

                double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

                setAllPower(out/(targetpos*10));

                lastError = error;

                // reset the timer for next time
                timer.reset();

                // Show the elapsed game time and wheel power.
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("FL Encoder", frontLeftDrive.getCurrentPosition());
                telemetry.addData("FR Encoder", frontRightDrive.getCurrentPosition());
                telemetry.addData("BL Encoder", backLeftDrive.getCurrentPosition());
                telemetry.addData("BR Encoder", backRightDrive.getCurrentPosition());
                telemetry.addData("Out", allPowerOut);
                telemetry.update();
                
                allPowerOut += out/(targetpos*10) + ", ";
                */

            }
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("FL Encoder", frontLeftDrive.getCurrentPosition());
            //telemetry.addData("FR Encoder", frontRightDrive.getCurrentPosition());
            //telemetry.addData("BL Encoder", backLeftDrive.getCurrentPosition());
            //telemetry.addData("BR Encoder", backRightDrive.getCurrentPosition());
            telemetry.update();

        }


    }