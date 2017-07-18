/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Blue1", group="blue")  // @Autonomous(...) is the other common choice

public class AutoBlue extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    Hardware r = new Hardware();
    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    boolean tape = false;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        r.init(hardwareMap); //reset servos, set motors to use encoders

        waitForStart();
        runtime.reset();
        telemetry.addData("Status", "Running");
        telemetry.update();
        launchParticles(0.35);

        r.rightMotor1.setPower(-0.8);
        r.rightMotor2.setPower(-0.8);
        r.leftMotor1.setPower(-0.8);
        r.leftMotor2.setPower(-0.8);

        sleep(1700);

        r.rightMotor1.setPower(0);
        r.rightMotor2.setPower(0);
        r.leftMotor1.setPower(0);
        r.leftMotor2.setPower(0);

        /*r.blueBar.setPosition(0.45);
        encoderDrive(0.6, -0.01, -0.01, 5); //drive forward 10 inches at 60% power with a 5 second timeout
         //turn right
         //drive forward, aligning with wall

        while(!tape && opModeIsActive()) {
            //find white line
            if(r.bottomLeft.blue() == 0) {
                r.leftMotor1.setPower(-0.3);
                r.rightMotor1.setPower(-0.3);
                r.leftMotor2.setPower(-0.3);
                r.rightMotor2.setPower(-0.3);

            }
            else {
                r.leftMotor1.setPower(0);
                r.rightMotor1.setPower(0);
                r.leftMotor2.setPower(0);
                r.rightMotor2.setPower(0);
                tape = true;
            }
        }

        if (isBlue()) {
            r.bluePress.setPosition(1);
            sleep(1000);
            r.bluePress.setPosition(0);
        }

        tape = false;

        while(!tape && opModeIsActive()) {
            //find second white line
            if(r.bottomLeft.blue() == 0) {
                r.leftMotor1.setPower(-0.3);
                r.rightMotor1.setPower(-0.3);
                r.leftMotor2.setPower(-0.3);
                r.rightMotor2.setPower(-0.3);

            }
            else {
                r.leftMotor1.setPower(0);
                r.rightMotor1.setPower(0);
                r.leftMotor2.setPower(0);
                r.rightMotor2.setPower(0);
                tape = true;
            }
        }

        if (isBlue()) {
            r.bluePress.setPosition(1);
            sleep(1000);
            r.bluePress.setPosition(0);
        }

        else {
            r.bluePress.setPosition(1);
            sleep(1000);
            r.bluePress.setPosition(0);
        }*/

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
            // leftMotor.setPower(-gamepad1.left_stick_y);
            // rightMotor.setPower(-gamepad1.right_stick_y);
        }
    }



    public void launchParticles(double speed) throws InterruptedException {
        r.launcher1.setPower(-speed);
        r.launcher2.setPower(speed);
        sleep(2000);
        r.particleFlip.setPosition(1);
        sleep(800);
        r.particleFlip.setPosition(0);
        sleep(800);
        r.particleFlip.setPosition(1);
        sleep(800);
        r.particleFlip.setPosition(0);
        sleep(800);
        r.particleFlip.setPosition(1);
        sleep(800);
        r.particleFlip.setPosition(0);
        sleep(800);
        r.launcher1.setPower(0);
        r.launcher2.setPower(0);

    }

    public boolean isBlue() {
        if (r.blueBeacon.red() > 0) {
            return false;
        } else {
            return true;
        }
    }

    public void drive(double rightPower, double leftPower, long time) throws InterruptedException {
        r.rightMotor1.setPower(rightPower);
        r.rightMotor2.setPower(rightPower);
        r.leftMotor1.setPower(leftPower);
        r.leftMotor2.setPower(leftPower);

        sleep(time);

        r.rightMotor1.setPower(0);
        r.rightMotor2.setPower(0);
        r.leftMotor1.setPower(0);
        r.leftMotor2.setPower(0);
    }

    public void encoderDrive(double speed, double leftIN, double rightIN, double time) throws InterruptedException {
        r.rightMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        r.rightMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        r.leftMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        r.leftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        r.rightMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r.rightMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r.leftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r.leftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r.rightMotor1.setPower(0);
        r.rightMotor2.setPower(0);
        r.leftMotor1.setPower(0);
        r.leftMotor2.setPower(0);
        r.rightMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        r.rightMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        r.leftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        r.leftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int rightTarget = r.rightMotor1.getCurrentPosition() + (int) (rightIN * COUNTS_PER_INCH);
        int leftTarget = r.leftMotor1.getCurrentPosition() + (int) (leftIN * COUNTS_PER_INCH);
        r.rightMotor1.setTargetPosition(rightTarget);
        r.rightMotor2.setTargetPosition(rightTarget);
        r.leftMotor1.setTargetPosition(leftTarget);
        r.leftMotor2.setTargetPosition(leftTarget);
        runtime.reset();
        r.rightMotor1.setPower(Math.abs(speed));
        r.rightMotor2.setPower(Math.abs(speed));
        r.leftMotor1.setPower(Math.abs(speed));
        r.leftMotor2.setPower(Math.abs(speed));
        while (opModeIsActive() && runtime.seconds() < time && r.rightMotor1.isBusy() && r.leftMotor1.isBusy()) {
            telemetry.addData("AUTONOMOUS", "Target Position(R,L): " + rightTarget + "," + leftTarget);
            telemetry.addData("AUTONOMOUS", "Current Position(R,L): " + r.rightMotor1.getCurrentPosition() + "," + r.leftMotor1.getCurrentPosition());
            telemetry.addData("Time: ", runtime.seconds());
            telemetry.update();
        }
        r.rightMotor1.setPower(0);
        r.rightMotor2.setPower(0);
        r.leftMotor1.setPower(0);
        r.leftMotor2.setPower(0);
    }


}
