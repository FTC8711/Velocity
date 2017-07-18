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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp", group="Iterative Opmode")  // @Autonomous(...) is the other common choice

public class TeleOp extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    Hardware r       = new Hardware();
    public boolean rbButtonWatch = false;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        r.init(hardwareMap);
        telemetry.addData("Status", "Running: " + runtime.toString());
        runtime.reset();



        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
            r.leftMotor1.setPower(gamepad1.left_stick_y);
            r.leftMotor2.setPower(gamepad1.left_stick_y);
            r.rightMotor1.setPower(gamepad1.right_stick_y);
            r.rightMotor2.setPower(gamepad1.right_stick_y);

            if (gamepad1.dpad_left) {
                r.intake.setPower(-1);
            }

            if (gamepad1.dpad_down) {
                r.intake.setPower(0);
            }

            if (gamepad2.left_bumper && !rbButtonWatch) {
                r.launcher1.setPower(0.8);
                r.launcher2.setPower(0.8);
            } else {
                r.launcher1.setPower(0);
                r.launcher2.setPower(0);
            }

            rbButtonWatch = gamepad1.right_bumper;

            if (gamepad2.y) {
                r.particleFlip.setPosition(1);
            }

            else {
                r.particleFlip.setPosition(0);
            }

            /*Thread operating = new Thread () { //includes all robot controls
                public void run() {
                    r.leftMotor1.setPower(gamepad1.left_stick_y);
                    r.leftMotor2.setPower(gamepad1.left_stick_y);
                    r.rightMotor1.setPower(gamepad1.right_stick_y);
                    r.rightMotor2.setPower(gamepad1.right_stick_y);

                    if (gamepad1.dpad_left) {
                        r.intake.setPower(-1);
                    }

                    if (gamepad1.dpad_down) {
                        r.intake.setPower(0);
                    }

                    if (gamepad2.left_bumper && !rbButtonWatch) {
                        r.launcher1.setPower(0.8);
                        r.launcher2.setPower(0.8);
                    } else {
                        r.launcher1.setPower(0);
                        r.launcher2.setPower(0);
                    }

                    rbButtonWatch = gamepad1.right_bumper;

                    if (gamepad2.y) {
                        r.particleFlip.setPosition(1);
                    }

                    else {
                        r.particleFlip.setPosition(0);
                    }
                }
            };

            Thread shooting = new Thread()  { //stops action and shoots 3 particles
                public void run() {
                    if (gamepad2.right_bumper) {
                        runLauncher(0.8);
                        mySleep(2000);
                        r.particleFlip.setPosition(1);
                        mySleep(800);
                        r.particleFlip.setPosition(0);
                        mySleep(1000);
                        r.particleFlip.setPosition(1);
                        mySleep(800);
                        r.particleFlip.setPosition(0);
                    }

                }
            };

            operating.start();
            shooting.start(); */



            // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
            // leftMotor.setPower(-gamepad1.left_stick_y);
            // rightMotor.setPower(-gamepad1.right_stick_y);
        }


    }

    public void stopRobot() {
        r.leftMotor1.setPower(0);
        r.leftMotor2.setPower(0);
        r.rightMotor1.setPower(0);
        r.rightMotor2.setPower(0);

    }

    public void runLauncher(double speed) {
        r.launcher1.setPower(-speed);
        r.launcher2.setPower(speed);
    }

    public void mySleep(long s) {
        runtime.reset();
        while (runtime.milliseconds() < s) {

        }
        return;
    }




}






