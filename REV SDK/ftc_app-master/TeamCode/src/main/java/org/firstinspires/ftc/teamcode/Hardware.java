package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Thread.sleep;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */

//IRI Hardware Class

public class Hardware
{
    /* Public OpMode members. */
    public DcMotor  leftMotor1  = null;
    public DcMotor  leftMotor2  = null;
    public DcMotor  rightMotor1 = null;
    public DcMotor  rightMotor2 = null;
    public DcMotor  launcher    = null;
    public DcMotor  intake      = null;

    public ColorSensor bottomRight = null;
    public ColorSensor bottomLeft = null;
    public ColorSensor blueBeacon = null;
    public ColorSensor redBeacon = null;

    public Servo    particleGate = null;
    public Servo blueBar = null;
    public Servo bluePress1 = null;
    public Servo bluePress2 = null;
    public Servo redPress1 = null;
    public Servo redPress2 = null;
    public Servo redBar = null;

    public DcMotor leftSet = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public Hardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftMotor1   = hwMap.dcMotor.get("L1");
        leftMotor2   = hwMap.dcMotor.get("L2");
        rightMotor1  = hwMap.dcMotor.get("R1");
        rightMotor2 = hwMap.dcMotor.get("R2");
        launcher    = hwMap.dcMotor.get("Launcher");
        particleGate = hwMap.servo.get("Gate");
        blueBar  = hwMap.servo.get("blueBar");
        redBar = hwMap.servo.get("redBar");
        bluePress1 = hwMap.servo.get("bluePress1");
        bluePress2 = hwMap.servo.get("bluePress2");
        redPress1 = hwMap.servo.get("redPress1");
        redPress2 = hwMap.servo.get("redPress2");

        leftMotor1.setDirection(DcMotor.Direction.REVERSE);
        leftMotor2.setDirection(DcMotor.Direction.REVERSE);// Set to REVERSE if using AndyMark motors
        rightMotor1.setDirection(DcMotor.Direction.FORWARD);
        rightMotor2.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

        bottomLeft.setI2cAddress(I2cAddr.create8bit(0x6c));
        bottomRight.setI2cAddress(I2cAddr.create8bit(0x7c));
        blueBeacon.setI2cAddress(I2cAddr.create8bit(0x4c));
        redBeacon.setI2cAddress(I2cAddr.create8bit(0x5c));



        // Set all motors to zero power
        leftMotor1.setPower(0);
        leftMotor2.setPower(0);
        rightMotor1.setPower(0);
        rightMotor2.setPower(0);
        launcher.setPower(0);

        leftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        blueBar.setPosition(0);
        redBar.setPosition(0);
        bluePress1.setPosition(0);
        bluePress2.setPosition(0);
        redPress1.setPosition(0);
        redPress2.setPosition(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }



    }


