package org.firstinspires.ftc.teamcode.HardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * WAGS:  Naming convention is camel case!
 *
 *          front
 *    (LF)--------(RF)
 *    |    robot   |
 *   (LB)--------(RB)
 *        back
 *
 * Motor channel:  Left Front (LF) drive motor:        "leftFront"
 * Motor channel:  Right Front (RF) drive motor:        "rightFront"
 * Motor channel:  Left Back (LB) drive motor:        "leftBack"
 * Motor channel:  Right Back (RB) drive motor:        "rightBack"
 */

public class HardwareMap_Arm1
{
    /* Public OpMode members. */
    public DcMotor  arm1  = null;
    //public DcMotor arm2 = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareMap_Arm1(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        // Save reference to Hardware map
        hwMap = ahwMap;

        arm1 = hwMap.get(DcMotor.class, "arm1");
        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm1.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        //arm2 = hwMap.get(DcMotor.class, "arm2");
        //arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //arm2.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        arm1.setPower(0);
        //arm2.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        arm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //arm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}