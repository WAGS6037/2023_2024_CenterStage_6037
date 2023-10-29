package org.firstinspires.ftc.teamcode.HardwareMap;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
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

public class HardwareMap_Intake
{
    /* Public OpMode members. */
    public CRServo Intake  = null;
    //public DcMotor arm2 = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareMap_Intake(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        // Save reference to Hardware map
        hwMap = ahwMap;

        Intake = hwMap.get(CRServo.class, "Intake");
        Intake.setDirection(CRServo.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

    }

}
