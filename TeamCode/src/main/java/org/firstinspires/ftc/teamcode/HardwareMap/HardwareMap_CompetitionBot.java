package org.firstinspires.ftc.teamcode.HardwareMap;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 *  * This is NOT an opmode.
 *  *
 *  * This hardware class assumes the following device names have been configured on the robot:
 *  * WAGS:  Naming convention is camel case!
 *  *
 *  *          front
 *  *    (LF)--------(RF)
 *  *    |    robot   |
 *  *   (LB)--------(RB)
 *  *        back
 *  *
 *  * Motor channel:  Left Front (LF) drive motor:        "leftFront"
 *  * Motor channel:  Right Front (RF) drive motor:        "rightFront"
 *  * Motor channel:  Left Back (LB) drive motor:        "leftBack"
 * Motor channel:  Right Back (RB) drive motor:        "rightBack"
 */

public class HardwareMap_CompetitionBot
{
    /* Public OpMode members. */
    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightBack = null;

    //public DcMotor duckMotor = null;

    //The slide system is a DC Motor
    public DcMotor arm1 = null;
    public DcMotor arm2 = null;

    //The wheels for the paper airplane shooter are both servos
    public DcMotor shooter = null; //was CRServo but it was changed to 180 degrees

    //Intake
    public Servo Intake1 = null;

    public Servo Intake2 = null;

    public DcMotor Pivot = null;

    public BNO055IMU imu;

    public final double THRESHOLD = 4;

   //encoder value for levels 1 and 2 of shipping hub
   public int level1 = 1805;
   public int level2 = 4461;
   public int down = 0;

    /* local OpMode members. */
    HardwareMap hwMap =  null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public HardwareMap_CompetitionBot(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        // Wheels
        leftFront = hwMap.get(DcMotor.class, "leftFront");
        rightFront = hwMap.get(DcMotor.class, "rightFront");
        leftBack = hwMap.get(DcMotor.class, "leftBack");
        rightBack = hwMap.get(DcMotor.class, "rightBack");

        // arm
        arm1 = hwMap.get(DcMotor.class, "arm1");
        arm2 = hwMap.get(DcMotor.class, "arm2");

        // Wheels for paper airplane shooter
        shooter = hwMap.get(DcMotor.class, "shooter");

        //Intake
        Intake1 = hwMap.get(Servo.class, "Intake1");

        Intake2 = hwMap.get(Servo.class, "Intake2");

        //Pivot
        Pivot = hwMap.get(DcMotor.class, "Pivot");

        //  OTHER ITEMS
        imu = hwMap.get(BNO055IMU.class, "imu");

        //////////////////////////////////////////////

        /// RESET ALL MOTORS THAT HAVE ENCODER WIRES
        // Wheels
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // arm
        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Shooter
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Pivot
        Pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ////// SETTING DIRECTIONS OF THE MOTOR
        // Wheels
        leftFront.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightFront.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        leftBack.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightBack.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // arm
        arm1.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        arm2.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Wheels for paper airplane shooter
        shooter.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark servos
        //rightWheel.setDirection(CRServo.Direction.FORWARD);// Set to FORWARD if using AndyMark servos

        //Pivot
        Pivot.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark servos

        // Set all motors and servos to power 0

        // Wheels
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        // arm
        arm1.setPower(0);
        arm2.setPower(0);

        // Shooter
        shooter.setPower(0); // --> need if the servo is continuous (we are using motor now)
        //rightWheel.setPower(0); // --> need if the servo is continuous

        // Intake
        //Intake1.setPosition(0);
        //Intake2.setPosition(0);

        // Pivot
        Pivot.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.

        // Wheels
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // arm
        arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // --> just added
        arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // --> just added

        // shooter
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Pivot
        Pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // --> just added

        //IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        //get and initialize IMU
        imu.initialize(parameters);

    }

}