package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptGamepadTouchpad;
import org.firstinspires.ftc.teamcode.HardwareMap.HardwareMap_Arm1;
import org.firstinspires.ftc.teamcode.HardwareMap.HardwareMap_CompetitionBot;

@TeleOp
public class ArmTest extends LinearOpMode {
    HardwareMap_CompetitionBot robot = new HardwareMap_CompetitionBot();
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData(">", "Press Start." );
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double rStickX;
            double rStickY;
            double lStickX;
            double targetAngle;
            double mag1;
            double mag2;
            double rotationPower;
            double maxPower;
            double scaleDown;

            //gamepad 1 driving
            rStickX = gamepad2.right_stick_x;
            rStickY = gamepad2.right_stick_y; // this used to be negative
            lStickX = gamepad2.left_stick_x;

            targetAngle = (Math.atan2(rStickY, rStickX));

            rotationPower = -lStickX;
            mag1 = Math.sqrt(Math.pow(rStickX, 2) + Math.pow(rStickY, 2)) * (Math.sin(targetAngle + Math.PI / 4));
            mag2 = Math.sqrt(Math.pow(rStickX, 2) + Math.pow(rStickY, 2)) * (Math.sin(targetAngle - Math.PI / 4));

            maxPower = Math.max(Math.abs(mag1) + Math.abs(rotationPower), Math.abs(mag2) + Math.abs(rotationPower)) + 0.15;
            scaleDown = 1.0;

            if (maxPower > 1)
                scaleDown = 1.0 / maxPower;

            robot.arm1.setPower((mag2 + rotationPower) * scaleDown);
        }

        telemetry.addData(">", "Done");
        telemetry.update();
    }
}
