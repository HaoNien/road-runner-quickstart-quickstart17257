package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp
public class SlideReset extends robotBase{
    @Override
    protected void robotInit() {
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setPower(0);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    @Override
    protected void robotInitLoop() {

    }

    @Override
    protected void robotStart(){

        slide.setPower(-gamepad2.right_stick_y);

    }
}
