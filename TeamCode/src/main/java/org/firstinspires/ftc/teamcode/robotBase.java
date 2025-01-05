package org.firstinspires.ftc.teamcode;

import androidx.core.content.pm.PermissionInfoCompat;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

@Config
public abstract class robotBase extends LinearOpMode {
    protected SampleMecanumDrive drive;

    protected DcMotorEx slide;
    protected DcMotorEx armR;
    protected DcMotorEx armL;


    //protected CRServo FrontL, FrontR;
    protected Servo FrontL, FrontR;

    protected AnalogInput FrontL_Pos, FrontR_Pos;
    protected Servo Claw;


    private PIDController ArmPID = new PIDController(0, 0, 0);
    private PIDController SlidePID = new PIDController(0, 0, 0);
    private PIDController WristLeftPID = new PIDController(0, 0, 0);
    private PIDController WristRightPID = new PIDController(0, 0, 0);

    /*------------------ARM_PIDF-----------------------*/
    public static double armTarget = 45;
    public static double armP = 0.1;
    public static double armI = 0.1;
    public static double armD = 0.004;
    public static double armF = 0;
    public static double arm_f_coeff = 0.006;
    public static double armOutput;
    public static double armPosNow = 0;

    public static double armUpLimit = 95;

    public static double armBottomLimit = -2;

    public static double armPowerMax = 1;
    public static double armPowerMin = -0.4;

    public static double arm2deg = 6.27;

    /*------------------Slide_PIDF-----------------------*/
    public static double slideP = 1;
    public static double slideI = 0;
    public static double slideD = 0.01;

    public static double slideF = 0;
    public static double slide_f_coeff = 0;

    public static double slide_motorEnc = 103.8;
    public static double slide_Ratio = 1/1.4;

    public static double slide2lenth = slide_motorEnc*slide_Ratio / 0.8;
    public static double smax = 98;
    public static double smax0 = 76;

    public static double smin = 40;
    public double slidePower;

    public static double slidePosNow = 0;

    public double slideTarget = 0;


    /*-----------------wrist----------------------*/
//    public static double wristLP = 0.008;
//    public static double wristLI = 0;
//    public static double wristLD = 0.0002;
//    public static double wristRP = 0.008;
//    public static double wristRI = 0;
//    public static double wristRD = 0.0002;
    public static double gear_ratio = 52 / 18;
    public static double currentAngleLeft;
    public static double currentAngleRight;

    public static double MAX_POWER = 1; // 最大輸出功率
    public static double tarAngleLeft = 0;  // 左側目標角度
    public static double tarAngleRight = 0; // 右側目標角度

    public static double claw_Open = 0.5;

    public static double claw_Close = 0.22;
    public static double lift_Offset = -10, turn_Offset = -15;//turnoff 15
    public static double lift_Max = 90, lift_Mini = -90;
    public static double turn_Max = 90, turn_Mini = -90;
    public static double maxServoAngleLeft = 270.078, maxServoAngleRight = 270.078;
    public static double tarPositionLeft, tarPositionRight;
    public boolean isHangingMode = false; // 預設為普通模式
    Gamepad.RumbleEffect effect = new Gamepad.RumbleEffect.Builder()
            .addStep(1.0, 1.0, 1000)  //  Rumble right motor 100% for 500 mSec
            .addStep(0.0, 0.0, 1000)  //  Pause for 300 mSec

            .build();
    double lift = 0, turn = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SampleMecanumDrive(hardwareMap);

        slide = hardwareMap.get(DcMotorEx.class, "slide");
        armL = hardwareMap.get(DcMotorEx.class, "armL");
        armR = hardwareMap.get(DcMotorEx.class, "armR");


        slide.setDirection(DcMotorSimple.Direction.FORWARD);
        armL.setDirection(DcMotorSimple.Direction.REVERSE);
        armR.setDirection(DcMotorSimple.Direction.FORWARD);
        FrontL = hardwareMap.get(Servo.class, "frontl");
        FrontR = hardwareMap.get(Servo.class, "frontr");

        Claw = hardwareMap.get(Servo.class, "claw");

        //FrontR.setDirection(DcMotorSimple.Direction.REVERSE);

        FrontL_Pos = hardwareMap.get(AnalogInput.class, "lpos");
        FrontR_Pos = hardwareMap.get(AnalogInput.class, "rpos");

        robotInit();
        robotStart();

    }

    protected abstract void robotInit();

    protected abstract void robotStart() throws InterruptedException;


    public void armTurn2angle(double target) {
        target = Math.max(Math.min(target, armUpLimit), armBottomLimit);

        if (isHangingMode) {
            armF = 0; // 關閉F控制
            armP= 0.2;
            if(!gamepad1.isRumbling()) gamepad1.runRumbleEffect(effect);
        } else {
            gamepad1.stopRumble();
            armP= 0.1;

            armF = Math.cos(Math.toRadians(armPosNow)) * slidePosNow * arm_f_coeff;
        }

        ArmPID.setPID(armP, armI, armD);
        armOutput = ArmPID.calculate(armPosNow, target) + armF;

        if (isHangingMode) {
            armOutput = Math.max(Math.min(armOutput, armPowerMax), -armPowerMax); // 移除下降限制
        } else {
            if (armPosNow < 90) armOutput = Math.max(Math.min(armOutput, armPowerMax), armPowerMin);
            else armOutput = Math.min(armOutput, armPowerMax);
        }

        armL.setPower(armOutput);
        armR.setPower(armOutput);
    }

    public void slideToPosition(double slidePos) {
        if (armPosNow < 60)
            slidePos = Math.max(Math.min(slidePos, Math.min(slidePos, smax0)), smin);
        else
            slidePos = Math.max(Math.min(slidePos, smax), smin);


        SlidePID.setPID(slideP, slideI, slideD);
        slidePower = SlidePID.calculate(slidePosNow, slidePos);
        slide.setPower(slidePower);
    }

    public void wristToPosition(double liftAng, double turnAng) {
        turnAng /= gear_ratio;//齒輪比
        turnAng = Math.max(Math.min(turnAng, turn_Max), turn_Mini);
        liftAng = Math.max(Math.min(liftAng, lift_Max), lift_Mini);

        turnAng -= turn_Offset;
        liftAng -= lift_Offset;
        tarAngleLeft = liftAng + (maxServoAngleLeft / 2);
        tarAngleRight = liftAng - turnAng + (maxServoAngleRight / 2);
        tarPositionLeft = tarAngleLeft / maxServoAngleLeft;
        tarPositionRight = tarAngleRight / maxServoAngleRight;

        FrontL.setPosition(1 - tarPositionLeft);
        FrontR.setPosition(tarPositionRight);


    }

    /*public void wristToPosition(double liftAng, double turnAng) {
        turnAng/=2.88;//齒輪比
        turnAng=Math.max(Math.min(turnAng,turn_Max),turn_Mini);
        liftAng=Math.max(Math.min(liftAng,lift_Max),lift_Mini);

        turnAng-=turn_Offset;
        liftAng-=lift_Offset;
        tarAngleLeft = -liftAng + turnAng;
        tarAngleRight = liftAng + turnAng;

        currentAngleLeft = FrontL_Pos.getVoltage() / 3.3 * 360.0 % 360;
        currentAngleRight = FrontR_Pos.getVoltage() / 3.3 * 360.0 % 360;

        double errorLeft = calculateAngleError(tarAngleLeft, currentAngleLeft);
        double errorRight = calculateAngleError(tarAngleRight, currentAngleRight);

        WristLeftPID.setPID(wristLP, wristLI, wristLD);
        WristRightPID.setPID(wristRP, wristRI, wristRD);
        double powerLeft = WristLeftPID.calculate(errorLeft);
        double powerRight = WristRightPID.calculate(errorRight);

        // 限制功率輸出範圍
        powerLeft = Math.max(-MAX_POWER, Math.min(MAX_POWER, powerLeft));
        powerRight = Math.max(-MAX_POWER, Math.min(MAX_POWER, powerRight));

        // 設置馬達功率
        FrontL.setPower(powerLeft);
        FrontR.setPower(powerRight);
    }*/


    private double calculateAngleError(double target, double current) {
        double error = target - current;
        if (error > 180) error -= 360;  // 誤差大於 180 時，取反方向
        if (error < -180) error += 360; // 誤差小於 -180 時，取反方向
        return error;
    }
}
