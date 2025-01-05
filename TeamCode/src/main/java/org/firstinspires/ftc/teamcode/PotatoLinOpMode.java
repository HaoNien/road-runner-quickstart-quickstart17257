/* Copyright (c) 2021 FIRST. All rights reserved.
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

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@Config
@TeleOp(name = " PotatoLinOpMode", group = "Linear OpMode")
public class PotatoLinOpMode extends robotBase {
    private final ElapsedTime runtime = new ElapsedTime();


    public static double slide_Speed = 2.5;
    public static double arm_Speed = 5;
    private long lastTime = System.currentTimeMillis();
    private int loopCount = 0;
    int loopCountHZ = 0;
    double clawPos = 0.5;

    // 定義兩個獨立的狀態機
    private enum RobotStateB {
        STATE_1, STATE_2, STATE_3, STATE_4, STATE_5, STATE_6, STATE_7
    }

    private enum RobotStateY {
        STATE_1, STATE_2, STATE_3, STATE_4, STATE_5, STATE_6, STATE_7
    }

    private enum RobotStateX {
        STATE_1, STATE_2, STATE_3, STATE_4, STATE_5, STATE_6, STATE_7, STATE_8
    }

    private enum RobotStateA {
        STATE_1, STATE_2
    }

    boolean togglePressed = false; // 防抖動變數

    private RobotStateB currentStateB = RobotStateB.STATE_7;
    private RobotStateY currentStateY = RobotStateY.STATE_7;

    private RobotStateX currentStateX = RobotStateX.STATE_7;

    private RobotStateA currentStateA = RobotStateA.STATE_1;

    private boolean wasButtonPressedB = false, stateExecutedB = true;
    private boolean wasButtonPressedY = false, stateExecutedY = true;
    private boolean wasButtonPressedX = false, stateExecutedX = true;

    private boolean wasButtonPressedA = false, stateExecutedA = true;


    public void robotInit() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setPower(0);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        armR.setPower(0);
        armL.setPower(0);
        armL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armPosNow = armL.getCurrentPosition() / arm2deg;

        wristToPosition(-90, -90);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // 在 INIT 階段持續維持手臂的角度直到 waitForStart()
        while (!isStarted() && !isStopRequested()) {
            armPosNow = armL.getCurrentPosition() / arm2deg; // 讀取手臂當前角度
            armTurn2angle(45);                       // 將手臂維持在目標角度

            telemetry.addData("Arm Position", armPosNow);
            telemetry.addData("Target Angle", armTarget);
            telemetry.update();
        }
        runtime.reset();
    }

    @Override
    public void robotStart() {
        // run until the end of the match (driver presses STOP)


        while (opModeIsActive()) {
            long currentTime = System.currentTimeMillis();
            loopCount++; // 每次迴圈執行時增加計數器
            armPosNow = armL.getCurrentPosition() / arm2deg;
            slidePosNow = (slide.getCurrentPosition() / slide2lenth) * 2 + smin;


            // 管理 B 按鈕的狀態機
            manageStateMachineB();
            // 管理 Y 按鈕的狀態機
            manageStateMachineY();
            manageStateMachineX();
            manageStateMachineA();

            if (gamepad2.left_bumper) Claw.setPosition(claw_Open);
            if (gamepad2.right_bumper) Claw.setPosition(claw_Close);

            //slide
            double gp2_r_Y = gamepad2.right_stick_y;

            if (gp2_r_Y > 0.1 || gp2_r_Y < -0.1)
                slideTarget = slidePosNow + (-gp2_r_Y * slide_Speed);
            slideTarget = Math.max(Math.min(slideTarget, smax), smin);

            slideToPosition(slideTarget);
            if (gamepad2.left_stick_button && !togglePressed) {
                isHangingMode = !isHangingMode; // 切換模式
                togglePressed = true; // 防抖動
            } else if (!gamepad2.left_stick_button) {
                togglePressed = false; // 重置防抖動
            }
            double gp2_l_Y = gamepad2.left_stick_y;

            if (gp2_l_Y < -0.3 || gp2_l_Y > 0.3) armTarget = armPosNow + (gp2_l_Y * arm_Speed);
            armTarget = Math.max(Math.min(armTarget, armUpLimit), armBottomLimit);

            armTurn2angle(armTarget);

//            if (gamepad2.a) {
//                lift = -90;
//                turn = 0;
//            }
//            if (gamepad2.x) {
//                lift = 0;
//                turn = 0;
//            }

            if (gamepad2.dpad_up) lift += 5;
            else if (gamepad2.dpad_down) lift -= 5;
            lift = Math.max(Math.min(lift, lift_Max), lift_Mini);

            if (gamepad2.dpad_left) turn += 5;
            else if (gamepad2.dpad_right) turn -= 5;
            turn = Math.max(Math.min(turn, turn_Max), turn_Mini);

            wristToPosition(lift, turn);
            double gp1ly = -gamepad1.left_stick_y;
            double gp1lx = -gamepad1.left_stick_x;
            double gp1rx = -gamepad1.right_stick_x;
            double gp1ltr = gamepad1.left_trigger;
            double gp1rtr = gamepad1.right_trigger;

            double axial, lateral, yaw;
            if (Math.abs(gp1ly) < 0.7 && Math.abs(gp1ly) > 0.1) axial = gp1ly / 2;
            else axial = gp1ly;
            if (Math.abs(gp1lx) < 0.7 && Math.abs(gp1lx) > 0.1) lateral = gp1lx / 2;
            else lateral = gp1lx;
            if (Math.abs(gp1rx) < 0.7 && Math.abs(gp1rx) > 0.1)
                yaw = (gp1rx) + (gp1ltr / 3) - (gp1rtr / 3);
            else yaw = gp1rx*1.33 + (gp1ltr / 3) - (gp1rtr / 3);
            //double axial = -gamepad1.left_stick_y;
            //double lateral = -gamepad1.left_stick_x;
            //double yaw = gamepad1.right_stick_x;


            drive.setDrivePower(new Pose2d(axial, lateral, yaw));
            telemetry.addData("Loop Frequency", "%d Hz", loopCountHZ);
            // 每隔一秒計算一次頻率
            if (currentTime - lastTime >= 1000) {
                loopCountHZ = loopCount;


                loopCount = 0; // 重置計數器
                lastTime = currentTime; // 重置時間戳
            }

            //telemetry.addData("pow", drive.getWheelVelocities());

            //telemetry.addData("slide", slide.getCurrentPosition());
            telemetry.addData("slideCM", slidePosNow);
            telemetry.addData("slideTAR", slideTarget);
            telemetry.addData("slidePOW", slidePower);

            telemetry.addData("armNOW", armPosNow);
            telemetry.addData("armTAR", armTarget);
            telemetry.addData("armPOWER", armOutput);
            telemetry.addData("armF", armF);
            telemetry.addData("armAmp", armL.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("armRAmp", armR.getCurrent(CurrentUnit.AMPS));


            telemetry.addData("wristAngL", tarAngleLeft);
            telemetry.addData("wristPosL", tarPositionLeft);
            telemetry.addData("wristAngR", currentAngleRight);
            telemetry.addData("wristRPosR", tarPositionRight);
            telemetry.addData("lift", lift);
            telemetry.addData("turn", turn);


            telemetry.update();
        }//loop

    }//start

    // **B 按鈕狀態機邏輯**
    private void manageStateMachineB() {
        if (gamepad2.b && !wasButtonPressedB) {
            switchStateB();
            wasButtonPressedB = true;
            stateExecutedB = false;
        } else if (!gamepad2.b) {
            wasButtonPressedB = false;
        }

        if (!stateExecutedB) {
            executeStateLogicB();
            stateExecutedB = true;
        }
    }

    private void switchStateB() {
        currentStateY = RobotStateY.STATE_7; // 重置另一個模式的狀態
        currentStateX = RobotStateX.STATE_8; // 重置另一個模式的狀態

        switch (currentStateB) {
            case STATE_1:
                currentStateB = RobotStateB.STATE_2;
                break;
            case STATE_2:
                currentStateB = RobotStateB.STATE_3;
                break;
            case STATE_3:
                currentStateB = RobotStateB.STATE_4;
                break;
            case STATE_4:
                currentStateB = RobotStateB.STATE_6;
                break;
            case STATE_5:
                currentStateB = RobotStateB.STATE_6;
                break;
            case STATE_6:
                currentStateB = RobotStateB.STATE_1;
                break;
            case STATE_7:
                currentStateB = RobotStateB.STATE_1;
                break;
        }
    }

    private void executeStateLogicB() {
        //鉤子模式
        switch (currentStateB) {
            case STATE_1:
                armTarget = 20;
                lift = -20;
                turn = 0;
                slideTarget = 40;
                Claw.setPosition(claw_Open);

                break;
            case STATE_2:
                Claw.setPosition(claw_Close);

                break;
            case STATE_3:
                armTarget = 45;

                break;
            case STATE_4:
                armTarget = 90;
                slideTarget = 52;
                lift = 90;
                break;
            case STATE_5:
                slideTarget = 68;
                break;
            case STATE_6:
                Claw.setPosition(claw_Open);
                break;
            case STATE_7:
                Claw.setPosition(claw_Open);

                break;

        }
    }

    // **Y 按鈕狀態機邏輯**
    private void manageStateMachineY() {
        if (gamepad2.y && !wasButtonPressedY) {
            switchStateY();
            wasButtonPressedY = true;
            stateExecutedY = false;
        } else if (!gamepad2.y) {
            wasButtonPressedY = false;
        }

        if (!stateExecutedY) {
            executeStateLogicY();
            stateExecutedY = true;
        }
    }

    private void switchStateY() {
        currentStateB = RobotStateB.STATE_7; // 重置另一個模式的狀態
        currentStateX = RobotStateX.STATE_8; // 重置另一個模式的狀態

        switch (currentStateY) {
            case STATE_1:
                currentStateY = RobotStateY.STATE_2;
                break;
            case STATE_2:
                currentStateY = RobotStateY.STATE_3;
                break;
            case STATE_3:
                currentStateY = RobotStateY.STATE_4;
                break;
            case STATE_4:
                currentStateY = RobotStateY.STATE_5;
                break;
            case STATE_5:
                currentStateY = RobotStateY.STATE_6;
                break;
            case STATE_6:
                currentStateY = RobotStateY.STATE_7;
                break;
            case STATE_7:
                currentStateY = RobotStateY.STATE_1;
                break;

        }
    }

    private void executeStateLogicY() {

        //高塔模式
        switch (currentStateY) {
            case STATE_1:
                armTarget = 10;
                slideTarget = 70;
                lift = 0;
                turn = 0;
                Claw.setPosition(claw_Open);

                break;
            case STATE_2:
                armTarget = 10;
                lift = -90;
                turn = 0;

                break;
            case STATE_3:
                Claw.setPosition(claw_Close);
                break;
            case STATE_4:
                armTarget = 20;
                slideTarget = 40;
                lift = 0;
                turn = 0;

                break;
            case STATE_5:
                armTarget = 90;
                lift = 20;
                turn = 0;
                slideTarget = smax;
                break;
            case STATE_6:
                Claw.setPosition(claw_Open);
                break;
            case STATE_7:
                armTarget = 25;
                slideTarget = 40;
                lift = 0;
                turn = 0;

                break;
        }
    }

    // **X 按鈕狀態機邏輯**
    private void manageStateMachineX() {
        if (gamepad2.x && !wasButtonPressedX) {
            switchStateX();
            wasButtonPressedX = true;
            stateExecutedX = false;
        } else if (!gamepad2.x) {
            wasButtonPressedX = false;
        }

        if (!stateExecutedX) {
            executeStateLogicX();
            stateExecutedX = true;
        }
    }

    private void switchStateX() {
        currentStateY = RobotStateY.STATE_1; // 重置另一個模式的狀態
        currentStateB = RobotStateB.STATE_1; // 重置另一個模式的狀態

        switch (currentStateX) {
            case STATE_1:
                currentStateX = RobotStateX.STATE_2;
                break;
            case STATE_2:
                currentStateX = RobotStateX.STATE_3;
                break;
            case STATE_3:
                currentStateX = RobotStateX.STATE_4;
                break;
            case STATE_4:
                currentStateX = RobotStateX.STATE_5;
                break;
            case STATE_5:
                currentStateX = RobotStateX.STATE_6;
                break;
            case STATE_6:
                currentStateX = RobotStateX.STATE_7;
                break;
            case STATE_7:
                currentStateX = RobotStateX.STATE_8;
                break;
            case STATE_8:
                currentStateX = RobotStateX.STATE_1;
                break;
        }
    }

    private void executeStateLogicX() {


        switch (currentStateX) {
            case STATE_1:
                slideTarget = 70;
                armTarget = 10;
                lift = -90;
                turn = 0;
                Claw.setPosition(claw_Open);

                break;
            case STATE_2:
                Claw.setPosition(claw_Close);

                break;


            case STATE_3:
                armTarget = 10;
                lift = -10;
                turn = 0;
                slideTarget = 40;
                break;

            case STATE_4:
                armTarget = 20;

                Claw.setPosition(claw_Open);
                break;

            case STATE_5:
                Claw.setPosition(claw_Close);

                break;

            case STATE_6:
                slideTarget = 65;
                armTarget = 50;
                lift=-10;

                break;

            case STATE_7:
                armTarget = 35;

                break;

            case STATE_8:
                Claw.setPosition(claw_Open);


                break;
        }
    }// **A 按鈕狀態機邏輯**

    private void manageStateMachineA() {
        if (gamepad2.a && !wasButtonPressedA) {
            switchStateA();
            wasButtonPressedA = true;
            stateExecutedA = false;
        } else if (!gamepad2.a) {
            wasButtonPressedA = false;
        }

        if (!stateExecutedA) {
            executeStateLogicA();
            stateExecutedA = true;
        }
    }

    private void switchStateA() {


        switch (currentStateA) {
            case STATE_1:
                currentStateA = RobotStateA.STATE_2;
                break;
            case STATE_2:
                currentStateA = RobotStateA.STATE_1;
                break;

        }
    }

    private void executeStateLogicA() {
        currentStateY = RobotStateY.STATE_1; // 重置另一個模式的狀態

        // 地面收集模式
        switch (currentStateA) {
            case STATE_1:
                armTarget = 10;
                slideTarget = 70;
                lift = 0;
                turn = 0;
                Claw.setPosition(claw_Open);
                lift = -90;

                break;

            case STATE_2:
                armTarget = 10;
                slideTarget = 70;
                lift = 0;
                turn = 0;
                Claw.setPosition(claw_Open);
                lift = -90;

                break;


        }
    }


}




