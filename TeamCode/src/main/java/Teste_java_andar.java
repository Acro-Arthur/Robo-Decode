/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="TeleOp_Java", group="Linear OpMode")

public class Teste_java_andar extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motor_esquerdo_frente = null;
    private DcMotor motor_esquerdo_tras = null;
    private DcMotor motor_direito_tras = null;
    private DcMotor motor_direito_frente = null;
    private DcMotor motor_medio_1 = null;
    private DcMotor motor_medio_2 = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Iniciado");
        telemetry.update();

        //motor_esquerdo_frente = hardwareMap.get(DcMotor.class, "esquerdo_frente");
        motor_esquerdo_tras  = hardwareMap.get(DcMotor.class, "Esquerdo_tras");
        //motor_direito_frente = hardwareMap.get(DcMotor.class, "direito_frente");
        motor_direito_tras = hardwareMap.get(DcMotor.class, "Direito_tras");
        //motor_medio_1 = hardwareMap.get(DcMotor.class, "Gobilda");
        //motor_medio_2 = hardwareMap.get(DcMotor.class, "Teste_2");


        motor_esquerdo_tras.setDirection(DcMotor.Direction.REVERSE);
        motor_direito_tras.setDirection(DcMotor.Direction.FORWARD);
        //motor_medio_1.setDirection(DcMotor.Direction.FORWARD);
        //motor_medio_2.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            motor_direito_tras.setPower(gamepad1.right_stick_y);
            motor_esquerdo_tras.setPower(gamepad1.right_stick_y);


            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "1 (%.2f)", motor_direito_tras.getPower());
            telemetry.update();
        }
    }
}
