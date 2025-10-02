/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * [Licença e termos omitidos para brevidade]
 */

import com.acmerobotics.dashboard.FtcDashboard;       // Biblioteca para dashboard web
import com.acmerobotics.dashboard.config.Config;       // Permite configuração via dashboard em tempo real
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;  // Base para código sequencial
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;         // Anotação para OpMode teleoperado
import com.qualcomm.robotcore.hardware.DcMotor;                // Classe para controlar motores DC
import com.qualcomm.robotcore.util.ElapsedTime;                // Para medir o tempo de execução
import com.qualcomm.robotcore.util.Range;                      // Para limitar valores (ex: potência entre 0 e 1)

import org.firstinspires.ftc.robotcore.external.Telemetry;    // Telemetria para exibir dados na tela

@Config  // Permite que variáveis públicas estáticas sejam configuradas via dashboard
@TeleOp(name="ProgramacaoMagola", group="Linear OpMode")  // Registro do OpMode com nome e grupo
public class programacao_magola extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime(); // Cronômetro para controlar o tempo do OpMode

    // Motores do lançador
    private DcMotor motorEsquerdoTras;
    private DcMotor motorDireitoTras;

    // Variáveis para controle do lançador
    private boolean atiradorLigado = false;      // Estado atual: ligado ou desligado
    private boolean estadoAnteriorBotaoA = false; // Para controlar o toggle do botão A (evitar múltiplos acionamentos)
    private boolean estadoAtualBotaoA = false;

    // Potência inicial do lançador, agora configurável em tempo real via dashboard
    public static double potenciaInicial = 0.7;

    private double potenciaLanca = 0.0;         // Potência atual do lançador (0 a 1)
    private static final double POTENCIA_MINIMA = 0.0;  // Limite mínimo da potência
    private static final double POTENCIA_MAXIMA = 1.0;  // Limite máximo da potência
    private static final double INCREMENTO = 0.01;      // Incremento para ajustar potência suavemente

    // Instâncias do dashboard e sua telemetria
    private FtcDashboard dashboard;
    private Telemetry telemetryDashboard;

    @Override
    public void runOpMode() {

        telemetry.setMsTransmissionInterval(100); // Atualiza telemetria Driver Station a cada 100ms para melhorar performance

        // Mapeia motores pelo nome configurado no Robot Controller
        motorEsquerdoTras = hardwareMap.get(DcMotor.class, "Esquerdo_tras");
        motorDireitoTras = hardwareMap.get(DcMotor.class, "Direito_tras");

        // Define sentido correto da rotação dos motores para sincronizar movimento
        motorEsquerdoTras.setDirection(DcMotor.Direction.REVERSE);
        motorDireitoTras.setDirection(DcMotor.Direction.FORWARD);

        // Define comportamento para parar os motores rapidamente quando potência for zero
        motorEsquerdoTras.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDireitoTras.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Mostra mensagem inicial para o piloto
        telemetry.addData("Status", "Pronto para iniciar");
        telemetry.update();

        // Inicializa dashboard e obtém telemetria dela para envio de dados
        dashboard = FtcDashboard.getInstance();
        telemetryDashboard = dashboard.getTelemetry();

        waitForStart(); // Espera o sinal para iniciar o OpMode
        runtime.reset(); // Reseta o cronômetro para contar o tempo a partir daqui

        // Loop principal que executa enquanto o OpMode estiver ativo
        while (opModeIsActive()) {
            processarInput();    // Lê controles do gamepad e atualiza lógica do lançador
            atualizarLançador(); // Aplica potência calculada nos motores
            atualizarDashboard();// Envia dados para Driver Station e Dashboard web

            idle();  // Dá tempo para o sistema rodar outras tarefas
        }
    }

    // Método que lê o gamepad e controla o estado do lançador e potência
    private void processarInput() {
        estadoAtualBotaoA = gamepad1.a; // Lê estado do botão A

        // Toggle: alterna ligar/desligar quando botão A for pressionado (rising edge)
        if (estadoAtualBotaoA && !estadoAnteriorBotaoA) {
            atiradorLigado = !atiradorLigado;

            if (atiradorLigado) {
                // Se ligar, usa potência inicial configurada (ajustável via dashboard)
                potenciaLanca = potenciaInicial;
            } else {
                // Se desligar, potência vai para zero
                potenciaLanca = 0.0;
            }
        }

        // Se lançador está ligado, permite ajuste fino da potência usando gatilhos
        if (atiradorLigado) {
            if (gamepad1.right_trigger > 0.1) {
                potenciaLanca += INCREMENTO * gamepad1.right_trigger; // Aumenta potência proporcionalmente ao gatilho direito
            }
            if (gamepad1.left_trigger > 0.1) {
                potenciaLanca -= INCREMENTO * gamepad1.left_trigger;  // Diminui potência proporcionalmente ao gatilho esquerdo
            }

            // Garante que potência fique entre mínimo e máximo definidos
            potenciaLanca = Range.clip(potenciaLanca, POTENCIA_MINIMA, POTENCIA_MAXIMA);
        }

        // Atualiza estado anterior do botão para detectar mudanças no próximo ciclo
        estadoAnteriorBotaoA = estadoAtualBotaoA;
    }

    // Aplica potência calculada nos motores do lançador
    private void atualizarLançador() {
        double power = atiradorLigado ? potenciaLanca : 0.0;

        motorEsquerdoTras.setPower(power);
        motorDireitoTras.setPower(power);
    }

    // Atualiza telemetria para Driver Station e para dashboard web
    private void atualizarDashboard() {
        // Envia dados para o Driver Station (tela do controle)
        telemetry.addData("Tempo", "%.2f s", runtime.seconds()); // Tempo em segundos desde o início
        telemetry.addData("Lançador", atiradorLigado ? "Ligado" : "Desligado"); // Estado do lançador
        telemetry.addData("Potência", String.format("%.2f", potenciaLanca));     // Potência atual do motor
        telemetry.addData("Encoder Esq", motorEsquerdoTras.getCurrentPosition()); // Posição do encoder do motor esquerdo
        telemetry.addData("Encoder Dir", motorDireitoTras.getCurrentPosition());  // Posição do encoder do motor direito
        telemetry.update();

        // Envia os mesmos dados para o dashboard web para visualização remota e ajustes
        telemetryDashboard.addData("Tempo", "%.2f s", runtime.seconds());
        telemetryDashboard.addData("Lançador", atiradorLigado ? "Ligado" : "Desligado");
        telemetryDashboard.addData("Potência", String.format("%.2f", potenciaLanca));
        telemetryDashboard.addData("Encoder Esq", motorEsquerdoTras.getCurrentPosition());
        telemetryDashboard.addData("Encoder Dir", motorDireitoTras.getCurrentPosition());
        telemetryDashboard.update();
    }
}
