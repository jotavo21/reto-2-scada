MODULE Reto2_Handshake
    !=====================================================================
    ! Reto 2 SCADA - Handshake PLC/SCADA <-> robot por OPC UA (IoT Gateway)
    ! Robot: IRB 140 / IRC5 / RobotWare 6.16 (tarea T_ROB1)
    ! Version 3 (2026-11-10): + paro por ROBOT_ESTOP y N ciclos (nMetaCiclos)
    ! Version 4 (2026-11-11): + contador de vida nVida (ITimer 0,5 s): el puente
    !   verifica que el programa EJECUTA; si nVida no avanza (simulacion o
    !   programa detenido) baja ROBOT_READY en el PLC y no se envia START.
    !   (v3 en RAPID/respaldo/Reto2_Handshake_v3.mod.txt)
    !
    ! Senales del enunciado (variables PERS bool, patron de DTPalletizer):
    !   ROBOT_READY  robot -> SCADA  disponible para una nueva solicitud
    !   ROBOT_START  SCADA -> robot  orden de iniciar la rutina (la escribe Node-RED)
    !   ROBOT_BUSY   robot -> SCADA  ejecutando la rutina
    !   ROBOT_DONE   robot -> SCADA  rutina terminada
    ! Senales propias del reto:
    !   ROBOT_ESTOP  SCADA -> robot  paro por emergencia de la planta (G0 en S2)
    !   bParoActivo  robot -> SCADA  el robot esta detenido por ROBOT_ESTOP
    !   nMetaCiclos  SCADA -> robot  N de repeticiones por cada START (1..5)
    !
    ! Secuencia (handshake de 4 pasos, sin ejecuciones repetidas):
    !   0. Rearme: si START esta en 1 (orden vieja), espera a que baje  [nEstado 4]
    !   1. READY=1 y espera START=1 (flanco 0 -> 1)                     [nEstado 1]
    !   2. READY=0, BUSY=1 -> N x (Path_10 + Path_20)                    [nEstado 2]
    !   3. BUSY=0, DONE=1 -> espera START=0 (START sostenido no repite)  [nEstado 3]
    !   4. DONE=0 -> vuelve al paso 0
    ! Regla: el robot NUNCA escribe ROBOT_START, ROBOT_ESTOP ni nMetaCiclos
    ! (los escribe solo el SCADA/PLC).
    !
    ! ROBOT_ESTOP (paro OPERATIVO por software, NO es una funcion de seguridad:
    ! la parada de emergencia real del IRC5 va cableada a su cadena de seguridad):
    !   ESTOP 0 -> 1: StopMove \Quick, el robot queda quieto en su trayectoria.
    !   ESTOP 1 -> 0: StartMove, retoma donde quedo y termina la rutina normal
    !   (manda DONE como siempre; el PLC no necesita nada especial).
    !   El PLC baja ROBOT_ESTOP con hongo liberado + Reset (G0: S2 -> S3).
    !
    ! N ciclos: al recibir START el robot COPIA nMetaCiclos en nCiclosLote
    ! (redondeado y limitado a 1..5); cambiar nMetaCiclos durante BUSY no
    ! afecta la rutina en curso.
    !
    ! nEstado: 0 iniciando/yendo a Home, 1 READY esperando START,
    !          2 BUSY ejecutando, 3 DONE esperando START=0,
    !          4 START sigue en 1: hay que bajarlo para rearmar
    !
    ! NodeIds en Node-RED (alias del Gateway = robot):
    !   ns=3;s=robot/RAPID/T_ROB1/Reto2_Handshake/<variable>
    !   ROBOT_READY/START/BUSY/DONE/ESTOP (Boolean), bParoActivo (Boolean),
    !   nMetaCiclos, nCiclosLote, nCicloActual, nPath, nEstado, nCiclosRobot, nVida (Float)
    ! IMPORTANTE: despues de cargar o cambiar este modulo, reiniciar el
    ! servidor del IoT Gateway (Server Control) para que publique las variables.
    !=====================================================================

    ! --- Senales de coordinacion (enunciado del Reto 2) ---
    PERS bool ROBOT_READY := FALSE;
    PERS bool ROBOT_START := FALSE;
    PERS bool ROBOT_BUSY := FALSE;
    PERS bool ROBOT_DONE := FALSE;

    ! --- Paro por emergencia de la planta ---
    PERS bool ROBOT_ESTOP := FALSE;
    PERS bool bParoActivo := FALSE;

    ! --- N ciclos por START ---
    CONST num nMIN_CICLOS := 1;
    CONST num nMAX_CICLOS := 5;
    PERS num nMetaCiclos := 1;
    PERS num nCiclosLote := 0;
    PERS num nCicloActual := 0;

    ! Trayectoria en ejecucion: 0 = en Home / quieto, 1..3 = Path_10..Path_30
    PERS num nPath := 0;

    ! Paso del handshake (ver tabla arriba) y rutinas completadas
    PERS num nEstado := 0;
    PERS num nCiclosRobot := 0;

    ! Contador de vida (v4): lo incrementa un TRAP por temporizador cada 0,5 s.
    ! Solo avanza mientras el programa ejecuta: con la simulacion o el
    ! programa detenido se congela, y el puente lo detecta en 3 s.
    PERS num nVida := 0;

    VAR intnum iEstop;
    VAR intnum iVida;

    !---------------------------------------------------------------------
    ! Posicion articular segura de arranque (equivale a Target_Home)
    !---------------------------------------------------------------------
    CONST jointtarget jHome := [[0,0,0,0,90,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];

    !---------------------------------------------------------------------
    ! Targets en wobj0 (base del robot), tool0 apuntando hacia abajo
    ! (orientacion [0,0,1,0]). Valores en mm. Plano de trabajo Z = 400 mm.
    !   Target_Home : sobre el robot, equivale a jHome
    !   Target_10   : al frente
    !   Target_20   : al frente a la izquierda (Y+)
    !   Target_30   : al frente a la derecha (Y-)
    ! Ver/ajustar en RobotStudio: RAPID -> Sincronizar con la estacion.
    !---------------------------------------------------------------------
    CONST robtarget Target_Home := [[450,0,650],[0,0,1,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_10 := [[550,0,400],[0,0,1,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_20 := [[450,200,400],[0,0,1,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_30 := [[350,-200,400],[0,0,1,0],[-1,0,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];

    PROC main()
        ConfJ \Off;
        ConfL \Off;
        TPErase;

        ! Interrupcion por cambio de ROBOT_ESTOP (0 -> 1 y 1 -> 0)
        IDelete iEstop;
        CONNECT iEstop WITH tEstop;
        IPers ROBOT_ESTOP, iEstop;

        ! Contador de vida cada 0,5 s (v4)
        IDelete iVida;
        CONNECT iVida WITH tVida;
        ITimer 0.5, iVida;

        ! Estado inicial: no disponible mientras va a Home
        nEstado := 0;
        ROBOT_READY := FALSE;
        ROBOT_BUSY := FALSE;
        ROBOT_DONE := FALSE;
        nPath := 0;
        nCicloActual := 0;
        bParoActivo := FALSE;

        ! Si el programa arranca con el paro activo, no se mueve hasta que baje
        IF ROBOT_ESTOP THEN
            bParoActivo := TRUE;
            TPWrite "ROBOT_ESTOP activo: esperando Reset de la planta";
            WaitUntil ROBOT_ESTOP = FALSE;
            bParoActivo := FALSE;
        ENDIF
        MoveAbsJ jHome, v200, fine, tool0;

        WHILE TRUE DO
            ! 0. Rearme: si START sigue en 1 (orden anterior o al arrancar),
            !    no se ejecuta: se espera a que baje para exigir un flanco 0 -> 1
            IF ROBOT_START THEN
                nEstado := 4;
                ROBOT_READY := FALSE;
                TPWrite "ROBOT_START sigue en 1: bajelo (START=0) para rearmar";
                WaitUntil ROBOT_START = FALSE;
            ENDIF

            ! 1. Disponible: espera la orden del SCADA
            ROBOT_READY := TRUE;
            nEstado := 1;
            TPWrite "ROBOT_READY: esperando ROBOT_START";
            WaitUntil ROBOT_START = TRUE;

            ! 2. Ocupado: fija N y ejecuta la rutina N veces
            ROBOT_READY := FALSE;
            ROBOT_BUSY := TRUE;
            nEstado := 2;
            rFijarCiclos;
            TPWrite "ROBOT_BUSY: ejecutando rutina, ciclos = "\Num:=nCiclosLote;
            ! Pausa corta para que el Gateway publique BUSY antes de mover
            WaitTime 0.3;
            rRutina;

            ! 3. Terminado: avisa y espera que el SCADA reconozca (START = 0)
            ROBOT_BUSY := FALSE;
            ROBOT_DONE := TRUE;
            nEstado := 3;
            Incr nCiclosRobot;
            TPWrite "ROBOT_DONE: esperando que ROBOT_START baje. Rutinas: "\Num:=nCiclosRobot;
            WaitUntil ROBOT_START = FALSE;

            ! 4. Reconocido: limpia DONE y vuelve al rearme / READY
            ROBOT_DONE := FALSE;
        ENDWHILE
    ENDPROC

    PROC rFijarCiclos()
        ! Copia N al inicio de la rutina: un cambio durante BUSY no la afecta.
        ! Metodo alternativo del curso (desde el FlexPendant):
        ! TPReadNum nCiclosLote, "Numero de ciclos (1 a 5)?";
        nCiclosLote := Round(nMetaCiclos);
        IF nCiclosLote < nMIN_CICLOS THEN
            TPWrite "nMetaCiclos fuera de rango, se usa "\Num:=nMIN_CICLOS;
            nCiclosLote := nMIN_CICLOS;
        ELSEIF nCiclosLote > nMAX_CICLOS THEN
            TPWrite "nMetaCiclos fuera de rango, se usa "\Num:=nMAX_CICLOS;
            nCiclosLote := nMAX_CICLOS;
        ENDIF
    ENDPROC

    PROC rRutina()
        ! Rutina del Reto 2: dos trayectorias, repetidas nCiclosLote veces
        FOR i FROM 1 TO nCiclosLote DO
            nCicloActual := i;
            Path_10;
            Path_20;
        ENDFOR
        nCicloActual := 0;
    ENDPROC

    !---------------------------------------------------------------------
    ! Paro por emergencia de la planta. IPers dispara en cada cambio:
    ! en 1 detiene el movimiento, en 0 lo reanuda en el mismo punto.
    ! Mientras el movimiento esta detenido, el programa queda esperando en
    ! la siguiente instruccion de movimiento (no avanza a DONE).
    !---------------------------------------------------------------------
    TRAP tEstop
        IF ROBOT_ESTOP THEN
            StopMove \Quick;
            bParoActivo := TRUE;
            TPWrite "ROBOT_ESTOP: robot detenido (paro de la planta)";
        ELSE
            bParoActivo := FALSE;
            TPWrite "ROBOT_ESTOP liberado: se reanuda el movimiento";
            StartMove;
        ENDIF
    ENDTRAP

    !---------------------------------------------------------------------
    ! Contador de vida (v4): prueba de que el programa esta ejecutando.
    ! Se reinicia en 0 al llegar a 100000 para no crecer sin limite.
    !---------------------------------------------------------------------
    TRAP tVida
        nVida := nVida + 1;
        IF nVida >= 100000 nVida := 0;
    ENDTRAP

    !---------------------------------------------------------------------
    ! Trayectorias: salen y vuelven a Home. Aproximacion en MoveJ y
    ! desplazamientos entre targets en MoveL (linea recta del TCP).
    ! Path_30 queda disponible pero no forma parte de la rutina.
    !---------------------------------------------------------------------
    PROC Path_10()
        ! Home -> frente -> izquierda -> Home
        nPath := 1;
        MoveJ Target_Home, v200, z10, tool0 \WObj:=wobj0;
        MoveJ Target_10, v200, z10, tool0 \WObj:=wobj0;
        MoveL Target_20, v150, z10, tool0 \WObj:=wobj0;
        MoveJ Target_Home, v200, fine, tool0 \WObj:=wobj0;
        nPath := 0;
    ENDPROC

    PROC Path_20()
        ! Home -> izquierda -> derecha -> Home
        nPath := 2;
        MoveJ Target_Home, v200, z10, tool0 \WObj:=wobj0;
        MoveJ Target_20, v200, z10, tool0 \WObj:=wobj0;
        MoveL Target_30, v150, z10, tool0 \WObj:=wobj0;
        MoveJ Target_Home, v200, fine, tool0 \WObj:=wobj0;
        nPath := 0;
    ENDPROC

    PROC Path_30()
        ! Home -> derecha -> frente -> Home (cierra el triangulo)
        nPath := 3;
        MoveJ Target_Home, v200, z10, tool0 \WObj:=wobj0;
        MoveJ Target_30, v200, z10, tool0 \WObj:=wobj0;
        MoveL Target_10, v150, z10, tool0 \WObj:=wobj0;
        MoveJ Target_Home, v200, fine, tool0 \WObj:=wobj0;
        nPath := 0;
    ENDPROC
ENDMODULE
