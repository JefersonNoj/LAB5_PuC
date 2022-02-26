; Archivo:	mainL5.s
; Dispositivo:	PIC16F887
; Autor:	Jeferson Noj
; Compilador:	pic-as (v2.30), MPLABX V5.40
;
; Programa:	
; Hardware:	LEDs en PORTA
;
; Creado: 21 feb, 2022
; Última modificación:  21 feb, 2022

PROCESSOR 16F887
#include <xc.inc>

; CONFIG1
  CONFIG  FOSC = INTRC_NOCLKOUT ; Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
  CONFIG  WDTE = OFF            ; Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
  CONFIG  PWRTE = ON            ; Power-up Timer Enable bit (PWRT enabled)
  CONFIG  MCLRE = OFF           ; RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
  CONFIG  CP = OFF              ; Code Protection bit (Program memory code protection is disabled)
  CONFIG  CPD = OFF             ; Data Code Protection bit (Data memory code protection is disabled)
  CONFIG  BOREN = OFF           ; Brown Out Reset Selection bits (BOR disabled)
  CONFIG  IESO = OFF            ; Internal External Switchover bit (Internal/External Switchover mode is disabled)
  CONFIG  FCMEN = OFF           ; Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
  CONFIG  LVP = OFF              ; Low Voltage Programming Enable bit (RB3/PGM pin has PGM function, low voltage programming enabled)

; CONFIG2
  CONFIG  BOR4V = BOR40V        ; Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
  CONFIG  WRT = OFF             ; Flash Program Memory Self Write Enable bits (Write protection off)

reset_tmr0 MACRO
    BANKSEL TMR0	    ; Cambiar de banco
    MOVLW   217		    ; 10ms = 4(1/4Mhz)(256-N)(256)
			    ; N = 256 - (10ms*4Mhz)/(4*256) = 217
    MOVWF   TMR0	    ; Configurar tiempo de retardo
    BCF	    T0IF	    ; Limpiar bandera de interrupción
    ENDM

PSECT udata_bank0	    ; Memoria común
  valor:	DS 1	    ; Almacena el valor del PORTA
  banderas:	DS 1	    ; Seleccionador de display
  nibbles:	DS 2	    ; Registro para almacenar nibble bajo y alto (hexadecimal)
  display:	DS 2	    ; Registro para almacenar valor equivalente,  para displays, del nibble bajo y alto
  decimal:	DS 1	    ; Registro temporal para hacer división de centenas 
  dec_temp1:	DS 1	    ; Registro temporal para hacer división de decenas
  dec_temp2:	DS 1	    ; Registro temporal para hacer división de unidades
  unidades:	DS 1	    ; Contador de unidades
  decenas:	DS 1	    ; Contador de decenas
  centenas:	DS 1	    ; Contador de centenas
  disp_dec:	DS 3	    ; Registro para almacenar valores equivalentes (para displays) de unidades, decenas y centenas

PSECT udata_shr		    ; Memoria compartida
  W_TEMP:	DS 1		
  STATUS_TEMP:	DS 1

PSECT resVect, class=CODE, abs, delta=2
;-------- VECTOR RESET ----------
ORG 00h			    ; Posición 0000h para el reset
resetVec:
    PAGESEL main
    GOTO main

PSECT intVect, class=CODE, abs, delta=2
;-------- INTERRUPT VECTOR ----------
ORG 04h			    ; Posición 0004h para interrupciones
push:
    MOVWF   W_TEMP	    ; Mover valor de W a W_TEMP
    SWAPF   STATUS, 0	    ; Intercambiar nibbles de registro STATUS y guardar en W
    MOVWF   STATUS_TEMP	    ; Mover valor de W a STATUS_TEMP
isr: 
    BTFSC   T0IF	    ; Evaluar bandera de interrupción de TMR0
    CALL    int_tmr0	    ; Si ocurre interrupción se llama a la subrutina indicada
    BTFSC   RBIF	    ; Evaluar bandera de interrupción del PORTB
    CALL    int_IocB	    ; Si ocurre interrupción se llama a la subrutina indicada
pop:			   
    SWAPF   STATUS_TEMP,0   ; Intercambiar nibbles de STATUS_TEMP y guardar en W
    MOVWF   STATUS	    ; Mover valor de W a registro STATUS
    SWAPF   W_TEMP, 1	    ; Intercambiar nibbles de W_TEMP y guardar en este mismo registro
    SWAPF   W_TEMP, 0	    ; Intercambiar nibbles de W_TEMP y gardar en W
    RETFIE
;------ Subrutinas de Interrupción -----
int_IocB:
    BTFSS   PORTB, 6	    ; Evaluar boton conectado al pin RB6
    INCF    PORTA	    ; Incrementar PORTA 
    BTFSS   PORTB, 7	    ; Evaluar boton conectado al pin RB7
    DECF    PORTA	    ; Decrementar PORTA
    BCF	    RBIF	    ; Limpiar bandera de interrupción del PORTB
    RETURN

int_tmr0:
    reset_tmr0		    ; Ejecutar macro
    CLRF    PORTD	    ; Limpiar PORTD
    BTFSC   banderas, 2	    ; Evaluar bit 2 del registro banderas
    GOTO    dispC	    ; Saltar a la instrucción indicada
    BTFSS   banderas, 1	    ; Evaluar bit 1 del registro banderas
    GOTO    $+4		    ; Saltar a la cuarta instrucción siguiente
    BTFSC   banderas, 0	    ; Evaluar bit 0 del registro banderas
    GOTO    dispD	    ; Saltar a la instrucción indicada
    GOTO    dispU	    ; Saltar a la instrucción indicada
    BTFSC   banderas, 0	    ; Evaluar bit 0 del registro banderas
    GOTO    disp1	    ; Saltar a la instrución indicada
    ;GOTO    disp0
    disp0:
	MOVF    display, 0	; Mover valor de display a W
	MOVWF   PORTC		; Mover valor de tabla a PORTC
	BSF	PORTD, 1	; Encender display de nibble bajo (hexadecimal)
	MOVLW	1		; Mover literal 1 a W
	MOVWF	banderas	; Mover valor de W a registro banderas
	RETURN

    disp1:
	MOVF    display+1, 0	; Mover display+1 a W
	MOVWF   PORTC		; Mover valor de tabla a PORTC
	BSF	PORTD, 0	; Encender display de nibble alto (hexadecimal)
	MOVLW	2		; Mover literal 2 a W
	MOVWF	banderas	; Mover valor de W a registro banderas
	RETURN

    dispU:
	MOVF    disp_dec, 0	; Mover disp_dec a W
	MOVWF   PORTC		; Mover Valor de tabla a PORTC
	BSF	PORTD, 4	; Encender display de unidades (decimal)
	MOVLW	3		; Mover literal 3 a W
	MOVWF	banderas	; Mover valor de W a registro banderas
	RETURN
    
    dispD:
	MOVF    disp_dec+1, 0	; Mover disp_dec+1 a W
	MOVWF   PORTC		; Mover valor de tabla a PORTC
	BSF	PORTD, 3	; Encender display de las decenas (decimal)
	MOVLW	4		; Mover literal 4 a W
	MOVWF	banderas	; Mover valor de W a registro banderas 
	RETURN

    dispC:
	MOVF    disp_dec+2, 0	; Mover disp_dec+2 a W
	MOVWF   PORTC		; Mover valor de tabla a PORTC
	BSF	PORTD, 2	; Encender display de centenas (decimal)
	MOVLW	0		; Mover literal 0 a W
	MOVWF	banderas	; Mover valor de W a registro banderas
	RETURN

PSECT code, delta=2, abs
ORG 100h		    ; Posición 0100h para el código

;-------- CONFIGURACION --------
main:
    CALL    config_clk	    ; Configuración del reloj
    CALL    config_io	    ; Configuración de entradas y salidas
    CALL    config_tmr0
    CALL    config_IocRB    ; Configuración de interruciones ON-CHANGE
    CALL    config_INT	    ; Configuración de interrupciones
    BANKSEL PORTA

;-------- LOOP RRINCIPAL --------
loop: 
    MOVF   PORTA, W		; Valor del PORTA a W
    MOVWF   valor		; Mover W a registro valor
    CALL    obtener_nibbles	; Guardar nibble alto y bajo del registro valor
    CALL    obtener_UDC		; Obtener valores equivalentes para los display
    CALL    config_displays	; Guardar los valores a enviar en PORTC para mostrar valor en hex
    GOTO    loop		; Saltar al loop principal

;---------- SUBRUTINAS ----------

obtener_nibbles:
    MOVF    valor, 0	    ; Mover registro valor a W
    ANDLW   0x0F	    ; AND entre W y la literal 0x0F para tener nibble bajo
    MOVWF   nibbles	    ; Mover nibble bajo al registro nibbles
    SWAPF   valor, 0	    ; Intercambiar nibbles del registro valor y guardar en W
    ANDLW   0x0F	    ; AND entre W y la literal 0x0F para tener nibble alto
    MOVWF   nibbles+1	    ; Mover nibble alto al registro nibbles+1
    RETURN

obtener_UDC:
    CLRF    centenas	    ; Limpiar registro de las centenas
    MOVF    valor, 0	    ; Mover registro valor a W
    MOVWF   decimal	    ; Mover valor de W a registro decimal 
    MOVF    decimal, 0	    ; Guardar valor de registro decimal en dec_temp1
    MOVWF   dec_temp1
    MOVLW   100		    ; Mover literal 100 a W
    SUBWF   decimal, 1	    ; Restar 100 al registro decimal y guardar en este mismo registro
    BTFSS   STATUS, 0	    ; Evaluar bit de CARRY del registro STATUS
    GOTO    ob_dec	    ; Saltar a la instrucción indicada si ocurrió ovwerflow en el rango
    MOVF    decimal, 0	    ; Guardar valor de registro decimal en dec_temp1
    MOVWF   dec_temp1
    INCF    centenas	    ; Incrementar el registro de centenas 
    GOTO    $-7		    ; Saltar a la séptima instrucción anterior 
    ob_dec:
	CLRF	decenas		; Limpiar registro de la decenas
	MOVF    dec_temp1, 0	; Guardar valor de registro dec_temp1 en dec_temp2
	MOVWF   dec_temp2	
	MOVLW	10		; Mover literal 10 a W
	SUBWF	dec_temp1, 1	; Restar 10 al registro dec_temp1 y guardar en este mismo registro
	BTFSS	STATUS, 0	; Evaluar bit de CARRY del registro STATUS
	GOTO	ob_u		; Saltar a la instrucción indicada si ocurrió overflow en el rango
	MOVF	dec_temp1, 0	; Guardar valor de registro dec_temp1 en dec_temp2 
	MOVWF	dec_temp2
	INCF	decenas		; Incrementear el registro de las decenas
	GOTO	$-7		; Saltar a la séptima instrucción anterior 
	ob_u:
	    CLRF    unidades	    ; Limpiar el registro de las unidades
	    MOVLW   1		    ; Mover literal 1 a W
	    SUBWF   dec_temp2, 1    ; Restar 1 al registro dec_temp2 y guardar en este mismo registro
	    BTFSS   STATUS, 0	    ; Evaluar vit de CARRY del registro STATUS
	    GOTO    $+3		    ; Saltar a la tercera instrucción siguiente si ocurrió overflow en el rango
	    INCF    unidades	    ; Incrementar el registro de las unidades
	    GOTO    $-5		    ; Saltar a la quinta instrucción anterior
	    RETURN

config_displays:
    MOVF    nibbles, 0	    ; Mover registro nibbles a W
    CALL    tabla	    ; Buscar valor de W en la tabla para display
    MOVWF   display	    ; Mover valor equivalente al registro display
    MOVF    nibbles+1, 0    ; Mover registro nibbles+1 a W
    CALL    tabla	    ; Buscar valor de W en la tabla para display
    MOVWF   display+1	    ; Mover valor equivalente al registro display+1

    MOVF    unidades, 0	    ; Mover registro de las unidades a W
    CALL    tabla	    ; Buscar valor de W en la tabla para display
    MOVWF   disp_dec	    ; Mover valor equivalente al registro disp_dec
    MOVF    decenas, 0	    ; Mover registro de las decenas a W
    CALL    tabla	    ; Buscar valor de W en la tabla para display
    MOVWF   disp_dec+1	    ; Mover valor equivalente al registro disp_dec+1
    MOVF    centenas, 0	    ; Mover registro de las centenas a W
    CALL    tabla	    ; Buscar valor de W en la tabla para display
    MOVWF   disp_dec+2	    ; Mover valor equivalente al registro disp_dec+2

    RETURN

config_clk:
    BANKSEL OSCCON
    BSF	    IRCF2	    ; IRCF/110/4MHz (frecuencia de oscilación)
    BSF	    IRCF1
    BCF	    IRCF0
    BSF	    SCS		    ; Reloj interno
    RETURN
   
config_io:
    BANKSEL ANSEL	
    CLRF    ANSEL	    ; I/O digitales
    CLRF    ANSELH
    BANKSEL TRISA
    CLRF    TRISA	    ; PORTA como salida
    CLRF    TRISC	    ; PORTC como salida
    CLRF    TRISD	    ; PORTD como salida
    BSF	    TRISB, 6	    ; RB6 como entrada
    BSF	    TRISB, 7	    ; RB7 como entrada
    BCF	    OPTION_REG, 7   ; Habilitación de Pull-ups en PORTB
    BSF	    WPUB, 6	    ; Habilitar Pull-up para RB6
    BSF	    WPUB, 7	    ; Habilitar Pull-up para RB7
    BANKSEL PORTA
    CLRF    PORTA	    ; Limpiar PORTA
    CLRF    PORTC	    ; Limpiar PORTC
    CLRF    PORTD	    ; Limpiar PORTD
    RETURN

config_tmr0:
    BANKSEL TRISA
    BCF	    T0CS	    ; Selección de reloj interno
    BCF	    PSA		    ; Asignación del Prescaler a TMR0
    BSF	    PS2
    BSF	    PS1
    BSF	    PS0		    ; Prescaler/111/1:256
    reset_tmr0		    
    RETURN 

config_IocRB:
    BANKSEL IOCB	    ; Cambiar a banco 01
    BSF	    IOCB, 6	    ; Habilitar interrupción ON-CHANGE para RB3
    BSF	    IOCB, 7	    ; Hablitiar interrupción ON-CHANGE para RB7
    BANKSEL INTCON	    ; Cambiar a banco 00
    MOVF    PORTB, 0	    ; Mover valor de PORTB a W
    BCF	    RBIF	    ; Limpiar bandera de interrupciones ON-CHANGE
    RETURN

config_INT:
    BANKSEL INTCON	    ; Cambiar a banco 00
    BSF	    GIE		    ; Habilitar interrupciones globales
    BSF	    RBIE	    ; Habilitar interrupciones ON-CHANGE del PORTB
    BCF	    RBIF	    ; Limpiar bandera de interrupciones ON-CHANGE
    BSF	    T0IE	    ; Habilitar interrupciones del TMR0
    BCF	    T0IF	    ; Limpiar bandera de interrupicón del TMR0
    RETURN

ORG 200h		    ; Establecer posición para la tabla
tabla:
    CLRF    PCLATH	    ; Limpiar registro PCLATH
    BSF	    PCLATH, 1	    ; Posicionar PC en 0x02xxh
    ANDLW   0x0F	    ; AND entre W y literal 0x0F
    ADDWF   PCL		    ; ADD entre W y PCL 
    RETLW   00111111B	    ; 0	en 7 seg
    RETLW   00000110B	    ; 1 en 7 seg
    RETLW   01011011B	    ; 2 en 7 seg
    RETLW   01001111B	    ; 3 en 7 seg
    RETLW   01100110B	    ; 4 en 7 seg
    RETLW   01101101B	    ; 5 en 7 seg
    RETLW   01111101B	    ; 6 en 7 seg
    RETLW   00000111B	    ; 7 en 7 seg
    RETLW   01111111B	    ; 8 en 7 seg
    RETLW   01101111B	    ; 9 en 7 seg
    RETLW   01110111B	    ; 10 en 7 seg
    RETLW   01111100B	    ; 11 en 7 seg
    RETLW   00111001B	    ; 12 en 7 seg
    RETLW   01011110B	    ; 13 en 7 seg
    RETLW   01111001B	    ; 14 en 7 seg
    RETLW   01110001B	    ; 15 en 7 seg
