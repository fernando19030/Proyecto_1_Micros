; Archivo:  Proyecto_1.s
; Dispositivo:	PIC16F887
; Autor:    Fernando Arribas
; Compilador:	pic-as (v2.31), MPLABX V5.45
; 
; Programa: Contador con interupción del Timer 1 cada segundo y Parpadea cada 250 ms con Timer 2  
; Hardware: 2 displays 7 segmentos en PORTC y una led en PORTD
;
; Creado: 16 mar, 2021
; Ultima modificacion: 02 abr, 2021

PROCESSOR 16F887  ;Definición del procesador a utilizar
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
  CONFIG  LVP = ON              ; Low Voltage Programming Enable bit (RB3/PGM pin has PGM function, low voltage programming enabled)

; CONFIG2
  CONFIG  BOR4V = BOR40V        ; Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
  CONFIG  WRT = OFF             ; Flash Program Memory Self Write Enable bits (Write protection off)

;------------------Macros---------------------
//<editor-fold defaultstate="collapsed" desc="Macros">
 reinicioTMR0 macro
    banksel PORTA   ;tiempo = 4*T_osc*prescaler*(256-TMR0) = 254
    movlw   254	    ;Movemos el valor que el timer debe tener para sumar cada 2ms
    movwf   TMR0    ;Movemos W al registro TMR0
    bcf	    T0IF    ;Limpiamos la bandera de Overflow del Timer 0
    endm

reinicioTMR1 macro
    banksel PORTA   ;tiempo = 4*T_osc*prescaler*(65536-TMR1) = 34286
    movlw   238	    ;Movemos el valor que el timer debe tener para sumar cada 1s
    movwf   TMR1L   ;Movemos W al registro TMR1L
    
    movlw   133
    movwf   TMR1H   ;Movemos W al registro TMR1H
    bcf	    TMR1IF  ;Limpiamos la bandera de Overflow del Timer 1
    endm
    
reinicioTMR2 macro
    banksel TRISA   ;tiempo = 4*T_osc*prescaler*postscaler*PR2 = 245
    movlw   245	    ;Movemos el valor que el timer debe tener para sumar cada 250ms
    movwf   PR2     ;Movemos W al registro PR2
    
    banksel PORTA
    bcf	    TMR2IF    ;Limpiamos la bandera de Overflow del Timer 2
    endm
    //</editor-fold>
;-----------------Variables-------------------
GLOBAL	w_temp, status_temp, flags, display, unidad, decena, tilt
GLOBAL	reseteo, semaforo1, semaforo2, semaforo1_temp
GLOBAL	semaforo3, decimal, contador, semaforo2_temp, semaforo3_temp, t_via
    
//<editor-fold defaultstate="collapsed" desc="Variables">
PSECT udata_shr ;common memory
    w_temp:	    DS 1	;1 byte
    status_temp:    DS 1

PSECT udata_bank0	
    flags:	    DS 1
    modos:	    DS 1	
    display:	    DS 8
    unidad:	    DS 5
    decena:	    DS 5
    reseteo:	    DS 1
    semaforo1:	    DS 1
    semaforo2:	    DS 1
    semaforo3:	    DS 1
    semaforo1_temp: DS 1
    semaforo2_temp: DS 1
    semaforo3_temp: DS 1
    decimal:	    DS 1
    contador:	    DS 1
    t_via:	    DS 3
    tilt:	    DS 1
    control:	    DS 2
    
    //</editor-fold>
;--------Definicion del Vector Reset----------
    
PSECT resVect, class=CODE, abs, delta=2    ;Caracteristicas del vector reset

;---------------vector reset------------------
ORG 00h			    ;Posicion del vector reset
resetVec: 
    PAGESEL main
    goto main
    
;--------------Vector Interrupcion------------
PSECT	intvect, class=CODE, abs, delta=2
org 04h

push:
    movwf   w_temp	    ;Guardar lo que esta en W para no modificarlo
    swapf   STATUS, W	    ;Mover lo que esta en STATUS a W cambiando los nibbles de lugar
    movwf   status_temp	    ;Mover lo que esta en W(STATUS) para evitar modificarlo
    
isr:    
    btfsc   T0IF	    ;Verificar si la bandera de overflow del TMR0 esta activa    
    call    intTMR0	    ;Si esta activa llamar la subrutina
    
    btfsc   TMR1IF	    ;Verificar si la bandera de overflow del TMR1 esta activa    
    call    intTMR1	    ;Si esta activa llamar la subrutina
    
    btfsc   TMR2IF	    ;Verificar si la bandera de overflow del TMR2 esta activa    
    call    intTMR2	    ;Si esta activa llamar la subrutina
pop: 
   swapf    status_temp, W  ;Rotar los nibbles y guardarlo en W
   movwf    STATUS	    ;Mover lo que esta en W a STATUS
   swapf    w_temp, F	    ;Rotar los nibbles y guardarlo en w_temp
   swapf    w_temp, W	    ;Rotarlo de nuevo para no modificar el orden y guardarlo en W
   retfie
   
;--------------subrutinas Interupcion--------
//<editor-fold defaultstate="collapsed" desc="Multiplexado">
intTMR0:
    reinicioTMR0	    ;Reiniciamos el TMR0
    clrf    PORTD	    ;Limpiamos el puerto D
    
    btfsc   flags, 0	    ;Verificamos si la bandera 0 esta prendida
    goto    display2	    ;Si esta prendido nos dirigimos al display 2
    
    btfsc   flags, 1	    ;Verificamos si la bandera 1 esta prendida
    goto    display3	    ;Si esta prendido nos dirigimos al display 3
    
    btfsc   flags, 2	    ;Verificamos si la bandera 2 esta prendida
    goto    display4	    ;Si esta prendido nos dirigimos al display 4
    
    btfsc   flags, 3	    ;Verificamos si la bandera 3 esta prendida
    goto    display5	    ;Si esta prendido nos dirigimos al display 5
    
    btfsc   flags, 4	    ;Verificamos si la bandera 4 esta prendida
    goto    display6	    ;Si esta prendido nos dirigimos al display 6
    
    btfsc   flags, 5	    ;Verificamos si la bandera 4 esta prendida
    goto    display7	    ;Si esta prendido nos dirigimos al display 7
    
    btfsc   flags, 6	    ;Verificamos si la bandera 4 esta prendida
    goto    display8	    ;Si esta prendido nos dirigimos al display 8
    
display1:
    movf    display, W	    ;Movemos lo que esta en la variable a W
    movwf   PORTC	    ;Lo movemos al PORTC
    bsf	    PORTD, 0	    ;Activamos RD0
    goto    next_display2   ;Nos dirigimos prepar las banderas para el siguiente display
   
display2:
    movf    display+1, W    ;Movemos lo que esta en la variable a W
    movwf   PORTC	    ;Lo movemos al PORTC
    bsf	    PORTD, 1	    ;Activamos RD1
    goto    next_display3   ;Nos dirigimos prepar las banderas para el siguiente display
   
display3:
    movf    display+2, W    ;Movemos lo que esta en la variable a W
    movwf   PORTC	    ;Lo movemos al PORTD
    bsf	    PORTD, 2	    ;Activamos RD2
    goto    next_display4   ;Nos dirigimos prepar las banderas para el siguiente display
   
display4:
    movf    display+3, W    ;Movemos lo que esta en la variable a W
    movwf   PORTC	    ;Lo movemos al PORTD
    bsf	    PORTD, 3	    ;Activamos RD3
    goto    next_display5   ;Nos dirigimos prepar las banderas para el siguiente display
   
display5:
    movf    display+4, W    ;Movemos lo que esta en la variable a W
    movwf   PORTC	    ;Lo movemos al PORTD
    bsf	    PORTD, 4	    ;Activamos RD4
    goto    next_display6   ;Nos dirigimos prepar las banderas para el siguiente display
   
display6:
    movf    display+5, W    ;Movemos lo que esta en la variable a W
    movwf   PORTC	    ;Lo movemos al PORTD
    bsf	    PORTD, 5	    ;Activamos RD5
    goto    next_display7   ;Nos dirigimos prepar las banderas para el siguiente display

display7:
    movf    display+6, W    ;Movemos lo que esta en la variable a W
    movwf   PORTC	    ;Lo movemos al PORTD
    bsf	    PORTD, 6	    ;Activamos RD6
    goto    next_display8   ;Nos dirigimos prepar las banderas para el siguiente display
   
display8:
    movf    display+7, W    ;Movemos lo que esta en la variable a W
    movwf   PORTC	    ;Lo movemos al PORTD
    bsf	    PORTD, 7	    ;Activamos RD7
    goto    next_display1   ;Nos dirigimos prepar las banderas para el siguiente display
   
next_display2:
    movlw   1		    ;Movemos 0000 0001B a W
    xorwf   flags, F	    ;Realizamos un XOR entre flags y W
    return
    
next_display3:
    movlw   3		    ;Movemos 0000 0011B a W
    xorwf   flags, F	    ;Realizamos un XOR entre flags y W
    return

next_display4:
    movlw   6		    ;Movemos 0000 0110B a W
    xorwf   flags, F	    ;Realizamos un XOR entre flags y W
    return
    
next_display5:
    movlw   12		    ;Movemos 0000 1100B a W
    xorwf   flags, F	    ;Realizamos un XOR entre flags y W
    return
    
next_display6:
    movlw   24		    ;Movemos 0001 1000B a W
    xorwf   flags, F	    ;Realizamos un XOR entre flags y W
    return
    
next_display7:
    movlw   48		    ;Movemos 0011 0000B a W
    xorwf   flags, F	    ;Realizamos un XOR entre flags y W
    return
    
next_display8:
    movlw   96		    ;Movemos 0110 0000B a W
    xorwf   flags, F	    ;Realizamos un XOR entre flags y W
    return
    
next_display1:
    clrf    flags, F	    ;Limpiamos todas las banderas
    return
    //</editor-fold>
   
//<editor-fold defaultstate="collapsed" desc="Decrementar Semaforos">
intTMR1:
    reinicioTMR1	    ;Reiniciamos el TMR1
    
    btfsc   control+1, 1	;Cuando se entra en el modo 4 y se acepta
    goto    reseteo_semaforos	;Se activa un reset
    
    decf    semaforo1, F    ;Decrementamos el semaforo 1
    
    decf    semaforo2, F    ;Decrementamos el semaforo 2
    
    decf    semaforo3, F    ;Decrementamos el semaforo 3
    
    return
    
reseteo_semaforos:
    decf    reseteo, F	    ;Decrementamos el reset
    return
    //</editor-fold>
    
//<editor-fold defaultstate="collapsed" desc="Parpadeo">
intTMR2:
    reinicioTMR2	    ;Reiniciamos el TMR2
    
    btfsc   tilt, 0	    ;Revisamos si el bit esta encendido
    goto    encendido
    
apagado:
    bsf	    tilt, 0	    ;Encender la bandera 
    return
    
encendido:
    bcf	    tilt, 0	    ;Apagamos la bandera
    return
    //</editor-fold>
;---------------Posicion codigo---------------    
PSECT code, delta=2, abs    ;Caracteristicas del codigo
ORG 100h

;-------------------Tabla---------------------
//<editor-fold defaultstate="collapsed" desc="Tabla">
tabla:
    clrf    PCLATH	    ;Limpiamos PCLATH
    bsf	    PCLATH, 0	    ;Colocamos el PCLATH como 01
    andlw   0x0f	    ;Realizamo un AND entre W y 0x0f para que W solo tenga los 4 bits menos signf
    addwf   PCL, 1	    ;Añadimos W a PCL para que asi el PC = PCLATH + PCL + W
    retlw   00111111B	    ;0
    retlw   00000110B	    ;1
    retlw   01011011B	    ;2
    retlw   01001111B	    ;3
    retlw   01100110B	    ;4
    retlw   01101101B	    ;5
    retlw   01111101B	    ;6
    retlw   00000111B	    ;7
    retlw   01111111B	    ;8
    retlw   01101111B	    ;9
    retlw   01110111B	    ;A
    retlw   01111100B	    ;b
    retlw   00111001B	    ;C
    retlw   01011110B	    ;d
    retlw   01111001B	    ;E
    retlw   01110001B	    ;F
    //</editor-fold>  
;---------------configuracion-----------------
main: 
    call    config_IO	    ;Configuracion de los pines
    call    config_CLK	    ;Configuracion del reloj
    call    config_TMR0	    ;Configuracion del Timer 0
    call    config_TMR1	    ;Condiguracion del Timer 1
    call    config_TMR2	    ;Condiguracion del Timer 2
    call    config_int	    ;Configuracion del las interupciones
 
//<editor-fold defaultstate="collapsed" desc="Valores Iniciales del Semaforo">
valores_iniciales:	    ;Colocamos los valores iniciales de los semaforos
    clrf    semaforo1	    ;Limpiamos todos los semaforos
    clrf    semaforo2
    clrf    semaforo3
    
    movlw   15		    ;Le ingresamos a las variables temporales de los
    movwf   semaforo1_temp  ;modos el valor de 15
    movwf   semaforo2_temp
    movwf   semaforo3_temp
    
    movwf   t_via	    ;Le ingresamos a los tiempos de via el valor de 15
    movwf   t_via+1
    movwf   t_via+2
    
    
    movf    t_via, W	    ;Cargamos el tiempo en los semaforos 1 y 2: 15 seg
    movwf   semaforo1
    movwf   semaforo2
    
    addwf   t_via+1, W	    ;sumamos el tiempo de via 2 al del 1 y lo colocamos
    movwf   semaforo3	    ;en el semaforo 3
    
    bsf	    control, 0	    ;Seteamos la bandera que le indica que el primer 
			    ;semaforo estara en verde
			    //</editor-fold>
 
    banksel PORTA
    
;---------------loop principal---------------
loop:
    btfss   PORTB, 0	    ;Al presionar el boton en RB0 se llama la subrutina
    call    cambio_modo	    ;para cambiar de modo
    
    btfsc   control, 0	    ;verificamos las banderas de control de los semaforos
    call    green1	    ;dependiendo de cual encienda en el estado del semaforo
			    ;verde del semaforo 1
    btfsc   control, 1
    call    green_tilt1	    ;verde titilante del semaforo 1
    
    btfsc   control, 2
    call    yellow1	    ;amarillo del semaforo 1
    
    btfsc   control, 3
    call    green2	    ;verde del semaforo 2
    
    btfsc   control, 4
    call    green_tilt2	    ;verde titilante del semaforo 2
    
    btfsc   control, 5
    call    yellow2	    ;amarillo del semaforo 2
    
    btfsc   control, 6
    call    green3	    ;verde del semaforo 3
    
    btfsc   control, 7
    call    green_tilt3	    ;verde titilante del semaforo 3
    
    btfsc   control+1, 0
    call    yellow3	    ;amarillo del semaforo 3
   
    btfsc   modos, 0	    ;Verificamos el primer bit del modo en el que estamos
    goto    mode_selectB    ;1
    goto    mode_selectA    ;0
    
    
//<editor-fold defaultstate="collapsed" desc="Verficar Modos">
mode_selectA:
    btfsc   modos, 1	    ;Verficamos el segundo bit del modo en el que estamos
    goto    mode_selectD    ;10
    goto    mode_selectC    ;00
    goto    loop
    
mode_selectB:
    btfsc   modos, 1	    ;Verficamos el tercer bit del modo en el que estamos
    goto    mode_selectE    ;11
    goto    mode_selectF    ;01
    goto    loop

mode_selectC:
    btfsc   modos, 2	    ;Verficamos el tercer bit del modo en el que estamos
    goto    modo4	    ;100 Modo 4 
    goto    modo0	    ;000 Modo 0
    goto    loop
    
mode_selectD:
    btfsc   modos, 2	    ;Verficamos el tercer bit del modo en el que estamos
    goto    modox	    ;110 6
    goto    modo2	    ;010 Modo 2
    goto    loop
    
mode_selectE:
    btfsc   modos, 2	    ;Verficamos el tercer bit del modo en el que estamos
    goto    modox	    ;111 9
    goto    modo3	    ;011 Modo 3
    goto    loop
    
mode_selectF:
    btfsc   modos, 2	    ;Verficamos el tercer bit del modo en el que estamos
    goto    modox	    ;101 5
    goto    modo1	    ;001 Modo 1
    goto    loop
    //</editor-fold>
   
//<editor-fold defaultstate="collapsed" desc="Modos de funcionamiento">
modo0:
    bcf	    PORTE, 0	    ;Apagamos todas las leds
    bcf	    PORTE, 1
    bcf	    PORTE, 2
    
    clrf    display+6	    ;Apagamos los displays de configuración
    clrf    display+7
    goto    loop
    
modo1:				;Modificamos el valor del semaforo 1
    movf    semaforo1_temp, W	;Movemos lo que esta en la variable temporal 
    movwf   contador		;a la variabel de modificacion
    bsf	    PORTE, 0		;Encendemos la primera led
    bcf	    PORTE, 1
    bcf	    PORTE, 2
    
    btfss   PORTB, 1		;Al presionar RB1 incrementamos la variable
    call    incremento
    
    btfss   PORTB, 2		;Al presionar RB2 decrementamos la variable
    call    decremento
    
    movf    contador, W		;Despues de modificar la variable la movemos a W
    movwf   semaforo1_temp	;La movemos a la variable temporal del semaforo 1
    movwf   decimal		;y a la variable de conversion a decimal
    
    call    Conversor		;Se convierte a decimal la variable
    
    movf    unidad, W		;Guardamos en la localidad de memoria la unidad
    movwf   unidad+4
    
    movf    decena, W		;Guardamos en la localidad de memoria la decena
    movwf   decena+4
    
    call    dispmode_on		;Configuramos los displays
    
    goto    loop
    
modo2:				;Modificamos el valor del semaforo 2
    movf    semaforo2_temp, W	;Movemos lo que esta en la variable temporal
    movwf   contador		;a la variabel de modificacion
    
    bcf	    PORTE, 0		;Encendemos la segunda led
    bsf	    PORTE, 1
    bcf	    PORTE, 2
    
    btfss   PORTB, 1		;Al presionar RB1 incrementamos la variable
    call    incremento
    
    btfss   PORTB, 2		;Al presionar RB2 decrementamos la variable
    call    decremento
    
    movf    contador, W		;Despues de modificar la variable la movemos a W
    movwf   semaforo2_temp	;La movemos a la variable temporal del semaforo 2
    movwf   decimal		;y a la variable de conversion a decimal
    
    call    Conversor		;Se convierte a decimal la variable
    
    movf    unidad, W		;Guardamos en la localidad de memoria la unidad
    movwf   unidad+4
    
    movf    decena, W		;Guardamos en la localidad de memoria la decena
    movwf   decena+4
    
    call    dispmode_on		;Configuramos los displays
    goto    loop
    
modo3:				;Modificamos el valor del semaforo 3
    movf    semaforo3_temp, W	;Movemos lo que esta en la variable temporal
    movwf   contador		;a la variabel de modificacion
    
    bcf	    PORTE, 0		;Encendemos la tercera led
    bcf	    PORTE, 1
    bsf	    PORTE, 2
    
    btfss   PORTB, 1		;Al presionar RB1 incrementamos la variable
    call    incremento
    
    btfss   PORTB, 2		;Al presionar RB2 decrementamos la variable
    call    decremento
    
    movf    contador, W		;Despues de modificar la variable la movemos a W
    movwf   semaforo3_temp	;La movemos a la variable temporal del semaforo 3
    movwf   decimal		;y a la variable de conversion a decimal
    
    call    Conversor		;Se convierte a decimal la variable
    
    movf    unidad, W		;Guardamos en la localidad de memoria la unidad
    movwf   unidad+4
    
    movf    decena, W		;Guardamos en la localidad de memoria la decena
    movwf   decena+4
    
    call    dispmode_on		;Configuramos los displays
    goto    loop
    
modo4:				;Aceptamos o rechazamos los cambios
    bsf	    PORTE, 0		;Encendemos las tres leds
    bsf	    PORTE, 1
    bsf	    PORTE, 2
    
    movlw   5			;Movemos la literal a la variabel de reseteo
    movwf   reseteo
    
    clrf    display+6		;Limpiamos los displays de configuracion
    clrf    display+7
    
    btfss   PORTB, 1		;Si presionamos RB1 aceptamos los cambios
    goto    acepto
    
    btfss   PORTB, 2		;Si presionamos RB2 rechazamos los cambios
    goto    rechazo
    
    goto    loop  
    
modox:				;Modo arbitrario
    clrf    modos		;Limpiamos la variable de modos
    goto    loop
    
acepto:
    btfss   PORTB, 1		;Antirebote
    goto    $-1
    clrf    modos		;Limpiamos la variable modos
    
    bsf	    PORTA, 0		;Colocamos los semaforos en rojo
    bcf	    PORTA, 1
    bcf	    PORTA, 2
    bsf	    PORTA, 3
    bcf	    PORTA, 4
    bcf	    PORTA, 5
    bsf	    PORTA, 6
    bcf	    PORTA, 7
    bcf	    PORTB, 3
    
    bsf	    control+1, 1	;Activamos la bandera de reseteo
       
    movf    reseteo, W		;Movemos el tiempo de reseteo en W
    movwf   decimal		;La movemos a la variable de conversion
    
    call    Conversor		;Convertimos a decimal
    
    movf    unidad, W		;Movemos la unidad a la variable
    movwf   unidad+4
    
    movf    decena, W		;Movemos la decena a la variable
    movwf   decena+4
    
    call    dispmode_on		;Activamos el display de configuracion
    
    clrf    display		;Apagamos los displays de los semaforos
    clrf    display+1
    
    clrf    display+2
    clrf    display+3
    
    clrf    display+4
    clrf    display+5
    
    movlw   1			
    subwf   reseteo, W		;Restamos la literal de la variable de reseteo
    
    btfsc   STATUS, 2		;Si la resta da 0 cargamos las variables
    goto    carga
    goto    $-31
    
    goto    loop
    
rechazo:
    btfss   PORTB, 2		;Antirebote
    goto    $-1
    clrf    modos		;Limpiamos la variable de modos
    goto    loop
    
carga:
    movf    semaforo1_temp, W	;Trasladamos los nuevos tiempos de via
    movwf   t_via
    
    movf    semaforo2_temp, W
    movwf   t_via+1
    
    movf    semaforo3_temp, W
    movwf   t_via+2
    
    movf    t_via, W		;Colocamos el tiempo de via 1
    movwf   semaforo1		;en el semaforo 1 y 2
    movwf   semaforo2
    
    addwf   t_via+1, W		;sumamos el tiempo de via 1 y 2 y lo guardamos en W
    movwf   semaforo3		;lo movemos al semaforo 3
    
    clrf    control		;Limpiamos la variable de control
    bcf	    control+1, 0
    bcf	    control+1, 1
    
    bsf	    control, 0		;Seteamos la variable que indica que el semaforo 1
    goto    loop		;esta en verde
    //</editor-fold>

;---------------sub rutinas------------------
//<editor-fold defaultstate="collapsed" desc="Puertos">
config_IO:
    banksel ANSEL   ;Nos movemos al banco 3
    clrf    ANSEL   ;I/O Digital
    clrf    ANSELH
   
    banksel TRISA   ;Nos movemos al banco 1  
    clrf    TRISA   ;todas son salidas
    movlw   007h    ;Le indicamos a los primeros 3 puertos de B
    movwf   TRISB   ;que son entradas y el resto salidas
    clrf    TRISC   ;todas son salidas 	   
    clrf    TRISD   ;todos son salidas
    clrf    TRISE
    
    bcf	    OPTION_REG, 7   ;Activar los PullUps internos en el puerto B
    bsf	    WPUB, 0	    ;Habilitar el pullup interno en RB0
    bsf	    WPUB, 1	    ;Habilitar el pullup interno en RB1
    bsf	    WPUB, 2	    ;Habilitar el pullup interno en RB2
      
    banksel PORTA   ;Nos movemos al banco 0  
    clrf    PORTA
    clrf    PORTB   ;Se limpian los puertos B, C ,D y A para que comienzen
    clrf    PORTC   ;en 0
    clrf    PORTD
    clrf    PORTE
    return
    //</editor-fold>
    
//<editor-fold defaultstate="collapsed" desc="Reloj">
config_CLK:
    banksel OSCCON  ;Banco 1
    bsf	    IRCF2   ;Reloj de 1 MHz IRCF = 111
    bcf	    IRCF1
    bcf	    IRCF0
    bsf	    SCS	    ;Reloj Interno
    return
    //</editor-fold>

//<editor-fold defaultstate="collapsed" desc="Interupciones">
config_int:
    banksel PORTA   ;Banco 0
    bsf	    GIE	    ;Habilitar las interupciones globales
    bsf	    PEIE    ;Habilitar las interupciones perifericas
    
    bsf	    T0IE    ;Habilitamos la interupcion por overflow del TMR0
    bcf	    T0IF    ;Limpiar la bandera de overflow del TMR0
    
    banksel TRISA
    bsf	    TMR1IE  ;Habilitamos la interupcion por overflow del TMR1
    bsf	    TMR2IE  ;Habilitamos la interupcion por overflow del TMR2
    
    banksel PORTA
    bcf	    TMR1IF  ;Limpiar la bandera de overflow del TMR1
    bcf	    TMR2IF  ;Limpiar la bandera de overflow del TMR2
    return
    //</editor-fold>

//<editor-fold defaultstate="collapsed" desc="Timer 0">
config_TMR0:
    banksel TRISA   ;Nos movemos al banco 1
    bcf	    T0CS    ;TMR0 usa el reloj interno
    bcf	    PSA	    ;Prescaler es utilizado por el TMR0
    bsf	    PS2
    bsf	    PS1
    bsf	    PS0	    ;Prescaler PS (111) = 1:256	 
    
    banksel PORTA   ;Nos movemos al banco 0
    reinicioTMR0
    return
    //</editor-fold>
   
//<editor-fold defaultstate="collapsed" desc="Timer 1">
config_TMR1:
    banksel PORTA
    bsf	    TMR1ON  ;TMR1 enable
    bcf	    TMR1CS  ;TMR1 clock internal
    bsf	    T1CKPS0 ;Prescaler (11) 1:8
    bsf	    T1CKPS1
    
    banksel PORTA
    reinicioTMR1
    return
    //</editor-fold>
    
//<editor-fold defaultstate="collapsed" desc="Timer 2">
config_TMR2:
    banksel PORTA
    bsf	    T2CKPS1 ;Prescaler (11): 1:16
    bsf	    T2CKPS0
    bsf	    TMR2ON  ;Encendemos TMR2
    bsf	    TOUTPS3 ;Postscaler (1111): 1:16
    bsf	    TOUTPS2
    bsf	    TOUTPS1
    bsf	    TOUTPS0
    
    banksel TRISA
    reinicioTMR2
    return
    //</editor-fold>
   
//<editor-fold defaultstate="collapsed" desc="Displays Semaforos">
display_setup:
    movf    unidad+1, W		;Mover la unidad a W
    call    tabla		;Llamar a tabla
    movwf   display, F		;Mover lo que esta en W al primer byte de la variable
    
    movf    decena+1, W		;Mover la decena a W
    call    tabla		;Llamar a tabla
    movwf   display+1, F	;Mover lo que esta en W al segundo byte de la variable

    movf    unidad+2, W		;Mover la unidad a W
    call    tabla		;Llamar a tabla
    movwf   display+2, F	;Mover lo que esta en W al tercer byte de la variable
    
    movf    decena+2, W		;Mover la decena a W
    call    tabla		;Llamar a tabla
    movwf   display+3, F	;Mover lo que esta en W al cuarto byte de la variable
    
    movf    unidad+3, W		;Mover la unidad a W
    call    tabla		;Llamar a tabla
    movwf   display+4, F	;Mover lo que esta en W al quinto byte de la variable
    
    movf    decena+3, W		;Mover la decena a W
    call    tabla		;Llamar a tabla
    movwf   display+5, F	;Mover lo que esta en W al sexto byte de la variable
    
    return
    //</editor-fold>
   
//<editor-fold defaultstate="collapsed" desc="Display Modos">
dispmode_on:
    movf    unidad+4, W		;Mover la unidad a W
    call    tabla		;Llamar a tabla
    movwf   display+6, F	;Mover lo que esta en W al tercer byte de la variable
    
    movf    decena+4, W		;Mover la decena a W
    call    tabla		;Llamar a tabla
    movwf   display+7, F	;Mover lo que esta en W al cuarto byte de la variable
    
    return//</editor-fold>
    
//<editor-fold defaultstate="collapsed" desc="Conversor a Decimal">
Conversor:
    clrf    decena		;Limpiamos la variable
    movlw   10			;Movemos 10 a W
    subwf   decimal, W		;Restamos W a la variable prueba
    btfsc   STATUS, 0		;Si se pudo hacer la operación haga lo siguiente
    incf    decena		;Incrementar la variable decena
    btfsc   STATUS, 0		;Si se pudo hacer la operación haga lo siguiente
    movwf   decimal		;Mover lo que esta en W a prueba
    btfsc   STATUS, 0		;Si se pudo hacer la operación haga lo siguiente
    goto    $-7			;Regresar 7 casillas hacia atras
    call    unidades		;Nos dirigimos a dividir las unidades
    return
    
unidades:
    clrf    unidad		;Limpiamos la variable
    movlw   1			;Restamos W a la variable prueba
    subwf   decimal, F		;Restamos W a la variable prueba
    btfsc   STATUS, 0		;Si se pudo hacer la operación haga lo siguiente
    incf    unidad		;Incrementar la variable unidad
    btfss   STATUS, 0		;Si no se pudo hacer la operación haga lo siguiente
    return
    goto    $-6			;Regresar 6 casillas hacia atras
    //</editor-fold>
  
//<editor-fold defaultstate="collapsed" desc="ModeChange">
cambio_modo:
    btfss   PORTB, 0	;Antirebote
    goto    $-1
    incf    modos	;Incrementamos la variable de cambio de modos
    return
    //</editor-fold>
   
//<editor-fold defaultstate="collapsed" desc="Incremento/Decremento">
incremento:
    btfss   PORTB, 1	    ;Antirebote
    goto    $-1
    incf    contador	    ;Incrementamos la variable de conteo general
    call    limite_superior ;Llamamos la subrutina que evita el overflow de 20
    
    movf    contador, W	    ;Movemos lo que esta en la variable a W
    return
    
decremento:
    btfss   PORTB, 2	    ;Antirebote
    goto    $-1
    decf    contador	    ;Decrementamos la variable de conteo general
    call    limite_inferior ;Llamamos la subrutina que evita el underflow de 10
    
    movf    contador, W	    ;Movemos lo que esta en la variable a W
    return
    //</editor-fold>
    
//<editor-fold defaultstate="collapsed" desc="Limites">
limite_inferior:
    movlw   9		;Movemos 9 a W
    subwf   contador, W	;Lo restamos del contador
    
    btfsc   STATUS, 2	;Si da 0 ocurre un underflow
    goto    underflow
    
    return
    
underflow:
    movlw   20		;Cargamos 20 en el contador
    movwf   contador
    return
    
limite_superior:
    movlw   21		;Movemos 21 a W
    subwf   contador, W	;Lo restamos del contador
    
    btfsc   STATUS, 2	;Si da 0 ocurre un overflow
    goto    overflow
    
    return
    
overflow:
    movlw   10		;Cargamos 10 en el contador
    movwf   contador
    return
    //</editor-fold>
  
//<editor-fold defaultstate="collapsed" desc="Convertir los Semaforos">
semaforoDecimal:
    movf    semaforo1, W    ;Movemos lo que esta en el semaforo 1 
    movwf   decimal	    ;al conversor decimal
    
    call    Conversor	    ;Convertimos a decimal
    movf    unidad, W	    ;Movemos la unidad a la variable del display 1
    movwf   unidad+1
    
    movf    decena, W	    ;Movemos la decena a la variable del display 2
    movwf   decena+1
    
    movf    semaforo2, W    ;Movemos lo que esta en el semaforo 2
    movwf   decimal	    ;al conversor decimal
    
    call    Conversor	    ;Convertimos a decimal
    movf    unidad, W	    ;Movemos la unidad a la variable del display 3
    movwf   unidad+2
    
    movf    decena, W	    ;Movemos la decena a la variable del display 4
    movwf   decena+2
    
    movf    semaforo3, W    ;Movemos lo que esta en el semaforo 3
    movwf   decimal	    ;al conversor decimal
    
    call    Conversor	    ;Convertimos a decimal
    movf    unidad, W	    ;Movemos la unidad a la variable del display 5
    movwf   unidad+3
    
    movf    decena, W	    ;Movemos la decena a la variable del display 6
    movwf   decena+3
    return
    //</editor-fold>
    
//<editor-fold defaultstate="collapsed" desc="Conteo Semaforos">
green1:			    ;Semaforo 1 en verde
    call    semaforoDecimal ;Convertimos a decimal los tres semaforos
    call    display_setup   ;Desplegamos en los displays
    
    bcf	    PORTB, 3	    ;Colocamos los semaforos 2 y 3 en rojo
    bcf	    PORTA, 0	    ;y el semaforo 1 en verde
    bcf	    PORTA, 7
    bsf	    PORTA, 2
    bsf	    PORTA, 3
    bsf	    PORTA, 6
    
    movlw   6		    ;Restamos 6 del valor del semaforo 2 y lo guardamos en W
    subwf   semaforo2, W
    
    btfsc   STATUS, 2	    ;Si da 0 cambiamos de estado
    call    push_tilt1
    return
    
push_tilt1:
    bcf	    control, 0	    ;Apagamos la bandera
    bsf	    control, 1	    ;Encendemos la bandera
    return
    
green_tilt1:
    call    semaforoDecimal ;Convertimos a decimal los tres semaforos
    call    display_setup   ;Desplegamos en los displays
    
    btfss   tilt, 0	    ;verficamos si la bandera esta apagada
    bcf	    PORTA, 2	    ;Apagamos la luz verde del semaforo 1
    
    btfsc   tilt, 0	    ;verficamos si la bandera esta encendida
    bsf	    PORTA, 2	    ;Encendemos la luz verde del semaforo 1
    
    movlw   3		    ;Restamos 3 del valor del semaforo 2 y lo guardamos en W
    subwf   semaforo2, W
    
    btfsc   STATUS, 2	    ;Si da 0 cambiamos de estado
    call    push_yellow1
    return
    
push_yellow1:
    bcf	    control, 1	    ;Apagamos la bandera
    bsf	    control, 2	    ;Encendemos la bandera
    return
    
yellow1:
    call    semaforoDecimal ;Convertimos a decimal los tres semaforos
    call    display_setup   ;Desplegamos en los displays
    
    bcf	    PORTA, 2	    ;Apagamos la luz verde del semaforo 1
    bsf	    PORTA, 1	    ;y encendemos la luz amarilla
    
    movf    t_via+1, w	    ;Restamos tiempo de via 2 del valor del semaforo 3
    subwf   semaforo3, W    ;y lo guardamos en W
    
    btfsc   STATUS, 2	    ;Si da 0 cambiamos de estado
    call    push_green2
    return
    
push_green2:
    bcf	    control, 2	    ;Apagamos la bandera
    bsf	    control, 3	    ;Encendemos la bandera
    
    movf    t_via+1, W	    ;Colocamos el tiempo de via 2 en el semaforo 2
    movwf   semaforo2
    
    addwf   t_via+2, W	    ;sumamos el tiempo de via 2 y 3 y lo colocamos
    movwf   semaforo1	    ;en el semaforo 1
    return
    
green2:
    call    semaforoDecimal ;Convertimos a decimal los tres semaforos
    call    display_setup   ;Desplegamos en los displays
    
    bcf	    PORTA, 1	    ;Apagamo la luz amarilla del semaforo 1
    bcf	    PORTA, 3	    ;y la luz roja del semaforo 2 para encender la luz
    bsf	    PORTA, 0	    ;verde del mismo y la roja del semaforo 1
    bsf	    PORTA, 5
    
    movlw   6		    ;Restamos 6 del valor del semaforo 2 y lo guardamos en W
    subwf   semaforo3, W
    
    btfsc   STATUS, 2	    ;Si da 0 cambiamos de estado
    call    push_tilt2
    return
    
push_tilt2:
    bcf	    control, 3	    ;Apagamos la bandera
    bsf	    control, 4	    ;Encendemos la bandera
    return
    
green_tilt2:
    call    semaforoDecimal ;Convertimos a decimal los tres semaforos
    call    display_setup   ;Desplegamos en los displays
    
    btfss   tilt, 0	    ;verficamos si la bandera esta apagada
    bcf	    PORTA, 5	    ;Apagamos la luz verde del semaforo 2
    
    btfsc   tilt, 0	    ;verficamos si la bandera esta encendida
    bsf	    PORTA, 5	    ;Encendemos la luz verde del semaforo 2
    
    movlw   3		    ;Restamos 3 del valor del semaforo 3 y lo guardamos en W
    subwf   semaforo3, W
    
    btfsc   STATUS, 2	    ;Si da 0 cambiamos de estado
    call    push_yellow2
    return
    
push_yellow2:
    bcf	    control, 4	    ;Apagamos la bandera
    bsf	    control, 5	    ;Encendemos la bandera
    return
    
yellow2:
    call    semaforoDecimal ;Convertimos a decimal los tres semaforos
    call    display_setup   ;Desplegamos en los displays
    
    bcf	    PORTA, 5	    ;Apagamos la luz verde del semaforo 2
    bsf	    PORTA, 4	    ;y encendemos la luz amarilla
    
    movf    t_via+2, w	    ;Restamos tiempo de via 3 del valor del semaforo 1
    subwf   semaforo1, W    ;y lo guardamos en W
    
    btfsc   STATUS, 2	    ;Si da 0 cambiamos de estado
    call    push_green3
    return
    
push_green3:
    bcf	    control, 5	    ;Apagamos la bandera
    bsf	    control, 6	    ;Encendemos la bandera
    
    movf    t_via+2, W	    ;Colocamos el tiempo de via 3 en el semaforo 3
    movwf   semaforo3
    
    addwf   t_via, W	    ;Sumamos el tiempo de via 1 y 3 y lo colocamos en el 
    movwf   semaforo2	    ;semaforo 2
    return
    
green3:
    call    semaforoDecimal ;Convertimos a decimal los tres semaforos
    call    display_setup   ;Desplegamos en los displays
    
    bcf	    PORTA, 4	    ;Apagamos la luz amarilla del semaforo 2 y la 
    bcf	    PORTA, 6	    ;luz roja del semaforo 3
    bsf	    PORTA, 3	    ;Encendemos la luz verde del semaforo 3 y la roja
    bsf	    PORTB, 3	    ;del semaforo 2
    
    movlw   6		    ;Restamos 6 del valor del semaforo 2 y lo guardamos en W
    subwf   semaforo1, W
    
    btfsc   STATUS, 2	    ;Si da 0 cambiamos de estado
    call    push_tilt3
    return
    
push_tilt3:
    bcf	    control, 6	    ;Apagamos la bandera
    bsf	    control, 7	    ;Encendemos la bandera
    return
    
green_tilt3:
    call    semaforoDecimal ;Convertimos a decimal los tres semaforos
    call    display_setup   ;Desplegamos en los displays
    
    btfss   tilt, 0	    ;verficamos si la bandera esta apagada
    bcf	    PORTB, 3	    ;Apagamos la luz verde del semaforo 3
    
    btfsc   tilt, 0	    ;verficamos si la bandera esta encendida
    bsf	    PORTB, 3	    ;Encendemos la luz verde del semaforo 3
    
    movlw   3		    ;Restamos 3 del valor del semaforo 1 y lo guardamos en W
    subwf   semaforo1, W
    
    btfsc   STATUS, 2	    ;Si da 0 cambiamos de estado
    call    push_yellow3
    return
    
push_yellow3:
    bcf	    control, 7	    ;Apagamos la bandera
    bsf	    control+1, 0    ;Encendemos la bandera
    return
    
yellow3:
    call    semaforoDecimal ;Convertimos a decimal los tres semaforos
    call    display_setup   ;Desplegamos en los displays
    
    bcf	    PORTB, 3	    ;Apagamos la luz verde del semaforo 3 y 
    bsf	    PORTA, 7	    ;encendemos la luz amarilla del semaforo 3
    
    movf    t_via, w	    ;Restamos tiempo de via 1 del valor del semaforo 2
    subwf   semaforo2, W    ;y lo guardamos en W
    
    btfsc   STATUS, 2	    ;Si da 0 cambiamos de estado
    call    push_green1
    return
    
push_green1:
    bcf	    control+1, 0    ;Apagamos la bandera
    bsf	    control, 0	    ;Encendemos la bandera
    
    movf    t_via, W	    ;Colocamos el tiempo de via 1 en el semaforo 1
    movwf   semaforo1	    
    
    addwf   t_via+1, W	    ;Sumamdos el tiempo de via 1 y 2 y lo colocamos en
    movwf   semaforo3	    ;el semaforo 3
    return
    //</editor-fold>
  
END