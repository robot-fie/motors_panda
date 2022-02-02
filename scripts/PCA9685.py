#! /usr/bin/python3
# -*- coding: utf-8 -*-
# ------------------------------------------------------------------------#
# File: PACA6885.py                                                       #
# Author: FIE Laboratorio de Robótica Autónoma                            #
# Developers:                                                             #
#       Maceira Coni, Carlos Alberto <cmaceira@fie.undef.edu.ar>          #
#       Muena, Guillermo Ariel       <gmuena@fie.undef.edu.ar>            #
# Brief:                                                                  #
#       Paquete de python que contiene la clase necesaria para configurar #
#       y manejar la placa de Adafruit basada en el PCA9685, un driver de #
#       servos de 16 canales que genera una señal PWM única por cada canal#
#       y que es controlado mediante el bus I2C. Esta clase se aprovecha  #
#       de la librería i2cdev para escribir los registros del microcontro-#
#       lador para generar diferentes salidas PWM.                        #
# ------------------------------------------------------------------------#

import i2cdev
import numpy
import time

# Dirección I2C para del driver PWM
ADDRESS_I2C_DEFAULT = 0x40

# Registros
# MODOS
MODE_1 = 0x00        # Registro de modo 1
MODE_2 = 0x01        # Registro de modo 2

# SUB-DIRECCIONES
SUBADR_1 = 0x02      # Bus I2C sub-dirección 1
SUBADR_2 = 0x03      # Bus I2C sub-dirección 2
SUBADR_3 = 0x04      # Bus I2C sub-dirección 3

# DIRECCCIÓN EN EL BUS I2C DEL REGISTRO ALLCALL
ALLCALLADR = 0x05    # Dirección del registro All Call de los canales

# REGISTROS INDIVIDUALES DEL CANAL 0
LED0_ON_L = 0x06     # Control de habilitación y brillo de la salida canal 0 (byte 0)
LED0_ON_H = 0x07     # Control de habilitación y brillo de la salida canal 0 (byte 1)
LED0_OFF_L = 0x08    # Control de habilitación y brillo de la salida canal 0 (byte 2)
LED0_OFF_H = 0x09    # Control de habilitación y brillo de la salida canal 0 (byte 3)

# REGISTROS DE CONTROL SIMULTÁNEO DE TODOS LOS CANALES
ALL_LED_ON_L = 0xFA  # Control de habilitación y brillo de todos los canales (byte 0)
ALL_LED_ON_H = 0xFB  # Control de habilitación y brillo de todos los canales (byte 1)
ALL_LED_OFF_L = 0xFC # Control de habilitación y brillo de todos los canales (byte 2)
ALL_LED_OFF_H = 0xFD # Control de habilitación y brillo de todos los canales (byte 3)

# REGISTRO DEL PRE-SCALER
PRE_SCALE = 0xFE     # Registro del pre-escaler del oscilador interno del generador de PWM

# REGISTRO DE MODO DE PRUEBA
TEST_MODE = 0xFF     # Registro del modo de prueba

# VALORES ÚTILES DE REGISTROS
# Estos valores permiten setar valores específicos de modo de manera
# sencilla sin tener que recurrir a la hoja de datos y al armado manual
# del valor necesario de un registro.
MODE_1_ALLCALL = 0x01
MODE_1_SUB3 = 0x02
MODE_1_SUB2 = 0x04
MODE_1_SUB1 = 0x08
MODE_1_SLEEP = 0x10
MODE_1_AI = 0x20
MODE_1_EXTCLK = 0x40
MODE_1_RESTART = 0x80

MODE_2_OUTNE = 0x00
MODE_2_OUTDRV = 0x04
MODE_2_OCH = 0x08
MODE_2_INVRT = 0x10

# VALORES DEL GENERADOR DE PULSOS
OSC_VAL = 25000000.0    # Valor del oscilador principal del generado de PWM
PRSCLR_CONST = 4096.0   # Constante para el cálculo del pre-scaler del generador de PWM

# Inicialización del bus I2C
bus = i2cdev.I2C(ADDRESS_I2C_DEFAULT,1 )

class PCA9685():

    def reset_PWM(self):
        # Primero se colocan en 0 todos los bits
        # de los registros que controlan todos los canales en
        # simultáneo.
        bus.write(bytes([ALL_LED_ON_L, 0]))
        bus.write(bytes([ALL_LED_ON_H, 0]))
        bus.write(bytes([ALL_LED_OFF_L, 0]))
        bus.write(bytes([ALL_LED_OFF_H, 0]))

        # El siguiente paso es configurar la topología eléctrica de las
        # salidas. Se colocan todas como una estrucuta totem pole, des-
        # cartando la configuración en estructura open-drain.
        bus.write(bytes([MODE_2, MODE_2_OUTDRV]))

        # Por último se configura el registro Mode 1 para que los canales
        # reaccionen a la dirección ALL-CALL del bus I2C.
        bus.write(bytes([MODE_1, MODE_1_ALLCALL]))

        # Se agrega un delay para que los comandos de escritura puedan tomar
        # efecto en el microcontrolador.
        time.sleep(0.01)

    def set_freq (self,freq_hz):

        # Cálculo del prescaler para setear la frecuencia correcta a partir
        # de la constante de prescaler y la frecuencia pasada como parámetro.
        prescale = int (numpy.floor((OSC_VAL/(PRSCLR_CONST * freq_hz))-1))

        # Se apaga el oscilador principal, llevando a estado de sleep al
        # microcontrolador.
        bus.write(bytes([MODE_1, MODE_1_SLEEP]))

        # Se agrega un delay para que los comandos de escritura puedan tomar
        # efecto en el microcontrolador.
        time.sleep(0.01)

        # Se escribe en el registro del pre-scaler el valor calculado pre-
        # viamente.
        bus.write(bytes([PRE_SCALE, prescale]))

        # Ahora se reinicia el microcontrolador colocando en 1 el bit corres-
        # pondiente del registro MODE 1.
        bus.write(bytes([MODE_1, MODE_1_RESTART]))

        # Se agrega un delay para que los comandos de escritura puedan tomar
        # efecto en el microcontrolador.
        time.sleep(0.01)

        # Es necesario colocar en 0 el bit de reset del registro MODE 1 para evitar
        # que el microntrolador vuelva a reiniciarse indefinidamente.
        bus.write(bytes([MODE_1, 0x00]))

        # Se agrega un delay para que los comandos de escritura puedan tomar
        # efecto en el microcontrolador.
        time.sleep(0.01)

        # Se configura la topología eléctrica de las salidas. Se colocan
        # todas como una estrucuta totem pole, des-
        # cartando la configuración en estructura open-drain.
        bus.write(bytes([MODE_2,MODE_2_OUTDRV]))

        # Se agrega un delay para que los comandos de escritura puedan tomar
        # efecto en el microcontrolador.
        time.sleep(0.01)

    def set_PWM(self, channel, duty):
        x = int((4095 * duty)/100)
        x = min(4095,x)
        x = max(0,x)

        # Los registros LEDn_ON (12 bits utilizables, 4 altos
        # y 8 bajos) son utilizados para determinar el tiempo
        # de delay en el cual la señal PWM comienza con el
        # estado alto. El tiempo de delay se setea de la si-
        # guiente manera: un periodo completo (común para
        # todos los canales, seteado mediante la frecuencia
        # del oscilador), se divide en 4096 'partes'. Este
        # registro indica en cual de esas partes la señal
        # pasa a estado alto. Esto permite que el estado alto
        # de la señal PWM ocurra en cualquier momento del pe-
        # riodo, flexibilizando la implementación (especial-
        # mente si se quisiera hacer que dos salidas estén en
        # cuadratura).
        # Los registros LEDn_OFF funcionan de manera similar.
        # La única diferencia es que estos registros indican
        # en cual de estas 'partes' la señal pasa a estado bajo.

        # Como criterio de diseño y para simplificar el código
        # y la implementacion, adoptamos que la señal de PWM
        # comience en alto el inicio del periodo. Por esta razón
        # se colocan en 0 todos los bits.
        bus.write(bytes([(LED0_ON_L+4*channel), 0x00]))
        bus.write(bytes([(LED0_ON_H+4*channel), 0x00]))

        # Ahora ajustamos el valor de los registros de off, que
        # se deben configurar de acuerdo al duty cycle selecciona-
        # do y pasado como parámetro de esta función. Es necesario
        # implementar máscaras binaraias para poder implementar el
        # valor de manera correcta.
        bus.write(bytes([(LED0_OFF_L+4*channel), (x & 0xFF)]))
        bus.write(bytes([(LED0_OFF_H+4*channel), (x >> 8)]))

        # Nótese que se utilizan los registros 0 como base para
        # obtener el número de registro a partir del canal que se
        # pasa como parámetro.
