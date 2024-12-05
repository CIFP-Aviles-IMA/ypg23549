#Aquí poneis el Docstring que querais
"""
Este script está creado para manejar un brazo robótico empleando servomotores y un controlador PWM PCA9685.
conectado a una tarjeta Jetson. El brazo robótico está compuesto por múltiples servomotores que permiten el movimiento de las articulaciones del
brazo (hombro, codo, muñeca, base y pinza) y sensores de localización (potenciómetros) que facilitan el manejo de
estos servos en tiempo presente. Asimismo, se añade un botón que, al ser presionado, regula la apertura y el cierre.
de la pinza del brazo automatizado.

Requisitos necesarios:
- Jetson.GPIO: Es el control de los pines GPIO para la placa Jetson.
- adafruit_servokit: Usado para la gestión de servomotores mediante la librería Adafruit.
- adafruit_pca9685: Usado para la comunicación con el controlador PWM PCA9685.
- time: Sirve para introducir los retrasos entre las acciones y configurar el sistema.

Funcionamiento del programa:
-El script ajusta los servos a través del controlador PWM PCA9685, que se encarga de las señales PWM requeridas
 para regular la ubicación de los motores.
-El valor de los potenciómetros se lee mediante los pines GPIO y se asignan a un rango de valores utilizados para 
 modificar el ángulo de los servos en las articulaciones (hombro, codo, muñeca, base).
-El valor del potenciómetro se transforma en un ancho de pulso PWM para regular el movimiento de los servomotores.
- Un pulsador conectado al pin GPIO 29 permite controlar la garra del brazo. Cuando el botón no está pulsado, 
  la garra se cierra, y cuando está pulsado, la garra se abre.

Funciones importanes:
- moveMotor(controlIn, motorOut): Esta funcion lee el valor del potenciometro conectado a un pin GPIO y ajusta el 
  movimiento del motor (servo) según el valor del potenciometro.
- El script repite constantemente las posiciones de los servos de acuerdo a los valores de los potenciómetros 
  y se controla la garra según el estado del pulsador.

Parámetros importantes:
- MIN_PULSE_WIDTH: Este parametro define el ancho de pulso mínimo para el movimiento de los servos (650 microsegundos).
- MAX_PULSE_WIDTH: Este parametro define el ancho de pulso máximo para el movimiento de los servos (2350 microsegundos).
- FREQUENCY: Esta es la frecuencia de actualización de la señal PWM (50 Hz).
"""

#import Wire 
#import Adafruit_PWMServoDriver
import board
import busio
import Jetson.GPIO as GPIO
import adafruit_pca9685
i2c = busio.I2C(board.SCL, board.SDA)
from adafruit_servokit import ServoKit
import time 

#Declaro las variables globales
MIN_PULSE_WIDTH=    650
MAX_PULSE_WIDTH=    2350
FREQUENCY      =    50


#Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
kit = ServoKit(channels=16)


#Configuro el SetUP
time.sleep(5)                           #<-- So I have time to get controller to starting position
pwm.frequency = FREQUENCY
GPIO.setmode(GPIO.BOARD)
hand = adafruit_motor.servo.Servo(0)   #Cualquiera de las dos opciones
wrist = adafruit_motor.servo.Servo(1)
potElbow = adafruit_motor.servo.Servo(2)  #//Assign Potentiometers to pins on Arduino Uno
elbow = adafruit_motor.servo.Servo(3) 
base = adafruit_motor.servo.Servo(4)
potWrist = adafruit_motor.servo.Servo(5)
shoulder = adafruit_motor.servo.Servo(6)                    
potShoulder = adafruit_motor.servo.Servo(7)
potBase = adafruit_motor.servo.Servo(8)

pwm.setPWMFreq(FREQUENCY)
pwm.setPWM(32, 0, 90)                  #Set Gripper to 90 degrees (Close Gripper) X en Jetson
pwm.begin()                            
GPIO.setup(29, GPIO.IN)          


#Asignamos pines
int potWrist    = A3
int potElbow    = A2                    #Assign Potentiometers to pins on Arduino Uno
int potShoulder = A1
int potBase     = A0

int hand      = 11
int wrist     = 12
int elbow     = 13                      #Assign Motors to pins on Servo Driver Board
int shoulder  = 14
int base      = 15


def moveMotor(controlIn, motorOut):
    """
    Funciones relacionadas con la función def MoveMotor(controlIN, motorOUT):

    Argumentos:
      controlIn (int):Se trata del pin GPIO donde se lee el valor del potenciometro el cual se obtiene mediante el pin seleccionado.
      motorOut (int):Se trata del pin de salida del motor.Este pin se utiliza envia la señal PWM al motor controlado.
    
    Returns:
      Esta función no devuelve ningún valor """

    pulse_wide, pulse_width, potVal = -7
  
    #potVal = analogRead(controlIn); (Lenguaje C)                                                  //Read value of Potentiometer
    potVal = GPIO.input(controlIn)
    pulse_wide = map(potVal, 800, 240, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH)
    pulse_width = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);                #Map Potentiometer position to Motor
  
    #pwm.setPWM(motorOut, 0, pulse_width); (Lenguaje C) 
    pwm = GPIO.PWM(motorOut, pulse widht)
 
while (True):
    moveMotor(potWrist, wrist)
    moveMotor(potElbow, elbow)            #Assign Motors to corresponding Potentiometers
    moveMotor(potShoulder, shoulder)
    moveMotor(potBase, base)
    int pushButton = GPIO.input(delcanalquesea)
    if(pushButton == GPIO.LOW):
    pwm.setPWM(hand, 0, 180)                             #Keep Gripper closed when button is not pressed
    print("Grab")
    else:
    pwm.setPWM(hand, 0, 90);                              #Open Gripper when button is pressed
    print("Release")
GPIO.cleanup()