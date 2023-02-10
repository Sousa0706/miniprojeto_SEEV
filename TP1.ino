/*Diogo Lopes Sousa - 2181235
Rúben Conde - 2203927
IPLEIRIA - Instituto Politécnico de Leiria
ESTG - Escola Superior de Tecnologia e Gestão
LEAU- Licenciatura em Engenharia Automóvel
SEEV - Sistemas Elétricos e Eletrónicos de Veículos

TP1: O nosso projeto consiste em fazer atuar um servo motor com acionamento de um potenciómetro. O servo motor simula uma borboleta de admissão de um motor otto e o potenciómetro
o pedal de acelerador de um automóvel, este poderá ser variado entre 0 e 100%. Temos também um botão, neste caso o do ESP32 que ao clicarmos poderemos alterar o modo de sensibilidade
do potenciómetro, entre modo confort e modo sport. O modo e a percentagem do acelerador poderá ser visualizado no display(LCD).

LINK: https://www.youtube.com/watch?v=7QIutIjMJEs&ab_channel=TCSR
*/

#include "Arduino.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_task_wdt.h"
#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>

//OLED display width, in pixels
#define SCREEN_WIDTH 128 
//OLED display height, in pixels
#define SCREEN_HEIGHT 64 

//Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
//The pins for I2C are defined by the Wire-library.
//Reset pin # (or -1 if sharing Arduino reset pin)
#define OLED_RESET     -1
///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
#define SCREEN_ADDRESS 0x3C

//Configuração pinos
#define PIN_LCD 7
#define PIN_ADC 35
#define PIN_SERVO 33
#define PIN_BUTTON 0
#define PIN_LED_1 26
#define PIN_LED_2 27
#define PIN_LED_3 14
#define PIN_LED_4 12

//Resolucao ADC
#define ADC_RESOLUTION 12

//Modos de conducao
#define MODE_CONFORT 0
#define MODE_SPORT 1

//Tasks
void vTaskReadValue(void *pvParameters);
void vTaskDisplay(void *pvParameters);
void vTaskServo(void *pvParameters);
void vTaskBrain(void *pvParameters);
void vTaskChangeMode(void *pvParameters);
void vTaskControlLed(void *pvParameters);

//Queues
QueueHandle_t xQueueADC;
QueueHandle_t xQueueServo;
QueueHandle_t xQueueLCD;
QueueHandle_t xQueueLED;

//Declare a variable of type SemaphoreHandle_t.  This is used to reference the semaphore that is used to synchronize a task with an interrupt. */
static SemaphoreHandle_t xBinarySemaphore;

//Declare a variable of type SemaphoreHandle_t.  This is used to reference the mutex type semaphore that is used to ensure mutual exclusive access to stdout.
SemaphoreHandle_t xMutex;

unsigned int modo = MODE_CONFORT;

//Estrutura que a Queue envia para o LCD
typedef struct xLCDData {
	float percentage;
	int mode;
};

//The service routine for the interrupt.  This is the interrupt that the task will be synchronized with. */
static void IRAM_ATTR vInterruptHandler(void);

/*------------------------------------------------------------------------------------------------*/

void setup(void) {

	//Inicializar a porta serie
	Serial.begin(115200);

	//Confirma se a porta serie está criada
	while (!Serial);

	//Dá maior prioridade a tarefas predefinidas no esp32
	vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);

	//Configuração dos pinos de entrada e saida
	pinMode(PIN_ADC, INPUT);
	pinMode(PIN_BUTTON, INPUT_PULLUP);

	pinMode(PIN_LED_1, OUTPUT);
	pinMode(PIN_LED_2, OUTPUT);
	pinMode(PIN_LED_3, OUTPUT);
	pinMode(PIN_LED_4, OUTPUT);

	attachInterrupt(digitalPinToInterrupt(PIN_BUTTON), &vInterruptHandler,
	FALLING);

	//Criação do semaforo
	vSemaphoreCreateBinary(xBinarySemaphore);

	//Criação do Mutex
	xMutex = xSemaphoreCreateMutex();

	//Configuração do tamanho da Queue
	xQueueADC = xQueueCreate(1, sizeof(unsigned int));
	xQueueServo = xQueueCreate(1, sizeof(float));
	xQueueLCD = xQueueCreate(1, sizeof(xLCDData));
	xQueueLED = xQueueCreate(1, sizeof(float));

	//Configuração das tarefas
	xTaskCreatePinnedToCore(vTaskBrain, "Brain", 2048, NULL, 4, NULL, 1);
	xTaskCreatePinnedToCore(vTaskReadValue, "Read", 2048, NULL, 2, NULL, 1);
	xTaskCreatePinnedToCore(vTaskServo, "Servo", 2048, NULL, 2, NULL, 1);
	xTaskCreatePinnedToCore(vTaskDisplay, "Display", 8192, NULL, 2, NULL, 1);
	xTaskCreatePinnedToCore(vTaskChangeMode, "Mode", 2048, NULL, 2, NULL, 1);
	xTaskCreatePinnedToCore(vTaskControlLed, "LED", 2048, NULL, 2, NULL, 1);

	Serial.println("****************************************");
	Serial.println("*       Diogo Sousa - Ruben Conde      *");
	Serial.println("*  Sistema Controlo Borboleta Admissao *");
	Serial.println("****************************************");
}



/*----------------------------Função de leitura do ADC----------------------------*/

void vTaskReadValue(void *pvParameters) {
	unsigned int analog_adc = 0;

	for (;;) {

		//Variavel que recebe o valor do ADC
		analog_adc = analogRead(PIN_ADC);

		//Queue que envia o valor para a tarefa Brain
		xQueueSend(xQueueADC, &analog_adc, 0);

		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}

/*-------------------------Função de controlo do Servo motor---------------------*/


void vTaskServo(void *pvParameters) {

	//Cria o servo object para o poder controlar
	Servo myServo;

	//Attaches ESP32 pin to the servo object
	myServo.attach(PIN_SERVO);

	float angle = 0;
	portBASE_TYPE xStatus;

	for (;;) {

		//Queue que recebe o valor do angulo da tarefa Brain
		xStatus = xQueueReceive(xQueueServo, &angle, portMAX_DELAY);

		if (xStatus == pdTRUE) {

			//Valor que faz atuar o servo
			myServo.write(angle);
			Serial.print("Servo angle: ");
			Serial.println(angle);

		}
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}


/*--------------------------------Função Brain-----------------------------------------*/
void vTaskBrain(void *pvParameters) {

	TickType_t pxPreviousWakeTime;
	pxPreviousWakeTime = xTaskGetTickCount();
	portBASE_TYPE xStatus;

	unsigned int valor_adc = 0, modo_tmp = 0;
	float angulo = 0.0;
	xLCDData LCDData;


	for (;;) {
		//Queue que recebe o valor do ADC
		xStatus = xQueueReceive(xQueueADC, &valor_adc, portMAX_DELAY);

		if (xStatus == pdPASS) {

			xSemaphoreTake(xMutex, portMAX_DELAY);
			modo_tmp = modo;
			xSemaphoreGive(xMutex);

			if (modo_tmp == MODE_CONFORT) {

				//Conversao do valor de adc para o angulo
				angulo = 100 / (pow(2, ADC_RESOLUTION)) * valor_adc;

				//Queue que manda o angulo
				xQueueSend(xQueueServo, &angulo, 0);

				Serial.print("Servo angle Confort: ");
				Serial.println(angulo);

			}

			else {

				//Conversao para outra sensibilidade
				angulo = 100 / (pow(2, ADC_RESOLUTION)) * valor_adc * 4;

				if (angulo > 99.98) {
					angulo = 99.98;
				}

				Serial.print("Servo angle Sport: ");
				Serial.println(angulo);

				xQueueSend(xQueueServo, &angulo, 0);
			}

			//Dados enviados para a estrutura
			LCDData.mode = modo_tmp;
			LCDData.percentage = angulo * 100 / 100;

			//Queue que envia o angulo para a estrutura usada no display
			xQueueSend(xQueueLCD, &LCDData, 0);
		}

		xQueueSend(xQueueLED, &angulo, 0);
		vTaskDelay(100 / portTICK_PERIOD_MS);

	}
}
/*--------------------------------Função Display-----------------------------------------*/
void vTaskDisplay(void *pvParameters) {

	xLCDData LCDData;
	portBASE_TYPE xStatus;
	Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

	do { 
	    //SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
		Serial.println(F("SSD1306 allocation failed"));
		vTaskDelay(100 / portTICK_PERIOD_MS);
	} 
	while (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS));

	vTaskDelay(10 / portTICK_PERIOD_MS);

	display.clearDisplay();
	display.setTextSize(1);
	display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);

	for (;;) {

		 //Estrutura recebida da queue
		xStatus = xQueueReceive(xQueueLCD, &LCDData, portMAX_DELAY);

		if (xStatus == pdPASS) {

			//Dados escritos no display
			display.setCursor(0, 0);
			display.printf("Percentage: %5.1f%% ", LCDData.percentage);
			display.setCursor(0, 25);

			if (LCDData.mode == MODE_CONFORT) {
				display.printf("Mode: Confort");
			}

			else {
				display.printf("Mode:   Sport");
			}

			display.display();
		}

		vTaskDelay(100 / portTICK_PERIOD_MS);
	}

}
/*----------------------------Interrupção----------------------------------------------*/

//Funçao de rotina da interrupçao, para dar ao semaforo a tarefa de troca de modo
static void IRAM_ATTR vInterruptHandler(void) {
	static signed portBASE_TYPE xHigherPriorityTaskWoken;

	xHigherPriorityTaskWoken = pdFALSE;

	Serial.println("INTERRUPT HANDLER");

	//Dá o semaforo para desbloquear a tarefa
	xSemaphoreGiveFromISR(xBinarySemaphore,(signed portBASE_TYPE*)&xHigherPriorityTaskWoken);

	if (xHigherPriorityTaskWoken == pdTRUE) {
		//Confirma se há uma tarefa de menor prioridade que precise do semaforo
		
		portYIELD_FROM_ISR();
	}
}
/*-------------------------------Função troca de modo-----------------------------------------*/

void vTaskChangeMode(void *pvParameters) {

	unsigned int modo_tmp=0;

	xSemaphoreTake(xBinarySemaphore, 0);

	for (;;) {

		xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);
		xSemaphoreTake(xMutex, portMAX_DELAY);
		modo_tmp = modo;
		xSemaphoreGive(xMutex);

		Serial.println("CHANGE MODE TASK");

		if (modo_tmp == MODE_CONFORT) {
			modo_tmp = MODE_SPORT;
		} else {
			modo_tmp = MODE_CONFORT;
		}

		xSemaphoreTake(xMutex, portMAX_DELAY);

		//Atualizar o modo
		modo = modo_tmp;
		xSemaphoreGive(xMutex);
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}

}
/*------------------------------Funçao controlo dos leds------------------------------------------*/

void vTaskControlLed(void *pvParameters) {
	portBASE_TYPE xStatus;
	float angulo = 0.0;

	for (;;) {

		xStatus = xQueueReceive(xQueueLED, &angulo, portMAX_DELAY);
		if (xStatus == pdTRUE) {

			if (angulo < 5) {
				digitalWrite(PIN_LED_1, LOW);
				digitalWrite(PIN_LED_2, LOW);
				digitalWrite(PIN_LED_3, LOW);
				digitalWrite(PIN_LED_4, LOW);

			} else if (angulo < 45) {
				digitalWrite(PIN_LED_1, HIGH);
				digitalWrite(PIN_LED_2, LOW);
				digitalWrite(PIN_LED_3, LOW);
				digitalWrite(PIN_LED_4, LOW);

			} else if (angulo < 75) {
				digitalWrite(PIN_LED_1, HIGH);
				digitalWrite(PIN_LED_2, HIGH);
				digitalWrite(PIN_LED_3, LOW);
				digitalWrite(PIN_LED_4, LOW);

			} else if (angulo < 90) {
				digitalWrite(PIN_LED_1, HIGH);
				digitalWrite(PIN_LED_2, HIGH);
				digitalWrite(PIN_LED_3, HIGH);
				digitalWrite(PIN_LED_4, LOW);

			} else if (angulo > 90) {
				digitalWrite(PIN_LED_1, HIGH);
				digitalWrite(PIN_LED_2, HIGH);
				digitalWrite(PIN_LED_3, HIGH);
				digitalWrite(PIN_LED_4, HIGH);

			}

			Serial.print("led: ");
			Serial.println(angulo);

		}

		vTaskDelay(100 / portTICK_PERIOD_MS);

	}
}

/*-----------------------------------------------------------------------------------------*/

void loop() {
	vTaskDelete(NULL);
}
