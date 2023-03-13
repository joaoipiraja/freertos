#include <Arduino_FreeRTOS.h>

#include <queue.h>
#include "Nokia_5110.h"
#include <afstandssensor.h>


QueueHandle_t queue_1;

 
//OBJETO DO TIPO Adafruit_PCD8544 COM OS PARÂMETROS (PINOS)
Nokia_5110 lcd = Nokia_5110(3,4,5,6,7);
AfstandsSensor afstandssensor(13, 12);  


void setup() {

  Serial.begin(9600);

  queue_1 = xQueueCreate(5, sizeof(double));

  if (queue_1 == NULL) {

    Serial.println("Queue can not be created");

  }

  xTaskCreate(TaskDisplay, "Display_task", 256, NULL, 2, NULL);

  xTaskCreate(TaskLDR, "LDR_task", 128, NULL, 1, NULL);

  vTaskStartScheduler();

}

void loop() {

}

void TaskDisplay(void * pvParameters) {

  double intensity = 0;


  while(1) {

    Serial.println("TaskDisplay");

    if (xQueueReceive(queue_1, &intensity, portMAX_DELAY) == pdPASS) {
      lcd.println("Distancia");
      lcd.print(intensity);
      lcd.print(" cm");
      lcd.clear();

    }

  }

}

void TaskLDR(void * pvParameters) {

  double current_intensity; 

  while(1) {


    current_intensity = afstandssensor.afstandCM();
    xQueueSend(queue_1, &current_intensity, portMAX_DELAY);
    vTaskDelay( 0.01 / portTICK_PERIOD_MS );

  }

}
