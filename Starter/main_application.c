// STANDARD INCLUDESS
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "extint.h"
#include "HW_access.h"

// ------------------- DEFINICIJE -------------------
#define COM_CH_0  0 // komande sa PC-a (UniCom)
#define COM_CH_1  1 // senzor udaljenosti (UniCom, 200 ms)
#define COM_CH_2  2 // slanje ka PC-u (opciono)

#define SPEED_TIMER_PERIOD_MS 3000UL // 3 sekunde

#define TASK_SERIAL_REC_PRI   (tskIDLE_PRIORITY + 3)  // Prijem komandi sa PC-a mora biti trenutan i ne sme da kasni pa se ovaj task mora definisati kao task sa najvisim prioritetom 
#define TASK_SERIAL_SEND_PRI  (tskIDLE_PRIORITY + 2)  // Ovaj task mora biti relativno brz ali njihov prijem nije toliko kritican pa moze imati nizi prioritet od prijema
#define TASK_SENSOR_PRI       (tskIDLE_PRIORITY + 2)  // Ovaj task mora biti relativno brz ali njihov prijem nije toliko kritican pa moze imati nizi prioritet od prijema
#define TASK_PROCESS_PRI      (tskIDLE_PRIORITY + 1)
#define TASK_DISPLAY_PRI      (tskIDLE_PRIORITY + 1)
#define TASK_ALARM_PRI        (tskIDLE_PRIORITY + 1)

// ------------------- GLOBALNE PROMENLJIVE -------------------
static SemaphoreHandle_t RXC_BinarySemaphore_0;
static SemaphoreHandle_t RXC_BinarySemaphore_1;
static SemaphoreHandle_t RXC_BinarySemaphore_2;
static SemaphoreHandle_t TBE_BinarySemaphore;
static SemaphoreHandle_t LED_INT_BinarySemaphore = NULL;
static SemaphoreHandle_t display_mutex; // štiti stotine/desetice/jedinice pri upisu

static QueueHandle_t Sensor_Queue;
static QueueHandle_t PC_Queue; // ako želiš da koristiš

// FreeRTOS timer handle
static TimerHandle_t speed_timer_handle = NULL;

// HEX kodovi za 7-seg
static const uint8_t hexnum[] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F};

// ------------------- DISPLAY VARS (protected by display_mutex) -------------------
static uint8_t stotine = 0, desetice = 0, jedinice = 0;

// ------------------- TEMPOMAT VARS (access synchronized where necessary) -------------------
static uint8_t tempomat_on = 0;     // 0=OFF, 1=ON
static uint16_t desired_speed = 0;   // target km/h (set by TEMPOMAT_XX.)
static uint16_t current_speed = 0;  // real (simulated) speed km/h

// latest distance from sensor (cm) — updated in Processing_Task
static uint16_t latest_distance_cm = 0;

// Sada imamo 7 segmenata ukupno (index 0-6)
static uint8_t display_digits[7] = { 0 }; // za 7 segmenata, 0..6

static volatile uint8_t alarm_active = 0;

// ------------------- ISR FUNKCIJE -------------------
static uint32_t prvProcessRXCInterrupt(void) {
    BaseType_t xHigherPTW = pdFALSE;
    if (get_RXC_status(COM_CH_0)) xSemaphoreGiveFromISR(RXC_BinarySemaphore_0, &xHigherPTW);
    if (get_RXC_status(COM_CH_1)) xSemaphoreGiveFromISR(RXC_BinarySemaphore_1, &xHigherPTW);
    if (get_RXC_status(COM_CH_2)) xSemaphoreGiveFromISR(RXC_BinarySemaphore_2, &xHigherPTW);
    portYIELD_FROM_ISR(xHigherPTW);
    return 0;
}

static uint32_t prvProcessTBEInterrupt(void) {
    BaseType_t xHigherPTW = pdFALSE;
    xSemaphoreGiveFromISR(TBE_BinarySemaphore, &xHigherPTW);
    portYIELD_FROM_ISR(xHigherPTW);
    return 0;
}

static uint32_t OnLED_ChangeInterrupt(void)
{
    BaseType_t xHigherPTW = pdFALSE;
    xSemaphoreGiveFromISR(LED_INT_BinarySemaphore, &xHigherPTW);
    portYIELD_FROM_ISR(xHigherPTW);
    return 0;
}

void send_message_COM2(const char* msg) {
    for (size_t i = 0; i < strlen(msg); i++) {
        send_serial_character(COM_CH_2, msg[i]);
        // čekamo da se karakter pošalje
        xSemaphoreTake(TBE_BinarySemaphore, portMAX_DELAY);
    }
}

// ------------------- HELPERS -------------------
static void update_display(uint16_t value) {
    uint16_t desired = tempomat_on ? desired_speed : 999;
    uint16_t current = current_speed;

    if (desired > 999) desired = 999;
    if (current > 999) current = 999;

    // Zaključaj mutex
    if (xSemaphoreTake(display_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        // Podešena brzina na prva tri mesta
        display_digits[0] = desired / 100;
        display_digits[1] = (desired / 10) % 10;
        display_digits[2] = desired % 10;

        // Četvrti segment prazan (isključen)
        display_digits[3] = 255; // recimo 255 znači "prazno"

        // Trenutna brzina na poslednja tri mesta (4,5,6)
        display_digits[4] = current / 100;
        display_digits[5] = (current / 10) % 10;
        display_digits[6] = current % 10;

        xSemaphoreGive(display_mutex);
    }
}

// ------------------- TIMER CALLBACK (svake 3s) -------------------

void vSpeedTimerCallback(TimerHandle_t xTimer) {
    (void)xTimer;

    uint8_t led_status = 0;
    if (get_LED_BAR(0, &led_status) == 0) { // 0 = uspešno
        if ((led_status & 0x07) != 0) {
            if (tempomat_on) {
                tempomat_on = 0;
                desired_speed = 999;
                update_display(desired_speed);
                printf("Tempomat resetovan zbog pritiska papučica (LED bar)\n");
                send_message_COM2("Tempomat OFF.\n");
            }
        }
    }
    uint8_t led_status_2 = 0;
    if (get_LED_BAR(1, &led_status_2) == 0) { // 0 = uspešno
        if ((led_status_2 & 0x01) != 0) {
            if (tempomat_on) {
                tempomat_on = 0;
                desired_speed = 999;
                update_display(desired_speed);
                printf("Tempomat resetovan\n");
                send_message_COM2("Tempomat OFF.\n");
            }
        }
    }
    // Proveravamo uslove za alarm
    static uint16_t last_speed = 0;
    if (tempomat_on) {
        uint16_t dist = latest_distance_cm;

        printf("[Timer] Distanca: %u cm, brzina: %u, željena: %u\n",
            dist, current_speed, desired_speed);

        if (dist < 500 && current_speed < last_speed) {
            alarm_active = 1;  // uključi alarm
        }
        else {
            alarm_active = 0;  // isključi alarm
        }

        last_speed = current_speed;
        if (dist < 500) {
            if (current_speed > 0) {
                current_speed--;
                update_display(current_speed);
                send_message_COM2("Automobil je preblizu\n");
            }
        }
        else {
            if (current_speed < desired_speed) {
                current_speed++;
                update_display(current_speed);
            }
            else if (current_speed > desired_speed) {
                current_speed--;
                update_display(current_speed);
            }
        }
    }
    else {
        alarm_active = 0;  // isključi alarm ako tempomat nije uključen
    }
}

// ------------------- TASKOVI -------------------

// Task koji blinka treći stubac LED bara sa periodom 1000ms ako je alarm aktivan
void Alarm_Task(void* pvParameters) {
    uint8_t led_state = 0;
    while (1) {
        if (alarm_active) {
            led_state = !led_state;
            set_LED_BAR(2, led_state ? 0xFF : 0x00);  // treći stubac, upali ili ugasi sve diode
        }
        else {
            set_LED_BAR(2, 0x00);  // ugasi sve diode trećeg stubca
            led_state = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(500)); // toggla na svakih 500ms => period blinka 1000ms
    }
}

void SerialReceive_Task(void* pvParameters) {
    char buffer[64];
    int idx = 0;
    uint8_t cc;

    memset(buffer, 0, sizeof(buffer));
    idx = 0;

    while (1) {
        // čekamo da ISR signalizira da je nešto stiglo na COM_CH_0
        xSemaphoreTake(RXC_BinarySemaphore_0, portMAX_DELAY);

        // pročitaj barem jedan znak (ISR je signalizirao)
        get_serial_character(COM_CH_0, &cc);

        // neke verzije UniCom šalju CR/LF - prihvatamo i to kao kraj poruke
        if (cc == '.') {
            // kraj poruke -> obradimo buffer (ako nije prazan)
            if (idx > 0) {
                buffer[idx] = '\0';
                // obrada komande
                if (strncmp(buffer, "TEMPOMAT_OFF", 12) == 0) {
                    tempomat_on = 0;
                    desired_speed = 0;
                    // ne menjamo current_speed da ostane trenutna brzina
                    update_display(current_speed);
                    printf("Tempomat OFF\n");
                    send_message_COM2("Tempomat OFF.\n");  // šaljemo obavještenje na PC
                }
                else if (strncmp(buffer, "TEMPOMAT_", 9) == 0) {
                    int speed = 0;
                    for (int i = 9; buffer[i]; i++) {
                        if (buffer[i] >= '0' && buffer[i] <= '9')
                            speed = speed * 10 + (buffer[i] - '0');
                        else break;
                    }
                    if (speed >= 0 && speed <= 200) {
                        desired_speed = (uint8_t)speed;
                        tempomat_on = 1;
                        // ne postavljamo odmah current_speed na desired_speed, nego kroz timer se povećava
                        printf("Tempomat ON, desired speed = %d km/h\n", desired_speed);
                        send_message_COM2("Tempomat ON.\n");  // šaljemo obavještenje na PC
                    }
                    else {
                        printf("Invalid speed: %d\n", speed);
                        send_message_COM2("Invalid speed: %d\n");
                    }
                }
                // resetujemo buffer i indeks da bismo bili spremni za sledeću poruku
                idx = 0;
                memset(buffer, 0, sizeof(buffer));
            }
            else {
                // primili smo samo delimiter, ignorišemo
                idx = 0;
                memset(buffer, 0, sizeof(buffer));
            }

            // --- FLUSH: očistimo sve preostale znakove koji su stigli pre nego što smo stigli da obradimo ---
            // Ako postoji još podataka u RX hardveru, čitamo ih i odbacujemo (ovo sprečava "slepljivanje" poruka)
            while (get_RXC_status(COM_CH_0)) {
                uint8_t tmpc;
                get_serial_character(COM_CH_0, &tmpc);
                // nepotrebno ih skladištimo, samo ih čitamo da "praznimo" UART
            }
        }
        else {
            // normalan znak (nije delimiter) -> dodaj u buffer ako ima mesta
            if (idx < (int)sizeof(buffer) - 2) { // ostavimo mesto za terminator i eventualno
                buffer[idx++] = (char)cc;
                buffer[idx] = '\0';
            }
            else {
                // buffer overflow – resetuj da se ne "zalepi"
                // ispiši upozorenje radi debug-a
                printf("Warning: RX buffer overflow, resetting buffer\n");
                idx = 0;
                memset(buffer, 0, sizeof(buffer));
                // takođe flush hardvera da ne ostane "preostalo"
                while (get_RXC_status(COM_CH_0)) {
                    uint8_t tmpc;
                    get_serial_character(COM_CH_0, &tmpc);
                }
            }            
        } // end else normal char
    } // end WHILE
}

// SENSOR TASK (kanal 1) — prima vrednosti distance u cm svakih 200 ms
void Sensor_Task(void* pvParameters) {
    char buffer[32];
    int idx = 0;
    uint8_t cc;
    memset(buffer, 0, sizeof(buffer));
    idx = 0;

    while (1) {
        // čekamo signal sa ISR da je stigao karakter na COM_CH_1
        if (xSemaphoreTake(RXC_BinarySemaphore_1, portMAX_DELAY) == pdTRUE) {
            get_serial_character(COM_CH_1, &cc);

            if (cc == '.') {
                // kraj poruke
                if (idx > 0) {
                    buffer[idx] = '\0';
                    // očekujemo format "DIST_xx"
                    if (strncmp(buffer, "DIST_", 5) == 0) {
                        int dist = 0;
                        // parsiraj broj iza DIST_
                        for (int i = 5; buffer[i] != '\0'; i++) {
                            if (buffer[i] >= '0' && buffer[i] <= '9') {
                                dist = dist * 10 + (buffer[i] - '0');
                            }
                            else {
                                break;
                            }
                        }
                        if (dist >= 0) { // provera granica distance
                            latest_distance_cm = (uint16_t)dist;
                            printf("Distance updated to %d cm\n", dist);
                        }
                        else {
                            printf("Invalid distance: %d\n", dist);
                            send_message_COM2("Invalid distance: %d\n");
                        }
                    }
                    else {
                        printf("Unknown command on sensor channel: %s\n", buffer);
                        send_message_COM2("Unknown command on sensor channel: %s\n");

                    }

                    // reset buffer
                    idx = 0;
                    memset(buffer, 0, sizeof(buffer));
                }
                else {
                    // primili samo delimiter, ignoriši
                    idx = 0;
                    memset(buffer, 0, sizeof(buffer));
                }
            }
            else {
                // dodaj znak u buffer ako ima mesta
                if (idx < (int)sizeof(buffer) - 1) {
                    buffer[idx++] = (char)cc;
                }
                else {
                    // buffer overflow, resetuj
                    printf("Warning: Sensor RX buffer overflow\n");
                    idx = 0;
                    memset(buffer, 0, sizeof(buffer));
                }
            }
        }
    }
}

// PROCESSING TASK — ažurira latest_distance_cm iz senzora
void Processing_Task(void* pvParameters) {
    uint16_t dist_cm;
    while (1) {
        if (xQueueReceive(Sensor_Queue, &dist_cm, portMAX_DELAY) == pdPASS) {
            // update latest distance (može se koristiti u timer callback-u)
            latest_distance_cm = dist_cm;
        }
    }
}

// DISPLAY TASK — multiplexing 3 cifre, bez vodećih nula
void Display_Task(void* pvParameters) {
    while (1) {
        for (int digit = 0; digit < 7; digit++) {
            select_7seg_digit(digit);
            if (xSemaphoreTake(display_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                if (display_digits[digit] == 255) {
                    set_7seg_digit(0x00); // prazan segment
                }
                else {
                    set_7seg_digit(hexnum[display_digits[digit]]);
                }
                xSemaphoreGive(display_mutex);
            }
            else {
                set_7seg_digit(0x00);
            }
            vTaskDelay(pdMS_TO_TICKS(3)); // manji delay za fluidniji multiplexing
        }
    }
}

// OPTIONAL: Serial send task (neobavezno)
void SerialSend_Task(void* pvParameters) {
    const char poruka[] = "READY";
    uint8_t i = 0;
    while (1) {
        send_serial_character(COM_CH_2, poruka[i++]);
        if (i >= sizeof(poruka) - 1) i = 0;
        xSemaphoreTake(TBE_BinarySemaphore, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ------------------- MAIN DEMO -------------------
void main_demo(void) {
    // inicijalizacija kanala
    init_serial_uplink(COM_CH_0);
    init_serial_downlink(COM_CH_0);
    init_serial_uplink(COM_CH_1);
    init_serial_downlink(COM_CH_1);
    init_serial_uplink(COM_CH_2);
    init_serial_downlink(COM_CH_2);
    
    // inicijalizacija simulatora
    init_7seg_comm();
    init_LED_comm();

    // ISR registracija
    vPortSetInterruptHandler(portINTERRUPT_SRL_RXC, prvProcessRXCInterrupt);  // Povezuje ISR sa prekidom kada UART primi podatak
    vPortSetInterruptHandler(portINTERRUPT_SRL_TBE, prvProcessTBEInterrupt);  // Povezuje ISR sa prekidom kada TBE bude prazan

    // semafori i mutex (Kreiranje semafora)
    RXC_BinarySemaphore_0 = xSemaphoreCreateBinary();
    RXC_BinarySemaphore_1 = xSemaphoreCreateBinary();
    RXC_BinarySemaphore_2 = xSemaphoreCreateBinary();
    TBE_BinarySemaphore = xSemaphoreCreateBinary();
    LED_INT_BinarySemaphore = xSemaphoreCreateBinary();
    display_mutex = xSemaphoreCreateMutex();

    // pocetne vrednosti
    tempomat_on = 0;
    desired_speed = 0;
    current_speed = 50; // postavi početnu brzinu na 50 km/h
    update_display(0);
    latest_distance_cm = 1000;

    // queue (Sensor_Queue šalje uint16_t distances)
    // Prave se FreeROTS redovi koji sluze za razmenu podataka izmedju taskova
    Sensor_Queue = xQueueCreate(10, sizeof(uint16_t));
    PC_Queue = xQueueCreate(10, sizeof(uint8_t)); // opcionalno

    // FreeRTOS tajmer koji periodicno poziva funkciju vSpeedTimerCallBack(svake 3 sekunde)
    speed_timer_handle = xTimerCreate("SpeedTimer", pdMS_TO_TICKS(SPEED_TIMER_PERIOD_MS), pdTRUE, NULL, vSpeedTimerCallback);
    if (speed_timer_handle != NULL) {
        xTimerStart(speed_timer_handle, 0);
    }

    // kreiranje taskova (Funkcija taska, Ime taska, velicina memorije, Parametar, Prioritet, Handler taska)
    xTaskCreate(SerialReceive_Task, "RX_TASK", configMINIMAL_STACK_SIZE + 200, NULL, TASK_SERIAL_REC_PRI, NULL);
    xTaskCreate(SerialSend_Task, "TX_TASK", configMINIMAL_STACK_SIZE + 200, NULL, TASK_SERIAL_SEND_PRI, NULL);
    xTaskCreate(Sensor_Task, "SENSOR_TASK", configMINIMAL_STACK_SIZE + 200, NULL, TASK_SENSOR_PRI, NULL);
    xTaskCreate(Processing_Task, "PROC_TASK", configMINIMAL_STACK_SIZE + 200, NULL, TASK_PROCESS_PRI, NULL);
    xTaskCreate(Display_Task, "DISP_TASK", configMINIMAL_STACK_SIZE + 200, NULL, TASK_DISPLAY_PRI, NULL);
    xTaskCreate(Alarm_Task, "ALARM_TASK", configMINIMAL_STACK_SIZE + 100, NULL, TASK_ALARM_PRI, NULL);

    // LED bar interrupt setup (Budi task preko semafora da reagune na promenu u LED Baru)
    vPortSetInterruptHandler(portINTERRUPT_SRL_OIC, OnLED_ChangeInterrupt);

    // start scheduler (Budim sve taskove, semafore, tajmere i queue-ove koji su podeseni)
    vTaskStartScheduler();

    while (1);
}