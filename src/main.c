#include <stdio.h>
#include <string.h>

#include <pico/stdlib.h>      // Picon perusjutut (printf, sleep_ms, gpio...)
#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

#include "hardware/i2c.h"     // I2C-väylä
#include "tkjhat/sdk.h"       // JTKJ Hat -SDK
#include <inttypes.h>         // uint8_t jne.

// ===============================
// MÄÄRITYKSET / DEFINET
// ===============================

// Accelerometrin X-akselin datarekisterit
#define ACCEL_DATA_X1  0x0B
#define ACCEL_DATA_X0  0x0C

// FreeRTOS-tehtävien pinon koko
#define DEFAULT_STACK_SIZE 2048 

// Morse-puskurin koko (kuinka pitkä viesti voi olla)
#define MORSE_BUFFER_SIZE 256 // Tätä kokoa voi vaihtaa. Tää saa nyt alkuun riittää.

// I2C-portti
#define I2C_PORT    i2c0

// IMU = gyroskooppi / liikeanturi (ICM-42670)
#define ICM42670_ADDR 0x69
#define REG_VIESTI      0x75   // WHO_AM_I -rekisteri (voi myöhemmin nimetä uudelleen)
#define REG_PWR_MGMT0   0x1F   // tehohallinta (gyro+accel päälle)


// ===============================
// TILAKONE
// ===============================

enum state {
    STATE_INIT = 0,      // aloitus / nollaus
    STATE_READ_INPUT,    // luetaan imu + napit
    STATE_CONVERT,       // (ei käytössä nyt)
    STATE_PRINT          // tulostetaan valmis morse-viesti
};

// Aloitetaan suoraan inputin luvusta, pidetään tämä simppelinä
enum state programState = STATE_READ_INPUT; 

// ===============================
// GLOBAALIT MUUTTUJAT
// ===============================

// Tänne tallennetaan morse-viesti merkkijonona, esim: ".- .- ...  \n"
char morseBuffer[MORSE_BUFFER_SIZE];

// morseIndex kertoo, mihin kohtaan puskurissa kirjoitetaan seuraava merkki
int morseIndex = 0; 

// spaceCount laskee peräkkäiset välilyönnit (BUTTON2 painallukset):
// 1 väli  = kirjainväli
// 2 väliä = sanaväli
// 3 väliä = viestin loppu
int spaceCount = 0;

// Napin edellinen tila, jotta voidaan tunnistaa "uusi painallus"
int prevButton1 = 1; // 1 = ei painettuna (pull-up)
int prevButton2 = 1;

// Viesti valmis -lippu, jota print_task tarkkailee
volatile int messageReady = 0;

// ===============================
// PROTOTYYPIT
// ===============================

// IMU-apufunktiot
void imu_write_reg(uint8_t reg, uint8_t value);
uint8_t imu_read_reg(uint8_t reg);
void imu_init(void);

// Kulman luku ja muunto piste/viiva-symboliksi
int read_angle_degrees(void);
char angle_to_symbol(int angle);

// Puskurin apufunktiot
void reset_morse(void);
void add_space_simple(void);

// FreeRTOS-tehtävät
static void sensor_task(void *arg);
static void print_task(void *arg);


// ===============================
// IMU-APUFUNKTIOT
// ===============================

// Luetaan accelerometrin X-akselin raaka-arvo (16-bittinen, signed).
int16_t imu_read_accel_x_raw(void) {
    uint8_t hi = imu_read_reg(ACCEL_DATA_X1);
    uint8_t lo = imu_read_reg(ACCEL_DATA_X0);

    int16_t value = (int16_t)((hi << 8) | lo);
    return value;
}

// Kirjoita yksi rekisteri IMU:lle I2C:n yli.
// reg   = rekisterin osoite
// value = kirjoitettava arvo
void imu_write_reg(uint8_t reg, uint8_t value) {
    uint8_t buf[2];
    buf[0] = reg;     // rekisterin osoite
    buf[1] = value;   // itse arvo

    i2c_write_blocking(I2C_PORT, ICM42670_ADDR, buf, 2, false);
}

// Lue yksi rekisteri IMU:lta I2C:n yli.
// reg = rekisterin osoite
// palauttaa rekisteristä luetun arvon
uint8_t imu_read_reg(uint8_t reg) {
    uint8_t value = 0;

    // ensin kerrotaan mille rekisterille halutaan luku
    i2c_write_blocking(I2C_PORT, ICM42670_ADDR, &reg, 1, true);
    // sitten luetaan 1 tavu kyseisestä rekisteristä
    i2c_read_blocking(I2C_PORT, ICM42670_ADDR, &value, 1, false);

    return value;
}

// Kytketään anturi päälle.
// Tässä asetetaan gyro + accel low noise -tilaan, jotta mittaus on tarkempi.
void imu_init(void) {
    sleep_ms(10); // pieni odotus, että anturi ehtii herätä

    // PWR_MGMT0 = 0x0F -> gyro ja accel päälle low-noise -tilassa.
    imu_write_reg(REG_PWR_MGMT0, 0x0F);
}

// ===============================
// KULMAFUNKTIOIDEN RUNGOT
// ===============================

// Tässä kohtaa "kulma" = raaka accelerometrin X-akselin arvo.
// Nimi on angle, mutta oikeasti se on vielä vain analoginen lukema.
int read_angle_degrees(void) {
    int16_t raw = imu_read_accel_x_raw();
    return (int)raw;
}

// Muunnetaan kulma morse-symboliksi.
// 0..2999 = '-' , muuten '.'
char angle_to_symbol(int angle) {
    if (angle >= 0 && angle < 3000) {
        return '-'; // esimerkki: "vaaka"
    } else {
        return '.'; // esimerkki: "pysty"
    }
}

// (Jätetään tämä käyttöön jos haluat joskus lisätä suoraan kulmasta puskuriin)
void angle_to_morse_buffer(int angle) {
    char symbol = angle_to_symbol(angle);
    if (morseIndex < MORSE_BUFFER_SIZE - 2) {
        morseBuffer[morseIndex++] = symbol;
        morseBuffer[morseIndex] = '\0';
    }
}

// ===============================
// PUSKURIN APUFUNKTIOT
// ===============================

void reset_morse(void) {
    morseIndex = 0;
    morseBuffer[0] = '\0';
    spaceCount = 0;
}

void add_space_simple(void) {
    if (morseIndex < MORSE_BUFFER_SIZE - 2) {
        morseBuffer[morseIndex++] = ' ';
        morseBuffer[morseIndex] = '\0';
    }
}

// ===============================
// FreeRTOS-TEHTÄVÄT
// ===============================

// sensor_task:
// Lue napit, tee reunantunnistus ja rakenna morseBuffer.
static void sensor_task(void *arg){
    (void)arg;

    // Alusta edelliset tilat kerran alussa
    prevButton1 = gpio_get(BUTTON1);
    prevButton2 = gpio_get(BUTTON2);

    for(;;){
        int b1 = gpio_get(BUTTON1); // 0 = painettu
        int b2 = gpio_get(BUTTON2);

        // BUTTON1: lisää piste/viiva kulmasta
        // Reuna: 1 -> 0 (uusi painallus)
        if (prevButton1 == 1 && b1 == 0) {
            printf("BUTTON1 pressed\n");
            int angle = read_angle_degrees();
            char symbol = angle_to_symbol(angle);

            if (morseIndex < MORSE_BUFFER_SIZE - 2) {
                morseBuffer[morseIndex++] = symbol;
                morseBuffer[morseIndex] = '\0';
                // Uusi merkki katkaisee välilyöntiketjun
                spaceCount = 0;
            }
        }

        // BUTTON2: lisää välilyönti, 3 peräkkäistä väliä = viesti valmis
        if (prevButton2 == 1 && b2 == 0) {
            printf("BUTTON2 pressed\n");
            add_space_simple();
            spaceCount++;
            if (spaceCount >= 3) {
                messageReady = 1; // print_task hoitaa tulostuksen ja resetin
            }
        }

        // Päivitä edelliset tilat
        prevButton1 = b1;
        prevButton2 = b2;

        // Yksinkertainen debouncaus ja CPU-lepo
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// print_task:
// Näyttää kulman debugina ja tulostaa valmiin morse-viestin kun messageReady=1.
static void print_task(void *arg){
    (void)arg;

    while(1){
        int angle = read_angle_degrees();
        printf("Kulma (raaka): %d\n", angle);

        if (messageReady) {
            printf(">>> MORSE-VIESTI: %s\n", morseBuffer);
            reset_morse();
            messageReady = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(500)); // puoli sekuntia väliä
    }
}

// ===============================
// PÄÄOHJELMA (main)
// ===============================

int main() {
    // Alustetaan stdio (USB, UART) niin että printf toimii.
    stdio_init_all();

    // (valinnainen) Odotellaan kunnes sarjamonitori on kiinni.
    /*
    while (!stdio_usb_connected()){
        sleep_ms(10);
    }
    */

    // Alustetaan JTKJ Hat -SDK (jos kurssi sitä käyttää)
    init_hat_sdk();
    sleep_ms(300); // Odotetaan hetki, että USB ja HAT ovat valmiit.

    // ===========================
    // NAPIT (BUTTON1, BUTTON2 tulevat HATin headerista)
    // ===========================
    gpio_init(BUTTON1);
    gpio_set_dir(BUTTON1, GPIO_IN);
    gpio_pull_up(BUTTON1);   // oletus = 1, painettaessa = 0

    gpio_init(BUTTON2);
    gpio_set_dir(BUTTON2, GPIO_IN);
    gpio_pull_up(BUTTON2);

    // ===========================
    // I2C + IMU
    // ===========================
    // I2C-nopeus: 400 kHz
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(DEFAULT_I2C_SCL_PIN);

    // Kytketään IMU päälle
    imu_init();

    // ===========================
    // FreeRTOS-tehtävien luonti
    // ===========================
    TaskHandle_t hSensorTask = NULL;
    TaskHandle_t hPrintTask  = NULL;

    BaseType_t result;

    // Sensoritehtävä – lukee napit ja rakentaa viestin
    result = xTaskCreate(
                sensor_task,          // tehtäväfunktio
                "sensor",             // nimi (debugia varten)
                DEFAULT_STACK_SIZE,   // pinon koko
                NULL,                 // ei parametreja
                2,                    // prioriteetti
                &hSensorTask);        // kahva

    if(result != pdPASS) {
        printf("Sensor task creation failed\n");
        return 0;
    }

    // Printtaustehtävä – näyttää kulman ja tulostaa valmiin viestin
    result = xTaskCreate(
                print_task,
                "print",
                DEFAULT_STACK_SIZE,
                NULL,
                2,
                &hPrintTask);

    if(result != pdPASS) {
        printf("Print task creation failed\n");
        return 0;
    }

    // Käynnistetään FreeRTOSin scheduler (ei palaa koskaan jos kaikki ok)
    vTaskStartScheduler();

    // Tänne ei normaalisti koskaan tulla.
    return 0;
}
