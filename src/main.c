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
    STATE_CONVERT,       // (jos joskus erotetaan muunnos omaksi vaiheeksi)
    STATE_PRINT          // tulostetaan valmis morse-viesti
};

enum state programState = STATE_INIT; // aloitetaan INIT-tilasta

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
// HUOM: tätä ei vielä käytetä testissä, mutta runko on valmiina.
// 0-45 astetta  = '-'
// 45-90 astetta = '.'
char angle_to_symbol(int angle) {
    if (angle >= 0 && angle < 45) {
        return '-'; // vaaka
    } else {
        return '.'; // pysty
    }
}

// ===============================
// FreeRTOS-TEHTÄVÄT
// ===============================

// sensor_task:
// Tulevaisuudessa: napit + morse.
// Nyt vain delay, ettei tee vielä mitään kriittistä.
static void sensor_task(void *arg){
    (void)arg;

    for(;;){
        // TODO: tänne myöhemmin napit + morseBuffer-logiikka
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// print_task:
// Testivaiheessa: tulostetaan raaka kulma 0.5 s välein.
static void print_task(void *arg){
    (void)arg;

    while(1){
        int angle = read_angle_degrees();
        printf("Kulma (raaka): %d\n", angle);

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

    // Sensoritehtävä (Veikka) – vielä tyhjä runko
    result = xTaskCreate(
                sensor_task,          // tehtäväfunktio
                "sensor",             // nimi (debugia varten)
                DEFAULT_STACK_SIZE,   // pinon koko
                NULL,                 // ei parametreja
                2,                    // prioriteetti
                &hSensorTask);        // kahva (voi olla myöhemmin hyödyksi)

    if(result != pdPASS) {
        printf("Sensor task creation failed\n");
        return 0;
    }

    // Printtaustehtävä (Oona) – tulostaa kulman
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
