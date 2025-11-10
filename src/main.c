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

// FreeRTOS-tehtävien pinon koko
#define DEFAULT_STACK_SIZE 2048 

// Morse-puskurin koko (kuinka pitkä viesti voi olla)
#define MORSE_BUFFER_SIZE 256 // Tätä kokoa voi vaihtaa. Tää saa nyt alkuun riittää.

// I2C-asetukset (MUUTA PINNIT OIKEIKSI)
#define I2C_PORT    i2c0
#define I2C_SDA_PIN X    // vaihda oikeiksi arvoiksi (esim. 4)
#define I2C_SCL_PIN Y    // vaihda oikeiksi arvoiksi (esim. 5)

// IMU = gyroskooppi / liikeanturi (ICM-42670)
#define ICM42670_ADDR 0x69
#define REG_VIESTI      0x75   // WHO_AM_I -rekisteri (voi myöhemmin nimetä uudelleen)
#define REG_PWR_MGMT0   0x1F   // tehohallinta (gyro+accel päälle)

// Nappulat (vaihda pinnit oikeiksi)
#define BUTTON1     A          // nappi 1: kirjaa piste/viiva
#define BUTTON2     B          // nappi 2: välilyönnit

// ===============================
// TILAKONE
// ===============================

// Ohjelman eri tilat. Voidaan käyttää kun viestiä rakennetaan ja tulostetaan.
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

// Kulman luku ja muunto piste/viiva-symboliksi (tulee tehtäväksi myöhemmin)
int read_angle_degrees(void);
char angle_to_symbol(int angle);

// FreeRTOS-tehtävät
static void sensor_task(void *arg);
static void print_task(void *arg);


// ===============================
// IMU-APUFUNKTIOT
// ===============================

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

// Tämä funktio tulee myöhemmin lukemaan kulman oikeasti IMU:sta.
// Nyt se voi vaikka palauttaa aina saman arvon testimielessä.
int read_angle_degrees(void) {
    int angle = 0;

    // TODO: lue kulma IMU:lta (0-90 astetta tms.)
    // angle = ...

    return angle;
}

// Muunnetaan kulma morse-symboliksi.
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
// Tänne tulee Veikan koodi, joka
// - lukee IMU:n kulman
// - lukee nappi 1 ja 2 tilan
// - nappi 1 painettaessa: lisää kulman mukaan '.' tai '-' morseBufferiin
// - nappi 2 painettaessa: lisää välilyönnin ja kasvattaa spaceCountia
// - kun spaceCount == 3 -> viesti valmis -> programState = STATE_PRINT
static void sensor_task(void *arg){
    (void)arg;

    for(;;){
        // Nappien tämänhetkiset tilat: 1 = ei painettu, 0 = painettu
        int currentButton1 = gpio_get(BUTTON1);
        int currentButton2 = gpio_get(BUTTON2);

        // Luetaan kulma anturilta (myöhemmin oikea imu-luku)
        int angle = read_angle_degrees();

        // NAPPI 1: uusi painallus -> kirjaa piste/viiva
        if (prevButton1 == 1 && currentButton1 == 0) {
            char symbol = angle_to_symbol(angle);

            if (morseIndex < MORSE_BUFFER_SIZE - 1) {
                morseBuffer[morseIndex] = symbol;
                morseIndex++;
                spaceCount = 0; // merkki katkaisee välilyöntijonon
            }
        }

        // NAPPI 2: uusi painallus -> lisää välilyönti
        if (prevButton2 == 1 && currentButton2 == 0) {
            if (morseIndex < MORSE_BUFFER_SIZE - 1) {
                morseBuffer[morseIndex] = ' ';
                morseIndex++;
            }

            spaceCount++;

            // kolme välilyöntiä peräkkäin -> päätetään viesti
            if (spaceCount == 3) {
                // muutetaan viimeinen lisätty merkki rivinvaihdoksi
                // jotta viesti päättyy "  \n"
                if (morseIndex > 0) {
                    morseBuffer[morseIndex - 1] = '\n';
                }

                // lisätään merkkijonon loppumerkki
                if (morseIndex < MORSE_BUFFER_SIZE) {
                    morseBuffer[morseIndex] = '\0';
                }

                // ilmoitetaan print_taskille, että viesti on valmis
                programState = STATE_PRINT;

                // seuraavaa viestiä varten
                spaceCount = 0;
            }
        }

        // talletetaan nappien tila seuraavaa kierrosta varten
        prevButton1 = currentButton1;
        prevButton2 = currentButton2;

        vTaskDelay(pdMS_TO_TICKS(20)); // pieni viive, ettei pyöri liian nopeasti
    }
}

// print_task:
// Tänne tulee Oonan koodi, joka
// - odottaa kunnes programState == STATE_PRINT
// - tulostaa morseBufferin USB:lle (printf)
// - tyhjentää puskurin ja asettaa tilan STATE_INIT / STATE_READ_INPUT
static void print_task(void *arg){
    (void)arg;

    while(1){
        if (programState == STATE_PRINT) {
            // Tulostetaan morse-viesti
            printf("%s", morseBuffer);

            // Tyhjennetään puskuri seuraavaa viestiä varten
            morseIndex = 0;
            morseBuffer[0] = '\0';

            // Palataan alku-/luku-tilaan
            programState = STATE_READ_INPUT;
        }

        vTaskDelay(pdMS_TO_TICKS(50)); // ei tarvitse tarkistaa koko ajan
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
    // NAPIT
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
    // I2C-nopeus: X * 1000 (esim. 400 * 1000 = 400kHz)
    i2c_init(I2C_PORT, X * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    // Kytketään IMU päälle
    imu_init();

    // ===========================
    // FreeRTOS-tehtävien luonti
    // ===========================
    TaskHandle_t hSensorTask = NULL;
    TaskHandle_t hPrintTask  = NULL;

    BaseType_t result;

    // Sensoritehtävä (Veikka)
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

    // Printtaustehtävä (Oona)
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
