#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/cyw43_arch.h"   // Wi-Fi + LED
#include "ssd1306.h"
#include "mpu6050.h"

// Bibliotecas personalizadas
#include "include/wifi_conn.h"
#include "include/mqtt_comm.h"

// ==== CONFIG DISPLAY OLED (i2c1) ====
#define I2C_PORT_OLED i2c1
#define OLED_SCL 14
#define OLED_SDA 15

// ==== LED Verde ====
#define LED_G_PIN 11

ssd1306_t disp;  // Estrutura do display

// ==== Inicializa OLED ====
void inicializa_oled() {
    i2c_init(I2C_PORT_OLED, 400 * 1000);
    gpio_set_function(OLED_SCL, GPIO_FUNC_I2C);
    gpio_set_function(OLED_SDA, GPIO_FUNC_I2C);
    gpio_pull_up(OLED_SCL);
    gpio_pull_up(OLED_SDA);

    disp.external_vcc = false;
    ssd1306_init(&disp, 128, 64, 0x3C, I2C_PORT_OLED);
}

// ==== Atualiza display estilo menu (como seu exemplo) ====
void atualizar_display_sensor(float ax, float ay, float az, 
                              float gx, float gy, float gz,
                              float temp, const char *wifi_status) {
    ssd1306_clear(&disp);

    char linha[32];

    snprintf(linha, sizeof(linha), "WiFi: %s", wifi_status);
    ssd1306_draw_string(&disp, 0, 0, 1, linha);

    snprintf(linha, sizeof(linha), "AX: %.2fg AY: %.2fg", ax, ay);
    ssd1306_draw_string(&disp, 0, 12, 1, linha);

    snprintf(linha, sizeof(linha), "AZ: %.2fg GX: %.1f", az, gx);
    ssd1306_draw_string(&disp, 0, 24, 1, linha);

    snprintf(linha, sizeof(linha), "GY: %.1f GZ: %.1f", gy, gz);
    ssd1306_draw_string(&disp, 0, 36, 1, linha);

    snprintf(linha, sizeof(linha), "Temp: %.1fC", temp);
    ssd1306_draw_string(&disp, 0, 48, 1, linha);

    ssd1306_show(&disp);
}

// ==== Pisca LED Verde por 0,5s ====
void led_pisca_meio_seg() {
    gpio_put(LED_G_PIN, 1);
    sleep_ms(500);
    gpio_put(LED_G_PIN, 0);
}

int main() {
    stdio_init_all();

    // Inicializa LED
    gpio_init(LED_G_PIN);
    gpio_set_dir(LED_G_PIN, GPIO_OUT);
    gpio_put(LED_G_PIN, 0);

    // Inicializa Wi-Fi
    if (cyw43_arch_init()) {
        inicializa_oled();
        atualizar_display_sensor(0,0,0,0,0,0,0,"Falha Init WiFi");
        return -1;
    }

    // Inicializa OLED
    inicializa_oled();

    // Inicializa MPU6050
    init_mpu6050();
    mpu6050_config();

    // Conecta Wi-Fi
    connect_to_wifi("Maíra 2", "@Ni06072012");

    sleep_ms(5000);

    // Checa se conectou
    const char *wifi_status = (cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_UP) ? "OK" : "Falhou";

    // Mostra status Wi-Fi antes de ler sensor
    atualizar_display_sensor(0,0,0,0,0,0,0,wifi_status);

    // Configura MQTT
    mqtt_setup("bitdoglab_anderson", "mqtt.iot.natal.br", "desafio05", "desafio05.laica");

    int16_t accel[3], gyro[3], temp_raw;
    char json_msg[512];

    while (true) {
        // Lê dados MPU
        mpu6050_read_raw(accel, gyro, &temp_raw);

        float ax_g = accel[0] / 16384.0;
        float ay_g = accel[1] / 16384.0;
        float az_g = accel[2] / 16384.0;

        float gx_dps = gyro[0] / 131.0;
        float gy_dps = gyro[1] / 131.0;
        float gz_dps = gyro[2] / 131.0;

        float temp_c = (temp_raw / 340.0) + 36.53;

        // Atualiza display com os valores lidos
        atualizar_display_sensor(ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps, temp_c, wifi_status);

        // Monta JSON
        snprintf(json_msg, sizeof(json_msg),
            "{"
                "\"team\":\"desafio05\","
                "\"device\":\"bitdoglab_anderson\","
                "\"ip\":\"0.0.0.0\","
                "\"ssid\":\"MinhaRedeWiFi\","
                "\"sensor\":\"MPU-6050\","
                "\"data\":{"
                    "\"accel\":{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f},"
                    "\"gyro\":{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f},"
                    "\"temperature\":%.2f"
                "},"
                "\"timestamp\":\"2025-09-07T21:00:00\""
            "}",
            ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps, temp_c
        );

        // Publica no MQTT
        mqtt_comm_publish("ha/desafio05/anderson.henrique/mpu6050", json_msg, strlen(json_msg));

        // Pisca LED verde por 0,5s após publicar
        led_pisca_meio_seg();

        // Aguarda 10 segundos
        sleep_ms(2000);
    }

    cyw43_arch_deinit();
    return 0;
}
