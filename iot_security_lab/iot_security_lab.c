#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/cyw43_arch.h"   // Wi-Fi + LED
#include "ssd1306.h"
#include "mpu6050.h"


// Bibliotecas personalizadas do seu exemplo de conexão
#include "include/wifi_conn.h"
#include "include/mqtt_comm.h"

// ==== CONFIG DISPLAY OLED (i2c1) ====
#define I2C_PORT_OLED i2c1
#define OLED_SCL 14
#define OLED_SDA 15

ssd1306_t disp;  // Estrutura do display

// ==== Inicialização do OLED ====
void inicializa_oled() {
    i2c_init(I2C_PORT_OLED, 400 * 1000);
    gpio_set_function(OLED_SCL, GPIO_FUNC_I2C);
    gpio_set_function(OLED_SDA, GPIO_FUNC_I2C);
    gpio_pull_up(OLED_SCL);
    gpio_pull_up(OLED_SDA);

    disp.external_vcc = false;
    ssd1306_init(&disp, 128, 64, 0x3C, I2C_PORT_OLED);
}

// ==== Print multilinha no OLED ====
void print_texto_multilinha(const char *msg, int font_size) {
    ssd1306_clear(&disp);
    int line_height = font_size * 16;
    int y = 0;

    const char *start = msg;
    while (*start) {
        const char *end = strchr(start, '\n');
        char linha[64] = {0};

        if (end) {
            strncpy(linha, start, end - start);
            linha[end - start] = '\0';
            start = end + 1;
        } else {
            strcpy(linha, start);
            start += strlen(start);
        }

        ssd1306_draw_string(&disp, 0, y, font_size, linha);
        y += line_height;
    }
    ssd1306_show(&disp);
}

// ==== Pisca LED Verde (LED Wi-Fi da BitDogLab) ====
void led_pisca() {
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    sleep_ms(100);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
}

// ==== MAIN ====
int main() {
    stdio_init_all();

    // Inicializa sistema WiFi
    if (cyw43_arch_init()) {
        printf("Falha ao inicializar Wi-Fi\n");
        return -1;
    }

    // Inicializa OLED
    inicializa_oled();

    // Inicializa MPU6050 (i2c0 -> GPIO0=SDA, GPIO1=SCL)
    init_mpu6050();
    mpu6050_config();

    // === Conecta Wi-Fi ===
    connect_to_wifi("Maíra 2", "@Ni06072012");

    // === Configura MQTT ===
    mqtt_setup("bitdoglab_anderson", "mqtt.iot.natal.br", "desafio05", "desafio05.laica");

    // Buffers
    int16_t accel[3], gyro[3], temp_raw;
    char buffer_oled[128];
    char json_msg[512];

    while (true) {
        // === Lê dados MPU ===
        mpu6050_read_raw(accel, gyro, &temp_raw);

        float ax_g = accel[0] / 16384.0;
        float ay_g = accel[1] / 16384.0;
        float az_g = accel[2] / 16384.0;

        float gx_dps = gyro[0] / 131.0;
        float gy_dps = gyro[1] / 131.0;
        float gz_dps = gyro[2] / 131.0;

        float temp_c = (temp_raw / 340.0) + 36.53;

        // === Exibe no OLED ===
        snprintf(buffer_oled, sizeof(buffer_oled),
            "AX:%.2fg AY:%.2fg\n"
            "AZ:%.2fg GX:%.1f\n"
            "GY:%.1f GZ:%.1f\n"
            "T:%.1fC",
            ax_g, ay_g,
            az_g, gx_dps,
            gy_dps, gz_dps,
            temp_c);

        print_texto_multilinha(buffer_oled, 1);

        // === Monta JSON ===
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
            ax_g, ay_g, az_g,
            gx_dps, gy_dps, gz_dps,
            temp_c
        );

        // === Publica no MQTT ===
        mqtt_comm_publish("ha/desafio05/anderson.henrique/mpu6050", json_msg, strlen(json_msg));

        // === Pisca LED ao publicar ===
        led_pisca();

        sleep_ms(10000); // publica a cada 10 segundos
    }

    cyw43_arch_deinit();
    return 0;
}
