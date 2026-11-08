#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>

#define LED_R_NODE DT_ALIAS(led_red)
#define LED_G_NODE DT_ALIAS(led_green)
#define LED_B_NODE DT_ALIAS(led_blue)
#define SW0_NODE   DT_ALIAS(sw0)
#define ADC_NODE   DT_PATH(zephyr_user)
#define I2C_NODE   DT_ALIAS(sensor_i2c)
#define UART_NODE  DT_ALIAS(gps_uart)

static const struct gpio_dt_spec led_r  = GPIO_DT_SPEC_GET(LED_R_NODE, gpios);
static const struct gpio_dt_spec led_g  = GPIO_DT_SPEC_GET(LED_G_NODE, gpios);
static const struct gpio_dt_spec led_b  = GPIO_DT_SPEC_GET(LED_B_NODE, gpios);
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(SW0_NODE, gpios);

static const struct adc_dt_spec ldr_adc = ADC_DT_SPEC_GET(ADC_NODE);
static const struct device *i2c_dev     = DEVICE_DT_GET(I2C_NODE);
static const struct device *uart_dev    = DEVICE_DT_GET(UART_NODE);

int main(void)
{
	int ret;
	int16_t sample_buffer;
	struct adc_sequence sequence = {
		.buffer = &sample_buffer,
		.buffer_size = sizeof(sample_buffer),
	};

	printk("\n==================================================\n");
	printk("   TEST DE VALIDATION MATERIELLE GLOBALE (PHASE 1) \n");
	printk("==================================================\n");

	/* 1. LEDs RGB (Anode commune) */
	if (gpio_is_ready_dt(&led_r) && gpio_is_ready_dt(&led_g) && gpio_is_ready_dt(&led_b)) {
		gpio_pin_configure_dt(&led_r, GPIO_OUTPUT_INACTIVE);
		gpio_pin_configure_dt(&led_g, GPIO_OUTPUT_INACTIVE);
		gpio_pin_configure_dt(&led_b, GPIO_OUTPUT_INACTIVE);
		printk("[ OK ] GPIO LEDs RGB initialises\n");
	} else {
		printk("[FAIL] Erreur GPIO LEDs RGB\n");
	}

	/* 2. Bouton B1 */
	if (gpio_is_ready_dt(&button)) {
		gpio_pin_configure_dt(&button, GPIO_INPUT);
		printk("[ OK ] GPIO Bouton B1 pret\n");
	}

	/* 3. ADC1 (LDR sur PB1 / Canal 5) */
	if (device_is_ready(ldr_adc.dev)) {
		adc_channel_setup_dt(&ldr_adc);
		printk("[ OK ] ADC1 pret (PB1 / Canal 5)\n");
	} else {
		printk("[FAIL] ADC1 non pret\n");
	}

	/* 4. Bus I2C1 (MPU6050 + Si7021) */
	if (device_is_ready(i2c_dev)) {
		printk("[ OK ] Bus I2C1 pret\n");
	} else {
		printk("[FAIL] Bus I2C1 non pret\n");
	}

	/* 5. USART1 (GPS PA6H) */
	if (device_is_ready(uart_dev)) {
		printk("[ OK ] USART1 (GPS) pret\n");
	} else {
		printk("[FAIL] USART1 non pret\n");
	}

	printk("--------------------------------------------------\n");
	printk("Lancement des mesures en temps réel...\n\n");

	uint8_t rx_char;
	int step = 0;

	while (1) {
		/* Clignotement LED RGB */
		gpio_pin_set_dt(&led_r, (step % 3 == 0) ? 1 : 0);
		gpio_pin_set_dt(&led_g, (step % 3 == 1) ? 1 : 0);
		gpio_pin_set_dt(&led_b, (step % 3 == 2) ? 1 : 0);
		step++;

		/* Lecture ADC LDR */
		adc_sequence_init_dt(&ldr_adc, &sequence);
		ret = adc_read_dt(&ldr_adc, &sequence);
		if (ret == 0) {
			printk("[LDR / PB1] Valeur brute ADC: %d\n", sample_buffer);
		}

		/* Ping I2C1 pour détecter MPU6050 (0x68) et Si7021 (0x40) */
		uint8_t dummy;
		int ret_mpu = i2c_read(i2c_dev, &dummy, 1, 0x68);
		int ret_si7 = i2c_read(i2c_dev, &dummy, 1, 0x40);

		printk("[I2C SCAN] MPU6050 (0x68): %s | Si7021 (0x40): %s\n",
		       (ret_mpu == 0) ? "DETECTE [OK]" : "NON DETECTE [FAIL]",
		       (ret_si7 == 0) ? "DETECTE [OK]" : "NON DETECTE [FAIL]");

		/* Communication UART GPS */
		int gps_bytes = 0;
		while (uart_poll_in(uart_dev, &rx_char) == 0) {
			gps_bytes++;
		}
		printk("[GPS UART] Octets recus: %d\n", gps_bytes);

		/* Bouton utilisateur B1 */
		int btn_val = gpio_pin_get_dt(&button);
		printk("[BOUTON B1] Etat: %s\n", (btn_val > 0) ? "PRESSE" : "RELACHE");

		printk("--------------------------------------------------\n");
		k_msleep(1500);
	}

	return 0;
}