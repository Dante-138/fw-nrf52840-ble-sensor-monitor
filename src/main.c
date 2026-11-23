/*
 * nRF52840 BLE Sensor Monitor
 *
 * Le sensores internos (temperatura do die + tensao VDD) e transmite
 * via BLE com notificacoes automaticas em tempo real.
 *
 * Copyright (c) 2026 Dante-138
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(sensor_monitor, LOG_LEVEL_INF);

/* ================================================================== */
/*  Configuracao                                                       */
/* ================================================================== */

#define SENSOR_INTERVAL_MS     2000    /* Leitura de sensores a cada 2s   */
#define LED_BLINK_MS           1000    /* Pisca LED1 a cada 1s            */
#define TEMP_ALERT_HIGH        40.0f   /* Limiar de alerta de temperatura */
#define BTN_DEBOUNCE_MS        50      /* Botao vale se segue apertado    */
#define NOTIFY_STACK_SIZE      1024
#define NOTIFY_PRIORITY        7

/* ================================================================== */
/*  LEDs  (aliases do DeviceTree)                                      */
/*  LED1 = P0.13 -> sistema rodando (pisca)                            */
/*  LED2 = P0.14 -> BLE conectado                                      */
/*  LED3 = P0.15 -> notificacao enviada (flash)                        */
/*  LED4 = P0.16 -> alerta de temperatura                              */
/* ================================================================== */

static const struct gpio_dt_spec led_run  = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec led_conn = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
static const struct gpio_dt_spec led_ntf  = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);
static const struct gpio_dt_spec led_alrt = GPIO_DT_SPEC_GET(DT_ALIAS(led3), gpios);

/* ================================================================== */
/*  Estado dos Sensores                                                */
/* ================================================================== */

/*
 * A thread de notificacao, o main, o work dos botoes e as leituras GATT
 * mexem neste estado: tudo passa por sensor_lock.
 */
K_MUTEX_DEFINE(sensor_lock);

static float    current_temp;
static float    temp_min =  999.0f;
static float    temp_max = -999.0f;
static int32_t  battery_mv;
static uint8_t  battery_pct;
static uint32_t reading_count;
static bool     notify_active = true;
static bool     ble_connected;

/* Copia coerente do estado, para imprimir e notificar fora do lock. */
struct snapshot {
	float    temp;
	float    temp_min;
	float    temp_max;
	int32_t  mv;
	uint8_t  pct;
	uint32_t count;
};

/* ================================================================== */
/*  Sensor de Temperatura Interna                                      */
/* ================================================================== */

/* Resolvido em compilacao: a thread de notificacao sobe no boot, antes do
 * main, e nunca le um ponteiro NULL. */
static const struct device *const temp_dev =
	DEVICE_DT_GET(DT_ALIAS(ambient_temp0));

static int read_temperature(void)
{
	struct sensor_value val;
	int ret;

	ret = sensor_sample_fetch(temp_dev);
	if (ret) {
		LOG_ERR("Temp fetch falhou: %d", ret);
		return ret;
	}

	ret = sensor_channel_get(temp_dev, SENSOR_CHAN_DIE_TEMP, &val);
	if (ret) {
		LOG_ERR("Temp get falhou: %d", ret);
		return ret;
	}

	current_temp = (float)sensor_value_to_double(&val);

	if (current_temp < temp_min) {
		temp_min = current_temp;
	}
	if (current_temp > temp_max) {
		temp_max = current_temp;
	}

	gpio_pin_set_dt(&led_alrt, current_temp >= TEMP_ALERT_HIGH ? 1 : 0);

	return 0;
}

/* ================================================================== */
/*  Bateria / Tensao VDD  (SAADC canal 0)                              */
/* ================================================================== */

static const struct adc_dt_spec adc_vdd =
	ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0);

static int16_t adc_buf;

static int read_battery(void)
{
	struct adc_sequence seq = {
		.buffer      = &adc_buf,
		.buffer_size = sizeof(adc_buf),
	};
	int ret;

	ret = adc_sequence_init_dt(&adc_vdd, &seq);
	if (ret) {
		LOG_ERR("ADC seq init: %d", ret);
		return ret;
	}

	ret = adc_read_dt(&adc_vdd, &seq);
	if (ret) {
		LOG_ERR("ADC read: %d", ret);
		return ret;
	}

	int32_t val_mv = (int32_t)adc_buf;

	ret = adc_raw_to_millivolts_dt(&adc_vdd, &val_mv);
	if (ret) {
		/* Fallback: gain=1/6, ref_interna=0.6V -> range=3.6V */
		val_mv = ((int32_t)adc_buf * 3600) / 4096;
	}

	battery_mv = val_mv;

	/* Mapeamento linear: 2100mV (0%) ate 3300mV (100%) */
	if (battery_mv >= 3300) {
		battery_pct = 100;
	} else if (battery_mv <= 2100) {
		battery_pct = 0;
	} else {
		battery_pct = (uint8_t)((battery_mv - 2100) * 100 / 1200);
	}

	return 0;
}

/* Chamar com sensor_lock. */
static void take_snapshot(struct snapshot *s)
{
	s->temp     = current_temp;
	s->temp_min = temp_min;
	s->temp_max = temp_max;
	s->mv       = battery_mv;
	s->pct      = battery_pct;
	s->count    = reading_count;
}

/* Le temperatura e VDD e devolve o estado. Nao chamar de ISR (bloqueia). */
static void read_sensors(struct snapshot *s, bool count_reading)
{
	k_mutex_lock(&sensor_lock, K_FOREVER);
	read_temperature();
	read_battery();
	if (count_reading) {
		reading_count++;
	}
	take_snapshot(s);
	k_mutex_unlock(&sensor_lock);
}

/* ================================================================== */
/*  BLE: Custom Sensor Service                                         */
/* ================================================================== */

/*
 * UUIDs do servico:
 *   Service :  12345678-1234-5678-1234-56789abcdef0
 *   Temp    :  ...def1    (READ + NOTIFY)
 *   Battery :  ...def2    (READ + NOTIFY)
 */

#define BT_UUID_SENSOR_SVC_VAL \
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)

static struct bt_uuid_128 sensor_svc_uuid =
	BT_UUID_INIT_128(BT_UUID_SENSOR_SVC_VAL);

#define BT_UUID_TEMP_CHAR_VAL \
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef1)

static struct bt_uuid_128 temp_char_uuid =
	BT_UUID_INIT_128(BT_UUID_TEMP_CHAR_VAL);

#define BT_UUID_BATT_CHAR_VAL \
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef2)

static struct bt_uuid_128 batt_char_uuid =
	BT_UUID_INIT_128(BT_UUID_BATT_CHAR_VAL);

/* --- CCC callbacks ------------------------------------------------ */

static void temp_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	LOG_INF("Temp notify %s", value == BT_GATT_CCC_NOTIFY ? "ON" : "OFF");
}

static void batt_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	LOG_INF("Batt notify %s", value == BT_GATT_CCC_NOTIFY ? "ON" : "OFF");
}

/* --- Read callbacks ----------------------------------------------- */

static ssize_t read_temp_char(struct bt_conn *conn,
			      const struct bt_gatt_attr *attr,
			      void *buf, uint16_t len, uint16_t offset)
{
	float temp;

	k_mutex_lock(&sensor_lock, K_FOREVER);
	temp = current_temp;
	k_mutex_unlock(&sensor_lock);

	return bt_gatt_attr_read(conn, attr, buf, len, offset,
				 &temp, sizeof(temp));
}

static ssize_t read_batt_char(struct bt_conn *conn,
			      const struct bt_gatt_attr *attr,
			      void *buf, uint16_t len, uint16_t offset)
{
	uint8_t pct;

	k_mutex_lock(&sensor_lock, K_FOREVER);
	pct = battery_pct;
	k_mutex_unlock(&sensor_lock);

	return bt_gatt_attr_read(conn, attr, buf, len, offset,
				 &pct, sizeof(pct));
}

/* --- Definicao do servico GATT ------------------------------------ */
/*
 *  Indices dos atributos:
 *    [0] Primary Service
 *    [1] Characteristic Declaration  (temperature)
 *    [2] Characteristic Value        (temperature)  <- notify aqui
 *    [3] CCC Descriptor              (temperature)
 *    [4] Characteristic Declaration  (battery)
 *    [5] Characteristic Value        (battery)      <- notify aqui
 *    [6] CCC Descriptor              (battery)
 */

BT_GATT_SERVICE_DEFINE(sensor_svc,
	BT_GATT_PRIMARY_SERVICE(&sensor_svc_uuid),

	/* Temperatura */
	BT_GATT_CHARACTERISTIC(&temp_char_uuid.uuid,
		BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
		BT_GATT_PERM_READ,
		read_temp_char, NULL, NULL),
	BT_GATT_CCC(temp_ccc_changed,
		BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	/* Bateria */
	BT_GATT_CHARACTERISTIC(&batt_char_uuid.uuid,
		BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
		BT_GATT_PERM_READ,
		read_batt_char, NULL, NULL),
	BT_GATT_CCC(batt_ccc_changed,
		BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

/* ================================================================== */
/*  BLE: Conexao                                                       */
/* ================================================================== */

static struct bt_conn *cur_conn;

static void on_connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		LOG_ERR("Conexao falhou (err %u)", err);
		return;
	}

	cur_conn = bt_conn_ref(conn);
	ble_connected = true;
	gpio_pin_set_dt(&led_conn, 1);
	LOG_INF("BLE conectado");
}

static void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("BLE desconectado (reason %u)", reason);

	if (cur_conn) {
		bt_conn_unref(cur_conn);
		cur_conn = NULL;
	}
	ble_connected = false;
	gpio_pin_set_dt(&led_conn, 0);
}

BT_CONN_CB_DEFINE(conn_cbs) = {
	.connected    = on_connected,
	.disconnected = on_disconnected,
};

/* --- Advertising data --------------------------------------------- */

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS,
		      (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_SENSOR_SVC_VAL),
};

static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, "nRF52840_Sensor",
		sizeof("nRF52840_Sensor") - 1),
};

/* ================================================================== */
/*  Botoes  (aliases do DeviceTree)                                    */
/*  BTN1 = P0.11 -> leitura imediata                                   */
/*  BTN2 = P0.12 -> liga/desliga notificacoes                          */
/*  BTN3 = P0.24 -> reset min/max                                      */
/*  BTN4 = P0.25 -> status completo via serial                         */
/*                                                                     */
/*  A interrupcao so agenda um work. A acao roda no workqueue do       */
/*  sistema, BTN_DEBOUNCE_MS depois da ultima borda, e so se o botao   */
/*  continua apertado: o repique da mecanica vira uma acao so, e a     */
/*  leitura de sensor (que bloqueia) sai do contexto de ISR.           */
/* ================================================================== */

static void btn1_action(void)
{
	struct snapshot s;

	read_sensors(&s, false);
	printk("[BTN1] Leitura forcada -> Temp: %.1f C | VDD: %d mV (%u%%)\n",
	       (double)s.temp, s.mv, s.pct);
}

static void btn2_action(void)
{
	notify_active = !notify_active;
	printk("[BTN2] Notificacoes BLE: %s\n", notify_active ? "ON" : "OFF");
}

static void btn3_action(void)
{
	float temp;

	k_mutex_lock(&sensor_lock, K_FOREVER);
	temp_min = current_temp;
	temp_max = current_temp;
	reading_count = 0;
	temp = current_temp;
	k_mutex_unlock(&sensor_lock);

	printk("[BTN3] Min/Max resetados para %.1f C\n", (double)temp);
}

static void btn4_action(void)
{
	struct snapshot s;

	k_mutex_lock(&sensor_lock, K_FOREVER);
	take_snapshot(&s);
	k_mutex_unlock(&sensor_lock);

	printk("\n===== STATUS =====\n");
	printk("Temp atual:    %.1f C\n", (double)s.temp);
	printk("Temp min:      %.1f C\n", (double)s.temp_min);
	printk("Temp max:      %.1f C\n", (double)s.temp_max);
	printk("VDD:           %d mV (%u%%)\n", s.mv, s.pct);
	printk("Leituras:      %u\n", s.count);
	printk("BLE:           %s\n",
	       ble_connected ? "Conectado" : "Desconectado");
	printk("Notificacoes:  %s\n", notify_active ? "ON" : "OFF");
	printk("==================\n\n");
}

struct button {
	struct gpio_dt_spec      spec;
	void                   (*action)(void);
	struct gpio_callback     cb;
	struct k_work_delayable  work;
};

static struct button buttons[] = {
	{ .spec = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios), .action = btn1_action },
	{ .spec = GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios), .action = btn2_action },
	{ .spec = GPIO_DT_SPEC_GET(DT_ALIAS(sw2), gpios), .action = btn3_action },
	{ .spec = GPIO_DT_SPEC_GET(DT_ALIAS(sw3), gpios), .action = btn4_action },
};

/* Contexto de ISR: so (re)agenda o work, que e seguro aqui. */
static void btn_isr(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins)
{
	struct button *b = CONTAINER_OF(cb, struct button, cb);

	k_work_reschedule(&b->work, K_MSEC(BTN_DEBOUNCE_MS));
}

static void btn_work_fn(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct button *b = CONTAINER_OF(dwork, struct button, work);

	if (gpio_pin_get_dt(&b->spec) == 1) {
		b->action();
	}
}

/* ================================================================== */
/*  Inicializacao — LEDs e Botoes                                      */
/* ================================================================== */

static int leds_init(void)
{
	const struct gpio_dt_spec *leds[] = {
		&led_run, &led_conn, &led_ntf, &led_alrt
	};

	for (int i = 0; i < 4; i++) {
		if (!gpio_is_ready_dt(leds[i])) {
			LOG_ERR("LED%d nao pronto", i + 1);
			return -ENODEV;
		}
		gpio_pin_configure_dt(leds[i], GPIO_OUTPUT_INACTIVE);
	}
	return 0;
}

static int buttons_init(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(buttons); i++) {
		struct button *b = &buttons[i];
		int ret;

		if (!gpio_is_ready_dt(&b->spec)) {
			LOG_ERR("BTN%u nao pronto", (unsigned int)(i + 1));
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&b->spec, GPIO_INPUT);
		if (ret) {
			LOG_ERR("BTN%u configure: %d", (unsigned int)(i + 1), ret);
			return ret;
		}

		/* Work e callback prontos antes de ligar a interrupcao. */
		k_work_init_delayable(&b->work, btn_work_fn);
		gpio_init_callback(&b->cb, btn_isr, BIT(b->spec.pin));

		ret = gpio_add_callback(b->spec.port, &b->cb);
		if (ret) {
			LOG_ERR("BTN%u callback: %d", (unsigned int)(i + 1), ret);
			return ret;
		}

		ret = gpio_pin_interrupt_configure_dt(&b->spec,
						      GPIO_INT_EDGE_TO_ACTIVE);
		if (ret) {
			LOG_ERR("BTN%u interrupt: %d", (unsigned int)(i + 1), ret);
			return ret;
		}
	}
	return 0;
}

/* ================================================================== */
/*  Thread de Notificacao BLE                                          */
/*  Le sensores periodicamente e envia notificacoes ao celular         */
/* ================================================================== */

static void notify_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		struct snapshot s;

		k_msleep(SENSOR_INTERVAL_MS);

		read_sensors(&s, true);

		LOG_INF("#%u | Temp: %.1f C | VDD: %d mV (%u%%)",
			s.count, (double)s.temp, s.mv, s.pct);

		if (!ble_connected || !notify_active) {
			continue;
		}

		/* Notifica temperatura (attr index 2) */
		int ret = bt_gatt_notify(cur_conn,
					 &sensor_svc.attrs[2],
					 &s.temp,
					 sizeof(s.temp));
		if (ret == 0) {
			gpio_pin_set_dt(&led_ntf, 1);
		}

		/* Notifica bateria (attr index 5) */
		bt_gatt_notify(cur_conn,
			       &sensor_svc.attrs[5],
			       &s.pct,
			       sizeof(s.pct));

		/* Flash breve no LED3 */
		k_msleep(100);
		gpio_pin_set_dt(&led_ntf, 0);
	}
}

K_THREAD_DEFINE(notify_tid, NOTIFY_STACK_SIZE,
		notify_thread_fn, NULL, NULL, NULL,
		NOTIFY_PRIORITY, 0, 0);

/* ================================================================== */
/*  Main                                                               */
/* ================================================================== */

int main(void)
{
	struct snapshot s;
	int ret;

	printk("\n");
	printk("========================================\n");
	printk("  nRF52840 BLE Sensor Monitor v1.0\n");
	printk("  github.com/Dante-138\n");
	printk("========================================\n\n");

	/* --- LEDs & Botoes --- */
	ret = leds_init();
	if (ret) {
		return ret;
	}

	/* --- Sensor de temperatura --- */
	if (!device_is_ready(temp_dev)) {
		LOG_ERR("Sensor de temperatura nao encontrado");
		return -ENODEV;
	}
	LOG_INF("Sensor de temperatura: OK");

	/* --- ADC (tensao VDD) --- */
	if (!adc_is_ready_dt(&adc_vdd)) {
		LOG_ERR("ADC nao pronto");
		return -ENODEV;
	}

	ret = adc_channel_setup_dt(&adc_vdd);
	if (ret) {
		LOG_ERR("ADC channel setup: %d", ret);
		return ret;
	}
	LOG_INF("ADC (VDD): OK");

	/* Botoes so depois dos sensores: BTN1 le os dois. */
	ret = buttons_init();
	if (ret) {
		return ret;
	}

	/* --- Leitura inicial --- */
	read_sensors(&s, false);
	printk("Temperatura inicial: %.1f C\n", (double)s.temp);
	printk("Tensao VDD:          %d mV\n\n", s.mv);

	/* --- Bluetooth --- */
	ret = bt_enable(NULL);
	if (ret) {
		LOG_ERR("BLE init falhou: %d", ret);
		return ret;
	}
	LOG_INF("Bluetooth: OK");

	ret = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad),
			      sd, ARRAY_SIZE(sd));
	if (ret) {
		LOG_ERR("Advertising falhou: %d", ret);
		return ret;
	}
	LOG_INF("Advertising ativo como 'nRF52840_Sensor'");

	printk("Controles:\n");
	printk("  BTN1 -> Leitura imediata\n");
	printk("  BTN2 -> Liga/Desliga notificacoes BLE\n");
	printk("  BTN3 -> Reset min/max\n");
	printk("  BTN4 -> Status completo\n\n");

	/* --- Loop principal: pisca LED1 (heartbeat) --- */
	while (1) {
		gpio_pin_toggle_dt(&led_run);
		k_msleep(LED_BLINK_MS);
	}

	return 0;
}
