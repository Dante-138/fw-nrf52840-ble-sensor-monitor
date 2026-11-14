<p align="center">
  <img src="https://img.shields.io/badge/MCU-nRF52840-blue?style=flat-square" alt="nRF52840"/>
  <img src="https://img.shields.io/badge/RTOS-Zephyr-green?style=flat-square" alt="Zephyr"/>
  <img src="https://img.shields.io/badge/SDK-nRF_Connect-red?style=flat-square" alt="nRF Connect SDK"/>
  <img src="https://img.shields.io/badge/Protocol-BLE-purple?style=flat-square" alt="BLE"/>
  <img src="https://img.shields.io/badge/Sensor-Temp_+_ADC-yellow?style=flat-square" alt="Sensors"/>
  <img src="https://img.shields.io/badge/License-MIT-lightgrey?style=flat-square" alt="MIT"/>
</p>

# nRF52840 BLE Sensor Monitor

Firmware para a **nRF52840 DK (PCA10056)** que le sensores internos do chip (temperatura do die e tensao de alimentacao VDD) e transmite os dados em tempo real para um celular via **Bluetooth Low Energy** com notificacoes automaticas.

---

## Funcionalidades

- Leitura periodica da **temperatura interna** do nRF52840 (sensor die)
- Leitura da **tensao VDD** via SAADC (analogico-digital)
- Servico BLE customizado com **notificacoes automaticas** a cada 2 segundos
- **4 LEDs** indicadores de status
- **4 botoes** para controle interativo
- Rastreamento de temperatura **minima e maxima**
- Alerta visual quando temperatura ultrapassa limiar configuravel
- Log de leituras via serial (UART J-Link VCOM)

---

## LEDs

| LED | Pino   | Funcao                              |
|-----|--------|-------------------------------------|
| LED1 | P0.13 | Heartbeat - pisca a cada 1s (sistema rodando) |
| LED2 | P0.14 | BLE conectado (aceso = conectado)   |
| LED3 | P0.15 | Flash ao enviar notificacao BLE     |
| LED4 | P0.16 | Alerta de temperatura (>= 40 C)    |

## Botoes

| Botao | Pino   | Funcao                              |
|-------|--------|-------------------------------------|
| BTN1  | P0.11  | Forca leitura imediata dos sensores |
| BTN2  | P0.12  | Liga/desliga notificacoes BLE       |
| BTN3  | P0.24  | Reseta valores min/max              |
| BTN4  | P0.25  | Exibe status completo via serial    |

---

## Servico BLE

Servico customizado **Custom Sensor Service** com duas characteristics:

| Characteristic | UUID (sufixo) | Propriedades    | Dados              |
|---------------|---------------|-----------------|---------------------|
| Temperature   | `...def1`     | READ + NOTIFY   | float (4 bytes, C)  |
| Battery       | `...def2`     | READ + NOTIFY   | uint8 (1 byte, %)   |

**UUIDs completos:**

```
Service:     12345678-1234-5678-1234-56789abcdef0
Temperature: 12345678-1234-5678-1234-56789abcdef1
Battery:     12345678-1234-5678-1234-56789abcdef2
```

O dispositivo anuncia como **`nRF52840_Sensor`** no scan BLE.

---

## Hardware

| Recurso     | Especificacao                     |
|-------------|-----------------------------------|
| MCU         | nRF52840 (ARM Cortex-M4F, 64 MHz)|
| Flash       | 1 MB                              |
| RAM         | 256 KB                            |
| Radio       | Bluetooth 5.0 LE                  |
| ADC         | SAADC 12-bit (leitura VDD)        |
| Temp Sensor | Die temperature (interno)         |
| Board       | nRF52840 DK (PCA10056)            |

---

## Pre-requisitos

- **nRF Connect SDK** v3.x (nRF Connect for VS Code)
- **Toolchain** correspondente ao SDK instalado
- **nRF52840 DK** conectada via USB (J-Link)
- **nRF Connect** app no celular (Android/iOS) para visualizar dados BLE
- **PuTTY** ou terminal serial para ver logs (opcional)

---

## Estrutura do Projeto

```
nrf52840_ble_sensor_monitor/
├── CMakeLists.txt          # Build system (Zephyr/CMake)
├── prj.conf                # Kconfig (BLE, GPIO, Sensor, ADC, Log)
├── app.overlay             # DeviceTree (TEMP + SAADC canal VDD)
├── README.md               # Este arquivo
├── LICENSE                 # MIT License
├── .gitignore              # Exclusoes de build/IDE
└── src/
    └── main.c              # Firmware completo (~400 linhas)
```

---

## Compilacao e Flash

### Via nRF Connect for VS Code (recomendado)

1. Abra a pasta `nrf52840_ble_sensor_monitor` no VS Code
2. No painel **nRF Connect**, clique em **Add Build Configuration**
3. Selecione board: **`nrf52840dk/nrf52840`**
4. Clique em **Build Configuration** (build automatico)
5. Conecte a DK via USB
6. Clique em **Flash**

### Via terminal (west)

```bash
# Build
west build -b nrf52840dk/nrf52840 -p always

# Flash
west flash --runner jlink
```

### Build com caminho completo

```bash
west build -b nrf52840dk/nrf52840 \
  C:\ncs\projects\nrf52840_ble_sensor_monitor \
  --no-sysbuild \
  --build-dir C:\ncs\projects\nrf52840_ble_sensor_monitor\build \
  --pristine

west flash --runner jlink \
  --build-dir C:\ncs\projects\nrf52840_ble_sensor_monitor\build
```

---

## Como Testar

### 1. Serial (monitorar logs)

Abra um terminal serial na porta COM do J-Link:
- **Baud rate:** 115200
- **Configuracao:** 8N1 (8 bits, sem paridade, 1 stop bit)

Saida esperada:

```
========================================
  nRF52840 BLE Sensor Monitor v1.0
  github.com/Dante-138
========================================

Temperatura inicial: 26.5 C
Tensao VDD:          3287 mV

Controles:
  BTN1 -> Leitura imediata
  BTN2 -> Liga/Desliga notificacoes BLE
  BTN3 -> Reset min/max
  BTN4 -> Status completo

[00:00:02.001] #1 | Temp: 26.5 C | VDD: 3287 mV (99%)
[00:00:04.002] #2 | Temp: 26.7 C | VDD: 3290 mV (99%)
```

### 2. BLE (celular)

1. Abra o app **nRF Connect** no celular
2. Escaneie dispositivos BLE
3. Conecte ao **`nRF52840_Sensor`**
4. Navegue ate o servico **UUID ...def0**
5. Ative notificacoes nas characteristics de temperatura e bateria
6. Os valores serao atualizados automaticamente a cada 2 segundos

### 3. Botoes (interacao)

- Pressione **BTN1** para forcar uma leitura imediata
- Pressione **BTN4** para ver o status completo no terminal serial
- Pressione **BTN2** para pausar/retomar notificacoes BLE
- Pressione **BTN3** para resetar os valores min/max

---

## Arquitetura

```
                  ┌───────────────────────┐
                  │     Main Thread       │
                  │  (LED1 heartbeat)     │
                  └───────────────────────┘

┌───────────────────────────────────────────────┐
│              Notify Thread (prio 7)           │
│                                               │
│  ┌──────────┐    ┌──────────┐    ┌────────┐  │
│  │ Temp     │    │ ADC/VDD  │    │ BLE    │  │
│  │ Sensor   │───>│ Leitura  │───>│ Notify │  │
│  │ (die)    │    │ (SAADC)  │    │ (GATT) │  │
│  └──────────┘    └──────────┘    └────────┘  │
└───────────────────────────────────────────────┘

┌───────────────────────────────────────────────┐
│              GPIO Interrupts                  │
│  BTN1: leitura | BTN2: toggle | BTN3: reset  │
│  BTN4: status via printk                      │
└───────────────────────────────────────────────┘
```

**Modelo de threads:**

| Thread         | Stack   | Prioridade | Funcao                           |
|----------------|---------|------------|----------------------------------|
| Main           | default | 0          | Init + heartbeat LED1            |
| Notify         | 1024 B  | 7          | Leitura ciclica + BLE notify     |

---

## Stack Tecnologico

| Componente   | Versao / Tecnologia              |
|-------------|----------------------------------|
| RTOS        | Zephyr OS v3.7                   |
| SDK         | nRF Connect SDK v3.0.2           |
| Build       | CMake + Ninja (via west)         |
| BLE Stack   | SoftDevice Controller (Zephyr)   |
| Linguagem   | C (puro, sem C++)                |
| Licenca     | MIT                              |

---

## Troubleshooting

| Problema | Causa | Solucao |
|----------|-------|---------|
| `Sensor de temperatura nao encontrado` | TEMP peripheral nao habilitado | Verifique `app.overlay` e `CONFIG_TEMP_NRF5=y` |
| `ADC nao pronto` | SAADC nao configurado | Verifique `app.overlay` (canal ADC) e `CONFIG_ADC=y` |
| `Advertising falhou` | BLE stack sem memoria | Aumente `CONFIG_HEAP_MEM_POOL_SIZE` |
| Nao aparece no scan | Celular com BLE desligado | Ative BLE e permissao de localizacao no celular |
| Notificacoes nao chegam | CCC nao habilitado | No nRF Connect app, clique no icone de seta para ativar notify |
| Temperatura fixa em 0 | Overlay nao aplicado | Rebuild com `--pristine` |
| VDD mostra 0 mV | Canal ADC incorreto | Verifique `zephyr,input-positive = <9>` no overlay |
| LED1 nao pisca | Build falhou | Verifique erros no terminal de build |

---

## Autor

**Dante-138** - [GitHub](https://github.com/Dante-138)
