# hw definition file for processing by chibios_pins.py

# uart port
# usart1 rx PA9
# uasrt1 tx PA10

# 485-1
# PWROUT    PA7
# PWRIN     PA5
# TXEN      PA1
# uasrt2 tx PA2
# usart2 rx PA3 

# 485-2 
# PWROUT    PA6
# PWRIN     PA4
# TXEN      PC8
# usart6 tx PC6
# usart6 rx PC7

# ms8537-1
# i2c1 sda PB7
# i2c1 scl PB6

# ms6837-2
# i2c2 sda PB11
# i2c2 scl PB10


# can1 tx  PB9
# can1 rx  PB8

# can2 tx  PB13
# can2 rx  PB5



# MCU class and specific type
MCU STM32F4xx STM32F405xx

# bootloader starts firmware at 64k
FLASH_RESERVE_START_KB 64

# store parameters in pages 2 and 3
STORAGE_FLASH_PAGE 2
define HAL_STORAGE_SIZE 15360

# board ID for firmware load
APJ_BOARD_ID 1014

# setup build for a peripheral firmware
env AP_PERIPH 1

define STM32_ST_USE_TIMER 5
define CH_CFG_ST_RESOLUTION 32

# enable watchdog

# crystal frequency
OSCILLATOR_HZ 8000000

# assume the 256k flash part for now
FLASH_SIZE_KB 1024



#PA11 OTG_FS_DM OTG1
#PA12 OTG_FS_DP OTG1

# USART1, for debug
PA9  USART1_TX USART1 SPEED_HIGH 
PA10 USART1_RX USART1 SPEED_HIGH 






define HAL_UART_MIN_TX_SIZE 256
define HAL_UART_MIN_RX_SIZE 128
define HAL_UART_STACK_SIZE 0x300

# ---------------------- I2C bus ------------------------
I2C_ORDER I2C1 I2C2

PB6 I2C1_SCL I2C1
PB7 I2C1_SDA I2C1

PB10 I2C2_SCL I2C2
PB11 I2C2_SDA I2C2

define HAL_USE_I2C TRUE
define STM32_I2C_USE_I2C1 TRUE
define STM32_I2C_USE_I2C2 TRUE

# ---------------------- CAN bus -------------------------
define HAL_CAN_POOL_SIZE 6000
# enable CAN support
PB8 CAN1_RX CAN1
PB9 CAN1_TX CAN1

PB5 CAN2_RX CAN2
PB13 CAN2_TX CAN2

#define CAN_APP_NODE_NAME "org.ardupilot.f405_Echo"


# --------------------- BAROS MS5837 ---------------------------
#define HAL_PERIPH_ENABLE_BAROd
define HAL_BARO_ALLOW_INIT_NO_BARO
#define HAL_PROBE_EXTERNAL_I2C_BAROS
define HAL_BARO_PROBE_EXT_DEFAULT 512
define AP_BARO_MS56XX_ENABLED 1
#BARO MS56XX I2C:0:0x76
#BARO MS56XX I2C:1:0x76

# ----------------- rangefinder,ADSB ----------------------
#define HAL_PERIPH_ENABLE_RANGEFINDER
#define RANGEFINDER_MAX_INSTANCES 2


# -------------------- Buzzer+NeoPixels --------------d------
#define HAL_PERIPH_NOTIFY_WITHOUT_RCOUT
#define HAL_PERIPH_ENABLE_NOTIFY
#define DEFAULT_NTF_LED_TYPES 455


# ---------------------- COMPASS ---------------------------
#define HAL_PERIPH_ENABLE_MAG
#define HAL_COMPASS_MAX_SENSORS 1


# debugger support
PA13 JTMS-SWDIO SWD
PA14 JTCK-SWCLK SWD

define HAL_USE_ADC FALSE
define STM32_ADC_USE_ADC1 FALSE
define HAL_DISABLE_ADC_DRIVER TRUE
# ----------- optimizations ----------------
define HAL_NO_GPIO_IRQ
# avoid timer and RCIN threads to save memory
define HAL_NO_RCIN_THREAD
define HAL_NO_RCOUT_THREAD


define HAL_USE_RTC FALSE
define DISABLE_SERIAL_ESC_COMM TRUE

define DMA_RESERVE_SIZE 0
define HAL_NO_MONITOR_THREAD

# keep ROMFS uncompressed as we don't have enough RAM
# to uncompress the bootloader at runtime
env ROMFS_UNCOMPRESSED True




define SERIAL_BUFFERS_SIZE 512
define PORT_INT_REQUIRED_STACK 64
define HAL_DEVICE_THREAD_STACK 0x300
define AP_PARAM_MAX_EMBEDDED_PARAM 512

define HAL_DISABLE_LOOP_DELAY

PB0 LED_GREEN2 OUTPUT LOW GPIO(11)  # GREEN