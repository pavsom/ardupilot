# hw definition file for processing by chibios_pins.py

# MCU class and specific type
MCU STM32F4xx STM32F405xx

# crystal frequency
OSCILLATOR_HZ 8000000

# board ID for firmware load
APJ_BOARD_ID 1014

# setup build for a peripheral firmware
env AP_PERIPH 1

FLASH_RESERVE_START_KB 0
FLASH_BOOTLOADER_LOAD_KB 64
# assume 256k flash part
FLASH_SIZE_KB 1024

PC8 LED_BOOTLOADER OUTPUT LOW GPIO(0)  # blue
define HAL_LED_ON 1

# reserve some space for params
#APP_START_OFFSET_KB 4

# order of UARTs
SERIAL_ORDER USART1

STDOUT_SERIAL SD1
STDOUT_BAUDRATE 57600



#define CH_CFG_ST_FREQUENCY 1000

# USART1, for debug
PA9  USART1_TX USART1 SPEED_HIGH 
PA10 USART1_RX USART1 SPEED_HIGH 

# USART2
PA2 USART2_TX USART2 SPEED_HIGH NODMA
PA3 USART2_RX USART2 SPEED_HIGH NODMA

PA7 EN_PWR_485_1 OUTPUT GPIO(1)
PA5 PWR_485_1 INPUT GPIO(2)
#PA1 EN_TX_485_1 OUTPUT (3)

# USART6, for MSP
PC6 USART6_TX USART6 SPEED_HIGH NODMA
PC7 USART6_RX USART6 SPEED_HIGH NODMA

PA6 EN_PWR_485_2 OUTPUT GPIO(4)
PA4 PWR_485_2 INPUT GPIO(5)
#PC8 EN_TX_485_2 OUTPUT (6)


# ---------------------- CAN bus -------------------------
define HAL_CAN_POOL_SIZE 6000
# enable CAN support
PB8 CAN1_RX CAN1
PB9 CAN1_TX CAN1

PB12 CAN2_RX CAN2
PB13 CAN2_TX CAN2


define HAL_USE_CAN TRUE
define STM32_CAN_USE_CAN1 TRUE

#define CAN_APP_NODE_NAME "org.ardupilot.f405_MatekGPS"

# debugger support
PA13 JTMS-SWDIO SWD
PA14 JTCK-SWCLK SWD

# use PB6 (normally I2C1_SCL) as "hold in bootloader" pin
# this has a hw pullup, so if we set it as input floating
# and look for it low then we know user has pulled it down and
# want to stay in the bootloader
#PC1 STAY_IN_BOOTLOADER INPUT FLOATING


# Add CS pins to ensure they are high in bootloader
#PB0 MAG_CS CS
