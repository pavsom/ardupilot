# hw definition file for processing by chibios_pins.py

# MCU class and specific type
MCU STM32F4xx STM32F405xx

FLASH_RESERVE_START_KB 0
FLASH_BOOTLOADER_LOAD_KB 64

# reserve some space for params
APP_START_OFFSET_KB 4

# board ID for firmware load
APJ_BOARD_ID 1014

# setup build for a peripheral firmware
env AP_PERIPH 1

# crystal frequency
OSCILLATOR_HZ 8000000

#define CH_CFG_ST_FREQUENCY 1000

# assume 256k flash part
FLASH_SIZE_KB 1024

STDOUT_SERIAL SD1
STDOUT_BAUDRATE 57600


# order of UARTs
SERIAL_ORDER OTG1 USART1


# USART1
PA9  USART1_TX USART1 SPEED_HIGH NODMA
PA10 USART1_RX USART1 SPEED_HIGH NODMA

# USART2
PA2 USART2_TX USART2 SPEED_HIGH NODMA
PA3 USART2_RX USART2 SPEED_HIGH NODMA

PC10  USART3_TX USART3
PC11  USART3_RX USART3

PA0 UART4_TX UART4
PA1 UART4_RX UART4

PC12 UART5_TX UART5
PD2  UART5_RX UART5

PA11 OTG_FS_DM OTG1
PA12 OTG_FS_DP OTG1

# enable CAN support
PB8 CAN1_RX CAN1
PB9 CAN1_TX CAN1

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
PC1 STAY_IN_BOOTLOADER INPUT FLOATING


# Add CS pins to ensure they are high in bootloader
#PB0 MAG_CS CS
