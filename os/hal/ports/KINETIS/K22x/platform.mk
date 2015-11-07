# List of all platform files.
PLATFORMSRC = ${CHIBIOS}/os/hal/ports/common/ARMCMx/nvic.c \
              ${CHIBIOS}/os/hal/ports/KINETIS/K22x/hal_lld.c \
              ${CHIBIOS}/os/hal/ports/KINETIS/K22x/pal_lld.c \
              ${CHIBIOS}/os/hal/ports/KINETIS/K22x/serial_lld.c \
              ${CHIBIOS}/os/hal/ports/KINETIS/K22x/spi_lld.c \
              ${CHIBIOS}/os/hal/ports/KINETIS/LLD/i2c_lld.c \
              ${CHIBIOS}/os/hal/ports/KINETIS/LLD/ext_lld.c \
              ${CHIBIOS}/os/hal/ports/KINETIS/LLD/adc_lld.c \
              ${CHIBIOS}/os/hal/ports/KINETIS/K22x/gpt_lld.c \
              ${CHIBIOS}/os/hal/ports/KINETIS/K22x/st_lld.c \
              ${CHIBIOS}/os/hal/ports/KINETIS/K22x/i2s_lld.c \
              ${CHIBIOS}/os/hal/ports/KINETIS/K22x/fsl_mcg_hal.c \
              ${CHIBIOS}/os/hal/ports/KINETIS/K22x/mcg_lld.c 

# Required include directories
PLATFORMINC = ${CHIBIOS}/os/hal/ports/common/ARMCMx \
              ${CHIBIOS}/os/hal/ports/KINETIS/K22x \
              ${CHIBIOS}/os/hal/ports/KINETIS/LLD
