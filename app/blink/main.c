#include <stdint.h>
#include <cnc1800l.h>

#define GPIO_READ 	0
#define GPIO_WRITE	1

static volatile char *gpio_base  = CNC1800L_GPIO_BASE;

int
gpio_hw_set_direct(int gpio_id, int dir) {
    uint32_t flags;

    flags = (1 << gpio_id);
    switch (dir) {
    case GPIO_READ:
        *(uint32_t *)(gpio_base+REG_GPIO_SWPORTA_DDR) =(*(uint32_t *)(CNC1800L_GPIO_BASE+REG_GPIO_SWPORTA_DDR))|(flags);
        break;
    case GPIO_WRITE:
        *(uint32_t *)(gpio_base+REG_GPIO_SWPORTA_DDR) =(*(uint32_t *)(CNC1800L_GPIO_BASE+REG_GPIO_SWPORTA_DDR))&(~flags);
        break;
    default:
        return -1;
    }
    return 0;
}

unsigned short
gpio_hw_read(int gpio_id) {
    gpio_hw_set_direct(gpio_id, GPIO_READ);

    return ((*(uint32_t *)(CNC1800L_GPIO_BASE+REG_GPIO_EXT_PORTA))>>gpio_id) & 0x1;
}

int
gpio_hw_write(int gpio_id, unsigned short data) {
    uint32_t flags ;
    gpio_hw_set_direct(gpio_id, GPIO_WRITE);

    flags = (1<<gpio_id);
    if (!data) {
        *(uint32_t *)(gpio_base+REG_GPIO_SWPORTA_DR) =(*(uint32_t *)(CNC1800L_GPIO_BASE+REG_GPIO_SWPORTA_DR))&(~flags);
    }
    else {
        *(uint32_t *)(gpio_base+REG_GPIO_SWPORTA_DR) =(*(uint32_t *)(CNC1800L_GPIO_BASE+REG_GPIO_SWPORTA_DR))|(flags);
    }
    return 0;
}

void
delay(uint32_t n) {
    while (n--) {
        asm("nop");
    }
}

void __attribute__((section(".text.main")))
main(void) {
    for (;;) {
        gpio_hw_set_direct(2, GPIO_WRITE);
        gpio_hw_write(2, 0);
        delay(3000000);
        gpio_hw_set_direct(2, GPIO_WRITE);
        gpio_hw_write(2, 1);
        delay(3000000);
    }
}
