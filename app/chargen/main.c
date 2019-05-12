#include <stdint.h>
#include <cnc1800l.h>
#include <ns16550.h>

#define GPIO_READ 	0
#define GPIO_WRITE	1

/**********************************************************************/

// from enum def in include/mach/mux.h
#define PINMUX_UART 2

// from ./cnc1800l_mux.c
#define  CNC18XX_PINMUX_BASE 0xB2110000
#define  CNC18XX_PINMUX_SIZE 0x100
#define REG_PINMUX_UART 0x008

static volatile uint16_t *pinmux_base = (char *)CNC18XX_PINMUX_BASE;

/*
  どこかの18, 19bit目のstatus bitを立てる必要がありそう
  if (strncmp("UART0", module, 5)==0){
  if (cnc_module_status == 1){
  gpio_configure_status_bit(18,"SET",1);
  gpio_configure_status_bit(19,"SET",1);
  }

  // 以下でPINMUX設定をしているようだ
  case PINMUX_UART:
  if(set) pinmux_base[REG_PINMUX_UART>>1] |= (1<<(bit-1));
  else pinmux_base[REG_PINMUX_UART>>1] &= ~(1<<(bit-1));
  break;

  // kernel module内のpinmux_baseはioremapされているもの
  pinmux_base =
  (volatile unsigned short *) ioremap(CNC18XX_PINMUX_BASE,
  CNC18XX_PINMUX_SIZE);

  // これがPAかな？
  cnc1800l_mux.c:#define  CNC18XX_PINMUX_BASE 0xB2110000
 */

/**********************************************************************/

/*
  // see u-boot ./include/configs/cnc1800l.h
  #define CONFIG_SYS_NS16550
  #define CONFIG_SYS_NS16550_SERIAL
  #define CONFIG_SYS_NS16550_CLK          CONFIG_SYS_PCLK_FREQ
  #define CONFIG_CONS_INDEX               1
  #define CONFIG_SYS_NS16550_REG_SIZE     (-4)
  #define CONFIG_BAUDRATE                 115200
  #define CONFIG_SYS_BAUDRATE_TABLE       { 9600, 19200, 38400, 57600, 115200 }
  #define CONFIG_SYS_NS16550_COM1         0x801F1000
  #define CONFIG_SYS_NS16550_COM2         0x801F2000

  // see u-boot common/serial.c
  #if defined(CONFIG_SYS_NS16550_SERIAL)
  #if defined(CONFIG_SYS_NS16550_COM1)
  serial_register(&eserial1_device);
  #endif
  #if defined(CONFIG_SYS_NS16550_COM2)
  serial_register(&eserial2_device);
  #endif

  // from U-Boot console
  CNC1800L> coninfo
  serial 80000003 SIO stdin stdout stderr
         ^^^^^^^^stdio_devオブジェクトのdev->flags。下の3はS/I/Oに対応かな

  // see struct serial_device *__default_serial_console (void)
  #if defined(CONFIG_CONS_INDEX) && defined(CONFIG_SYS_NS16550_SERIAL)
  #if (CONFIG_CONS_INDEX==1)
        return &eserial1_device;

  このコードパスになっているはず？
  ただ、これは CONFIG_SERIAL_MULTI が定義されているケースの話で、該当しない？

  // drivers/serial/serial.c
  void
  _serial_putc(const char c,const int port) {
  if (c == '\n')
  NS16550_putc(PORT, '\r');
  NS16550_putc(PORT, c);
  }

  結局これ？

  void NS16550_putc (NS16550_t com_port, char c) {
  while ((serial_in(&com_port->lsr) & UART_LSR_THRE) == 0);
  serial_out(c, &com_port->thr);

  こうなって・・・

  #ifdef CONFIG_SYS_NS16550_PORT_MAPPED
  #define serial_out(x,y) outb(x,(ulong)y)
  #define serial_in(y)    inb((ulong)y)
  #else
  #define serial_out(x,y) writeb(x,y)
  #define serial_in(y)    readb(y)
  #endif

  こうなる。このwriteb/readbはARM genericなマクロを使っているかな？
  なおyがアドレス。

  #define writeb(v,c)     ({ u8  __v = v; __iowmb(); __arch_putb(__v,c); __v; })
  #define readb(c)        ({ u8  __v = __arch_getb(c); __iormb(); __v; })

  #define __arch_getb(a)                  (*(volatile unsigned char *)(a))
  #define __arch_putb(v,a)                (*(volatile unsigned char *)(a) = (v))

  なので、

  void NS16550_putc (NS16550_t com_port, char c) {
  while ((serial_in(&com_port->lsr) & UART_LSR_THRE) == 0);
  serial_out(c, &com_port->thr);

  このcom_port構造体でアドレスを管理しているようだが、これがどこで
  どう初期化されているのか、初期化パラメータは何なのか見えない。
 */

/*
  アドレスっぽいマクロはCONFIG_SYS_NS16550_COM1なので、これを使っている場所を
  見てみる。

  // from u-boot drivers/serial/serial.c
  static NS16550_t serial_ports[4] = {
  #ifdef CONFIG_SYS_NS16550_COM1
  (NS16550_t)CONFIG_SYS_NS16550_COM1,
  #else
  ...
  #if defined(CONFIG_SERIAL_MULTI)
  ...
  #define DECLARE_ESERIAL_FUNCTIONS(port)         \
  int  eserial##port##_init (void) {                \
    int clock_divisor;                                                  \
    clock_divisor = calc_divisor(serial_ports[port-1]);                 \
    NS16550_init(serial_ports[port-1], clock_divisor);                  \
    return(0);}                                                         \
  void eserial##port##_setbrg (void) {                                  \
      serial_setbrg_dev(port);}                                         \
  ...

  CONFIG_SERIAL_MULTIが生きているか曖昧だが、
  ここで各所のeserial[1234]_*を定義している。コンパイル時に作っているので
  ソースで追いかけても出ないわけだ。

        clock_divisor = calc_divisor(serial_ports[port-1]); \
        NS16550_init(serial_ports[port-1], clock_divisor); \

  これがキモだな。

  // from u-boot drivers/serial/serial.c
  static int calc_divisor (NS16550_t port) ...
  ...
  #define MODE_X_DIV 16
  // Compute divisor value. Normally, we should simply return:
  //
  //   CONFIG_SYS_NS16550_CLK) / MODE_X_DIV / gd->baudrate
  //
  // but we need to round that value by adding 0.5.
  // Rounding is especially important at high baud rates.
  return (CONFIG_SYS_NS16550_CLK + (gd->baudrate * (MODE_X_DIV / 2))) /
  (MODE_X_DIV * gd->baudrate);

  一部例外を除き機種共通でclock divider設定値を出せるようだ。
  で、NS16550_initの方は

  void NS16550_init (NS16550_t com_port, int baud_divisor) {
      serial_out(CONFIG_SYS_NS16550_IER, &com_port->ier);

  なるほど、NS16550_tの構造体がそのままレジスタ構成と同じっぽい。
      
  #if !defined(CONFIG_SYS_NS16550_REG_SIZE) || (CONFIG_SYS_NS16550_REG_SIZE == 0)
  #error "Please define NS16550 registers size."
  #elif (CONFIG_SYS_NS16550_REG_SIZE > 0)
  #define UART_REG(x)                                                \
      unsigned char prepad_##x[CONFIG_SYS_NS16550_REG_SIZE - 1];     \
      unsigned char x;
  #elif (CONFIG_SYS_NS16550_REG_SIZE < 0)
  #define UART_REG(x)                                                     \
      unsigned char x;                                                  \
      unsigned char postpad_##x[-CONFIG_SYS_NS16550_REG_SIZE - 1];
  #endif
      
  // from uboot include/ns16550.h
  struct NS16550 {
    UART_REG(rbr);
    ...
  };

  #define thr rbr
  #define iir fcr
  #define dll rbr
  #define dlm ier

  typedef volatile struct NS16550 *NS16550_t;

  これがメモリにマップされているでOKかな？このUART_REGマクロがちょい謎。

  $ grep -r CONFIG_SYS_NS16550_REG_SIZE |grep 1800l
  include/configs/cnc1800l.h:#define CONFIG_SYS_NS16550_REG_SIZE     (-4)

  なんだこれ？とみると、endian対応の話で、各レジスタは8bitなのだが
  配置は32bit単位になっている。で、先頭8bitなのか末尾8bitなのかを
  吸収するためにマクロで前後どちらか3byteをpaddingしている。

 */

//
#define CS_IRQ_UART0 12
#define PA_IO_REGS_BASE 0x80100000
#define PA_UART0_BASE (PA_IO_REGS_BASE + 0xf1000)

static volatile NS16550_t com1 = (NS16550_t)CONFIG_SYS_NS16550_COM1;

/**********************************************************************/

#define UART_LCRVAL UART_LCR_8N1
#define UART_MCRVAL (UART_MCR_DTR|UART_MCR_RTS)
#define UART_FCRVAL (UART_FCR_FIFO_EN | UART_FCR_RXSR | UART_FCR_TXSR)

#ifndef CONFIG_SYS_NS16550_IER
#define CONFIG_SYS_NS16550_IER  0x00
#endif

// from globaldata struct in u-boot
#define gd_baudrate 115200

// from drivers/serial/serial.c
#define MODE_X_DIV 16
static int
calc_divisor(NS16550_t port) {
    /* Compute divisor value. Normally, we should simply return:
     *   CONFIG_SYS_NS16550_CLK) / MODE_X_DIV / gd->baudrate
     * but we need to round that value by adding 0.5.
     * Rounding is especially important at high baud rates.
     */
    return (CONFIG_SYS_NS16550_CLK + (gd_baudrate * (MODE_X_DIV / 2))) /
        (MODE_X_DIV * gd_baudrate);
}

#define readb(y) \
    (*(volatile uint32_t *)y)
    
#define writeb(x, y) \
    do { *(volatile uint32_t *)y = x; } while (0)

#define serial_out(x, y) writeb(x, y)
#define serial_in(y) readb(y)

void
NS16550_init(NS16550_t com_port, int baud_divisor) {
    serial_out(CONFIG_SYS_NS16550_IER, &com_port->ier);
    serial_out(UART_LCR_BKSE | UART_LCRVAL, (uint32_t)&com_port->lcr);
    serial_out(0, &com_port->dll);
    serial_out(0, &com_port->dlm);
    serial_out(UART_LCRVAL, &com_port->lcr);
    serial_out(UART_MCRVAL, &com_port->mcr);
    serial_out(UART_FCRVAL, &com_port->fcr);
    serial_out(UART_LCR_BKSE | UART_LCRVAL, &com_port->lcr);
    serial_out(baud_divisor & 0xff, &com_port->dll);
    serial_out((baud_divisor >> 8) & 0xff, &com_port->dlm);
    serial_out(UART_LCRVAL, &com_port->lcr);
}

void
NS16550_reinit(NS16550_t com_port, int baud_divisor) {
    serial_out(CONFIG_SYS_NS16550_IER, &com_port->ier);
    serial_out(UART_LCR_BKSE | UART_LCRVAL, &com_port->lcr);
    serial_out(0, &com_port->dll);
    serial_out(0, &com_port->dlm);
    serial_out(UART_LCRVAL, &com_port->lcr);
    serial_out(UART_MCRVAL, &com_port->mcr);
    serial_out(UART_FCRVAL, &com_port->fcr);
    serial_out(UART_LCR_BKSE, &com_port->lcr);
    serial_out(baud_divisor & 0xff, &com_port->dll);
    serial_out((baud_divisor >> 8) & 0xff, &com_port->dlm);
    serial_out(UART_LCRVAL, &com_port->lcr);
}

void
NS16550_putc(NS16550_t com_port, char c) {
    while ((serial_in(&com_port->lsr) & UART_LSR_THRE) == 0);
    serial_out(c, &com_port->thr);
}

/**********************************************************************/

static volatile char *gpio_base  = (char *)CNC1800L_GPIO_BASE;

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

void
init_uart(void) {
    // see:
    // - int pinmux_enable_uart(UART_ID UART)
    // - int pinmux_enable_irda(void)
    pinmux_base[REG_PINMUX_UART>>1] |= 1;

    // init COM1
    int clock_divisor = calc_divisor(com1);
    NS16550_init(com1, clock_divisor);

    NS16550_putc(com1, 'h');
    NS16550_putc(com1, 'e');
    NS16550_putc(com1, 'l');
    NS16550_putc(com1, 'l');
    NS16550_putc(com1, 'o');
    NS16550_putc(com1, '\r');
    NS16550_putc(com1, '\n');
}

void __attribute__((section(".text.main")))
main(void) {
    init_uart();

    for (;;) {
        gpio_hw_set_direct(2, GPIO_WRITE);
        gpio_hw_write(2, 0);
        delay(3000000);
        gpio_hw_set_direct(2, GPIO_WRITE);
        gpio_hw_write(2, 1);
        delay(3000000);
    }
}
