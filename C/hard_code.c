#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>

#include "bcm2835.h"

#define BCM2708_PERI_BASE   0x20000000
#define BCM2708_GPIO_BASE   (BCM2708_PERI_BASE + 0x200000)
#define BCM2709_PERI_BASE   0x3F000000
#define BCM2709_GPIO_BASE   (BCM2709_PERI_BASE + 0x200000)

static volatile unsigned *gpio;
static int  mem_fd;

static int setup_gpiomem_access(void)
{
  if ((mem_fd = open("/dev/gpiomem", O_RDWR|O_SYNC)) < 0) {
    printf("HAL_PI_GPIO: can't open /dev/gpiomem:  %d - %s\n"
        "If the error is 'permission denied' then try adding the user who runs\n"
        "LinuxCNC to the gpio group: sudo gpasswd -a username gpio\n", errno, strerror(errno));
    return -1;
  }

  gpio = mmap(NULL, BCM2835_BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, 0);

  if (gpio == MAP_FAILED) {
    close(mem_fd);
    mem_fd = -1;
    printf( "HAL_PI_GPIO: mmap failed: %d - %s\n", errno, strerror(errno));
    return -1;
  }


  return 0;
}

static int  setup_gpio_access(int rev, int ncores)
{
  // open /dev/mem
  if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
      printf("HAL_PI_GPIO: can't open /dev/mem:  %d - %s\n",
		      errno, strerror(errno));
    return -1;
  }

  if (rev <= 2  || ncores <= 2)
       gpio = mmap(NULL, BCM2835_BLOCK_SIZE, PROT_READ|PROT_WRITE,
		   MAP_SHARED, mem_fd, BCM2708_GPIO_BASE);
    else
       gpio = mmap(NULL, BCM2835_BLOCK_SIZE, PROT_READ|PROT_WRITE,
		   MAP_SHARED, mem_fd, BCM2709_GPIO_BASE);

  if (gpio == MAP_FAILED) {
    printf(
		    "HAL_PI_GPIO: mmap failed: %d - %s\n",
		    errno, strerror(errno));
    return -1;;
  }
  return 0;
}

static __inline__ void bcm2835_peri_write(volatile uint32_t* paddr, uint32_t value)
{
  // Make sure we don't rely on the first write, which may get
  // lost if the previous access was to a different peripheral.
  *paddr = value;
  *paddr = value;
}
static __inline__ uint32_t bcm2835_peri_read(volatile uint32_t* paddr)
{
  // Make sure we dont return the _last_ read which might get lost
  // if subsequent code changes to a different peripheral
  volatile uint32_t ret = *paddr;
  return ret;
}

static __inline__ void bcm2835_peri_set_bits(volatile uint32_t* paddr, uint32_t value, uint32_t mask)
{
  uint32_t v = bcm2835_peri_read(paddr);
  v = (v & ~mask) | (value & mask);
  bcm2835_peri_write(paddr, v);
}

void bcm2835_gpio_fsel(uint8_t pin, uint8_t mode)
{
  // Function selects are 10 pins per 32 bit word, 3 bits per pin
  volatile uint32_t* paddr = gpio + BCM2835_GPFSEL0/4 + (pin/10);
  uint8_t   shift = (pin % 10) * 3;
  uint32_t  mask = BCM2835_GPIO_FSEL_MASK << shift;
  uint32_t  value = mode << shift;
  bcm2835_peri_set_bits(paddr, value, mask);
}

static __inline__ void bcm2835_gpio_set(uint8_t pin)
{
  volatile uint32_t* paddr = gpio + BCM2835_GPSET0/4 + pin/32;
  uint8_t shift = pin % 32;
  bcm2835_peri_write(paddr, 1 << shift);
}

static __inline__ void bcm2835_gpio_clr(uint8_t pin)
{
  volatile uint32_t* paddr = gpio + BCM2835_GPCLR0/4 + pin/32;
  uint8_t shift = pin % 32;
  bcm2835_peri_write(paddr, 1 << shift);
}

int main() {
  setup_gpiomem_access();
  setup_gpio_access(5, 4);
  bcm2835_gpio_fsel(2,BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_set(2);
  uint32_t u = -1;
  while(u--);
  bcm2835_gpio_clr(2);

  if (gpio)
    munmap((void *) gpio, BCM2835_BLOCK_SIZE);
  if (mem_fd > -1)
      close(mem_fd);
}
