#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

static void gpio_setup(void) {
  rcc_periph_clock_enable(RCC_GPIOD);
  gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12);
}

int main(void) {
  int i;

  gpio_setup();

  for(;;) {
    gpio_clear(GPIOC, GPIO12);
    for(i=0; i<1500000; i++)
      __asm__("nop");

    gpio_set(GPIOC, GPIO12);
    for(i=0; i<500000; i++)
      __asm__("nop");
  }
  return 0;
}