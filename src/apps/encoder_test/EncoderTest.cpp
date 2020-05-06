#include "Config.h"

#include "drivers/Encoder.h"
#include "drivers/Console.h"
#include "drivers/System.h"
#include "drivers/HighResolutionTimer.h"

#include "rtos/Os.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>


// static Encoder encoder(HardwareConfig::reverseEncoder());
static Encoder encoder;

static constexpr uint32_t TaskAliveCount = 4;
static constexpr uint32_t TaskAliveMask = (1 << TaskAliveCount) - 1;
static uint32_t taskAliveState;

static void taskAlive(uint32_t task) {
    os::InterruptLock lock;
    taskAliveState |= (1 << task);
    if ((taskAliveState & TaskAliveMask) == TaskAliveMask) {
        System::resetWatchdog();
        taskAliveState = 0;
    }
}

static os::PeriodicTask<CONFIG_DRIVER_TASK_STACK_SIZE> driverTask("driver", CONFIG_DRIVER_TASK_PRIORITY, os::time::ms(1), [] () {
    encoder.process();
    taskAlive(0);
});

static os::PeriodicTask<CONFIG_DRIVER_TASK_STACK_SIZE> ledTask("leds", CONFIG_DRIVER_TASK_PRIORITY, os::time::ms(300), [] () {
    gpio_toggle(GPIOD, GPIO12);
    taskAlive(1);
});

/*
static void assert_handler(const char *filename, int line, const char *msg) {
    ui.showAssert(filename, line, msg);
    // keep watchdog satisfied but reset on encoder down
    while (1) {
        System::resetWatchdog();
        encoder.process();
        Encoder::Event event;
        if (encoder.nextEvent(event) && event == Encoder::Event::Down) {
            System::reset();
            while (1) {}
        }
    }
}
*/

int main(void) {
    System::init();
    System::startWatchdog(1000);
    Console::init();
    HighResolutionTimer::init();

    //HighResolutionTimer::init();

    //dbg_set_assert_handler(&assert_handler);

    //profiler.init();
    rcc_periph_clock_enable(RCC_GPIOD);
    gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12);

    encoder.init();

    System::resetWatchdog();

	os::startScheduler();
}
