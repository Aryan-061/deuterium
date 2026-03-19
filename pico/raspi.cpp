#include "raspi.hpp"

void raspi::init() {
    uart_init(RP_UARTID, RP_BAUDRATE);
    gpio_set_function(RP_TX, GPIO_FUNC_UART);
    gpio_set_function(RP_RX, GPIO_FUNC_UART);
}

void raspi::update() {

}