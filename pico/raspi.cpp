#include "raspi.hpp"

extern RaspiRecFrame raspi_rec_frame;

void raspi::init() {
    uart_init(RP_UARTID, RP_BAUDRATE);
    gpio_set_function(RP_TX, GPIO_FUNC_UART);
    gpio_set_function(RP_RX, GPIO_FUNC_UART);
}

void raspi::update() {
    raspi_rec_frame.sof = uart_getc(RP_UARTID);
    raspi_rec_frame.dx = uart_getc(RP_UARTID);
    raspi_rec_frame.dy = uart_getc(RP_UARTID);
    raspi_rec_frame.dz = uart_getc(RP_UARTID);
    raspi_rec_frame.dyaw = uart_getc(RP_UARTID);
    raspi_rec_frame.crc0 = uart_getc(RP_UARTID);
    raspi_rec_frame.crc1 = uart_getc(RP_UARTID);
}