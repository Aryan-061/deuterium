#include "raspi.hpp"

int recstate = 0;
int recbuffindex = 0;
uint8_t recbuff[18];


void raspi::init() {
    uart_init(RASPI_UARTID, RASPI_BAUDRATE);
    gpio_set_function(RASPI_TX, GPIO_FUNC_UART);
    gpio_set_function(RASPI_RX, GPIO_FUNC_UART);
}

void raspi::update() {

    while (uart_is_readable(RASPI_UARTID)) {
        switch (recstate) {

        case 0:
            if (uart_getc(RASPI_UARTID) == RASPI_SOF0)
                recstate = 1;
            break;

        case 1:
            if (uart_getc(RASPI_UARTID) == RASPI_SOF1)
                recstate = 2;
            break;

        case 2:
            recbuff[recbuffindex] = uart_getc(RASPI_UARTID);
            recbuffindex++;
            if (recbuffindex >= 18) {
                recbuffindex = 0;
                recstate = 0;
            }
            break;

        }
    }
}