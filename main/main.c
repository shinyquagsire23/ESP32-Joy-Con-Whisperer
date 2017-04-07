/*
* ESP32 Joy-Con Whisperer
*
* Joy-Con pins 5, 6, 8 and 10 go to ESP32 GPIO 10, 27, 26, and 14
* Joy-Con pin 2 must be pulled to GND for communication to occur after handshake
* Joy-Con pin 4 can have 5V applied for charging
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "soc/uart_struct.h"

#define BUF_SIZE (1024)

char magic_start[0x4] = {0xA1, 0xA2, 0xA3, 0xA4};
char handshake_start[0xC] = {0x19, 0x01, 0x03, 0x07, 0x00, 0xA5, 0x02, 0x01, 0x7E, 0x00, 0x00, 0x00};
char get_mac[0xC] = {0x19, 0x01, 0x03, 0x07, 0x00, 0x91, 0x01, 0x00, 0x00, 0x00, 0x00, 0x24};
char switch_baud[0x14] = {0x19, 0x01, 0x03, 0x0F, 0x00, 0x91, 0x20, 0x08, 0x00, 0x00, 0xBD, 0xB1, 0xC0, 0xC6, 0x2D, 0x00, 0x00, 0x00, 0x00, 0x00};
char controller_status[0xD] = {0x19, 0x01, 0x03, 0x08, 0x00, 0x92, 0x00, 0x01, 0x00, 0x00, 0x69, 0x2D, 0x1F};
char unk_1[0xC] = {0x19, 0x01, 0x03, 0x07, 0x00, 0x91, 0x11, 0x00, 0x00, 0x00, 0x00, 0x0E};
char unk_2[0xC] = {0x19, 0x01, 0x03, 0x07, 0x00, 0x91, 0x10, 0x00, 0x00, 0x00, 0x00, 0x3D};
char unk_3[0x10] = {0x19, 0x01, 0x03, 0x0B, 0x00, 0x91, 0x12, 0x04, 0x00, 0x00, 0x12, 0xA6, 0x0F, 0x00, 0x00, 0x00};
char read_spi[0x3D] = {0x19, 0x1, 0x3, 0x38, 0x0, 0x92, 0x0, 0x31, 0x0, 0x0, 0xd4, 0xe6, 0x1, 0xc, 0x0, 0x1, 0x40, 0x40, 0x0, 0x1, 0x40, 0x40, 0x10, 0x0, 0x0, 0x0, 0x0, 0x1C, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};

void joycon_send_command_dat(int uart_num, char *command, int command_len, char *out, int expected_len)
{
    while(1) {
        //Write data back to UART
        uart_write_bytes(uart_num, command, command_len);
        if(uart_wait_tx_done(uart_num, 10) == ESP_ERR_TIMEOUT)
        {
            printf("Command send timed out.\n");
            continue;
        }
        
        int len = uart_read_bytes(uart_num, (void*)out, BUF_SIZE, 20 / portTICK_RATE_MS);
        
        
        if(len == expected_len || expected_len == 0)
        {
            for(int i = 0x20; i < len-1; i++)
            {
                printf("%02x ", out[i]);
            }
            break;
        }
            
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}


void joycon_send_command(int uart_num, char *command, int command_len, char *out, int expected_len)
{
    while(1) {
        //Write data back to UART
        uart_write_bytes(uart_num, command, command_len);
        if(uart_wait_tx_done(uart_num, 10) == ESP_ERR_TIMEOUT)
        {
            printf("Command send timed out.\n");
            continue;
        }
        
        int len = uart_read_bytes(uart_num, (void*)out, BUF_SIZE, 20 / portTICK_RATE_MS);
        printf("Got data! %08x", len);
        
        for(int i = 0; i < len; i++)
        {
            printf(" %02x", out[i]);
        }
        printf("\n");
        
        if(len == expected_len || expected_len == 0)
            break;
            
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

static void joycon_whisperer_task()
{
    uint32_t ret = 0;

    const int uart_num = UART_NUM_2;
    uart_config_t uart_config = {
        .baud_rate = 1000000,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
        .rx_flow_ctrl_thresh = 122,
    };
    //Configure UART1 parameters
    uart_param_config(uart_num, &uart_config);
    //Set UART1 pins(TX: IO4, RX: I05, RTS: IO18, CTS: IO19)
    uart_set_pin(uart_num, 10, 26, 14, 27);
    uart_set_line_inverse(uart_num, UART_INVERSE_TXD | UART_INVERSE_RTS);
    //Install UART driver (we don't need an event queue here)
    //In this example we don't even use a buffer for sending data.
    ret = uart_driver_install(uart_num, BUF_SIZE * 2, 0, 0, NULL, 0);
    printf("driver install %08x\n", ret);
    
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    char *ret_data = malloc(BUF_SIZE);
    while(1) {
        vTaskDelay(100 / portTICK_PERIOD_MS);
        
        //Write data back to UART
        uart_write_bytes(uart_num, magic_start, 0x4);
        if(uart_wait_tx_done(uart_num, 1) == ESP_ERR_TIMEOUT)
        {
            printf("Magic send timed out.\n");
            continue;
        }
        uart_write_bytes(uart_num, handshake_start, 0xC);
        if(uart_wait_tx_done(uart_num, 1) == ESP_ERR_TIMEOUT)
        {
            printf("Command send timed out.\n");
            continue;
        }
        
        int len = uart_read_bytes(uart_num, (void*)ret_data, BUF_SIZE, 20 / portTICK_RATE_MS);
        printf("Got data! %08x", len);
        
        for(int i = 0; i < len; i++)
        {
            printf(" %02x", ret_data[i]);
        }
        printf("\n");
        
        if(len == 0xC)
            break;
    }
    
    joycon_send_command(uart_num, get_mac, 0xC, ret_data, 0x14);
    //joycon_send_command(uart_num, switch_baud, 0x14, ret_data, 0xC);
    //uart_set_baudrate(uart_num, 3125000);
    //printf("Baud Switched\n");
    
    joycon_send_command(uart_num, unk_1, 0xC, ret_data, 0xC);
    printf("unk1 done\n");
    joycon_send_command(uart_num, unk_2, 0xC, ret_data, 0xC);
    printf("unk2 done\n");
    joycon_send_command(uart_num, unk_3, 0x10, ret_data, 0xC);
    printf("unk3 done\n");
    
    vTaskDelay(500 / portTICK_PERIOD_MS);
    
    // Poll input forever
    /*while(1)
    {
        joycon_send_command(uart_num, controller_status, 0xD, ret_data, 0x3D);
    }*/
    
    // Have the Joy-Con tell us its deepest, darkest secrets
    // by doing a hex dump of SPI flash.
    //
    // Reads begin to mirror after 0x80000.
    while(1)
    {
        joycon_send_command_dat(uart_num, read_spi, 0x3D, ret_data, 0x3D);
        *(uint32_t*)(&read_spi[0x17]) += 0x1C;
    }
}

void app_main()
{
    xTaskCreate(&joycon_whisperer_task, "joycon_whisperer_task", 2048, NULL, 1, NULL);
}
