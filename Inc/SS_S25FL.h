#include "stm32f4xx_hal.h"
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"


extern uint8_t  MEASURE_ENABLE_FLAG;
extern uint32_t global_curr_flash_page;
extern uint32_t global_flash_timestamp ;
extern uint8_t DMA_TX_IN_PROGRESS;
extern uint8_t DMA_RX_IN_PROGRESS;

typedef struct{
    uint8_t byte_num_of_zeroes; //from 7 to 0  MSB---> LSB
    uint32_t byte_addr;
} Addr_section_last_written_t;
typedef enum { TX,RX } DATA_TRANSFER_DIR_t;
typedef enum { ERR,OK } TIMEOUT_STATE_t;
typedef enum { B_BUSY,B_IDLE } B_STATE_t;
typedef enum { B_ERROR,B_SUCCESS } ERR_FLAG_t;
typedef enum { FULL,EMPTY,NOT_FULL } FLASH_SPACE_STATE_t;

#define TRUE 1
#define FALSE 0

//------------------USER DEFINES ( YOU CAN CHANGE IT :) )------------------------------->
#define DEBUG_LVL_1
//#define DEBUG_LVL_2
//#define DEBUG_LVL_3

#define FLASH_TIMESTAMP_INC_FACTOR 50

#define MAX_HAL_TIMEOUT 250       //250 ms
#define MAX_SPI_DMA_TIMEOUT 250
#define MAX_WAIT_4_FLASH_TIMEOUT 1000    //1 s
#define MAX_WAIT_4_FLASH_ERASE_TIMEOUT 300000 //5 min

#define FLASH_256MB
//#define FLASH_512MB

#define HAL_SPI_INSTANCE SPI2
#define HAL_TIM_INSTANCE TIM2

extern SPI_HandleTypeDef hspi2;
extern DMA_HandleTypeDef hdma_spi2_tx;
extern DMA_HandleTypeDef hdma_spi2_rx;

#define SPI_STRUCT hspi2
#define SPI_DMA_TX_STRUCT hdma_spi2_tx
#define SPI_DMA_RX_STRUCT hdma_spi2_rx
#define UART_DEBUG_STRUCT huart2
//---------------------------------------------------------------------------------------<
//------------------------------------------------->
#if defined  FLASH_256MB

#define FLASH_SIZE_BYTES ((uint32_t) 256000000 )    //256Mb
#define FIRST_LOG_PAGE   ((uint32_t) 501 )

#elif defined FLASH_512MB

#define FLASH_SIZE_BYTES ((uint32_t) 512000000 )   //512Mb
#define FIRST_LOG_PAGE   ((uint32_t) 1001 )

#endif

#define FLASH_MAX_SIZE_OFFSET ((uint32_t) 50 ) // aby nie powzwolic na dotarcie do konca pamieci

#define FLASH_SIZE_PAGES ( (FLASH_SIZE_BYTES/256) - FLASH_MAX_SIZE_OFFSET )
//-------------------------------------------------<
//------------------------------------------------->
#define __print_debug_to_UART(x) debug_tx_length = sprintf(debug_tx_char_buff,x);\
HAL_UART_Transmit(&UART_DEBUG_STRUCT,(uint8_t *)debug_tx_char_buff,debug_tx_length, MAX_HAL_TIMEOUT );

#define __print_debug_to_UART_1(x,y) debug_tx_length = sprintf(debug_tx_char_buff,x,y);\
HAL_UART_Transmit(&UART_DEBUG_STRUCT,(uint8_t *)debug_tx_char_buff,debug_tx_length, MAX_HAL_TIMEOUT );

#define __print_debug_to_UART_2(x,y,z) debug_tx_length = sprintf(debug_tx_char_buff,x,y,z);\
HAL_UART_Transmit(&UART_DEBUG_STRUCT,(uint8_t *)debug_tx_char_buff,debug_tx_length, MAX_HAL_TIMEOUT );

#define __print_debug_to_UART_3(x,y,z,m) debug_tx_length = sprintf(debug_tx_char_buff,x,y,z,m);\
HAL_UART_Transmit(&UART_DEBUG_STRUCT,(uint8_t *)debug_tx_char_buff,debug_tx_length, MAX_HAL_TIMEOUT );
//-------------------------------------------------<
// --------------------------DEBUG----------------->
#if defined DEBUG_LVL_1 || defined DEBUG_LVL_2 || defined DEBUG_LVL_3
char debug_tx_char_buff [500];
uint16_t debug_tx_length;
#endif
// ----------------------------DEBUG---------------<

void SS_S25FL_reset_init (void);
void SS_S25FL_saving_init(void);

void SS_S25FL__dma_transfer_cs_handler (void);
void SS_S25FL_inc_timestamp_handler(void);  // place somewhere in a timer interupt routine (called every 50 uS?)

void SS_S25FL_erase_only_written_pages (void);
ERR_FLAG_t SS_S25FL_erase_full_chip(void);
void SS_S25FL_start_logging(void);
void SS_S25FL_stop_logging(void);
ERR_FLAG_t SS_S25FL_save_variable_u32(uint8_t variable_id, uint32_t data);
uint32_t SS_S25FL_get_free_space_in_pages(void);
TIMEOUT_STATE_t SS_S25FL_get_timeout (void);

///--------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef DEBUG_LVL_1 // --------------------------DEBUG----------------->
void SS_S25FL_dump_page_to_uart(uint32_t page_num);
void SS_S25FL_read_data_logs_to_uart(uint8_t id);
#endif

/*
uint32_t SS_S25FL_get_addr_from_page (uint32_t page_number ,uint8_t offset);
uint32_t SS_S25FL_get_addr_from_sector (uint32_t sector_number ,uint16_t offset) ;
uint32_t SS_S25FL_get_addr_from_halfblock (uint32_t halfblock_number ,uint16_t offset);
uint32_t SS_S25FL_get_addr_from_block (uint32_t block_number ,uint16_t offset) ;
//-------------------------------------
uint32_t SS_S25FL_get_sector_num_from_page(uint32_t page_num);
uint32_t SS_S25FL_get_page_num_from_addr(uint32_t addr);
// --------------------- DATA TRANSFER STATE HANDLING FUNC-------------->>>>>>>>
B_STATE_t SS_S25FL_get_dma_transfer_state (DATA_TRANSFER_DIR_t x);
B_STATE_t SS_S25FL_get_dma_spi_state (void);
void SS_S25FL_wait_4_spi_bus(void);
// --------------------- BASIC FLASH HANDLING FUNC-------------->>>>>>>>
void SS_S25FL_reset_init(void);
uint8_t SS_S25FL_read_status(void);
B_STATE_t SS_S25FL_check_write_progress(void);
uint8_t SS_S25FL_get_id(void);
void SS_S25FL_write_enable(void);
// --------------------- ERASE OPERATIONS WITHOUT DMA-------------->>>>>>>>
ERR_FLAG_t SS_S25FL_erase_sector(uint32_t sector_num) //Erases given sector
ERR_FLAG_t SS_S25FL_erase_halfblock(uint32_t adress) ////Erases given halfblock ---->  Not implemented , not tested !!!!!!!!!
ERR_FLAG_t SS_S25FL_erase_block(uint32_t block_num) // //Erases given block
ERR_FLAG_t SS_S25FL_erase_full_chip(void) // Erases full FLASH ,very long command , up to 140 s !!!
// --------------------- READ WRITE OPERATIONS WITHOUT DMA-------------->>>>>>>>
ERR_FLAG_t SS_S25FL_write_bytes(uint32_t adress, uint8_t *array, uint8_t length) // Write specified number of bytes to FLASH , non DMA
ERR_FLAG_t SS_S25FL_write_page(uint32_t page, uint8_t *array) // Write single page to FLASH , non DMA
ERR_FLAG_t SS_S25FL_read_page(uint32_t page, uint8_t *array) // Read single page from FLASH , non DMA
// --------------------- READ WRITE OPERATIONS WITH DMA-------------->>>>>>>>
void SS_S25FL_write_page_dma(uint32_t page, uint8_t *array) // Write single page to FLASH , with use of DMA
void SS_S25FL_read_page_dma(uint32_t page, uint8_t *array) // Read single page from FLASH , with use of DMA
// --------------------- ADDRESS SECTION HANDLING FUNTIONS-------------->>>>>>>
Addr_section_last_written_t SS_S25FL__addr_section__find_last_written_byte (void)
void SS_S25FL__addr_section__byte_zeroes_inc (void) // num. of zeroes overflow handling
uint32_t SS_S25FL__addr_section__get_last_log_page_init (void) // execute once on startup to get initial page number and init global var
uint32_t SS_S25FL__addr_section__get_last_log_page (void) // returns last log page (written)
void SS_S25FL__addr_section__inc_last_log_page (void) // icrements addr section by one zero
// --------------------- ERASE ONLY WRITTEN PAGES FUNC-------------->>>>>>>
void SS_S25FL_erase_in_page_range (uint32_t start_page, uint32_t stop_page) // erases in given page range including these pages
void SS_S25FL__addr_section__erase (void) // erases addr section
void SS_S25FL_erase_only_written_pages(void) // erases only written pages in addr section and log section
// --------------------- DATA LOGGING FUNC -------------->>>>>>>
void SS_S25FL_saving_init(void) // var logging init function , call once on startup
void SS_S25FL_start_logging(void) // call to start logging data to flash
void SS_S25FL_stop_logging(void) // call to stop logging data to flash
void SS_S25FL_save_variable_u32(uint8_t variable_id, uint32_t data)
// --------------------- OTHER -------------->>>>>>>
uint32_t SS_S25FL_get_free_space_in_pages(void) // call to get free space left in pages (256 bytes per page) -------> not tested!
TIMEOUT_STATE_t SS_S25FL_get_timeout(void) // get global timeout flag
//------------------------------ TEST FUNC-------------------------------- >>>>>>>>>>>>>>>>>>>
#ifdef DEBUG_LVL_1 // --------------------------DEBUG----------------->

void SS_S25FL_test_get_addr_from_page(void)
void SS_S25FL_test_get_addr_from_sector(void)
void SS_S25FL_test_get_addr_from_block(void)
void SS_S25FL_test_get_sector_num_from_page(void)
void SS_S25FL_test_get_page_num_from_addr(void)
#endif
//---------------------------------------------------------------------------------------
void SS_S25FL_test__addr_section__get_last_log_page_init (void);

void SS_S25FL__addr_section__test_1 (void)
void SS_S25FL_test_2 (void)
void SS_S25FL_test_logs (void)
void SS_S25FL_test_erase_in_range(void)
void SS_S25FL_test_1(void)
*/
