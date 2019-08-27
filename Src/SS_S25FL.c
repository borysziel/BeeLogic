//==========================================
// Title:  FLASH data logging "file system" library
// Author: Andrzej Laczewski
// Date:   2019-02-24
// Ver:    1
// Organisation: AGH SPACE SYSTEMS
//==========================================

#include "SS_S25FL.h"

//--------CubeMX USATWIENIA:
//---> GPIO: pin od resetu jako wyjœcie (FLASH_RST)
//---> GPIO: pin od CS jako wyjscie (FLASH_CS)
//---> Standardowa konfig SPI (FULL DUPLEX MASTER,HARDWARE NSS DISABLE) + odpowiedni prescaler + SPI2 global interupt
//---> DMA od SPI: Mode Normal, FIFO off, Mem to Periph (SPI2_TX), Periph To Mem (SPI2_RX) , High piority , DMA stream global interupt enabled , data width BYTE
//---> TIMER generacja cyklicznych przerwañ co 50 uS lub inny okres czasu
//--->
//--->


//  --------------------> Example usage:
/*
int main(void)
{

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_RTC_Init();


  //-------------------------------------------
  SS_S25FL_reset_init();

  SS_S25FL_erase_only_written_pages();
  //SS_S25FL_erase_full_chip(); // wykasowac cala pamiec przed pierwszym wykorzystaniem biblioteki, potem mo¿na kasowac zapisane dane funkcja SS_S25FL_erase_only_written_pages();

  SS_S25FL_saving_init();

  SS_S25FL_start_logging();

  uint16_t i;

  for(i=1;i<=3072;i++)
  {
  	SS_S25FL_save_variable_u32(1, i);
  }

  SS_S25FL_stop_logging();

  SS_S25FL_read_data_logs_to_uart(1);

  __print_debug_to_UART_1("------------FLASH free space %ld ----------\r\n",SS_S25FL_get_free_space_in_pages())

  //-------------------------------------------

  while (1)
  {

  }
}
*/
//  --------------------<

// ---------------------GLOBAL VAR-------------->>>>>>>>
uint8_t  MEASURE_ENABLE_FLAG = 0;
FLASH_SPACE_STATE_t FLASH_SPACE_STATE = EMPTY;

uint32_t global_curr_flash_page = FIRST_LOG_PAGE;
uint32_t global_flash_timestamp = 0;
//--------------------------
uint8_t DMA_TX_IN_PROGRESS = 0;
uint8_t DMA_RX_IN_PROGRESS = 0;
TIMEOUT_STATE_t TIMEOUT_STATE = OK;
Addr_section_last_written_t  global_addr_section_lw; // Contains info about current zero position in addr section
// ---------------------GLOBAL VAR--------------<<<<<<<<

/*
 * Funkcje do obs³ugi GPIO
 */
// --------------------- EXTERNAL GPIO HANDLING-------------->>>>>>>>
void SS_S25FL_select(void)
{
	HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, RESET);
}
void SS_S25FL_deselect(void)
{
	HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, SET);
}
void SS_S25FL_reset_activ(void)
{
	HAL_GPIO_WritePin(FLASH_RST_GPIO_Port, FLASH_RST_Pin, RESET);
}
void SS_S25FL_reset_deactiv(void)
{
	HAL_GPIO_WritePin(FLASH_RST_GPIO_Port, FLASH_RST_Pin, SET);
}
// --------------------- EXTERNAL GPIO HANDLING--------------<<<<<<<<

/*
 * Systick handler odpowiedzilny za przesy³ danych poprzez DMA/SPI do FLASH,a , Konieczne wywo³ywanie w przerwaniu systick minimum raz na 1 ms, funkcja odpowiedzialna za prze³¹czenie linii CS
 * inc_timestamp_handler odpowiedzialny za inkrementacje zmiennej od time-stamp'a logów, nalezy wywolywac w przerwaniu od timera, jednostki do ustalenia (teraz jest chyba inkrementacja o 50 uS)
 */
// --------------------- INTERUPT ROUTINE HANDLERS-------------->>>>>>>>

void SS_S25FL__dma_transfer_cs_handler (void)// call inside SYSTICK interupt (every 1ms, or less if SPI bitrate is high) or somewhere else, responsible for switching CS after transmision,
{
	if( (!((HAL_DMA_GetState(&SPI_DMA_TX_STRUCT) == HAL_DMA_STATE_BUSY) || (HAL_SPI_GetState(&SPI_STRUCT) == HAL_SPI_STATE_BUSY_TX))) && (DMA_TX_IN_PROGRESS == 1) ){
		DMA_TX_IN_PROGRESS = 0;
		SS_S25FL_deselect();
	}
	if( (!((HAL_DMA_GetState(&SPI_DMA_RX_STRUCT) == HAL_DMA_STATE_BUSY) || (HAL_SPI_GetState(&SPI_STRUCT) == HAL_SPI_STATE_BUSY_RX))) && (DMA_RX_IN_PROGRESS == 1) ){
		DMA_RX_IN_PROGRESS = 0;
		SS_S25FL_deselect();
	}
}
void SS_S25FL_inc_timestamp_handler(void) // place somewhere in a timer interupt routine (called every 50 uS?)
{
	if(MEASURE_ENABLE_FLAG)
	{
		global_flash_timestamp += FLASH_TIMESTAMP_INC_FACTOR; // overflow after 71min??
	}
	else
	{
		global_flash_timestamp = 0;
	}
}



// --------------------- INTERUPT ROUTINE HANDLERS--------------<<<<<<<<


/*
 * Funkcje do konwersji adresu pomiêdzy ró¿nymi "rodzajami" adresowania pamiêci
 */
// ---------------------ADDRESS COVERSION FUNC-------------->>>>>>>>
uint32_t SS_S25FL_get_addr_from_page (uint32_t page_number ,uint8_t offset) // 1 000 000 pages max (256Mb flash), 256 bytes per page, max offset 255, counting pages from 0
{
	uint32_t address = page_number;
	address = (address << 8) + offset;
	return address;
}
uint32_t SS_S25FL_get_addr_from_sector (uint32_t sector_number ,uint16_t offset) //62500 sectors max (256Mb flash), 4096 bytes per sector, max offset 4095, counting sectors from 0
{
	uint32_t address = sector_number;
	address = (address << 12) + offset;
	return address;
}
uint32_t SS_S25FL_get_addr_from_halfblock (uint32_t halfblock_number ,uint16_t offset)
{
	uint32_t address = halfblock_number;
	address = (address << 15) + offset;
	return address;
}
uint32_t SS_S25FL_get_addr_from_block (uint32_t block_number ,uint16_t offset) //3906 blocks max (256Mb flash), 65536 bytes per block, max offset 65535, counting blocks from 0
{
	uint32_t address = block_number;
	address = (address << 16) + offset;
	return address;
}
//-------------------------------------
/*
 * Funkcje do sprawdzania na której stronie czy sektorze znajduje siê dany adres w danym formacie
 */
uint32_t SS_S25FL_get_sector_num_from_page(uint32_t page_num) // check in which sector given page is
{
	uint32_t sector_num = 0;
	int64_t curr_page;

	for (curr_page = page_num; curr_page >= 0 ; (curr_page = curr_page - 16) )
	{
		sector_num++;
	}

	return sector_num - 1; // we start from zero
}
uint32_t SS_S25FL_get_page_num_from_addr(uint32_t addr) // check in which page given addr is
{
	uint32_t page_num = 0;
	int64_t curr_addr;

	for (curr_addr = addr; curr_addr >= 0 ; (curr_addr = curr_addr - 256) )
	{
		page_num ++;
	}

	return page_num - 1; // we start from zero
}
// ---------------------ADDRESS COVERSION FUNC--------------<<<<<<<<

/*
 * Funkcje do sprawdzania zajêtoœci peryferiów (DMA i SPI) odpowiedzialnych za przesy³ danych do flash,a
 * w celu zapobiegania wywo³aniu funkcji w trakcie pracy DMA lub standardowego wysy³ania przez SPI bez wykorzystania DMA ,
 * wait_4_spi_bus mo¿e opózniac wywo³anie danej fukcji czekajac na zakonczenie transferu DMA/SPI a¿ do przekroczenia maks czasu MAX_HAL_TIMEOUT,
 * Po przekroczeniu tego czasu dana funkcja zwraca b³¹d (nie odczeka³a siê na zakoñczenie transferu)
 */
// --------------------- DATA TRANSFER STATE HANDLING FUNC-------------->>>>>>>>
B_STATE_t SS_S25FL_get_dma_transfer_state (DATA_TRANSFER_DIR_t x)  // 1 = BUSY , return dma transfer state based on global flags
{
	if(x == TX)
	{
		if(DMA_TX_IN_PROGRESS == 1) return B_BUSY;
		else return B_IDLE;
	}
	if(x == RX)
	{
		if(DMA_RX_IN_PROGRESS == 1) return B_BUSY;
		else return B_IDLE;
	}
}
B_STATE_t SS_S25FL_get_dma_spi_state (void)
{
	if( (HAL_DMA_GetState(&SPI_DMA_RX_STRUCT) == HAL_DMA_STATE_BUSY) || (HAL_DMA_GetState(&SPI_DMA_TX_STRUCT) == HAL_DMA_STATE_BUSY) || (HAL_SPI_GetState(&SPI_STRUCT) == HAL_SPI_STATE_BUSY_RX) || (HAL_SPI_GetState(&SPI_STRUCT) == HAL_SPI_STATE_BUSY_TX) || (SS_S25FL_get_dma_transfer_state(TX) == B_BUSY) || (SS_S25FL_get_dma_transfer_state(RX) == B_BUSY) )
	{
		return B_BUSY;
	}
	else
	{
		return B_IDLE;
	}
}
TIMEOUT_STATE_t SS_S25FL_wait_4_spi_bus(void)
{
	uint32_t tickstart = HAL_GetTick();
	while((SS_S25FL_get_dma_spi_state() == B_BUSY))
	{
		if((HAL_GetTick()- tickstart) > MAX_SPI_DMA_TIMEOUT)
		{
			TIMEOUT_STATE = ERR;
			return ERR;
		}
	}
	return OK;
}

// --------------------- DATA TRANSFER STATE HANDLING FUNC--------------<<<<<<<<

/*
 * Funkcje do obs³ugi podstawowych funkcjonalnoœci FLASH,a
 * Tylko do u¿ytku wenêtrznego (nie zabezbieczone przed wywo³anie w trakcie transferu DMA)
 */
// --------------------- BASIC FLASH HANDLING FUNC-------------->>>>>>>>
void SS_S25FL_reset_init(void)
{
	TIMEOUT_STATE = OK; // init vars

	SS_S25FL_reset_activ();
	HAL_Delay(10);
	SS_S25FL_reset_deactiv();
	HAL_Delay(50);
}
uint8_t SS_S25FL_read_status(void) // unprotected (this func is not protected against calling during DMA transmission) , only for internal use
{
	uint8_t reg = 0x05;
	SS_S25FL_select();
	HAL_SPI_Transmit(&SPI_STRUCT, &reg, 1, MAX_HAL_TIMEOUT);
	HAL_SPI_Receive(&SPI_STRUCT, &reg, 1, MAX_HAL_TIMEOUT);
	SS_S25FL_deselect();
	return reg;
}
B_STATE_t SS_S25FL_check_write_progress(void)// unprotected (this func is not protected against calling during DMA transmission) , only for internal use
{
	if( SS_S25FL_read_status() & 0x01 ) return B_BUSY;
	else return B_IDLE;
}
TIMEOUT_STATE_t SS_S25FL_wait_4_flash(uint32_t timeout)
{
	uint32_t tickstart = HAL_GetTick();
	while((SS_S25FL_check_write_progress() == B_BUSY))
	{
		if((HAL_GetTick()- tickstart) > timeout)
		{
			TIMEOUT_STATE = ERR;
			return ERR;
		}
	}
	return OK;
}
uint8_t SS_S25FL_get_id(void)// unprotected (this func is not protected against calling during DMA transmission) , only for internal use
{
	uint8_t dummy [3], ID = 0x9F;
	SS_S25FL_select();
	HAL_SPI_Transmit(&SPI_STRUCT, &ID, 1, MAX_HAL_TIMEOUT);
	HAL_SPI_Receive(&SPI_STRUCT, dummy, 2, MAX_HAL_TIMEOUT);
	HAL_SPI_Receive(&SPI_STRUCT, &ID, 1, MAX_HAL_TIMEOUT);
	SS_S25FL_deselect();
	return ID;
}
ERR_FLAG_t SS_S25FL_write_enable (void)// unprotected (this func is not protected against calling during DMA transmission) , only for internal use
{
	uint8_t reg = 0x06;
	SS_S25FL_select();
	HAL_SPI_Transmit(&SPI_STRUCT, &reg, 1, MAX_HAL_TIMEOUT);
	SS_S25FL_deselect();
	if (SS_S25FL_wait_4_flash(MAX_WAIT_4_FLASH_TIMEOUT) == ERR) return B_ERROR;
	else return B_SUCCESS;
	//while(SS_S25FL_check_write_progress() == B_BUSY);
}
// --------------------- BASIC FLASH HANDLING FUNC--------------<<<<<<<<

/*
 * Funkcje do kasowania róznych "obszarów" pamiêci,
 */
// --------------------- ERASE OPERATIONS WITHOUT DMA-------------->>>>>>>>
ERR_FLAG_t SS_S25FL_erase_sector(uint32_t sector_num) //Erases given sector
{
	if(SS_S25FL_wait_4_spi_bus() == OK){

		uint8_t config_buff [5];
		uint32_t adress;

		adress = SS_S25FL_get_addr_from_sector(sector_num,0);
		config_buff[0] = 0x21;
		config_buff[1] = (adress>>24) & 0xff;
		config_buff[2] = (adress>>16) & 0xff;
		config_buff[3] = (adress>>8) & 0xff;
		config_buff[4] = (adress) & 0xff;

		if (SS_S25FL_wait_4_flash(MAX_WAIT_4_FLASH_TIMEOUT) == ERR) goto FLASH_ERROR;
		SS_S25FL_write_enable();
		SS_S25FL_select();

		HAL_SPI_Transmit(&SPI_STRUCT, config_buff, 5, MAX_HAL_TIMEOUT);

		SS_S25FL_deselect();

		if (SS_S25FL_wait_4_flash(MAX_WAIT_4_FLASH_ERASE_TIMEOUT) == ERR) goto FLASH_ERROR;

		return B_SUCCESS;
	}
	else {
		FLASH_ERROR:
	    return B_ERROR;
	}
}
ERR_FLAG_t SS_S25FL_erase_halfblock(uint32_t adress) ////Erases given halfblock ---->  Not implemented , not tested !!!!!!!!!
{
	if(SS_S25FL_wait_4_spi_bus() == OK){

		uint8_t config_buff [5];
		config_buff[0] = 0x53;
		config_buff[1] = (adress>>24) & 0xff;
		config_buff[2] = (adress>>16) & 0xff;
		config_buff[3] = (adress>>8) & 0xff;
		config_buff[4] = (adress) & 0xff;

		if (SS_S25FL_wait_4_flash(MAX_WAIT_4_FLASH_TIMEOUT) == ERR) goto FLASH_ERROR;
		SS_S25FL_write_enable();
		SS_S25FL_select();

		HAL_SPI_Transmit(&SPI_STRUCT, config_buff, 5, MAX_HAL_TIMEOUT);

		SS_S25FL_deselect();

		if (SS_S25FL_wait_4_flash(MAX_WAIT_4_FLASH_ERASE_TIMEOUT) == ERR) goto FLASH_ERROR;

		return B_SUCCESS;
	}
	else {
		FLASH_ERROR:
		return B_ERROR;
	}
}
ERR_FLAG_t SS_S25FL_erase_block(uint32_t block_num) // //Erases given block
{
	if(SS_S25FL_wait_4_spi_bus() == OK){

		uint8_t config_buff [5];
		uint32_t adress;

		adress = SS_S25FL_get_addr_from_block(block_num,0);
		config_buff[0] = 0xDC;
		config_buff[1] = (adress>>24) & 0xff;
		config_buff[2] = (adress>>16) & 0xff;
		config_buff[3] = (adress>>8) & 0xff;
		config_buff[4] = (adress) & 0xff;

		if (SS_S25FL_wait_4_flash(MAX_WAIT_4_FLASH_TIMEOUT) == ERR) goto FLASH_ERROR;
		SS_S25FL_write_enable();
		SS_S25FL_select();

		HAL_SPI_Transmit(&SPI_STRUCT, config_buff, 5, MAX_HAL_TIMEOUT);

		SS_S25FL_deselect();

		if (SS_S25FL_wait_4_flash(MAX_WAIT_4_FLASH_ERASE_TIMEOUT) == ERR) goto FLASH_ERROR;

		return B_SUCCESS;
	}
	else {
		FLASH_ERROR:
		return B_ERROR;
	}
}
ERR_FLAG_t SS_S25FL_erase_full_chip(void) // Erases full FLASH ,very long command , up to 140 s !!!
{
	if(SS_S25FL_wait_4_spi_bus() == OK){
		uint8_t reg = 0x60;
		if (SS_S25FL_wait_4_flash(MAX_WAIT_4_FLASH_TIMEOUT) == ERR) goto FLASH_ERROR;
		SS_S25FL_write_enable();
		SS_S25FL_select();
		HAL_SPI_Transmit(&SPI_STRUCT, &reg, 1, MAX_HAL_TIMEOUT);
		SS_S25FL_deselect();
		if (SS_S25FL_wait_4_flash(MAX_WAIT_4_FLASH_ERASE_TIMEOUT) == ERR) goto FLASH_ERROR;

		return B_SUCCESS;
	}
	else {
		FLASH_ERROR:
		return B_ERROR;
	}
}
// --------------------- ERASE OPERATIONS WITHOUT DMA--------------<<<<<<<

/*
 * Funkcje do zapisu/odczytu FLASH,a bez wykorzystania DMA
 */
// --------------------- READ WRITE OPERATIONS WITHOUT DMA-------------->>>>>>>>
ERR_FLAG_t SS_S25FL_write_bytes(uint32_t adress, uint8_t *array, uint8_t length) // Write specified number of bytes to FLASH , non DMA
{
	if(SS_S25FL_wait_4_spi_bus() == OK){

		uint8_t config_buff [5];
		config_buff[0] = 0x12;
		config_buff[1] = (adress>>24) & 0xff;
		config_buff[2] = (adress>>16) & 0xff;
		config_buff[3] = (adress>>8) & 0xff;
		config_buff[4] = (adress) & 0xff;

		if (SS_S25FL_wait_4_flash(MAX_WAIT_4_FLASH_TIMEOUT) == ERR) goto FLASH_ERROR;

		SS_S25FL_write_enable();
		SS_S25FL_select();

		HAL_SPI_Transmit(&SPI_STRUCT, config_buff, 5, MAX_HAL_TIMEOUT);
		HAL_SPI_Transmit(&SPI_STRUCT, array, length, MAX_HAL_TIMEOUT);

		SS_S25FL_deselect();

		return B_SUCCESS;
	}
	else {
		FLASH_ERROR:
	    return B_ERROR;
	}
}
ERR_FLAG_t SS_S25FL_write_page(uint32_t page, uint8_t *array) // Write single page to FLASH , non DMA
{
	if(SS_S25FL_wait_4_spi_bus() == OK){

		uint32_t adress = SS_S25FL_get_addr_from_page(page,0);
		uint8_t config_buff [5];
		config_buff[0] = 0x12;
		config_buff[1] = (adress>>24) & 0xff;
		config_buff[2] = (adress>>16) & 0xff;
		config_buff[3] = (adress>>8) & 0xff;
		config_buff[4] = (adress) & 0xff;

		if (SS_S25FL_wait_4_flash(MAX_WAIT_4_FLASH_TIMEOUT) == ERR) goto FLASH_ERROR;

		SS_S25FL_write_enable();
		SS_S25FL_select();

		HAL_SPI_Transmit(&SPI_STRUCT, config_buff, 5, MAX_HAL_TIMEOUT);
		HAL_SPI_Transmit(&SPI_STRUCT, array, 256, MAX_HAL_TIMEOUT);

		SS_S25FL_deselect();

		return B_SUCCESS;
	}
	else {
		FLASH_ERROR:
	    return B_ERROR;
	}
}
ERR_FLAG_t SS_S25FL_read_page(uint32_t page, uint8_t *array) // Read single page from FLASH , non DMA
{
	if(SS_S25FL_wait_4_spi_bus() == OK){

		uint32_t adress = SS_S25FL_get_addr_from_page(page,0);
		uint8_t config_buff [5];
		config_buff[0] = 0x13;
		config_buff[1] = (adress>>24) & 0xff;
		config_buff[2] = (adress>>16) & 0xff;
		config_buff[3] = (adress>>8) & 0xff;
		config_buff[4] = (adress) & 0xff;

		if (SS_S25FL_wait_4_flash(MAX_WAIT_4_FLASH_TIMEOUT) == ERR) goto FLASH_ERROR;

		SS_S25FL_write_enable();
		SS_S25FL_select();

		HAL_SPI_Transmit(&SPI_STRUCT, config_buff, 5, MAX_HAL_TIMEOUT);
		HAL_SPI_Receive(&SPI_STRUCT, array, 256, MAX_HAL_TIMEOUT);

		SS_S25FL_deselect();

		return B_SUCCESS;
	}
	else {
		FLASH_ERROR:
	    return B_ERROR;
	}
}
// --------------------- READ WRITE OPERATIONS WITHOUT DMA--------------<<<<<<<

/*
 * Funkcje do zapisu/odczytu FLASH,a z wykorzystaniem DMA
 */
// --------------------- READ WRITE OPERATIONS WITH DMA-------------->>>>>>>>
ERR_FLAG_t SS_S25FL_write_page_dma(uint32_t page, uint8_t *array) // Write single page to FLASH , with use of DMA
{
	if(SS_S25FL_wait_4_spi_bus() == OK){ // Protection against too early func call

		uint32_t adress = SS_S25FL_get_addr_from_page(page,0);
		uint8_t config_buff [5];
		config_buff[0] = 0x12;
		config_buff[1] = (adress>>24) & 0xff;
		config_buff[2] = (adress>>16) & 0xff;
		config_buff[3] = (adress>>8) & 0xff;
		config_buff[4] = (adress) & 0xff;

		if (SS_S25FL_wait_4_flash(MAX_WAIT_4_FLASH_TIMEOUT) == ERR) goto FLASH_ERROR;

		SS_S25FL_write_enable();
		SS_S25FL_select();

		HAL_SPI_Transmit(&SPI_STRUCT, config_buff, 5, MAX_HAL_TIMEOUT);

		HAL_SPI_Transmit_DMA(&SPI_STRUCT, array, 256);

		DMA_TX_IN_PROGRESS = 1;

		return B_SUCCESS;
	}
	else {
		FLASH_ERROR:
		return B_ERROR;
	}

}
ERR_FLAG_t SS_S25FL_read_page_dma(uint32_t page, uint8_t *array) // Read single page from FLASH , with use of DMA
{
	if(SS_S25FL_wait_4_spi_bus() == OK){ // Protection against too early func call

		uint32_t adress = SS_S25FL_get_addr_from_page(page,0);
		uint8_t config_buff [5];
		config_buff[0] = 0x13;
		config_buff[1] = (adress>>24) & 0xff;
		config_buff[2] = (adress>>16) & 0xff;
		config_buff[3] = (adress>>8) & 0xff;
		config_buff[4] = (adress) & 0xff;

		if (SS_S25FL_wait_4_flash(MAX_WAIT_4_FLASH_TIMEOUT) == ERR) goto FLASH_ERROR;

		SS_S25FL_write_enable();
		SS_S25FL_select();

		HAL_SPI_Transmit(&SPI_STRUCT, config_buff, 5, MAX_HAL_TIMEOUT);

		HAL_SPI_Receive_DMA(&SPI_STRUCT, array, 256);

		DMA_RX_IN_PROGRESS = 1;

		return B_SUCCESS;
	}
	else {
		FLASH_ERROR:
		return B_ERROR;
	}

}
// --------------------- READ WRITE OPERATIONS WITH DMA--------------<<<<<<<<

/*
 * Funkcje odpowiedzialne za obs³ugê sekcji pierwszych 500 stron w pamiêci odpowiedzialnych za odresowanie ostatniej strony na której zapisane s¹ logi,
 * Zapis iloœci zapisanych stron polega na kasowaniu kolejnych jedynek zaczynaj¹c od MSB w kolejnych bajtach w tej sekcji poprzez zast¹pienie ich zerami,
 * Kazde zero w tej sekcji oznacza jedna zapisan¹ stronê zaczynaj¹c od 500,nej
 * przyk³ad MSB ->>> (adres zerowy) [0001 1111] ->>> LSB = 3 zapisane strony
 */
// --------------------- ADDRESS SECTION HANDLING FUNTIONS-------------->>>>>>>
Addr_section_last_written_t SS_S25FL__addr_section__find_last_written_byte (void)
{
	uint8_t flash_buff[256];
	uint32_t page_num;
	uint16_t byte_num;
	uint8_t found_end = 0;

	uint8_t last_written_byte;
	uint8_t last_written_byte_num_of_zeroes = 0;
	uint32_t last_written_byte_addr = 0;

	Addr_section_last_written_t last_written_index;

	#ifdef DEBUG_LVL_1 // --------------------------DEBUG----------------->
	__print_debug_to_UART("Last written page found\r\n")
	#endif// ---------------------------------------DEBUG-----------------<

	for (page_num = 0; page_num <= 500 ; page_num ++ )
	{
		#ifdef DEBUG_LVL_3 // --------------------------DEBUG----------------->
		__print_debug_to_UART("__page_loop_iter__\r\n")
		#endif// ---------------------------------------DEBUG-----------------<

		SS_S25FL_read_page(page_num, flash_buff);

		 	for (byte_num = 0; byte_num <= 255 ; byte_num ++ ) // find last unwritten byte (0xFF)
			{
				#ifdef DEBUG_LVL_3 // --------------------------DEBUG----------------->
		 		__print_debug_to_UART_1("LST UN WRI BYT: %lu\r\n",byte_num)
				#endif// ---------------------------------------DEBUG-----------------<

				if(flash_buff[byte_num] == 0xFF)
				{
					found_end = 1;
					break;
				}
				else
				{
					last_written_byte_addr ++;
				}
			}

			if (found_end == 1)
			{
				found_end = 0;
				break;
			}
		}

	if (last_written_byte_addr == 0) // handle overflow (-1)
	{
		last_written_index.byte_addr = 0;
		last_written_index.byte_num_of_zeroes = 0;
	}
	else
	{
		last_written_byte = ~flash_buff[byte_num-1]; // invert to read num of zeroes

		while(last_written_byte != 0)
		{
			if((last_written_byte) & 0b10000000)
			{
				last_written_byte_num_of_zeroes ++;
			}
			last_written_byte = last_written_byte << 1;
		}

		last_written_index.byte_addr = last_written_byte_addr-1;
		last_written_index.byte_num_of_zeroes = last_written_byte_num_of_zeroes-1; // -1 for bit shift
	}

	return last_written_index;
}
void SS_S25FL__addr_section__byte_zeroes_inc (void) // num. of zeroes overflow handling
{
	if(global_addr_section_lw.byte_num_of_zeroes < 7)
	{
		global_addr_section_lw.byte_num_of_zeroes ++;
	}
	else
	{
		global_addr_section_lw.byte_num_of_zeroes = 0;
		global_addr_section_lw.byte_addr ++;
	}
}
uint32_t SS_S25FL__addr_section__get_last_log_page_init (void) // execute once on startup to get initial page number and init global var
{
	uint32_t last_log_page;

	global_addr_section_lw = SS_S25FL__addr_section__find_last_written_byte();

	last_log_page = (FIRST_LOG_PAGE - 1) + (global_addr_section_lw.byte_addr*8) + (global_addr_section_lw.byte_num_of_zeroes + 1); // start from 501

	#ifdef DEBUG_LVL_1 // --------------------------DEBUG----------------->
	__print_debug_to_UART_2("Byte_addr_get/num of zzeroes_get_init %lu,%u\r\n",global_addr_section_lw.byte_addr,global_addr_section_lw.byte_num_of_zeroes)
	#endif// ---------------------------------------DEBUG-----------------<

	if(last_log_page != FIRST_LOG_PAGE)
	{
		SS_S25FL__addr_section__byte_zeroes_inc(); // move to next pos for further saving (prevent saving twice on the same pos)

		#ifdef DEBUG_LVL_1 // --------------------------DEBUG----------------->
		__print_debug_to_UART_2("Byte_addr_get/num of zzeroes_get_init(inc zeroes) %lu,%u\r\n",global_addr_section_lw.byte_addr,global_addr_section_lw.byte_num_of_zeroes)
		#endif// ---------------------------------------DEBUG-----------------<
	}

	return last_log_page;
}
uint32_t SS_S25FL__addr_section__get_last_log_page (void) // returns last log page (written)
{
	uint32_t last_log_page;
	Addr_section_last_written_t last_written_index;

	last_written_index = SS_S25FL__addr_section__find_last_written_byte();

	last_log_page = (FIRST_LOG_PAGE - 1) + (last_written_index.byte_addr*8) + (last_written_index.byte_num_of_zeroes + 1); // start from 501 or 1001

	#ifdef DEBUG_LVL_1 // --------------------------DEBUG----------------->
	__print_debug_to_UART_1("Last written page %ld\r\n",last_log_page)
	#endif// ---------------------------------------DEBUG-----------------<

	return last_log_page;
}
void SS_S25FL__addr_section__inc_last_log_page (void) // icrements addr section by one zero
{
	uint8_t write_byte = 0;

	write_byte = ~ ( 0b10000000 >> global_addr_section_lw.byte_num_of_zeroes);
	SS_S25FL_write_bytes(global_addr_section_lw.byte_addr,&write_byte,1);

	#ifdef DEBUG_LVL_1 // --------------------------DEBUG----------------->
	__print_debug_to_UART_3("write_byte/addr/num of zeroes: %d,%ld,%d\r\n",write_byte,global_addr_section_lw.byte_addr,global_addr_section_lw.byte_num_of_zeroes)
	#endif// ---------------------------------------DEBUG-----------------<

	SS_S25FL__addr_section__byte_zeroes_inc();
}
// --------------------- ADDRESS SECTION HANDLING FUNTIONS--------------<<<<<<<

/*
 * erase_only_written_pages s³y¿y do kasowania tylko zapisanych stron w pamiêci, najpierw odczytywana jest iloœc zapisanych stron a potem na tej podstawie kasowane sa zapisane strony
 * w sekcji za odpowiedzialnej za przchowywanie logów (stony od 500) , nastêpnie ksaowana jest sekcja przechowyuj¹ca adres ostatniej zapisanej strony z logami (strony od 0 do 500)
 */
// --------------------- ERASE ONLY WRITTEN PAGES FUNC-------------->>>>>>>
void SS_S25FL_erase_in_page_range (uint32_t start_page, uint32_t stop_page) // erases in given page range including these pages
{
	uint32_t curr_sector;
	uint32_t start_sector;
	uint32_t stop_sector;

	start_sector = SS_S25FL_get_sector_num_from_page(start_page);
	stop_sector = SS_S25FL_get_sector_num_from_page(stop_page);

	for (curr_sector = start_sector; curr_sector <=  stop_sector ; curr_sector++ )
	{
		SS_S25FL_erase_sector(curr_sector);
	}
}
void SS_S25FL__addr_section__erase (void) // erases addr section
{
 	#if defined FLASH_256MB
	SS_S25FL_erase_block(0);     // 0 -- 256 page
	SS_S25FL_erase_block(1);     // 256 -- 512 page
	#endif

	#if defined FLASH_512MB
	SS_S25FL_erase_block(0);   // 0-----
	SS_S25FL_erase_block(1);
	SS_S25FL_erase_block(2);
	SS_S25FL_erase_block(3);   // -----1024
	#endif

	global_addr_section_lw.byte_addr = 0;
	global_addr_section_lw.byte_num_of_zeroes = 0;

	#ifdef DEBUG_LVL_1 // --------------------------DEBUG----------------->
	__print_debug_to_UART("------------Erased addr section ----------\r\n")
	#endif// ---------------------------------------DEBUG-----------------<
}
void SS_S25FL_erase_only_written_pages(void) // erases only written pages in addr section and log section
{
	uint32_t last_written_page;

	last_written_page = SS_S25FL__addr_section__get_last_log_page();

	SS_S25FL_erase_in_page_range (FIRST_LOG_PAGE,last_written_page);

	SS_S25FL__addr_section__erase();

	FLASH_SPACE_STATE = EMPTY;

	#ifdef DEBUG_LVL_1 // --------------------------DEBUG----------------->
	__print_debug_to_UART("------------Erased Written pages ----------\r\n")
	#endif// ---------------------------------------DEBUG-----------------<
}
// --------------------- ERASE ONLY WRITTEN PAGES FUNC--------------<<<<<<<

/*
 *  Funkcje do obs³ugi zapisu logów
 */
// --------------------- DATA LOGGING FUNC -------------->>>>>>>
void SS_S25FL_check_flash_overflow (void)
{
	if (global_curr_flash_page >= FLASH_SIZE_PAGES )
	{
		FLASH_SPACE_STATE = FULL;
	}
	else
	{
		FLASH_SPACE_STATE = NOT_FULL;
	}
}


void SS_S25FL_saving_init(void) // var logging init function , call once on startup
{
	volatile uint32_t last_log_page = SS_S25FL__addr_section__get_last_log_page_init();

	if(last_log_page == FIRST_LOG_PAGE) // handle overwritting last page on startup
	{
		global_curr_flash_page = FIRST_LOG_PAGE;
	}
	else
	{
		global_curr_flash_page = last_log_page + 1 ;
	}

	SS_S25FL_check_flash_overflow();

	global_flash_timestamp = 0;
	MEASURE_ENABLE_FLAG = 0;

	#ifdef DEBUG_LVL_1 // --------------------------DEBUG----------------->
	__print_debug_to_UART_1("------------Logging init, first page to write: %ld ----------\r\n",global_curr_flash_page)
	#endif// ---------------------------------------DEBUG-----------------<
}
void SS_S25FL_start_logging(void) // call to start logging data to flash
{
	MEASURE_ENABLE_FLAG = 1;
}
void SS_S25FL_stop_logging(void) // call to stop logging data to flash
{
	MEASURE_ENABLE_FLAG = 0;
}
ERR_FLAG_t SS_S25FL_save_variable_u32(uint8_t variable_id, uint32_t data)
{
	static uint8_t flash_bufor1[256];
	static uint8_t flash_bufor2[256];

	static uint16_t flash_bufor_inc = 0;
	static uint8_t *flash_bufor_pointer = flash_bufor1;

	uint32_t flash_time;

	SS_S25FL_check_flash_overflow();

	if( (MEASURE_ENABLE_FLAG == 1) && ( FLASH_SPACE_STATE == EMPTY || FLASH_SPACE_STATE == NOT_FULL ) )
	{
		flash_time = global_flash_timestamp/100; // divide to get 0.1ms per 1?

		flash_bufor_pointer[flash_bufor_inc++] = variable_id;
		flash_bufor_pointer[flash_bufor_inc++] = (uint8_t)(flash_time>>16);
		flash_bufor_pointer[flash_bufor_inc++] = (uint8_t)(flash_time>>8);
		flash_bufor_pointer[flash_bufor_inc++] = (uint8_t)(flash_time);

		flash_bufor_pointer[flash_bufor_inc++] = (uint8_t)(data>>24);
		flash_bufor_pointer[flash_bufor_inc++] = (uint8_t)(data>>16);
		flash_bufor_pointer[flash_bufor_inc++] = (uint8_t)(data>>8);
		flash_bufor_pointer[flash_bufor_inc++] = (uint8_t)(data);

		if(flash_bufor_inc == 256)
		{

			if(flash_bufor_pointer == flash_bufor1)
			{
				#ifdef DEBUG_LVL_1 // --------------------------DEBUG----------------->
				__print_debug_to_UART_1(",Saving page at %lu,",global_curr_flash_page)
				#endif// ---------------------------------------DEBUG-----------------<

				SS_S25FL__addr_section__inc_last_log_page();

				if ( SS_S25FL_write_page_dma(global_curr_flash_page, flash_bufor1) == B_ERROR ) goto SAVE_ERROR;
				flash_bufor_pointer = flash_bufor2;

				global_curr_flash_page ++;
			}
			else
			{
				#ifdef DEBUG_LVL_1 // --------------------------DEBUG----------------->
				__print_debug_to_UART_1(",Saving page at %lu,",global_curr_flash_page)
				#endif// ---------------------------------------DEBUG-----------------<

				SS_S25FL__addr_section__inc_last_log_page();

				if ( SS_S25FL_write_page_dma(global_curr_flash_page, flash_bufor2) == B_ERROR ) goto SAVE_ERROR;
				flash_bufor_pointer = flash_bufor1;

				global_curr_flash_page ++;
			}

			flash_bufor_inc = 0;

		}
		return B_SUCCESS;
	}
	else
	{
		SAVE_ERROR:
		return B_ERROR;
	}

}
// --------------------- DATA LOGGING FUNC --------------<<<<<<<


// --------------------- OTHER -------------->>>>>>>
uint32_t SS_S25FL_get_free_space_in_pages(void) // call to get free space left in pages (256 bytes per page)
{
	if ( global_curr_flash_page < FLASH_SIZE_PAGES)
	{
		return  (FLASH_SIZE_PAGES - global_curr_flash_page);
	}
	else
	{
		return 0;
	}
}
TIMEOUT_STATE_t SS_S25FL_get_timeout(void) // get global timeout flag
{
	return TIMEOUT_STATE;
}
// --------------------- OTHER --------------<<<<<<<








//------------------------------ TEST FUNC-------------------------------- >>>>>>>>>>>>>>>>>>>
#ifdef DEBUG_LVL_1 // --------------------------DEBUG----------------->
void SS_S25FL_dump_page_to_uart(uint32_t page_num)
{
	uint16_t i;
	uint8_t page_buff[256];

	debug_tx_length = sprintf(debug_tx_char_buff, "<<---- PAGE NUM: %ld ---->>\r\n",page_num);
	HAL_UART_Transmit(&UART_DEBUG_STRUCT,(uint8_t *)debug_tx_char_buff,debug_tx_length, MAX_HAL_TIMEOUT);


	SS_S25FL_read_page(page_num, page_buff);

	for (i=0;i<=255;i++)
	{
		debug_tx_length = sprintf(debug_tx_char_buff, "%X,",page_buff[i]);
		HAL_UART_Transmit(&UART_DEBUG_STRUCT,(uint8_t *)debug_tx_char_buff,debug_tx_length, MAX_HAL_TIMEOUT);
	}

	debug_tx_length = sprintf(debug_tx_char_buff, "\r\n<<---------------------->>\r\n");
	HAL_UART_Transmit(&UART_DEBUG_STRUCT,(uint8_t *)debug_tx_char_buff,debug_tx_length, MAX_HAL_TIMEOUT);
}
void SS_S25FL_test_get_addr_from_page(void)
{
	uint32_t curr_page;

	for(curr_page = 0 ; curr_page <= 500 ; curr_page++)
	{
		debug_tx_length = sprintf(debug_tx_char_buff, "Address:%ld,Page num:%ld \r\n",SS_S25FL_get_addr_from_page (curr_page,0),curr_page);
		HAL_UART_Transmit(&UART_DEBUG_STRUCT,(uint8_t *)debug_tx_char_buff,debug_tx_length, MAX_HAL_TIMEOUT);
	}
}
void SS_S25FL_test_get_addr_from_sector(void)
{
	uint32_t curr_sector;

	for(curr_sector = 0 ; curr_sector <= 10 ; curr_sector++)
	{
		debug_tx_length = sprintf(debug_tx_char_buff, "Address:%ld,Sector num:%ld \r\n",SS_S25FL_get_addr_from_sector(curr_sector,0),curr_sector);
		HAL_UART_Transmit(&UART_DEBUG_STRUCT,(uint8_t *)debug_tx_char_buff,debug_tx_length, MAX_HAL_TIMEOUT);
	}
}
void SS_S25FL_test_get_addr_from_block(void)
{
	uint32_t curr_block;

	for(curr_block =3900 ; curr_block <= 3906 ; curr_block ++)
	{
		debug_tx_length = sprintf(debug_tx_char_buff, "Address:%ld,Block num:%ld \r\n",SS_S25FL_get_addr_from_block(curr_block,0),curr_block);
		HAL_UART_Transmit(&UART_DEBUG_STRUCT,(uint8_t *)debug_tx_char_buff,debug_tx_length, MAX_HAL_TIMEOUT);
	}
}
void SS_S25FL_test_get_sector_num_from_page(void)
{
	uint32_t curr_page;
	for(curr_page = 0 ; curr_page <= 500 ; curr_page++)
	{
		debug_tx_length = sprintf(debug_tx_char_buff, "Sector num:%ld,Page num:%ld \r\n",SS_S25FL_get_sector_num_from_page(curr_page),curr_page);
		HAL_UART_Transmit(&UART_DEBUG_STRUCT,(uint8_t *)debug_tx_char_buff,debug_tx_length, MAX_HAL_TIMEOUT);
	}
}
void SS_S25FL_test_get_page_num_from_addr(void)
{
	uint32_t curr_addr;
	for(curr_addr = 0 ; curr_addr <= 1000 ; curr_addr ++)
	{
		debug_tx_length = sprintf(debug_tx_char_buff, "Page num:%ld,Addr num:%ld \r\n",SS_S25FL_get_page_num_from_addr(curr_addr),curr_addr);
		HAL_UART_Transmit(&UART_DEBUG_STRUCT,(uint8_t *)debug_tx_char_buff,debug_tx_length, MAX_HAL_TIMEOUT);
	}
}
//---------------------------------------------------------------------------------------
void SS_S25FL_test__addr_section__get_last_log_page_init (void)
{
	SS_S25FL__addr_section__erase ();
	uint32_t i;
	for (i=0;i<=500;i++)
	{
		SS_S25FL_dump_page_to_uart(i);
	}

	debug_tx_length = sprintf(debug_tx_char_buff, "Page num:%ld \r\n",SS_S25FL__addr_section__get_last_log_page_init());
	HAL_UART_Transmit(&UART_DEBUG_STRUCT,(uint8_t *)debug_tx_char_buff,debug_tx_length, MAX_HAL_TIMEOUT);
}
void SS_S25FL_read_data_logs_to_uart(uint8_t id) // read data logs to UART ,     old function (not mine)!!!!
{
	uint16_t inc = 0;
	uint32_t check_page = 501;
	uint8_t check_array[256];

	do
	{
		SS_S25FL_read_page(check_page, (uint8_t*)check_array);
		do
		{
			if (check_array[inc] == id)
			{
				debug_tx_length = sprintf(debug_tx_char_buff, "Time: %ld [x0.1ms],Value: %ld\r\n", (uint32_t)(check_array[inc+1] << 16) + (uint32_t)(check_array[inc+2] << 8) +
						(uint32_t)(check_array[inc+3]), (uint32_t)(check_array[inc+4]) + (uint32_t)(check_array[inc+5] << 16) +
						(uint32_t)(check_array[inc+6] << 8) + (uint32_t)(check_array[inc+7]));

				HAL_UART_Transmit(&UART_DEBUG_STRUCT,(uint8_t *)debug_tx_char_buff, debug_tx_length, MAX_HAL_TIMEOUT);
				inc += 8;
			}
			else
			{
				inc += 8;
			}
		}
		while (inc < 256);
		inc = 0;
		check_page++;
	}
	while(check_page <= 36210);
	//while ((check_array[0] != 0xff) && (check_array[7] != 0xff) && (check_array[8] != 0xff) && (check_array[63] != 0xff));

	debug_tx_length = sprintf(debug_tx_char_buff, "--------------------\r\n");
	HAL_UART_Transmit(&UART_DEBUG_STRUCT,(uint8_t *)debug_tx_char_buff,debug_tx_length, MAX_HAL_TIMEOUT);
	debug_tx_length = sprintf(debug_tx_char_buff, "--------------------\r\n");
	HAL_UART_Transmit(&UART_DEBUG_STRUCT,(uint8_t *)debug_tx_char_buff,debug_tx_length, MAX_HAL_TIMEOUT);
	debug_tx_length = sprintf(debug_tx_char_buff, "--------------------\r\n");
	HAL_UART_Transmit(&UART_DEBUG_STRUCT,(uint8_t *)debug_tx_char_buff,debug_tx_length, MAX_HAL_TIMEOUT);
	debug_tx_length = sprintf(debug_tx_char_buff, "--------------------\r\n");
	HAL_UART_Transmit(&UART_DEBUG_STRUCT,(uint8_t *)debug_tx_char_buff,debug_tx_length, MAX_HAL_TIMEOUT);
	debug_tx_length = sprintf(debug_tx_char_buff, "--------------------\r\n");
	HAL_UART_Transmit(&UART_DEBUG_STRUCT,(uint8_t *)debug_tx_char_buff,debug_tx_length, MAX_HAL_TIMEOUT);
	debug_tx_length = sprintf(debug_tx_char_buff, "--------------------\r\n");
	HAL_UART_Transmit(&UART_DEBUG_STRUCT,(uint8_t *)debug_tx_char_buff,debug_tx_length, MAX_HAL_TIMEOUT);
	debug_tx_length = sprintf(debug_tx_char_buff, "--------------------\r\n");
	HAL_UART_Transmit(&UART_DEBUG_STRUCT,(uint8_t *)debug_tx_char_buff,debug_tx_length, MAX_HAL_TIMEOUT);

}
void SS_S25FL__addr_section__test_1 (void)
{
	//SS_S25FL__addr_section__erase();

	debug_tx_length = sprintf(debug_tx_char_buff, "Last page num: %lu\r\n",SS_S25FL__addr_section__get_last_log_page_init());
	HAL_UART_Transmit(&UART_DEBUG_STRUCT,(uint8_t *)debug_tx_char_buff,debug_tx_length, MAX_HAL_TIMEOUT);

	for(uint16_t cnt = 1; cnt <= 4; cnt++)
	{
		SS_S25FL__addr_section__inc_last_log_page();
	}

	debug_tx_length = sprintf(debug_tx_char_buff, "After inc page num: %lu\r\n",SS_S25FL__addr_section__get_last_log_page());
	HAL_UART_Transmit(&UART_DEBUG_STRUCT,(uint8_t *)debug_tx_char_buff,debug_tx_length, MAX_HAL_TIMEOUT);
}
void SS_S25FL_test_2 (void)
{

	//SS_S25FL__addr_section__erase();

	uint8_t write_byte = 0b10111111;
	SS_S25FL_write_bytes(0,&write_byte,1);

	HAL_Delay(10);

	uint8_t buf_rcv [256];
	SS_S25FL_read_page_dma(0,buf_rcv);

	HAL_UART_Transmit(&UART_DEBUG_STRUCT, "xxxxx",5, MAX_HAL_TIMEOUT);
	HAL_UART_Transmit(&UART_DEBUG_STRUCT, buf_rcv, 256, MAX_HAL_TIMEOUT);

}
void SS_S25FL_test_logs (void)
{

	MEASURE_ENABLE_FLAG = 1;

	uint16_t i;

	for(i=0;i<=512;i++)
	{
		SS_S25FL_save_variable_u32(1, i);
	}

	SS_S25FL_read_data_logs_to_uart(1);

	for (i=0;i<=50;i++)
	{
		SS_S25FL_dump_page_to_uart(i);
	}

	for (i=470;i<560;i++)
	{
		SS_S25FL_dump_page_to_uart(i);
	}

}
void SS_S25FL_test_erase_in_range(void)
{
	uint8_t page_buff [256];
	uint32_t i;

	for (i = 0; i <= 255 ; i++)
	{
		page_buff[i]= 0x00;
	}

	for (i=0;i<=500;i++)
	{
		SS_S25FL_write_page(i,page_buff);
	}

	SS_S25FL_erase_in_page_range (16,96);

	for (i=0;i<=500;i++)
	{
		SS_S25FL_dump_page_to_uart(i);
	}

}
void SS_S25FL_test_1(void)
{

	//SS_S25FL_erase_full_chip();
    //HAL_Delay(200);

	/*uint8_t buf [256];


	for (uint32_t i = 0; i<256;i++){ // fill with val
		buf [i]= 0xAA;
	}
	buf [0] = 0xAF;
	buf [1] = 0xAE;
	buf [2] = 0xAD;
	buf [3] = 0xAC;
	buf [254] = 0xA9;
	buf [255] = 0xAB;

	SS_S25FL_write_page_dma(116, buf);*/

	uint8_t buf_rcv [256];
	SS_S25FL__addr_section__erase();
	HAL_Delay(10);
	SS_S25FL_read_page_dma(400,buf_rcv);

	HAL_UART_Transmit(&UART_DEBUG_STRUCT, "Test start:", 12, MAX_HAL_TIMEOUT);
	HAL_UART_Transmit(&UART_DEBUG_STRUCT, buf_rcv, 256, MAX_HAL_TIMEOUT);

	//uint8_t data;
	//data = SS_S25FL_get_id();
	//HAL_UART_Transmit(&UART_DEBUG_STRUCT, "abcd", 4, 200);
	//HAL_UART_Transmit(&UART_DEBUG_STRUCT, &data, 1, 200);

}
#endif// ---------------------------------------DEBUG-----------------<
//------------------------------ TEST FUNC-------------------------------- >>>>>>>>>>>>>>>>>>>

//------------------------------- TRASH:   [XXXXXXXXXXXXXXXXXXXX]
/*uint32_t SS_S25FL_REETURN_LAST_PAGE_NUM(void)
{
	return page;
	HAL_PWR_EnableBkUpAccess();
	uint32_t page = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1);
	HAL_PWR_DisableBkUpAccess();
}
void SS_S25FL_SAVE_LAST_PAGE_NUM(uint32_t page)
{
	HAL_PWR_EnableBkUpAccess();
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, page);
	HAL_PWR_DisableBkUpAccess();
}*/
