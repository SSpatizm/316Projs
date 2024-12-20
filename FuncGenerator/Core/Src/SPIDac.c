#include "SPIDac.h"
void spi_setup(){
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; //enables SPI1 clock
	// sets Baud Rate to 000
	SPI1->CR1 &= ~(SPI_CR1_BR);
//	SPI1->CR1 |= (SPI_CR1_BR);
	//===================================
	// Register COnfiguration for SPI_CR1
	//===================================
	// sets clock phase shift and clock polarity to 0, 0
	SPI1->CR1 &= ~(SPI_CR1_CPOL | SPI_CR1_CPHA);
	SPI1->CR1 |= (SPI_CR1_BIDIMODE); // sets bidirectional mode to 1 (1-line bidirectional mode)
	SPI1->CR1 |= (SPI_CR1_BIDIOE);	// sets output enable High so data is only being sent, not received
	SPI1->CR1 &= ~(SPI_CR1_RXONLY); // set to transmit mode only
	SPI1->CR1 &= ~(SPI_CR1_LSBFIRST); //sets data transfer mode to MSB first
    //	SPI1->CR1 &= ~(SPI_CR1_CRCEN); //re-enable this later
	SPI1->CR1 &= ~(SPI_CR1_SSM); //sets software CS management to hardware mode
	SPI1->CR1 |= (SPI_CR1_MSTR);
	//==================================
	//Register Configuration for SPI_CR2
	//==================================
	SPI1->CR2 |= (0xF << SPI_CR2_DS_Pos); // sets DS to 1111, or 16 bit Frame size
    //	SPI1->CR2 &=
	SPI1->CR2 |= (SPI_CR2_SSOE); // Sets CS to output mode
	SPI1->CR2 |= SPI_CR2_NSSP_Msk; //mask nssp
	//disable interrupts
	SPI1->CR2 &= ~(SPI_CR2_TXEIE | SPI_CR2_RXNEIE);
	// FRXTH enabled since DS is 12
	SPI1->CR2 &= ~SPI_CR2_FRXTH;
	SPI1->CR1 |= (SPI_CR1_SPE); //enables SPI1
}



void send_dac(uint16_t in_volt){
	//data frame to send data with
	uint16_t data_frame = 0x0000;
	data_frame &= ~(1 << DAC_DACA_Pos); // Sets DAC select to DAC A(0xF)
	data_frame &= ~(1 << DAC_BUFF_Pos);  // sets Buffer to disabled(0xE)
	data_frame |= (1 << DAC_GA_Pos); 	// Sets Gain to 1(0xD)
	data_frame |= (1 << DAC_SHDN_Pos); 	// Sets output enable to true(0xC)
	in_volt &= GPIO_DATA_Mask; // keeps the first 12 bits
	data_frame |= in_volt; //sets 12 lowest bits to in_volt
	// if transmit FIFO is empty, load data
	SPI1->DR = data_frame;
	while(SPI1->SR & SPI_SR_BSY); //wait after sending
	while (!(SPI1->SR & SPI_SR_TXE)); //wait after sending
}
