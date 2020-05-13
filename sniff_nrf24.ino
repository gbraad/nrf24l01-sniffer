#include "iface_nrf24l01.h" // from nrf24_multipro, also requires nrf24l01.ino
#include <stdarg.h>

//SPI Comm.pins with nRF24L01
#define MOSI_pin  3  // MOSI - D3
#define SCK_pin   4  // SCK  - D4
#define CE_pin    5  // CE   - D5
#define MISO_pin  A0 // MISO - A0
#define CS_pin    A1 // CS   - A1

#define ledPin    13 // LED  - D13

// SPI outputs
#define MOSI_on PORTD |= _BV(3)  // PD3
#define MOSI_off PORTD &= ~_BV(3)// PD3
#define SCK_on PORTD |= _BV(4)   // PD4
#define SCK_off PORTD &= ~_BV(4) // PD4
#define CE_on PORTD |= _BV(5)    // PD5
#define CE_off PORTD &= ~_BV(5)  // PD5
#define CS_on PORTC |= _BV(1)    // PC1
#define CS_off PORTC &= ~_BV(1)  // PC1
// SPI input
#define  MISO_on (PINC & _BV(0)) // PC0

#define RF_POWER TX_POWER_80mW

uint8_t packet[32];
uint8_t packet_size = 32;

// serial printf
void sp(char *fmt, ... ){
    char buf[32]; // resulting string limited to 32 chars
    va_list args;
    va_start (args, fmt );
    vsnprintf(buf, 128, fmt, args);
    va_end (args);
    Serial.print(buf);
}

void setup()
{
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, LOW); //start LED off
    pinMode(MOSI_pin, OUTPUT);
    pinMode(SCK_pin, OUTPUT);
    pinMode(CS_pin, OUTPUT);
    pinMode(CE_pin, OUTPUT);
    pinMode(MISO_pin, INPUT);
    NRF24L01_Reset();
    NRF24L01_Initialize();
    NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);      // No Auto Acknoledgement
    
    NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0, packet_size); // rx pipe 0 size
    
    NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x01);  // Enable data pipe 0 only
    NRF24L01_SetBitrate(NRF24L01_BR_1M);             // 1Mbps, todo: try 250kbps as well
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);     // Clear data ready, data sent, and retransmit
    
    // sort output to find potential address:
    //
    // $> sort data.txt | uniq -c | sort -nr > out.txt
    uint8_t addr[] = {0x00,0x55}; // for scanning, todo: alternate with 0x00,0xaa
    //uint8_t addr[] = {0x00,0xaa};
     
    const u8 address_length = 2;
    
    NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, address_length -2);   // RX address length
    NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, addr, address_length);
    NRF24L01_WriteReg(NRF24L01_1C_DYNPD, 0x00);       // Disable dynamic payload length on all pipes
    NRF24L01_WriteReg(NRF24L01_1D_FEATURE, 0x00);     // Set feature bits
    NRF24L01_SetPower(2);
    NRF24L01_FlushRx();
    NRF24L01_FlushTx();
    Serial.begin(115200);
    Serial.println("Hello captain!");
}

void loop()
{
    const uint8_t channel = 0; // TODO: try with 0-127
    uint8_t i;
    NRF24L01_SetTxRxMode(RX_EN);
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
    NRF24L01_FlushRx();
    NRF24L01_WriteReg(NRF24L01_05_RF_CH, channel);
    
    NRF24L01_WriteReg(NRF24L01_00_CONFIG, _BV(NRF24L01_00_PRIM_RX) | _BV(NRF24L01_00_PWR_UP)); // rx mode, no crc
    while(!(NRF24L01_ReadReg(NRF24L01_07_STATUS) & _BV(NRF24L01_07_RX_DR))) {
        ; // wait rx fifo ready    
    }
    
    NRF24L01_ReadPayload(packet,packet_size);
    
    // dump packet
    sp("\n");
    for(i=0; i<packet_size; i++) {
        sp("%02x " , packet[i] );
    } 
 }
