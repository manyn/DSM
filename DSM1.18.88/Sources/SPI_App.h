
#ifndef __SPI_APP
#define __SPI_APP

/* MODULE SPI. */

//Flash
#define WREN 0x06   //Write Enable
#define WRDI 0x04   //Write Disable

#define RDSR 0x05	//Read Status Register
#define WRSR 0x01	//Write Status Register

#define READ 0x03	//Read Data Bytes
#define PP   0x02	//Page Program

#define BE   0xC7	//Bulk Erase
#define SE	 0xD8	//Sector erase
#define RDID 0x9F	//Read Identification
#define RES  0xAB   //Read Electronic signature

void InitSPI(void);

float ReadTemp();   //read temprature
void SectorEraseU(dword *adr);                     
void WriteFDataU(dword *adr,byte *Buf,word Length);
void ReadFDataU(dword *adr,byte *Buf,word Length);
void LB_OffsetToAdr(dword LB,word Offset,dword *adr);
void SenToAdr(word SeN,dword *adr);  //SeN :0~511
word LB_ToSen(dword LB);
word Adr_ToSen(dword adr);
dword LookForBlkAdr(DMSTAT *dms);   //UnitLength <= 256/4


#endif 
