#ifndef _EEEPROM_H
#define _EEEPROM_H

typedef struct    //Used to copy and store Flash error status registers.
{
  unsigned char fstat_var;
  unsigned char ferfstat_var;
}ErrType;
             
/**** P-Flash and D-Flash Commands ****/

#define ERASE_VERIFY_ALL_BLOCKS  0x01 
// Verify that all program and data Flash blocks are erased.
// CCOBIX end = 0
// CCOB Params - NONE
// MGSTAT set if fault
                                             
#define ERASE_VERIFY_BLOCK       0X02      
// Verify that a Flash block is erased.
// CCOBIX end = 0
// CCOB Params - gpage
// MGSTAT set if fault

#define ERASE_ALL_BLOCKS         0x08 
// Erase all program and data Flash blocks.
// An erase of all Flash blocks is only possible when the FPLDIS, FPHDIS, and FPOPEN
// bits in the FPROT register and the EPDIS and EPOPEN bits in the EPROM register are
// set prior to launching the command.
// CCOBIX end = 0
// CCOB Params - NONE
// MGSTAT set if fault, FPVIOL / ACCERR set where appropriate

#define UNSECURE_FLASH           0x0B 
// Supports a method of releasing MCU security by erasing all program and data Flash
// blocks and verifying that all program and data Flash blocks are erased.
// CCOBIX end = 0
// CCOB Params - NONE
// MGSTAT set if fault

#define SET_USER_MARGIN_LEVEL    0x0D 
// Specifies a user margin read level for all program Flash blocks.
// CCOBIX end = 1
// CCOB Params - gpage, level setting (0-2) in CCOB[1]
// ACCERR set if invalid level

#define SET_FIELD_MARGIN_LEVEL   0x0E 
// Specifies a field margin read level for all program Flash blocks (special modes only).
// CCOBIX end = 1
// CCOB Params - gpage, level setting (0-4) in CCOB[1]
// ACCERR set if invalid level

/* **** P-Flash Only Commands ****/

#define ERASE_VERIFY_P_FLASH_SECTION 0x03  
// Verify that a given number of words starting at the address provided are erased.
// CCOBIX end = 2
// CCOB Params - global address, number of phrases in CCOB[2]
// MGSTAT set if fault

#define READ_ONCE	               0x04  
// Read a phrase from a dedicated 64 word area in a hidden region of a programFlash block
// that was previously programmed using the Program Once command.
// CCOBIX end = 1
// CCOB Params - read once index (0-3) in CCOB[1], phrase in CCOB [5:2]
// returns phrase in CCOB [4:1]

#define LOAD_DATA_FIELD          0x05 
// Load data for simultaneous multiple program Flash block operations.
// CCOBIX end = 5
// CCOB Params - global address, phrase in CCOB [5:2]
#define PROGRAM_P_FLASH          0x06 
// Program a phrase in a program Flash block and any previously loaded phrases for any
// other program Flash block (see Load Data Field command).
// CCOBIX end = 5
// CCOB Params - global address, phrase in CCOB [5:2]
// MGSTAT set if fault, FPVIOL / ACCERR set where appropriate

#define PROGRAM_ONCE             0x07 
// Program a dedicated 64 word area in a hidden region of a program Flash block that is
// allowed to be programmed only once.
// CCOBIX end = 5
// CCOB Params - read once index (0-3) in CCOB[1], phrase in CCOB [5:2]
// MGSTAT set if fault

#define ERASE_P_FLASH_BLOCK      0x09 
// Erase a program Flash block.
// An erase of the full program Flash block is only possible when FPLDIS, FPHDIS and
// FPOPEN bits in the FPROT register are set prior to launching the command.
// CCOBIX end = 1
// CCOB Params - global address
// MGSTAT set if fault, FPVIOL / ACCERR set where appropriate

#define ERASE_P_FLASH_SECTOR 0x0A 
// Erase all bytes in a program Flash sector.
// CCOBIX end = 1
// CCOB Params - global address
// MGSTAT set if fault, FPVIOL / ACCERR set where appropriate

#define VERIFY_BACKDOOR_ACCESS_KEY 0x0C 
// Supports a method of releasing MCU security by verifying a set of security keys.
// CCOBIX end = 4
// CCOB Params - backdoor key in CCOB [1:4]
// ACCERR set if not verified

/**** D-Flash Only Commands ****/

#define ENABLE_D_FLASH 0x0F 
// Partition a section of D-Flash for user access and EEE.
// CCOBIX end = 2
// CCOB Params - number of sectors for D-Flash in CCOB[1],  number of sectors for EEE in CCOB[2]
// ACCERR set if fault

#define FULL_PARTITION_D_FLASH 0x0F 
// Partition a section of D-Flash for user access and EEE.
// CCOBIX end = 2
// CCOB Params - number of sectors for D-Flash in CCOB[1],  number of sectors for EEE in CCOB[2]
// ACCERR set if fault

#define ERASE_VERIFY_D_FLASH_SECTION 0x10 
// Verify that a given number of words starting at the address provided are erased.
// CCOBIX end = 2
// CCOB Params - global address of first word, number of words to verify CCOB[2]
// MGSTAT set if fault
#define PROGRAM_D_FLASH         0x11 
// Program up to four words in the data Flash block (see Load Data Field command).
// CCOBIX end = 2
// CCOB Params - global address, up to 4 data words in CCOB [2:5]
// MGSTAT set if fault, EPVIOL / ACCERR set where appropriate

#define ERASE_D_FLASH_SECTOR    0x12 
// Erase all bytes in a data Flash sector.
// CCOBIX end = 2
// CCOB Params - global address
// MGSTAT set if fault, EPVIOL  set where appropriate

#define ENABLE_EEPROM_EMULATION    0x13 
// Requests the FTMSM to enable EEPROM emulation.
// CCOBIX end = 0
// CCOB Params - NONE

#define DISABLE_EEPROM_EMULATION   0x14 
// Requests the FTMSM to suspend all current erase and program activity related to
// EEPROM emulation but leave current EEE tags set.
// CCOBIX end = 0
// CCOB Params - NONE

#define CANCEL_EEPROM_EMULATION    0x15   /* M22E mask only */
// Requests the FTMSM to suspend all current erase and program activity related to
// EEPROM emulation and clear all outstanding EEE tags.
// CCOBIX end = 0
// CCOB Params - NONE

#define EEPROM_QUERY    0x15   /* M48H mask only */
// Requests EEE status information.
// CCOBIX end = 0
// CCOB Return Params -
// CCOB[1] DFPART - size of D-Flash user partition (x256 bytes)
// CCOB[2] ERPART - size of EEE ram (x256 bytes)
// CCOB[3] ECOUNT - typical number of erase cycles for the EEE sectors
// CCOB[4] Dead sector count / Ready sector count

#define PARTITION_D_FLASH 0x20  /* M48H mask only */
// Partition a section of D-Flash for user access and EEE.
// CCOBIX end = 2
// CCOB Params - number of sectors for D-Flash in CCOB[1],  number of sectors for EEE in CCOB[2]
// ACCERR set if fault

//Uncomment the FCLK_DIV value according to the oscillator crystal.
//These values are chosen from the MC9S12XE100 datasheet.
//#define FCLK_DIV 0x1    // Flash clock divider for 2MHz crystal
//#define FCLK_DIV 0x3    // Flash clock divider for 4MHz crystal
//#define FCLK_DIV 0x7    // Flash clock divider for 8MHz crystal
//#define FCLK_DIV 0x9    // Flash clock divider for 10MHz crystal
#define FCLK_DIV 0xF    // Flash clock divider for 16MHz crystal
//#define FCLK_DIV 0x18   // Flash clock divider for 26MHz crystal
#define FDIV_NOT_SET       0x01
#define COMMAND_BUSY       0x02
#define PARTITION_MISMATCH 0x04
#define CCIF_MASK          0x80

// bit masks for FSTAT
#define mgstat0 0x01  
#define mgstat1 0x02
#define mgbusy  0x08
#define fpviol  0x10
#define accerr  0x20
#define ccif    0x80

//bit masks for FERSTAT
#define sfdif    0x01
#define dfdif    0x02
#define ersvif0  0x04
#define ersvif1  0x08
#define epviolif 0x10
#define pgmerif  0x40
#define erserif  0x80



//EEE SIZE SECTION
/**** SET THE SIZE OF THE EEE HERE ****/
#define EEE_RAM 1 //Specify the # of EEE RAM sections for the FTM (Flash
                  //module) to use as EEE. The valid range is 0-16 sections.

/**** CALCULATES AMOUNT OF DATA FLASH FOR GENERAL USE ****/
#if(EEE_RAM == 1)
   #define DFPART (128 - 12)
#else
   #define DFPART (128 - (8 * EEE_RAM))
#endif


//function

//EEEPROM initial

void Eeeprom_intial(void);   //Inital EEEPROM

ErrType LaunchFlashCommand(char params, unsigned char ccob0high, unsigned char ccob0low, unsigned int ccob1,
unsigned int ccob2,unsigned int ccob3,unsigned int ccob4,unsigned int ccob5, unsigned int ccob6, unsigned int ccob7);
void PartionEEPROM(void);



//user defined partion switch
//#define FORCE_PARTITON_FOR_DEBUG 





#endif