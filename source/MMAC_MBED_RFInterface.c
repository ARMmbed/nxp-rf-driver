/*
 * Copyright (c) 2014-2015 ARM. All rights reserved.
 */
#include "ns_types.h"
#include "cmsis.h"
#include "platform/arm_hal_interrupt.h"
#include "nanostack/platform/arm_hal_phy.h"
#include "nxp-rf-driver/MMAC.h"
#include "nxp-rf-driver/BbcAndPhyRegs.h"
#include "MicroSpecific.h"

#define RF_BUFFER_SIZE 128

#define BBC_MIN_BE         (3) 
#define BBC_MAX_BE         (5)
#define BBC_NUM_BACKOFFS   (5)

#define BBC_CHAN_COUNT     (16) 
#define BBC_CHAN_MASK      (0x07FFF800)
#define BBC_MAX_MTU_SIZE   (127) 

/*RF receive buffer*/
static tsMacFrame sRxFrame, sTxFrame;
const teTxOption eOptions_Tx = 0x1A; //(E_MMAC_TX_START_NOW | E_MMAC_TX_USE_AUTO_ACK | E_MMAC_TX_USE_CCA);   /* Start Now, Auto ACK, Use CCA */
const teRxOption eOptions_Rx = (0x1A | E_MMAC_RX_ADDRESS_MATCH); // (E_MMAC_RX_START_NOW | E_MMAC_RX_USE_AUTO_ACK | E_MMAC_RX_ALLOW_MALFORMED | E_MMAC_RX_ALLOW_FCS_ERROR | E_MMAC_RX_ADDRESS_MATCH);
static phy_device_driver_s device_driver;
static uint8_t nxp_MAC[8] = {1, 2, 3, 4, 5, 6, 7, 8};
static phy_device_channel_info_s channel_info;
static int8_t rf_radio_driver_id = -1;
static uint8_t mac_tx_handle = 0;
static int8_t i8MMAC_Extension(phy_extension_type_e extension_type,uint8_t *data_ptr);
static int8_t i8MMAC_Address_Write(phy_address_type_e address_type,uint8_t *address_ptr);
int8_t i8MMAC_Start_Cca(uint8_t *data_ptr, uint16_t data_length, uint8_t tx_handle, data_protocol_e data_protocol);
static int8_t i8MMAC_Interface_State_Control(phy_interface_state_e new_state, uint8_t rf_channel);
void vMMAC_Set_PanId(uint8_t *address_ptr);
void vMMAC_Set_Short_Address(uint8_t *address_ptr);
void vMMAC_Set_Ext_Address(uint8_t *address_ptr);
void vRadioToOff(void);
static void vMMAC_Set_CSMA_Parameters(uint8 u8MinBe, uint8 u8MaxBe, uint8 u8CsmaBackoffs);
static volatile bool bReceiving = FALSE;
static uint8_t rf_phy_channel = 12;


/*
 * \brief Function initialises and registers the RF driver.
 *
 * \param none
 *
 * \return rf_radio_driver_id Driver ID given by NET library
 */
int8_t rf_device_register(void)
{
	tsExtAddr MAC_addr;

	/* Read MAC address from HW */
	vMMAC_GetMacAddress(&MAC_addr);
	nxp_MAC[0] = (MAC_addr.u32H >> 24) & 0xFF; 
	nxp_MAC[1] = (MAC_addr.u32H >> 16) & 0xFF;
	nxp_MAC[2] = (MAC_addr.u32H >> 8) & 0xFF;
	nxp_MAC[3] = (MAC_addr.u32H) & 0xFF;
	nxp_MAC[4] = (MAC_addr.u32L >> 24) & 0xFF; 
	nxp_MAC[5] = (MAC_addr.u32L >> 16) & 0xFF;
	nxp_MAC[6] = (MAC_addr.u32L >> 8) & 0xFF;
	nxp_MAC[7] = (MAC_addr.u32L) & 0xFF;
	
    /* Set up BBC; may need to access BBC registers in reset code below */
    vMMAC_Enable();
    vMMAC_ConfigureRadio();
	
	/* Set Default MAC Address */
	//vMMAC_Set_Ext_Address(nxp_MAC);
	
	/*Set pointer to MAC address*/
	device_driver.PHY_MAC = nxp_MAC;
	device_driver.driver_description = "NXP_MAC";
	
	/*Number of channels in PHY*/
	channel_info.channel_count = BBC_CHAN_COUNT;
	
	/*Channel mask 26-11*/
	channel_info.channel_mask = BBC_CHAN_MASK;
	
	/* Type of RF PHY 802.15.4 2.4GHz */
	device_driver.link_type = PHY_LINK_15_4_2_4GHZ_TYPE;	
	device_driver.link_channel_info = &channel_info;
	
	/*Maximum size of payload is 127*/
	device_driver.phy_MTU = BBC_MAX_MTU_SIZE;
	
	/*No header in PHY*/
	device_driver.phy_header_length = 0;
	
	/*No tail in PHY*/
	device_driver.phy_tail_length = 0;	
	
	/*Set address write function*/
	device_driver.address_write = &i8MMAC_Address_Write;
	
	/*Set RF extension function*/
	device_driver.extension = &i8MMAC_Extension;
	
	/*Set RF state control function*/
	device_driver.state_control = &i8MMAC_Interface_State_Control;
	
	/*Set transmit function*/
	device_driver.tx = &i8MMAC_Start_Cca;
	
	/* Set Min Be, Max Be and CSMA CA back off */
	vMMAC_Set_CSMA_Parameters(BBC_MIN_BE, BBC_MAX_BE, BBC_NUM_BACKOFFS);
	
	/* Enable BBC interrupt */
	NVIC_EnableIRQ(BBCSELECT_IRQn);
	
	/*Register device driver*/
	rf_radio_driver_id = arm_net_phy_register(&device_driver);	
	
	return rf_radio_driver_id;
}

static void vMMAC_Set_CSMA_Parameters(uint8 u8MinBe, uint8 u8MaxBe, uint8 u8CsmaBackoffs)
{
    uint32 u32RegValue;

    u32RegValue = u8MinBe
                  | (((uint32)u8CsmaBackoffs)
                     << REG_BBC_TXMBEBT_MAXBO_BIT)
                  | (((uint32)u8MaxBe)
                     << REG_BBC_TXMBEBT_MAXBE_BIT);
					 
	 vREG_BbcWrite(REG_BBC_TXMBEBT, u32RegValue);
}

/*
 * \brief Function sets the addresses to RF address filters.
 *
 * \param address_type Type of address
 * \param address_ptr Pointer to given address
 *
 * \return 0 Success
 */
static int8_t i8MMAC_Address_Write(phy_address_type_e address_type, uint8_t *address_ptr)
{
    int8_t ret_val = 0;
	
    switch (address_type)
    {
        /*Set 48-bit address*/
        case PHY_MAC_48BIT:
			break;
			
		/*Set 64-bit address*/
        case PHY_MAC_64BIT:
            vMMAC_Set_Ext_Address(address_ptr);
			break;
			
        /*Set 16-bit address*/
        case PHY_MAC_16BIT:
            vMMAC_Set_Short_Address(address_ptr);
			break;
			
        /*Set PAN Id*/
        case PHY_MAC_PANID:
            vMMAC_Set_PanId(address_ptr);
			break;
			
		default:
            break;

    }

    return ret_val;
}

/*
 * \brief Function controls the ACK pending, channel setting and energy detection.
 *
 * \param extension_type Type of control
 * \param data_ptr Data from NET library
 *
 * \return 0 Success
 */
static int8_t i8MMAC_Extension(phy_extension_type_e extension_type, uint8_t *data_ptr)
{
	static uint8 bLastAckPending = 0;
    switch (extension_type)
    {
        /* Control MAC pending bit for Indirect data transmission */
        case PHY_EXTENSION_CTRL_PENDING_BIT:
		{
			bool_t	bFramePending;
            if(*data_ptr)
            {
				bFramePending = 1;
                vREG_BbcWrite(REG_BBC_TXPEND, bFramePending);
            }
            else
            {
				bFramePending = 0;
                vREG_BbcWrite(REG_BBC_TXPEND, bFramePending);
            }
			bLastAckPending = bFramePending;
			break;
		}
        
		/* Return frame pending status*/
        case PHY_EXTENSION_READ_LAST_ACK_PENDING_STATUS:
            *data_ptr = bLastAckPending;
		    break;
			
        /* Set channel */
        case PHY_EXTENSION_SET_CHANNEL:
		    break;
			
        /* Read energy on the channel */
        case PHY_EXTENSION_READ_CHANNEL_ENERGY:
		    break;
			
        /* Read status of the link */
        case PHY_EXTENSION_READ_LINK_STATUS:
		    break;
			
		default:
            break;

    }
    return 0;
}


/*
 * \brief Function starts the CCA process before starting data transmission and copies the data to RF TX FIFO.
 *
 * \param data_ptr Pointer to TX data
 * \param data_length Length of the TX data
 * \param tx_handle Handle to transmission
 * \return 0 Success
 * \return -1 Busy
 */
int8_t i8MMAC_Start_Cca(uint8_t *data_ptr, uint16_t data_length, uint8_t tx_handle, data_protocol_e data_protocol )
{
	mac_tx_handle = tx_handle;

	/* send packet */
	uint8 u8Index = 0;
			
	/* Frame Control Field */
	sTxFrame.u16FCF = data_ptr[u8Index++];
	sTxFrame.u16FCF |= data_ptr[u8Index++] << 8;
	
	/* Sequence Number */
	sTxFrame.u8SequenceNum = data_ptr[u8Index++];
	
	/* Build Dest & Source addresses */
	if(sTxFrame.u16FCF & MAC_FCF_DEST_ADDR_MODE_MASK)
	{
		sTxFrame.u16DestPAN = data_ptr[u8Index++];
		sTxFrame.u16DestPAN |= (data_ptr[u8Index++] << 8);
		
		if(((sTxFrame.u16FCF & MAC_FCF_DEST_ADDR_MODE_MASK) >> MAC_FCF_DEST_ADDR_MODE_BIT) == MAC_FCF_ADDR_16BIT_SHRT_ADDR)
		{					
			sTxFrame.uDestAddr.u16Short = data_ptr[u8Index++];
			sTxFrame.uDestAddr.u16Short |= (data_ptr[u8Index++] << 8);				
		}
		else if(((sTxFrame.u16FCF & MAC_FCF_DEST_ADDR_MODE_MASK) >> MAC_FCF_DEST_ADDR_MODE_BIT) == MAC_FCF_ADDR_64BIT_LONG_ADDR)
		{
			sTxFrame.uDestAddr.sExt.u32L = data_ptr[u8Index++];
			sTxFrame.uDestAddr.sExt.u32L |= data_ptr[u8Index++] << 8;
			sTxFrame.uDestAddr.sExt.u32L |= data_ptr[u8Index++] << 16;
			sTxFrame.uDestAddr.sExt.u32L |= data_ptr[u8Index++] << 24;
				sTxFrame.uDestAddr.sExt.u32H = data_ptr[u8Index++];
				sTxFrame.uDestAddr.sExt.u32H |= data_ptr[u8Index++] << 8;
				sTxFrame.uDestAddr.sExt.u32H |= data_ptr[u8Index++] << 16;
				sTxFrame.uDestAddr.sExt.u32H |= data_ptr[u8Index++] << 24;				
			
		}
	}
		
	if(sTxFrame.u16FCF & MAC_FCF_SRC_ADDR_MODE_MASK)
	{
		if((sTxFrame.u16FCF & MAC_FCF_SRC_ADDR_MODE_MASK) && !(sTxFrame.u16FCF & 0x0040))
		{
			sTxFrame.u16SrcPAN = data_ptr[u8Index++];
			sTxFrame.u16SrcPAN |= data_ptr[u8Index++] << 8;
		}
			
		if(((sTxFrame.u16FCF & MAC_FCF_SRC_ADDR_MODE_MASK) >> MAC_FCF_SRC_ADDR_MODE_BIT) == MAC_FCF_ADDR_16BIT_SHRT_ADDR)
		{
			sTxFrame.uSrcAddr.u16Short = data_ptr[u8Index++];
			sTxFrame.uSrcAddr.u16Short |= data_ptr[u8Index++] << 8;				
		}
		else if(((sTxFrame.u16FCF & MAC_FCF_SRC_ADDR_MODE_MASK) >> MAC_FCF_SRC_ADDR_MODE_BIT) == MAC_FCF_ADDR_64BIT_LONG_ADDR)
		{
			sTxFrame.uSrcAddr.sExt.u32L = data_ptr[u8Index++];
			sTxFrame.uSrcAddr.sExt.u32L |= data_ptr[u8Index++] << 8;
			sTxFrame.uSrcAddr.sExt.u32L |= data_ptr[u8Index++] << 16;
			sTxFrame.uSrcAddr.sExt.u32L |= data_ptr[u8Index++] << 24;
			sTxFrame.uSrcAddr.sExt.u32H = data_ptr[u8Index++];
			sTxFrame.uSrcAddr.sExt.u32H |= data_ptr[u8Index++] << 8;
			sTxFrame.uSrcAddr.sExt.u32H |= data_ptr[u8Index++] << 16;
			sTxFrame.uSrcAddr.sExt.u32H |= data_ptr[u8Index++] << 24;					
		}
	}	

	if (bReceiving || (u32REG_BbcRead(REG_BBC_TXBUFAD) & 0xffff))
	{			
		//printf("Transmission Failure \r\n");
		return -1; //busy
	}			

	sTxFrame.u8PayloadLength = (uint8)data_length - u8Index;			
	memcpy(sTxFrame.uPayload.au8Byte, &data_ptr[u8Index], sTxFrame.u8PayloadLength);			
		
	vMMAC_StartMacTransmit(&sTxFrame, eOptions_Tx);
	
    /*Return success*/
    return 0;
}

/*
 * \brief Function gives the control of RF states to MAC.
 *
 * \param new_state RF state
 * \param rf_channel RF channel
 *
 * \return 0 Success
 */
static int8_t i8MMAC_Interface_State_Control(phy_interface_state_e new_state, uint8_t rf_channel)
{
    int8_t ret_val = 0;
	
    switch (new_state)
    {
        /*Reset PHY driver and set to idle*/
        case PHY_INTERFACE_RESET:
			break;
			
        /*Disable PHY Interface driver*/
        case PHY_INTERFACE_DOWN:
			vRadioToOff();
			break;
			
        /*Enable PHY Interface driver*/
        case PHY_INTERFACE_UP:
		{	
			rf_phy_channel = rf_channel;
            vMMAC_SetChannelAndPower(rf_channel, 11);				//TODO by later
			memset(&sRxFrame, 0, sizeof(tsMacFrame));
            vMMAC_StartMacReceive(&sRxFrame, eOptions_Rx);
		}
		    break;
			
        /*Enable wireless interface ED scan mode*/
        case PHY_INTERFACE_RX_ENERGY_STATE:
			break;
			
        case PHY_INTERFACE_SNIFFER_STATE:             /**< Enable Sniffer state */
		{
            //rf_mode = RF_MODE_SNIFFER;              //TODO by later
			vMMAC_SetChannelAndPower(rf_channel, 11);			  //TODO by later	
			memset(&sRxFrame, 0, sizeof(tsMacFrame));
            vMMAC_StartMacReceive(&sRxFrame, eOptions_Rx);
			break;
		}
		
		default:
            break;
    }

    return ret_val;
}

/*
 * \brief Function set MAC Extended Address of chip.
 *
 * \param address_ptr Pointer to given address
 *
 */
void vMMAC_Set_Ext_Address(uint8_t *address_ptr)
{
	uint32 u32LowAddr, u32HighAddr;
	
	/* copy to NXP MAC */
	memcpy(nxp_MAC, address_ptr, 8);
	
	u32LowAddr = ((address_ptr[4] << 24) | (address_ptr[5] << 16) | (address_ptr[6] << 8) | (address_ptr[7] ));
	u32HighAddr = ((address_ptr[0] << 24) | (address_ptr[1] << 16) | (address_ptr[2] << 8) | (address_ptr[3] ));
		
	vREG_BbcWrite(REG_BBC_RXMEADL, u32LowAddr);
    vREG_BbcWrite(REG_BBC_RXMEADH, u32HighAddr);
}

/*
 * \brief Function set MAC Short Address of chip.
 *
 * \param address_ptr Pointer to given address
 *
 */
void vMMAC_Set_Short_Address(uint8_t *address_ptr)
{	
	uint16 u16Addr;
	
	u16Addr = ((address_ptr[0] << 8) | (address_ptr[1]));
	vREG_BbcWrite(REG_BBC_RXMSAD, u16Addr);
}

/*
 * \brief Function set panid of chip.
 *
 * \param address_ptr Pointer to given address
 *
 */
void vMMAC_Set_PanId(uint8_t *address_ptr)
{	
	uint16 u16PanId;
	
	u16PanId = ((address_ptr[0] << 8) | (address_ptr[1]));
	vREG_BbcReadModWrite32(REG_BBC_RXMPID, REG_BBC_RXMPID_PAN_ID_MASK,
                           u16PanId << REG_BBC_RXMPID_PAN_ID_BIT);
}

/*
 * \brief Function set panid of chip.
 *
 * \param address_ptr Pointer to given address
 *
 */
void vRadioToOff(void)
{
    uint32 u32State;

    vMMAC_RadioOff();

    /* Wait for radio to go idle: if transmitting, this will be allowed to
       complete */
    do
    {
        u32State = u32REG_BbcRead(REG_BBC_SM_STATE) & REG_BBC_SM_STATE_SUP_MASK;
        u32State |= (u32REG_PhyRead(REG_PHY_STAT) & REG_PHY_STAT_STATE_MASK);
    } while (u32State != 0);
}

/*
 * \brief Handler BBC.
 *
 * \param void
 *
 */
void vAHI_IntHandlerBbc_select(void)
{
    uint32 u32IntMask;
    uint32 u32SetBit;

    /* Read enabled interrupts */
    u32IntMask = u32REG_BbcRead(REG_BBC_MISR);

    /* Clear them */
    vREG_BbcWrite(REG_BBC_ISR, u32IntMask);

    while (u32IntMask)
    {
        /* Get next set bit */
        u32SetBit = FF1(u32IntMask) - 1;

        /* Clear the bit */
        u32IntMask ^= (1UL << u32SetBit);

        /* Act on interrupt */
        switch (u32SetBit)
        {
			case REG_BBC_INT_TX_BIT:
			{
				vREG_BbcWrite(REG_BBC_TXBUFAD, 0);
				
				/* Check status Tx */
				uint32 u32TxStatus = u32MMAC_GetTxErrors();
				
				/* Act upon TX status */
				if (u32TxStatus & REG_BBC_TXSTAT_RXABT_MASK)
				{
					/* 	TX aborted due to ongoing or new RX.
					Check that frame has not been expired whilst being sent. If it has,
					and is currently not finished, change it to finished and set the
					status to 'transaction expired' */
					arm_net_phy_tx_done(rf_radio_driver_id, mac_tx_handle, PHY_LINK_TX_FAIL, 1, 1);
				}
				else if (u32TxStatus & REG_BBC_TXSTAT_CCAE_MASK)
				{
					arm_net_phy_tx_done(rf_radio_driver_id, mac_tx_handle, PHY_LINK_CCA_FAIL, 8, 1);
				}
				else if (u32TxStatus & REG_BBC_TXSTAT_ACKE_MASK)
				{
					arm_net_phy_tx_done(rf_radio_driver_id, mac_tx_handle, PHY_LINK_TX_FAIL, 1, 1);
				}
				else
				{
					arm_net_phy_tx_done(rf_radio_driver_id, mac_tx_handle, PHY_LINK_TX_DONE, 1, 1);
				}			
				
				/* Receive mode */
				vMMAC_StartMacReceive(&sRxFrame, eOptions_Rx);
			}
				break;

			case REG_BBC_INT_RX_H_BIT:
				bReceiving = TRUE;
				break;

			case REG_BBC_INT_RX_BIT:
			{
				uint8 u8PktData[RF_BUFFER_SIZE];
				uint8 u8Index = 0;
				//uint8 u8Lqi = 3; /* Need scaling from LQI value */
				uint8 u8Rssi = u8MMAC_GetRxLqi(NULL);
			
				/* copy frame header */
				u8PktData[u8Index++] = (uint8)(sRxFrame.u16FCF & 0x00FF);
				u8PktData[u8Index++] = (uint8)((sRxFrame.u16FCF & 0xFF00) >> 8);
			
				/* Copy Seq No */
				u8PktData[u8Index++] = sRxFrame.u8SequenceNum;
			
				/* Copy Dest Pan Id */
				if(sRxFrame.u16FCF & MAC_FCF_DEST_ADDR_MODE_MASK)
				{
					u8PktData[u8Index++] = (uint8)(sRxFrame.u16DestPAN & 0x00FF);
					u8PktData[u8Index++] = (uint8)((sRxFrame.u16DestPAN & 0xFF00) >> 8);
				
					if(((sRxFrame.u16FCF & MAC_FCF_DEST_ADDR_MODE_MASK) >> MAC_FCF_DEST_ADDR_MODE_BIT) == MAC_FCF_ADDR_16BIT_SHRT_ADDR)
					{
						u8PktData[u8Index++] = (uint8)(sRxFrame.uDestAddr.u16Short & 0x00FF);
						u8PktData[u8Index++] = (uint8)((sRxFrame.uDestAddr.u16Short & 0xFF00) >> 8);					
					}
					else if(((sRxFrame.u16FCF & MAC_FCF_DEST_ADDR_MODE_MASK) >> MAC_FCF_DEST_ADDR_MODE_BIT) == MAC_FCF_ADDR_64BIT_LONG_ADDR)
					{					
						u8PktData[u8Index++] = (uint8)(sRxFrame.uDestAddr.sExt.u32L & 0x000000FF);
						u8PktData[u8Index++] = (uint8)((sRxFrame.uDestAddr.sExt.u32L & 0x0000FF00) >> 8);	
						u8PktData[u8Index++] = (uint8)((sRxFrame.uDestAddr.sExt.u32L & 0x00FF0000) >> 16);
						u8PktData[u8Index++] = (uint8)((sRxFrame.uDestAddr.sExt.u32L & 0xFF000000) >> 24);

						u8PktData[u8Index++] = (uint8)(sRxFrame.uDestAddr.sExt.u32H & 0x000000FF);
						u8PktData[u8Index++] = (uint8)((sRxFrame.uDestAddr.sExt.u32H & 0x0000FF00) >> 8);	
						u8PktData[u8Index++] = (uint8)((sRxFrame.uDestAddr.sExt.u32H & 0x00FF0000) >> 16);
						u8PktData[u8Index++] = (uint8)((sRxFrame.uDestAddr.sExt.u32H & 0xFF000000) >> 24);					
					}
				}
			
				if(sRxFrame.u16FCF & MAC_FCF_SRC_ADDR_MODE_MASK)
				{
					if((sRxFrame.u16FCF & MAC_FCF_SRC_ADDR_MODE_MASK) && !(sRxFrame.u16FCF & 0x0040))
					{
						u8PktData[u8Index++] = (uint8)(sRxFrame.u16SrcPAN & 0x00FF);
						u8PktData[u8Index++] = (uint8)((sRxFrame.u16SrcPAN & 0xFF00) >> 8);					
					}
				
					if(((sRxFrame.u16FCF & MAC_FCF_SRC_ADDR_MODE_MASK) >> MAC_FCF_SRC_ADDR_MODE_BIT) == MAC_FCF_ADDR_16BIT_SHRT_ADDR)
					{
						u8PktData[u8Index++] = (uint8)(sRxFrame.uSrcAddr.u16Short & 0x00FF);
						u8PktData[u8Index++] = (uint8)((sRxFrame.uSrcAddr.u16Short & 0xFF00) >> 8);
					}
					else if(((sRxFrame.u16FCF & MAC_FCF_SRC_ADDR_MODE_MASK) >> MAC_FCF_SRC_ADDR_MODE_BIT) == MAC_FCF_ADDR_64BIT_LONG_ADDR)
					{	
						u8PktData[u8Index++] = (uint8)(sRxFrame.uSrcAddr.sExt.u32L & 0x000000FF);
						u8PktData[u8Index++] = (uint8)((sRxFrame.uSrcAddr.sExt.u32L & 0x0000FF00) >> 8);	
						u8PktData[u8Index++] = (uint8)((sRxFrame.uSrcAddr.sExt.u32L & 0x00FF0000) >> 16);
						u8PktData[u8Index++] = (uint8)((sRxFrame.uSrcAddr.sExt.u32L & 0xFF000000) >> 24);

						u8PktData[u8Index++] = (uint8)(sRxFrame.uSrcAddr.sExt.u32H & 0x000000FF);
						u8PktData[u8Index++] = (uint8)((sRxFrame.uSrcAddr.sExt.u32H & 0x0000FF00) >> 8);	
						u8PktData[u8Index++] = (uint8)((sRxFrame.uSrcAddr.sExt.u32H & 0x00FF0000) >> 16);
						u8PktData[u8Index++] = (uint8)((sRxFrame.uSrcAddr.sExt.u32H & 0xFF000000) >> 24);					
					}
				}
					
				memcpy(&u8PktData[u8Index], sRxFrame.uPayload.au8Byte, sRxFrame.u8PayloadLength);
				//vREG_BbcWrite(REG_BBC_RXBUFAD, 0);
				arm_net_phy_rx(PHY_LAYER_PAYLOAD, u8PktData, (sRxFrame.u8PayloadLength + u8Index), 0xff, 0xff, rf_radio_driver_id);
				i8MMAC_Interface_State_Control(PHY_INTERFACE_UP, rf_phy_channel);
				bReceiving = FALSE;
				break;
			}
			
			case REG_BBC_INT_M0_BIT:
				//vTimerFired();
				break;
			
			default:
				break;
		}
	}
}

/*
 * \brief Function reads the MAC address array.
 *
 * \param ptr Pointer to read array
 *
 * \return none
 */
void rf_read_mac_address(uint8_t *ptr)
{
    memcpy(ptr, nxp_MAC, 8);
}

PUBLIC void vJPT_TxPowerAdjust(uint8 u8PowerAdj, uint8 u8Att3db, uint8 u8Channel)
{
#if (defined JPT_JN5164)
    static uint32 u32PhyPACTRL = 0;

    u32PhyPACTRL = READ_REG32(ADDR_PHY_PA_CTRL);

    if (u8Att3db != 0) // 3dB attenaution
    {
        // set PA_ATTEN
        u32PhyPACTRL |= (0x1 << 4);
    }

    WRITE_REG32(ADDR_PHY_PA_CTRL, u32PhyPACTRL);

#endif
}
