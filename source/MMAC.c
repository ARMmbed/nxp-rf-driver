/****************************************************************************
 *
 * MODULE:             Micro MAC
 *
 * DESCRIPTION:
 * Low-level functions for MAC/BBC control
 *
 ****************************************************************************
 *
 * This software is owned by NXP B.V. and/or its supplier and is protected
 * under applicable copyright laws. All rights are reserved. We grant You,
 * and any third parties, a license to use this software solely and
 * exclusively on NXP products [NXP Microcontrollers such as JN5148, JN5142, JN5139].
 * You, and any third parties must reproduce the copyright and warranty notice
 * and any other legend of ownership on each copy or partial copy of the
 * software.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Copyright NXP B.V. 2013. All rights reserved
 *
 ****************************************************************************/

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/
#include "jendefs.h"
#include "nxp-rf-driver/BbcAndPhyRegs.h"
#include "PeripheralRegs.h"
#include "MicroSpecific.h"
#include "nxp-rf-driver/MMAC.h"
#include "nxp-rf-driver/xcv_pub.h"
//#include "nxp-rf-driver/JPT.h"

#if (defined JENNIC_CHIP_FAMILY_JN516x) && !(defined JENNIC_CHIP_JN5169)
/* Recent changes can be applied to JN5168. However, to avoid changing the
   existing certified library there is a build option to avoid the changes. A
   future rebuild of the JN5168 could remove this restriction */
//#define JN5168_NO_CHANGE
#endif

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
#define INDEX_ADDR(PAGE,WORD) (0x01001000 + ((PAGE) << 8) + ((WORD) << 4))
#define LOOKUP_PAGE           (4)
#define LOOKUP_START_WORD     (3)
#define LOOKUP_END_WORD       (7)
#define PHY_BASE_ADDR         (REG_SYS_BASE + (PHY_OFFSET << 2))

/* Customer MAC address at page 5, word 7 */
#define MAC_ADDR_CUSTOMER 0x01001570
/* Default MAC address at page 5, word 8 */
#define MAC_ADDR_DEFAULT 0x01001580

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/
#if !(defined JN5168_NO_CHANGE)
/* Copied from MiniMac.h. Only used internally here so not exposed in public
   API, which makes it tricky to manage. Hence a copy rather than a
   reference */
typedef enum PACK
{
    E_MODULE_STD   = 0,
    E_MODULE_HPM05 = 1,
    E_MODULE_HPM06 = 2
} teModuleType;
#endif

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/
PUBLIC void vMMAC_IntHandlerBbc(void);
#if !(defined JN5168_NO_CHANGE)
PRIVATE uint8 u8GetPapValue(int8 i8TxPower);
#endif
#ifdef INCLUDE_ECB_API
PRIVATE void vAesCmdWaitBusy(void);
#endif
PUBLIC void vJPT_TxPowerAdjust(uint8 u8PowerAdj, uint8 u8Att3db, uint8 u8Channel);
/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/
#if (defined JENNIC_CHIP_FAMILY_JN516x) && !(defined JENNIC_CHIP_JN5169)
#if !(defined JN5168_NO_CHANGE)
PUBLIC teModuleType eMMacModuleType;
#endif
#endif

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/
PRIVATE void (*prIntHandler)(uint32);
PRIVATE uint8 u8SctlMask;

#if (defined JENNIC_CHIP_JN5169) || (defined JENNIC_CHIP_FAMILY_JN517x)
/* It fortuitously happens that the default for these is 0, because the
   default power setting for the radio is 8 (actually 8.5) dBm; see the
   table in u8GetPapValue */
PRIVATE uint8 u8PowerAdj = 0;
PRIVATE uint8 u8Atten3db = 0;
#endif

/****************************************************************************/
/***        Public Functions                                              ***/
/****************************************************************************/
/* Initialisation */
PUBLIC void vMMAC_Enable(void)
{
    /* Enable protocol domain: stack won't work without it */
    vREG_SysWrite(REG_SYS_PWR_CTRL,
                  u32REG_SysRead(REG_SYS_PWR_CTRL) | REG_SYSCTRL_PWRCTRL_PPDC_MASK);

    /* Ensure protocol domain is running */
    while ((u32REG_SysRead(REG_SYS_STAT)
            & REG_SYSCTRL_STAT_PROPS_MASK) == 0);

    /* Clear out interrupt registers */
    vREG_BbcWrite(REG_BBC_ISR, 0xffffffff);

    /* Enable TX and RX interrupts within BBC: allows them to wake CPU from
       doze, but not enabled enough to generate an interrupt (see
       vMMAC_EnableInterrupts for that) */
    vREG_BbcWrite(REG_BBC_IER, REG_BBC_INT_TX_MASK
                               | REG_BBC_INT_RX_MASK
                               | REG_BBC_INT_RX_H_MASK);
}

PUBLIC void vMMAC_EnableInterrupts(void (*prHandler)(uint32))
{
    /* Store user handler */
    prIntHandler = prHandler;

#if !(defined JENNIC_CHIP_FAMILY_JN517x)
    /* Set up BBC interrupt handler. JN517x sets this at compile time */
    isr_handlers[MICRO_ISR_NUM_BBC] = vMMAC_IntHandlerBbc;
#endif

    /* Enable BBC interrupt in PIC */
    MICRO_SET_PIC_ENABLE(MICRO_ISR_MASK_BBC);

    /* Enable interrupts in CPU */
    MICRO_ENABLE_INTERRUPTS();
}

PUBLIC void vMMAC_SetChannel(uint8 u8Channel)
{
    uint32 u32State;
    uint32 u32RegData;
#ifdef WIFICM_SUPPORT
    uint32 u32RxMctrlData;
#endif
    MICRO_INT_STORAGE;

    /* Disable interrupts */
    MICRO_INT_ENABLE_ONLY(0);

    /* Read current RX control setting in case we want to re-enable it later */
    u32RegData = u32REG_BbcRead(REG_BBC_RXCTL);
#ifdef WIFICM_SUPPORT
    u32RxMctrlData = u32REG_BbcRead(REG_PHY_MCTRL);
#endif

    /* Turn radio off and wait for it to be off. If we were sending or
       receiving, this might result in an interrupt to be processed */
#ifdef WIFICM_SUPPORT
    vJPT_WifiCmToOff();
#endif
    vREG_BbcWrite(REG_BBC_RXCTL, 0);
#ifdef WIFICM_SUPPORT
    vREG_PhyWrite(REG_PHY_MCTRL, 0);
#endif

    do
    {
        u32State = u32REG_BbcRead(REG_BBC_SM_STATE) & REG_BBC_SM_STATE_SUP_MASK;
        u32State |= (u32REG_PhyRead(REG_PHY_STAT) & REG_PHY_STAT_STATE_MASK);
    } while (u32State != 0);

    /* Change channel */
    vREG_PhyWrite(REG_PHY_CHAN, u8Channel);

#if (defined JENNIC_CHIP_JN5169) || (defined JENNIC_CHIP_FAMILY_JN517x)
    /* Change BMOD setting */
    vJPT_TxPowerAdjust(u8PowerAdj, u8Atten3db, u8Channel);
#endif

    /* Check for pending RX interrupts: if not, return RX to previous state */
    if (0 == (u32REG_BbcRead(REG_BBC_MISR)
              & (REG_BBC_INT_TX_MASK | REG_BBC_INT_RX_MASK)
             )
       )
    {
        vREG_BbcWrite(REG_BBC_RXCTL, u32RegData);
#ifdef WIFICM_SUPPORT
        vREG_PhyWrite(REG_PHY_MCTRL, u32RxMctrlData);
#endif
#ifdef WIFICM_SUPPORT
        if (u32RegData)
        {
            vJPT_WifiCmToRx();
        }
#endif
    }

    /* Restore interrupts */
    MICRO_INT_RESTORE_STATE();
}

#if !(defined JN5168_NO_CHANGE)
/* Note: API has been written so that either this or the simpler
   vMMAC_SetChannel can be used independently. Applications that don't need
   TX power control can use vMMAC_SetChannel and leave the TX power at the
   default, which leaves out vMMAC_SetChannelAndPower and other functions and
   variables from the build. Conversely, applications that do support TX power
   control can use this function exclusively and so vMMAC_SetChannel is left
   out of the build */
PUBLIC void vMMAC_SetChannelAndPower(uint8 u8Channel, int i8TxPower)
{
    uint32 u32RegData;
    uint32 u32PapValue;
    uint32 u32State;
    uint32 u32RxCtlData;
#ifdef WIFICM_SUPPORT
    uint32 u32RxMctrlData;
#endif
    MICRO_INT_STORAGE;

    /* Disable interrupts */
    //MICRO_INT_ENABLE_ONLY(0);

    /* Read current RX control setting in case we want to re-enable it later */
    u32RxCtlData = u32REG_BbcRead(REG_BBC_RXCTL);
#ifdef WIFICM_SUPPORT
    u32RxMctrlData = u32REG_BbcRead(REG_PHY_MCTRL);
#endif

    /* Turn radio off and wait for it to be off. If we were sending or
       receiving, this might result in an interrupt to be processed */
#ifdef WIFICM_SUPPORT
    vJPT_WifiCmToOff();
#endif
    vREG_BbcWrite(REG_BBC_RXCTL, 0);
#ifdef WIFICM_SUPPORT
    vREG_PhyWrite(REG_PHY_MCTRL, 0);
#endif

    do
    {
        u32State = u32REG_BbcRead(REG_BBC_SM_STATE) & REG_BBC_SM_STATE_SUP_MASK;
        u32State |= (u32REG_PhyRead(REG_PHY_STAT) & REG_PHY_STAT_STATE_MASK);
    } while (u32State != 0);

    /* Change channel */
    vREG_PhyWrite(REG_PHY_CHAN, u8Channel);

    /* For M06 on channel 26, must turn down power to stay within
       standards. In other cases, set power to requested level. Call to set
       power level also stores u8PowerAdj and u8Atten3db if appropriate */
    u32RegData = u32REG_PhyRead(REG_PHY_PA_CTRL);

#if (defined JENNIC_CHIP_FAMILY_JN516x) && !(defined JENNIC_CHIP_JN5169)
    if (   (E_MODULE_HPM06 == eMMacModuleType)
        && (26             == u8Channel)
        && (0              <= i8TxPower)
       )
    {
        u32PapValue = u8GetPapValue(-9);
    }
    else
#endif
    {
        u32PapValue = u8GetPapValue(i8TxPower);
    }

    u32RegData &= ~REG_PHY_PA_CTRL_PAP_MASK;
    u32RegData |= u32PapValue << REG_PHY_PA_CTRL_PAP_BIT;
    vREG_PhyWrite(REG_PHY_PA_CTRL, u32RegData);

#if (defined JENNIC_CHIP_JN5169) || (defined JENNIC_CHIP_FAMILY_JN517x)
    /* Change BMOD setting and apply other settings*/
    vJPT_TxPowerAdjust(u8PowerAdj, u8Atten3db, u8Channel);
#endif

    /* Check for pending RX interrupts: if not, return RX to previous state */
    if (0 == (u32REG_BbcRead(REG_BBC_MISR)
              & (REG_BBC_INT_TX_MASK | REG_BBC_INT_RX_MASK)
             )
       )
    {
        vREG_BbcWrite(REG_BBC_RXCTL, u32RxCtlData);
#ifdef WIFICM_SUPPORT
        vREG_PhyWrite(REG_PHY_MCTRL, u32RxMctrlData);
#endif
#ifdef WIFICM_SUPPORT
        if (u32RxCtlData)
        {
            vJPT_WifiCmToRx();
        }
#endif
    }

    /* Restore interrupts */
    //MICRO_INT_RESTORE_STATE();
}
#endif

/* Miscellaneous */
PUBLIC uint32 u32MMAC_GetTime(void)
{
    return u32REG_BbcRead(REG_BBC_SCFRC);
}

PUBLIC void vMMAC_RadioOff(void)
{
#ifdef WIFICM_SUPPORT
    vJPT_WifiCmToOff();
#endif
    vREG_BbcWrite(REG_BBC_RXCTL, 0x0);
}

PUBLIC void vMMAC_SetCutOffTimer(uint32 u32CutOffTime, bool_t bEnable)
{
    /* Using temporary variable reduces code size, thanks to compiler needing
       a hint to not save static variable mid-process */
    uint8 u8TempVal = u8SctlMask;

    vREG_BbcWrite(REG_BBC_SCESL, u32CutOffTime);

    u8TempVal |= REG_BBC_SCTL_CE_MASK;
    if (FALSE == bEnable)
    {
        u8TempVal ^= REG_BBC_SCTL_CE_MASK;
    }

    u8SctlMask = u8TempVal;
}

PUBLIC void vMMAC_SynchroniseBackoffClock(bool_t bEnable)
{
    /* Using temporary variable reduces code size, thanks to compiler needing
       a hint to not save static variable mid-process */
    uint8 u8TempVal = u8SctlMask;

    u8TempVal |= REG_BBC_SCTL_SNAP_MASK;
    if (FALSE == bEnable)
    {
        u8TempVal ^= REG_BBC_SCTL_SNAP_MASK;
    }

    u8SctlMask = u8TempVal;
}

PUBLIC uint8 u8MMAC_EnergyDetect(uint32 u32DurationSymbols)
{
    /* Energy detect is performed synchronously */
    
    /* Reset energy detect accumulator */
    uint8  u8AccumulatedEnergy = 0;
    uint8  u8SampleEnergy;
    uint32 u32EndTime;

    /* Turn on PHY in RX mode */
    vREG_PhyWrite(REG_PHY_MCTRL, REG_PHY_MCTRL_MPHYON_MASK
                                 | REG_PHY_MCTRL_MIOM_MASK);

    /* Wait for correct state */
    while ((u32REG_PhyRead(REG_PHY_STAT) & REG_PHY_STAT_STATE_MASK)
           != REG_PHY_STAT_STATE_RX);

    /* Use SCFRC to time directly */
    u32EndTime = u32REG_BbcRead(REG_BBC_SCFRC) + u32DurationSymbols;

    while (((int32)(u32EndTime - u32REG_BbcRead(REG_BBC_SCFRC))) > 0)
    {
        /* Clear event status */
        vREG_PhyWrite(REG_PHY_IS, REG_PHY_INT_ED_MASK);

        /* Start energy detect */
        vREG_PhyWrite(REG_PHY_MCTRL, REG_PHY_MCTRL_MPHYON_MASK
                                     | REG_PHY_MCTRL_MIOM_MASK
                                     | REG_PHY_MCTRL_MEDT_MASK);

        /* Wait for completion */
        while (0 == (u32REG_PhyRead(REG_PHY_IS) & REG_PHY_INT_ED_MASK));

        /* Read value */
        u8SampleEnergy = u8MMAC_GetRxLqi(NULL);
        if (u8SampleEnergy > u8AccumulatedEnergy)
        {
            u8AccumulatedEnergy = u8SampleEnergy;
        }
    }

    /* Clear event status */
    vREG_PhyWrite(REG_PHY_IS, REG_PHY_INT_ED_MASK);
    
    /* Turn off PHY */
    vREG_PhyWrite(REG_PHY_MCTRL, 0);
    
    return u8AccumulatedEnergy;
}

/* Receive */
PUBLIC void vMMAC_SetRxAddress(uint32 u32PanId, uint16 u16Short, tsExtAddr *psMacAddr)
{
    vREG_BbcWrite(REG_BBC_RXMPID, u32PanId);
    vREG_BbcWrite(REG_BBC_RXMSAD, u16Short);
    vREG_BbcWrite(REG_BBC_RXMEADL, psMacAddr->u32L);
    vREG_BbcWrite(REG_BBC_RXMEADH, psMacAddr->u32H);
}

PUBLIC void vMMAC_SetRxStartTime(uint32 u32Time)
{
    vREG_BbcWrite(REG_BBC_RXETST, u32Time);
}

PUBLIC void vMMAC_StartMacReceive(tsMacFrame *psFrame, teRxOption eOptions)
{
    uint32 u32RxOptions = ((uint32)eOptions) & 0xff;
    uint32 u32RxConfig = (((uint32)eOptions) >> 8) & 0xff;

    /* Disable TX, just in case */
    vREG_BbcWrite(REG_BBC_TXCTL, 0x0);

    /* Ensure MAC mode is enabled, with any pre-configured settings */
    vREG_BbcWrite(REG_BBC_SCTL, u8SctlMask);

    /* Set RX buffer pointer */
    vREG_BbcWrite(REG_BBC_RXBUFAD, (uint32)psFrame);

    /* Start RX */
    vREG_BbcWrite(REG_BBC_RXPROM, u32RxConfig);
    vREG_BbcWrite(REG_BBC_RXCTL,  u32RxOptions);
#ifdef WIFICM_SUPPORT
    vJPT_WifiCmToRx();
#endif
}

PUBLIC void vMMAC_StartPhyReceive(tsPhyFrame *psFrame, teRxOption eOptions)
{
    uint32 u32RxOptions = ((uint32)eOptions) & 0xff;
    uint32 u32RxConfig = (((uint32)eOptions) >> 8) & 0xff;

    /* Disable TX, just in case */
    vREG_BbcWrite(REG_BBC_TXCTL, 0x0);

    /* Ensure PHY mode is enabled, with any pre-configured settings */
    vREG_BbcWrite(REG_BBC_SCTL, 0x20 | u8SctlMask);

    /* Set RX buffer pointer */
    vREG_BbcWrite(REG_BBC_RXBUFAD, (uint32)psFrame);

    /* Start RX */
    vREG_BbcWrite(REG_BBC_RXPROM, u32RxConfig);
    vREG_BbcWrite(REG_BBC_RXCTL,  u32RxOptions);
#ifdef WIFICM_SUPPORT
    vJPT_WifiCmToRx();
#endif
}

PUBLIC bool_t bMMAC_RxDetected(void)
{
    if (u32REG_BbcRead(REG_BBC_RXSTAT) & REG_BBC_RXSTAT_INPKT_MASK)
    {
        return TRUE;
    }

    return FALSE;
}

PUBLIC uint32 u32MMAC_GetRxErrors(void)
{
    return u32REG_BbcRead(REG_BBC_RXSTAT);
}

PUBLIC uint32 u32MMAC_GetRxTime(void)
{
    return u32REG_BbcRead(REG_BBC_RXTSTP);
}

PUBLIC uint8 u8MMAC_GetRxLqi(uint8 *pu8Msq)
{
    uint32 u32MStat = u32REG_PhyRead(REG_PHY_MSTAT0);
    uint8  u8Ed;
    
    if (pu8Msq)
    {
        *pu8Msq = (u32MStat >> REG_PHY_MSTAT_SQI_BIT) & BIT_W_8;
    }

    u8Ed = (u32MStat >> REG_PHY_MSTAT_ED_BIT) & BIT_W_8;

#if (defined JENNIC_CHIP_JN5169) || (defined JENNIC_CHIP_FAMILY_JN517x)
    /* Modify reported ED value to match JN5168 */
    if (u8Ed > 8)
    {
        u8Ed -= 8;
    }
    else
    {   
        u8Ed = 0;
    }
#endif

    return u8Ed;
}

/* Transmit */
PUBLIC void vMMAC_SetTxParameters(uint8 u8Attempts, uint8 u8MinBE,
                                  uint8 u8MaxBE, uint8 u8MaxBackoffs)
{
    vREG_BbcWrite(REG_BBC_TXRETRY, u8Attempts);
    vREG_BbcWrite(REG_BBC_TXMBEBT, u8MinBE
                                   | (u8MaxBackoffs << 4)
                                   | (((uint32)u8MaxBE) << 8));
}

PUBLIC void vMMAC_SetTxStartTime(uint32 u32Time)
{
    vREG_BbcWrite(REG_BBC_TXTSTP, u32Time);
}

PUBLIC void vMMAC_SetCcaMode(teCcaMode eCcaMode)
{
    uint32 u32Val;

    /* Store value directly into register */
    u32Val = u32REG_PhyRead(REG_PHY_MCCA);
    u32Val &= ~REG_PHY_MCCA_CCAM_MASK;
    u32Val |= ((uint32)eCcaMode) << REG_PHY_MCCA_CCAM_BIT;
    vREG_PhyWrite(REG_PHY_MCCA, u32Val);
}

PUBLIC void vMMAC_StartMacTransmit(tsMacFrame *psFrame, teTxOption eOptions)
{
    /* Disable RX and reset CSMA context, just in case */
#ifdef WIFICM_SUPPORT
    vJPT_WifiCmToOff();
#endif
    vREG_BbcWrite(REG_BBC_RXCTL, 0x0);
    vREG_BbcWrite(REG_BBC_TXCSMAC, 0x0);

    /* Ensure MAC mode is enabled, with any pre-configured settings */
    vREG_BbcWrite(REG_BBC_SCTL, u8SctlMask);

    /* Set TX buffer pointer */
    vREG_BbcWrite(REG_BBC_TXBUFAD, (uint32)psFrame);

    /* Start TX */
    vREG_BbcWrite(REG_BBC_TXCTL, (uint32)eOptions);
}

PUBLIC void vMMAC_StartPhyTransmit(tsPhyFrame *psFrame, teTxOption eOptions)
{
    /* Disable RX and reset CSMA context, just in case */
#ifdef WIFICM_SUPPORT
    vJPT_WifiCmToOff();
#endif
    vREG_BbcWrite(REG_BBC_RXCTL, 0x0);
    vREG_BbcWrite(REG_BBC_TXCSMAC, 0x0);

    /* Ensure PHY mode is enabled, with any pre-configured settings */
    vREG_BbcWrite(REG_BBC_SCTL, 0x20 | u8SctlMask);

    /* Set TX buffer pointer */
    vREG_BbcWrite(REG_BBC_TXBUFAD, (uint32)psFrame);

    /* Start TX */
    vREG_BbcWrite(REG_BBC_TXCTL, (uint32)eOptions);
}

PUBLIC uint32 u32MMAC_GetTxErrors(void)
{
    return u32REG_BbcRead(REG_BBC_TXSTAT);
}

PUBLIC void vMMAC_GetMacAddress(tsExtAddr *psMacAddr)
{
    /* Get MAC address from index sector. First check for MAC in customer
       area */
    psMacAddr->u32L = *(uint32 *)(MAC_ADDR_CUSTOMER + 4);
    psMacAddr->u32H = *(uint32 *)(MAC_ADDR_CUSTOMER);

   /* If customer MAC is blank, use default instead. Index sector is all
      1s if blank */
   if (   (psMacAddr->u32L == 0xffffffff)
       && (psMacAddr->u32H == 0xffffffff)
      )
   {
       psMacAddr->u32L = *(uint32 *)(MAC_ADDR_DEFAULT + 4);
       psMacAddr->u32H = *(uint32 *)(MAC_ADDR_DEFAULT);
   }
}

PUBLIC uint32 u32MMAC_PollInterruptSource(uint32 u32Mask)
{
    uint32 u32Isr;

    /* Read pending interrupt sources and apply mask */
    u32Isr = u32REG_BbcRead(REG_BBC_ISR) & u32Mask;

    /* Clear them */
    vREG_BbcWrite(REG_BBC_ISR, u32Isr);

    /* Return them */
    return u32Isr;
}

PUBLIC uint32 u32MMAC_PollInterruptSourceUntilFired(uint32 u32Mask)
{
    uint32 u32Isr;

    /* Read pending interrupt sources until masked value gives non-zero
       result (assumes interrupts are not enabled, otherwise the sources
       will be cleared automatically when generated) */
    do
    {
        u32Isr = u32REG_BbcRead(REG_BBC_ISR) & u32Mask;
    } while (u32Isr == 0);

    /* Clear them */
    vREG_BbcWrite(REG_BBC_ISR, u32Isr);

    /* Return them */
    return u32Isr;
}

PUBLIC void vMMAC_ConfigureInterruptSources(uint32 u32Mask)
{
    /* Enable TX and RX interrupts within BBC: allows them to wake CPU from
       doze, but not enabled enough to generate an interrupt (see
       vMMAC_EnableInterrupts for that) */
    vREG_BbcWrite(REG_BBC_IER, u32Mask);
}

PUBLIC void vUpdateRegisters(void)
{
    uint8  *pu8LookupAddr = (uint8 *)INDEX_ADDR(LOOKUP_PAGE, LOOKUP_START_WORD);
    uint8  *pu8EndAddr = (uint8 *)INDEX_ADDR(LOOKUP_PAGE, LOOKUP_END_WORD + 1);
    uint32  u32RegValue;
    uint32  u32RegAddr;
    uint32  u32NewValue;
    uint8   u8BitShift;

    do
    {
        /* Extract next address (bits 15:8 of AVP) */
        u32RegAddr = pu8LookupAddr[1];
        if (u32RegAddr & (1 << 7))
        {
            /* Reached end of list */
            pu8LookupAddr = pu8EndAddr;
        }
        else
        {
            /* Work out how many bits to shift the new byte value by */
            u8BitShift = (u32RegAddr & 0x3) * 8;

            /* Determine address (word aligned) and extract value (bits 7:0
               of AVP) */
            u32RegAddr = PHY_BASE_ADDR + (u32RegAddr & 0x7c);
            u32NewValue = pu8LookupAddr[0];

            /* Read current value of register */
            u32RegValue = *(volatile uint32 *)u32RegAddr;

            /* Mask out then insert new value based on byte offset within
               word */
            u32RegValue |= (0xffUL << u8BitShift);
            u32RegValue ^= (0xffUL << u8BitShift);
            u32RegValue |= (u32NewValue << u8BitShift);

            /* Write value into register */
            *(volatile uint32 *)u32RegAddr = u32RegValue;

            /* Move to next entry in list */
            pu8LookupAddr += 2;
        }
    } while (pu8LookupAddr < pu8EndAddr);
}

#if (defined JENNIC_CHIP_FAMILY_JN516x) && !(defined JENNIC_CHIP_JN5169)
#if !(defined JN5168_NO_CHANGE)
PUBLIC void vMMAC_SetHighPowerOptions(void)
{
    /* Module settings                  STD   M05   M06 */
    uint8 const au8CcaThresholds[3] = {  57,   68,   96};
    uint8 const au8PaAtten[3]       = {   0,    0,    1};
    uint8 const au8TxOffset[3]      = {0x7f, 0x7f, 0x7c};

    uint32 u32RegData;
    
    /* Set CCA threshold based on module type */
    u32RegData = u32REG_PhyRead(REG_PHY_MCCA);
    u32RegData &= 0xfffff00fUL;
    u32RegData |= ((uint32)(au8CcaThresholds[(uint8)eMMacModuleType])) << 4;
    vREG_PhyWrite(REG_PHY_MCCA, u32RegData);

    /* Set PA attenuation based on module type */
    u32RegData = u32REG_PhyRead(REG_PHY_PA_CTRL);
    u32RegData &= 0xffffff8fUL;
    u32RegData |= ((uint32)(au8PaAtten[(uint8)eMMacModuleType])) << 4;
    vREG_PhyWrite(REG_PHY_PA_CTRL, u32RegData);

    /* Change IDLE to TX offset based on module type */
    vREG_PhyWrite(REG_PHY_VCO_TXO, au8TxOffset[(uint8)eMMacModuleType]);
}
#endif
#endif

#ifdef INCLUDE_ECB_API
/* Security */
PUBLIC void MMAC_EcbWriteKey(uint32 au32KeyData[4])
{
    /* Wait until idle */
    vAesCmdWaitBusy();

    /* Write the new key */
    vREG_AesWrite(REG_AES_DATA_IN + 0, au32KeyData[0]);
    vREG_AesWrite(REG_AES_DATA_IN + 1, au32KeyData[1]);
    vREG_AesWrite(REG_AES_DATA_IN + 2, au32KeyData[2]);
    vREG_AesWrite(REG_AES_DATA_IN + 3, au32KeyData[3]);

    /* Issue command */
    vREG_AesWrite(REG_AES_ACL_CMD, REG_AES_ACL_CMD_SET_KEY);
}

/****************************************************************************
 *
 * NAME:       vMMAC_EcbEncodeStripe
 *
 * DESCRIPTION:
 * Encodes a 128-bit data stripe using the AES Coprocessor. The input buffers MUST
 * be multiples of 128-bits. The function, upon return indicates how many stripes
 * in the input buffer have been decoded. In cases where the software loses context
 * on the AES Coprocessor, this allows the the process to continue where it left off
 * once it manages to regain context.
 *
 * PARAMETERS:      Name                RW  Usage
 *                  pau8inputData       R   pointer to user allocated input data buffer
 *                  pau8outputData      W   pointer to user allocated output data buffer
 *
 ****************************************************************************/
PUBLIC void vMMAC_EcbEncodeStripe(
        uint32 au32InputData[4],
        uint32 au32OutputData[4])
{
    /* Wait */
    vAesCmdWaitBusy();

    /* Pass in data */
    vREG_AesWrite(REG_AES_DATA_IN + 0, au32InputData[0]);
    vREG_AesWrite(REG_AES_DATA_IN + 1, au32InputData[1]);
    vREG_AesWrite(REG_AES_DATA_IN + 2, au32InputData[2]);
    vREG_AesWrite(REG_AES_DATA_IN + 3, au32InputData[3]);

    /* Issue command */
    vREG_AesWrite(REG_AES_ACL_CMD, REG_AES_ACL_CMD_GO);

    /* Blocking wait for encode to complete */
    vAesCmdWaitBusy();

    /* copy data into the user supplied buffer */
    au32OutputData[0] = u32REG_AesRead(REG_AES_DATA_OUT + 0);
    au32OutputData[1] = u32REG_AesRead(REG_AES_DATA_OUT + 1);
    au32OutputData[2] = u32REG_AesRead(REG_AES_DATA_OUT + 2);
    au32OutputData[3] = u32REG_AesRead(REG_AES_DATA_OUT + 3);
}
#endif

/****************************************************************************/
/***        Private Functions                                             ***/
/****************************************************************************/
PUBLIC void vMMAC_IntHandlerBbc(void)
{
    uint32 u32Isr;

    /* Read enabled interrupts */
    u32Isr = u32REG_BbcRead(REG_BBC_MISR);

    /* Clear them */
    vREG_BbcWrite(REG_BBC_ISR, u32Isr);

    /* Pass result to callback, if registered */
    if (prIntHandler)
    {
        prIntHandler(u32Isr);
    }
}

#if !(defined JN5168_NO_CHANGE)
PRIVATE uint8 u8GetPapValue(int8 i8TxPower)
{
#if ((defined JENNIC_CHIP_FAMILY_JN514x) || (defined JENNIC_CHIP_FAMILY_JN516x)) && !(defined JENNIC_CHIP_JN5169)
    uint8 u8TxPower;

    /* Obtain value to store in PAP register.
       Possible PAP values are  meaning (dBm)
                  3              0
                  2             -9
                  1             -20
                  0             -32
    */

    /* If value is positive we truncate it to 0dBm by setting the value to 64.
       Hence we end up with values from 64 down to 32 to indicate 0 to -32 */
    if (i8TxPower >= 0)
    {
        u8TxPower = 64;
    }
    else
    {
        u8TxPower = (uint8)((int8)64 + i8TxPower);
    }

    /* Now map the requested TX power to the available values.
       Doing the sums:
         u8TxPower  meaning (dBm)  gives u8PapValue so (dBm)
             64         0                    3           0
             63-52    -1 to -12              2          -9
             51-40   -13 to -24              1         -20
             39-32   -25 to -32              0         -32

       This is actually outside tolerance for -1, -2, -13 and -25 dBm!
       Subtracting 26 or 25 instead of 28 gives the best fit and within tolerance
     */
    return (uint8)(((uint32)(u8TxPower - 28)) / (uint32)12);
#else
    /* For JN5169 onwards, we have 6 power levels from -32 to +10 dBm.
          Possible PAP | meaning
           values are  |  (dBm)
          -------------+---------
                5      |   8.5
                4      |   3.7
                3      |  -7.2
                2      | -18.3
                1      | -29.4
                0      | -32.2

       There are also two adjustments that can be made:
         increments of 0, +0.8dB, +1.2dB and +1.6dB
         attenuation of 2.5dB

       To map all of this from the requested dBm level we're going to use a
       table */
#pragma GCC diagnostic ignored "-Wpacked"
#pragma GCC diagnostic ignored "-Wattributes"
    static struct __attribute__ ((__packed__))
    {
        unsigned int iPapValue      : 3;
        unsigned int iTXPowerAdjust : 2;
        unsigned int iAtten3db      : 1;
        unsigned int iPad           : 2;
    } const asPowerSettings[] = { 
                            {5, 3, 0}, /* +10dBm */
                            {5, 1, 0},
                            {5, 0, 0},
                            {5, 2, 1},
                            {5, 0, 1},
                            {4, 3, 0},
                            {4, 0, 0},
                            {4, 3, 1},
                            {4, 1, 1},
                            {4, 0, 1},
                            {4, 0, 1}, /*   0dBm */
                            {4, 0, 1},
                            {4, 0, 1},
                            {3, 3, 0},
                            {3, 3, 0},
                            {3, 3, 0},
                            {3, 3, 0},
                            {3, 0, 0},
                            {3, 3, 1},
                            {3, 2, 1},
                            {3, 0, 1}, /* -10dBm */
                            {3, 0, 1},
                            {3, 0, 1},
                            {3, 0, 1},
                            {2, 3, 0},
                            {2, 3, 0},
                            {2, 3, 0},
                            {2, 2, 0},
                            {2, 0, 0},
                            {2, 3, 1},
                            {2, 2, 1}, /* -20dBm */
                            {2, 1, 1},
                            {2, 0, 1},
                            {2, 0, 1},
                            {2, 0, 1},
                            {1, 3, 0},
                            {1, 3, 0},
                            {1, 3, 0},
                            {1, 2, 0},
                            {1, 0, 0},
                            {1, 0, 0}, /* -30dBm */
                            {1, 3, 1},
                            {0, 0, 0}};
#pragma GCC diagnostic pop

    uint8 u8Row;

    /* Element 0 of array relates to +10dBm, element 1 is +9dBm, etc., all
       the way down to -32dBm */
    if (i8TxPower > 10)
    {
        i8TxPower = 10;
    }
    if (i8TxPower < -32)
    {
        i8TxPower = -32;
    }

    u8Row = (uint8)((int8)10 - i8TxPower);

    /* Store settings for use by vJPT_TxPowerAdjust */
    u8PowerAdj = (uint8)asPowerSettings[u8Row].iTXPowerAdjust;
    u8Atten3db = (uint8)asPowerSettings[u8Row].iAtten3db;

    return (uint8)asPowerSettings[u8Row].iPapValue;
#endif
}
#endif

#ifdef INCLUDE_ECB_API
/****************************************************************************
 *
 * NAME:       vAesCmdWaitBusy
 *
 * DESCRIPTION:
 * waits until the cmd is complete. Blocking function, polls the wishbone
 * until not busy.
 *
 ****************************************************************************/
PRIVATE void vAesCmdWaitBusy(void)
{
    uint32 u32CmdDataReturn;

    do
    {
        u32CmdDataReturn = u32REG_AesRead(REG_AES_ACL_CMD);
    } while ((u32CmdDataReturn & REG_AES_ACL_CMD_DONE_MASK) == 0);
}
#endif

/****************************************************************************/
/***        End of file                                                   ***/
/****************************************************************************/
