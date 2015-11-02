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

extern PUBLIC void vUpdateRegisters(void);

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
/* PHY Controller Register Addresses */
enum
{
    SYS_PWR_CTRL        = 0x40000000,
    TST_3V              = 0x400000c8,
    PHY_CTRL            = 0x40001e00,
    PHY_STAT            = 0x40001e04,
    PHY_VCO_FTO         = 0x40001e24,
    PHY_PA_CTRL         = 0x40001e0c,
    PHY_VCO_CAL_CTRL    = 0x40001e14,
    PHY_MRX_CTRL        = 0x40001e3c,
    PHY_RC_CAL          = 0x40001e54,
    PHY_RC_CALO         = 0x40001e58,
    PHY_MCCA            = 0x40001e38,
    PHY_TST0            = 0x40001e70,
    PHY_TST2            = 0x40001e78,
    PHY_TST3            = 0x40001e7c,
    PHY_VCO_TXO         = 0x40001e28,
    PHY_VCO_RXO         = 0x40001e2c,
    PHY_VCO_CHG         = 0x40001e1c,
    PHY_SYNTH_CTRL      = 0x40001e60,
    PHY_AGC_TH          = 0x40001eb0,
    PHY_INJ_SEL         = 0x40001eac,
    PHY_VCO_CTRL        = 0x40001ebc,
    PHY_VCO_AUTO_CAL    = 0x40001e10,
    DCDC_CFG_2          = 0x400001f4,
    BBC_PTTT            = 0x40001448
}eRegAddr;

#define ZedRead(A,S)    *(volatile uint32 *)(A);
#define ZedWrite(A,B,S) *(volatile uint32 *)(A) = (B);
#define ZedPoll(A,S,B,C)                             \
    while (((*(volatile uint32 *)(A)) & (B)) != (C)) \
    {                                                \
        /*vAHI_CpuDoze();*/                              \
    }

#define ZedPrint(A)

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/

/****************************************************************************/
/***        Public Functions                                              ***/
/****************************************************************************/
PUBLIC void vMMAC_ConfigureRadio(void)
{
    volatile uint32 u32VAL;

    /***** Ensure BBC & Phy Stay On bit are Cleared ****/
    vREG_BbcWrite(REG_BBC_SCTL, u32REG_BbcRead(REG_BBC_SCTL) & 0xfffffff7);
    vREG_PhyWrite(REG_PHY_CTRL, u32REG_PhyRead(REG_PHY_CTRL) & 0xfffffffe);

    //GNATS 653 and OAD 2nd stage boot trac 65
    /***** Ensure TX & RX are Cleared ****/
    vREG_BbcWrite(REG_BBC_TXCTL,0);
    vREG_BbcWrite(REG_BBC_RXCTL,0);

    while ((u32REG_PhyRead(REG_PHY_STAT) & 0xf) != 0);

#ifndef FPGA_BUILD
//    vREG_PhyWrite(REG_PHY_IE, XCV_REG_PHY_IS_VCO_CAL_MASK);
#endif

    /* Following code is taken from Python test settings script. Following
       changes are necessary:
       1. Script enables protocol power at start: we can remove this
       2. Need to add call to vUpdateRegisters just before turning the radio
          on
       3. Need to add register write to clear PHY VCO_CAL_MASK after polling
          for calibration complete
       4. Change # to //, of course
       5. Remove '[1]' from register accesses
     */

    //########################################################
    //#######          DCDC settings begin         ###########
    //########################################################

    // Bit Field SOC_EN Value = 1
    u32VAL = ZedRead(DCDC_CFG_2,32)
    ZedWrite(DCDC_CFG_2,(u32VAL | 0x1),32)

    // Bit Field SETVP Value = 8
    u32VAL = ZedRead(DCDC_CFG_2,32)
    ZedWrite(DCDC_CFG_2,(u32VAL | 0x00008000),32)
    u32VAL = ZedRead(DCDC_CFG_2,32)
    ZedWrite(DCDC_CFG_2,(u32VAL & 0xffff8fff),32)

    //########################################################
    //#######           DCDC settings end          ###########
    //########################################################

    // Disable PAP ramp-up and Max output power ~9dBm
    // PHY_PA_CTRL: Read-modify write to set bit [11] (DIS_PAP_RAMP_UP=1 PAP=5)
    ZedWrite(PHY_PA_CTRL,0x5800,32)

    // Default value of the XTAL oscillator bias current after started
    // TST_XTAL_BIAS_3V=0 (253uA)
    // Set XTAL internal resistance
    // XTAL_RTRIM=1 (21kOhms)
    u32VAL = ZedRead(TST_3V,32)
    ZedWrite(TST_3V,((u32VAL & 0xfffffff8) | 0x2000),32)

    // Set R_OFFSET to zero
    // PHY_RC_CALO: Write zero to clear bits [7:4] are R_OFFSET=0 (remaining bits also zero)
    ZedWrite(PHY_RC_CALO,0x0,32)

    // Pre-enable RC (not /R/C) calibraiton and set RC_TRIM=6, R_TRIM=4 and C_TRIM=4
    // PHY_RC_CAL: Write to set bits [2:0] (RC_CALE=1, R_CALE=1, C_CALE=0)
    // PHY_RC_CAL: Write to set bits [26:24]=6, [22:20]=4, [18:16]=4 (RC_TRIM=6, R_TRIM=4, C_TRIM=4)
    // PHY_RC_CAL: Write to clear remaining bits (ok to do this before calibration)
    // ZedWrite(PHY_RC_CAL,0x06440007,32)
    ZedWrite(PHY_RC_CAL,0x06440006,32)

    // Change Idle to TX mode code offset
    // PHY_VCO_TXO:VCO_TXO=+2
    ZedWrite(PHY_VCO_TXO,0x2,32)

    // Change Idle to RX mode code offset
    // PHY_VCO_RXO:VCO_RXO=+4
    ZedWrite(PHY_VCO_RXO,0x4,32)

    // Change Idle/RX/TXmode mode channel gradient ~1 code per channel(5MHz)
    // PHY_VCO_CHG:VCO_CHG_RX=1,VCO_CHG_TX=1
    ZedWrite(PHY_VCO_CHG,0x1351,32)

    // Change IVCO to 0 (i.e. 2.4 mA) to calibrate
    // PHY_VCO_AUTO_CAL:VCOCAL_IVCO=0)
    ZedWrite(PHY_VCO_AUTO_CAL,0x00900710,32)

    // Pre-enable VCO calibration
    // PHY_VCO_CAL_CTRL: Read-modify write to set bit [0] (VCOCAL_CAL_ST=1)
    u32VAL = ZedRead(PHY_VCO_CAL_CTRL,32)
    ZedWrite(PHY_VCO_CAL_CTRL,(u32VAL | 0x1),32)

    // PHY_TST0: TST_SYNTH_DIV_BOOST=1 TST_IVCO=1 (1.2mA), TST_BVCM=3 (+0.1V), TST_BREF=3 (+0.1V), TST_BMOD=0x12 (0.2V*18/127=28.34mV, Kmod~=17.67MHz/V) TST_BSELREF=1 (VB_VCO ref) TST_SYNTH_DIV_BOOST=1 TST_SD_SEL_SHORT_DITHER=1
    ZedWrite(PHY_TST0,0x0406c911,32)

    // PHY_TST3:TST_127=1
    u32VAL = ZedRead(PHY_TST3,32)
    ZedWrite(PHY_TST3,(u32VAL | 0x80000000),32)

    // Disable dithering
    // SD_EN_DITHER=0
    // u32VAL = ZedRead(PHY_SYNTH_CTRL,32)[1]
    // ZedWrite(PHY_SYNTH_CTRL,(u32VAL & 0xffdfffff),32)

    // PHY_AGC_TH: Med2Low and Low2Med Thresholds decreased respectively by 4 dB and 3 dB compared to default thresholds
    ZedWrite(PHY_AGC_TH, 0x14cc6b5a,32)

    // PHY_INJ_SEL: All Rx channels in low side injection except channel 17 (still high side)
    ZedWrite(PHY_INJ_SEL, 0x40,32)

    // PHY_VCO_CTRL: TX_IVCO_MD=1, TX_IVCO=0 (i.e. 2.4 mA), RX_IVCO=0 (i.e. 2.4 mA)
    ZedWrite(PHY_VCO_CTRL,0x0,32)

    // Set delay PA for best Burst emission
    ZedWrite(PHY_TST2, 0x6008,32)

    // Change BBC timings to allow for TX fine tune algorithm and PA turn on
    ZedWrite(BBC_PTTT, 0x00001c28, 32)

    vUpdateRegisters();

    // Turn-on Radio by setting STAY_ON bit: VCO and RC/R/C calibrations will be performed
    // PHY_CTRL: Read-modify write to set bit 0 (STAY_ON=1)
    u32VAL = ZedRead(PHY_CTRL,32)
    ZedWrite(PHY_CTRL,(u32VAL | 0x1),32)

    // Wait for calibrations to complete by detecting PHY state in IDLE
    // Calibration duration ~ VRSP(16u)+VCO_SETTLE(16u)+SSP(32u)+Caltime(124u) = 188us
    // PHY_STAT: Poll for IDLE state (bits[3:0] = 0x3)
    ZedPoll(PHY_STAT,32,0xf,0x3)

    vREG_PhyWrite(REG_PHY_IS, XCV_REG_PHY_IS_VCO_CAL_MASK);

    // Radio should now be calibrated - turn-off Radio by clearing STAY_ON bit
    // PHY_CTRL: Read-modify write to clear bit 0 (STAY_ON=0)
    u32VAL = ZedRead(PHY_CTRL,32)
    ZedWrite(PHY_CTRL,(u32VAL & 0xfffffffe),32)

    // Set CCA_ED_THR (bits 11:4) in PHY_MCCA to 0x39 (decimal 57)
    //#u32VAL = ZedRead(PHY_MCCA,32)
    //#ZedWrite(PHY_MCCA,((u32VAL & 0xfffff00f) | 0x00000390),32)

    // For v0.4 remove this mode due to issues seen with false reseting during a packet
    //   // Enable AGC Reset in Packet Mode (set bit 16 of PHY_MRX_CTRL)
    //   u32VAL = ZedRead(PHY_MRX_CTRL,32)
    //   ZedWrite(PHY_MRX_CTRL,(u32VAL | 0x10000),32)

    // Over write Ccal
    // PHY_RC_CAL: Set Ccal=4
    // u32VAL = ZedRead(PHY_RC_CAL,32)
    // ZedWrite(PHY_RC_CAL,(u32VAL | 0x40),32)

    ZedPrint('')
    ZedPrint('')
    ZedPrint('')
    ZedPrint('JN5172 defaults V0.8 just ran,')
    ZedPrint('Rx iVCO = 2.4 mA & Tx iVCO = 2.4 mA')
    ZedPrint('PAP=5 max power')
    ZedPrint('TST127=1')
    ZedPrint('M2L_TH=default-4dB & L2M_TH=default-3dB')
    ZedPrint('All channels in LSI except 17 in HSI')
    ZedPrint('Default value of the XTAL oscillator bias current after started (253uA)')
    ZedPrint('TST_SD_SEL_SHORT_DITHER=1')
    ZedPrint('set XTAL internal resistance at 21kOhms')
    ZedPrint('DCDC output to 2.8Volts')
    ZedPrint('')
    ZedPrint('')
    ZedPrint('')
}

/****************************************************************************/
/***        Private Functions                                             ***/
/****************************************************************************/

/****************************************************************************/
/***        End of file                                                   ***/
/****************************************************************************/
