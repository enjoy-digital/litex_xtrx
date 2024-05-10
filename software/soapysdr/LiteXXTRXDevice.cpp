//
// SoapySDR driver for the LiteX-based alternative gateware for the Fairwaves/Lime XTRX.
//
// Copyright (c) 2021-2024 Enjoy Digital.
// Copyright (c) 2021 Julia Computing.
// Copyright (c) 2015-2015 Fairwaves, Inc.
// Copyright (c) 2015-2015 Rice University
// SPDX-License-Identifier: Apache-2.0
// http://www.apache.org/licenses/LICENSE-2.0
//

#include <fstream>
#include <memory>
#include <sys/mman.h>

#include "LiteXXTRXDevice.hpp"

#include <lms7002mNG/IComms.h>
#include <lms7002mNG/OpStatus.h>
#include <lms7002mNG/LMS7002M_parameters.h>

#include <SoapySDR/Registry.hpp>
#include <SoapySDR/Logger.hpp>

// TODO define    : What the issue/limitation?
// FIXME define   : What the issue/limitation?
// FORNEXT define : What the issue/limitation?

/***************************************************************************************************
 *                                     Default Configuration
 **************************************************************************************************/

const std::vector<std::pair<uint16_t, uint16_t>> xtrx_default_cfg = {
    /* LMS7002M generic */

    // Register 0x0023 - **LML1_LML2_MODE**
    {0x0023, 0x5542},  // 0b0101010101000010
    // [15]   DIQDIRCTR2    : DIQ2 direction control mode (0: Automatic, 1: Manual) = 0
    // [14]   DIQDIR2       : DIQ2 direction (0: Output, 1: Input) = 1
    // [13]   DIQDIRCTR1    : DIQ1 direction control mode (0: Automatic, 1: Manual) = 0
    // [12]   DIQDIR1       : DIQ1 direction (0: Output, 1: Input) = 1
    // [11]   ENABLEDIRCTR2 : ENABLE2 direction control mode (0: Automatic, 1: Manual) = 0
    // [10]   ENABLEDIR2    : ENABLE2 direction (0: Output, 1: Input) = 1
    // [9]    ENABLEDIRCTR1 : ENABLE1 direction control mode (0: Automatic, 1: Manual) = 0
    // [8]    ENABLEDIR1    : ENABLE1 direction (0: Output, 1: Input) = 1
    // [7]    Reserved
    // [6]    MOD_EN        : LimeLightTM interface enable (0: Disabled, 1: Enabled) = 1
    // [5]    LML2_FIDM     : Frame start ID selection for Port 2 (0: Frame start when 0, 1: Frame start when 1) = 0
    // [4]    LML2_RXNTXIQ  : TXIQ/RXIQ mode selection for Port 2 (0: BB2RF, 1: RF2BB) = 1
    // [3]    LML2_MODE     : Mode of LimeLightTM Port 2 (0: TRXIQ, 1: JESD207) = 0
    // [2]    LML1_FIDM     : Frame start ID selection for Port 1 (0: Frame start when 0, 1: Frame start when 1) = 1
    // [1]    LML1_RXNTXIQ  : TXIQ/RXIQ mode selection for Port 1 (0: BB2RF, 1: RF2BB) = 0
    // [0]    LML1_MODE1    : Mode of LimeLightTM Port 1 (0: TRXIQ, 1: JESD207) = 1

    // Register 0x002a - **LML_CLK_CTRL**
    {0x002a, 0x0192},  // 0b0000000110010010
    // [15:14] FCLK2_DLY   : FCLK2 clock internal delay (00: No delay, 01: 1x delay, 10: 2x delay, 11: 3x delay) = 00
    // [13:12] FCLK1_DLY   : FCLK1 clock internal delay (00: No delay, 01: 1x delay, 10: 2x delay, 11: 3x delay) = 00
    // [11:10] RX_MUX      : RxFIFO data source selection (00: RxTSPCLK, 01: TxFIFO, 10/11: LFSR) = 00
    // [9:8]   TX_MUX      : Port selection for data transmit to TSP (00: Port 1, 01: Port 2, 10/11: RxTSP) = 10
    // [7:6]   TXRDCLK_MUX : TX FIFO read clock selection (10/11: TxTSPCLK, 01: FCLK2, 00: FCLK1) = 01
    // [5:4]   TXWRCLK_MUX : TX FIFO write clock selection (10/11: RxTSPCLK, 01: FCLK2, 00: FCLK1) = 00
    // [3:2]   RXRDCLK_MUX : RX FIFO read clock selection (11: FCLK2, 10: FCLK1, 01: MCLK2, 00: MCLK1) = 00
    // [1:0]   RXWRCLK_MUX : RX FIFO write clock selection (10/11: RxTSPCLK, 01: FCLK2, 00: FCLK1) = 10

    // Register 0x002b - **LML_SRC**
    {0x002b, 0x002c},  // 0b0000001000111100
    // [15]   FCLK2_INV  : FCLK2 clock inversion (0: Not inverted, 1: Inverted) = 0
    // [14]   FCLK1_INV  : FCLK1 clock inversion (0: Not inverted, 1: Inverted) = 0
    // [13:12] MCLK2_DLY : MCLK2 clock internal delay (00: No delay, 01: 1x delay, 10: 2x delay, 11: 3x delay) = 01
    // [11:10] MCLK1_DLY : MCLK1 clock internal delay (00: No delay, 01: 1x delay, 10: 2x delay, 11: 3x delay) = 01
    // [9]    MCLK2_INV  : MCLK2 clock inversion (0: Not inverted, 1: Inverted) = 0
    // [8]    MCLK1_INV  : MCLK1 clock inversion (0: Not inverted, 1: Inverted) = 0
    // [7:6]  Reserved
    // [5:4]  MCLK2_SRC  : MCLK2 clock source (00: TxTSPCLKA after divider, 01: RxTSPCLKA after divider, 10: TxTSPCLKA, 11: RxTSPCLKA) = 01
    // [3:2]  MCLK1_SRC  : MCLK1 clock source (00: TxTSPCLKA after divider, 01: RxTSPCLKA after divider, 10: TxTSPCLKA, 11: RxTSPCLKA) = 10
    // [1]    TXDIVEN    : TX clock divider enable (0: Divider disabled, 1: Divider enabled) = 1
    // [0]    RXDIVEN    : RX clock divider enable (0: Divider disabled, 1: Divider enabled) = 1

    // Register 0x002c - **LML_CLK_DIV**
    {0x002c, 0xffff},  // 0b1111111111111111
    // [15:8]  TXTSPCLKA_DIV : TxTSP clock divider (unsigned integer, 0-255, default is 255) = 255
    // [7:0]   RXTSPCLKA_DIV : RxTSP clock divider (unsigned integer, 0-255, default is 255) = 255

    // Register 0x00ad - **CDS**
    {0x00ad, 0x03f3},  // 0b0000001111110011
    // [15:14] CDS_MCLK2   : MCLK2 clock delay (00: 400ps, 01: 500ps, 10: 600ps, 11: 700ps) = 00
    // [13:12] CDS_MCLK1   : MCLK1 clock delay (00: 400ps, 01: 500ps, 10: 600ps, 11: 700ps) = 00
    // [11:10] Reserved
    // [9]     CDSN_TXBTSP : TX TSPB clock inversion control (0: Clock inverted, 1: Not inverted) = 1
    // [8]     CDSN_TXATSP : TX TSPA clock inversion control (0: Clock inverted, 1: Not inverted) = 1
    // [7]     CDSN_RXBTSP : RX TSPB clock inversion control (0: Clock inverted, 1: Not inverted) = 1
    // [6]     CDSN_RXATSP : RX TSPA clock inversion control (0: Clock inverted, 1: Not inverted) = 1
    // [5]     CDSN_TXBLML : TX LMLB clock inversion control (0: Clock inverted, 1: Not inverted) = 1
    // [4]     CDSN_TXALML : TX LMLA clock inversion control (0: Clock inverted, 1: Not inverted) = 1
    // [3]     CDSN_RXBLML : RX LMLB clock inversion control (0: Clock inverted, 1: Not inverted) = 1
    // [2]     CDSN_RXALML : RX LMLA clock inversion control (0: Clock inverted, 1: Not inverted) = 1
    // [1]     CDSN_MCLK2  : MCLK2 clock inversion control (0: Clock inverted, 1: Not inverted) = 1
    // [0]     CDSN_MCLK1  : MCLK1 clock inversion control (0: Clock inverted, 1: Not inverted) = 1

    // Register 0x0092 - **LDO_CTRL1**
    {0x0092, 0x0D15},  // 0b0000110100010101
    // [15]  EN_LDO_DIG    : Enable LDO (0: Powered down, 1: Enabled) = 0
    // [14]  EN_LDO_DIGGN  : Enable LDO (0: Powered down, 1: Enabled) = 0
    // [13]  EN_LDO_DIGSXR : Enable LDO (0: Powered down, 1: Enabled) = 1
    // [12]  EN_LDO_DIGSXT : Enable LDO (0: Powered down, 1: Enabled) = 1
    // [11]  EN_LDO_DIVGN  : Enable LDO (0: Powered down, 1: Enabled) = 1
    // [10]  EN_LDO_DIVSXR : Enable LDO (0: Powered down, 1: Enabled) = 1
    // [9]   EN_LDO_DIVSXT : Enable LDO (0: Powered down, 1: Enabled) = 0
    // [8]   EN_LDO_LNA12  : Enable LDO (0: Powered down, 1: Enabled) = 0
    // [7]   EN_LDO_LNA14  : Enable LDO (0: Powered down, 1: Enabled) = 0
    // [6]   EN_LDO_MXRFE  : Enable LDO (0: Powered down, 1: Enabled) = 1
    // [5]   EN_LDO_RBB    : Enable LDO (0: Powered down, 1: Enabled) = 1
    // [4]   EN_LDO_RXBUF  : Enable LDO (0: Powered down, 1: Enabled) = 1
    // [3]   EN_LDO_TBB    : Enable LDO (0: Powered down, 1: Enabled) = 0
    // [2]   EN_LDO_TIA12  : Enable LDO (0: Powered down, 1: Enabled) = 1
    // [1]   EN_LDO_TIA14  : Enable LDO (0: Powered down, 1: Enabled) = 0
    // [0]   EN_G_LDO      : Enable control for all LDO power downs (0: All powered down, 1: Controlled by individual registers) = 1

    // Register 0x0093 - **LDO_CTRL2**
    {0x0093, 0x01B1},  // 0b0000000110110001
    // [15]  EN_LOADIMP_LDO_TLOB   : Enable load-dependent bias (0: Constant bias, 1: Load dependent) = 0
    // [14]  EN_LOADIMP_LDO_TPAD   : Enable load-dependent bias (0: Constant bias, 1: Load dependent) = 0
    // [13]  EN_LOADIMP_LDO_TXBUF  : Enable load-dependent bias (0: Constant bias, 1: Load dependent) = 0
    // [12]  EN_LOADIMP_LDO_VCOGN  : Enable load-dependent bias (0: Constant bias, 1: Load dependent) = 0
    // [11]  EN_LOADIMP_LDO_VCOSXR : Enable load-dependent bias (0: Constant bias, 1: Load dependent) = 0
    // [10]  EN_LOADIMP_LDO_VCOSXT : Enable load-dependent bias (0: Constant bias, 1: Load dependent) = 1
    // [9]   EN_LDO_AFE            : Enable LDO (0: Powered down, 1: Enabled) = 1
    // [8]   EN_LDO_CPGN           : Enable LDO (0: Powered down, 1: Enabled) = 0
    // [7]   EN_LDO_CPSXR          : Enable LDO (0: Powered down, 1: Enabled) = 1
    // [6]   EN_LDO_TLOB           : Enable LDO (0: Powered down, 1: Enabled) = 1
    // [5]   EN_LDO_TPAD           : Enable LDO (0: Powered down, 1: Enabled) = 1
    // [4]   EN_LDO_TXBUF          : Enable LDO (0: Powered down, 1: Enabled) = 1
    // [3]   EN_LDO_VCOGN          : Enable LDO (0: Powered down, 1: Enabled) = 0
    // [2]   EN_LDO_VCOSXR         : Enable LDO (0: Powered down, 1: Enabled) = 1
    // [1]   EN_LDO_VCOSXT         : Enable LDO (0: Powered down, 1: Enabled) = 1
    // [0]   EN_LDO_CPSXT          : Enable LDO (0: Powered down, 1: Enabled) = 1

    // Register 0x00A6 - **LDO_MISC_CTRL**
    {0x00A6, 0x000F},  // 0b0000000000001111
    // [15:13] ISINK_SPIBUFF         : SPIBUF LDO output resistive load (0: Off, 1: 10kΩ, ..., 7: 476Ω) = 000
    // [12]    SPDUP_LDO_SPIBUF      : Speed up settling time for LDO (0: Resistor in place, 1: Bypass resistor) = 0
    // [11]    SPDUP_LDO_DIGIp2      : Speed up settling time for LDO (0: Resistor in place, 1: Bypass resistor) = 0
    // [10]    SPDUP_LDO_DIGIp1      : Speed up settling time for LDO (0: Resistor in place, 1: Bypass resistor) = 0
    // [9]     BYP_LDO_SPIBUF        : Bypass signal for LDO (0: Normal operation, 1: Bypass LDO) = 0
    // [8]     BYP_LDO_DIGIp2        : Bypass signal for LDO (0: Normal operation, 1: Bypass LDO) = 0
    // [7]     BYP_LDO_DIGIp1        : Bypass signal for LDO (0: Normal operation, 1: Bypass LDO) = 0
    // [6]     EN_LOADIMP_LDO_SPIBUF : Enable load-dependent bias (0: Constant bias, 1: Load dependent) = 0
    // [5]     EN_LOADIMP_LDO_DIGIp2 : Enable load-dependent bias (0: Constant bias, 1: Load dependent) = 0
    // [4]     EN_LOADIMP_LDO_DIGIp1 : Enable load-dependent bias (0: Constant bias, 1: Load dependent) = 0
    // [3]     PD_LDO_SPIBUF         : Power down signal for LDO (0: Active, 1: Powered down) = 0
    // [2]     PD_LDO_DIGIp2         : Power down signal for LDO (0: Active, 1: Powered down) = 0
    // [1]     PD_LDO_DIGIp1         : Power down signal for LDO (0: Active, 1: Powered down) = 0
    // [0]     EN_G_LDOP             : Enable control for all LDO power downs (0: All powered down, 1: Controlled individually) = 1

    /* XTRX specific */

    // Register 0x0085 - **XBUF_CTRL**
    {0x0085, 0x0019},  // 0b0000000000011001
    // [8] SLFB_XBUF_RX      : Self biasing digital control (0: Disabled, 1: Enabled) = 0
    // [7] SLFB_XBUF_TX      : Self biasing digital control (0: Disabled, 1: Enabled) = 0
    // [6] BYP_XBUF_RX       : Bypass input buffer (0: Not active, 1: Active) = 0
    // [5] BYP_XBUF_TX       : Bypass input buffer (0: Not active, 1: Active) = 0
    // [4] EN_OUT2_XBUF_TX   : Enable 2nd output of TX XBUF (0: Active, 1: Disabled) = 1
    // [3] EN_TBUFIN_XBUF_RX : RX XBUF input from TX (0: From external XOSC, 1: From TX) = 1
    // [2] PD_XBUF_RX        : Power down RX XBUF (0: Active, 1: Powered down) = 0
    // [1] PD_XBUF_TX        : Power down TX XBUF (0: Active, 1: Powered down) = 0
    // [0] EN_G_XBUF         : Enable control for all XBUF power downs (0: All powered down, 1: Controlled individually) = 1
};

/***************************************************************************************************
 *                                     SPI API
 **************************************************************************************************/

//#define LITEPCIE_SPI_DEBUG
#define LITEPCIE_SPI_CS_HIGH (0 << 0)
#define LITEPCIE_SPI_CS_LOW  (1 << 0)
#define LITEPCIE_SPI_START   (1 << 0)
#define LITEPCIE_SPI_DONE    (1 << 0)
#define LITEPCIE_SPI_LENGTH  (1 << 8)

extern "C" {

    #include "liblitepcie.h"

    #include <fcntl.h>
    #include <unistd.h>

    static inline uint32_t litepcie_lms7002m_spi_xfer(void *handle, const uint32_t mosi)
    {
        int *fd = (int *)handle;
        uint32_t miso;

        /* Load SPI MOSI Data */
        litepcie_writel(*fd, CSR_LMS7002M_SPI_MOSI_ADDR, mosi);

        /* Start SPI Xfer */
        litepcie_writel(*fd, CSR_LMS7002M_SPI_CONTROL_ADDR, 32*LITEPCIE_SPI_LENGTH | LITEPCIE_SPI_START);

        /* Wait for SPI Xfer */
        while ((litepcie_readl(*fd, CSR_LMS7002M_SPI_STATUS_ADDR) & LITEPCIE_SPI_DONE) == 0);

        /* Get MISO Data */
        miso = litepcie_readl(*fd, CSR_LMS7002M_SPI_MISO_ADDR) & 0xffff;

    #ifdef LITEPCIE_SPI_DEBUG
        /* Debug */
        printf("%s addr: 0x%04x, mosi: 0x%08x, miso: 0x%08x\n",
            __func__,
            0x7fff & (mosi >> 16),
            mosi & 0xffff,
            miso & 0xffff
        );
    #endif

        /* Return MISO Data */
        return miso;
    }

}

class DLL_EXPORT LMS_SPI: public lime::ISPI {
    public:
        LMS_SPI(int fd):_fd(fd) {}

        lime::OpStatus SPI(const uint32_t* MOSI, uint32_t* MISO, uint32_t count) override
        {
            for (uint32_t i = 0; i < count; i++) {
                    bool readback = !(MOSI[i] >> 31);
                    uint32_t rd;
                    uint32_t wr = MOSI[i];
                    if (readback)
                        wr = (0xffff & wr) << 16;
                    rd = litepcie_lms7002m_spi_xfer(&_fd, wr);
                    if (readback)
                        MISO[i] = rd;
            }
            return lime::OpStatus::Success;
        }

        lime::OpStatus SPI(uint32_t spiBusAddress, const uint32_t* MOSI,
            uint32_t* MISO, uint32_t count) override
        {
            for (uint32_t i = 0; i < count; i++) {
                printf("%d addr %08x ", i, spiBusAddress);
                if (MOSI)
                    printf("MOSI %02x ", MOSI[i]);
                if (MISO)
                    printf("MISO %02x ", MISO[i]);
                printf("\n");
            }
            return lime::OpStatus::Success;
        }

    private:
        int _fd;
};


/***************************************************************************************************
 *                                     Constructor
 **************************************************************************************************/

std::string getLiteXXTRXSerial(int fd);
std::string getLiteXXTRXIdentification(int fd);

void dma_set_loopback(int fd, bool loopback_enable) {
    struct litepcie_ioctl_dma m;
    m.loopback_enable = loopback_enable ? 1 : 0;
    checked_ioctl(fd, LITEPCIE_IOCTL_DMA, &m);
}

SoapyLiteXXTRX::SoapyLiteXXTRX(const SoapySDR::Kwargs &args)
    : _fd(-1),
    _masterClockRate(1.0e6), _refClockRate(26e6) {
    //LMS7_set_log_handler(&customLogHandler);
    //LMS7_set_log_level(LMS7_TRACE);
    SoapySDR::logf(SOAPY_SDR_INFO, "SoapyLiteXXTRX initializing...");
    setvbuf(stdout, NULL, _IOLBF, 0);

    // open LitePCIe descriptor
    if (args.count("path") == 0) {
        // if path is not present, then findLiteXXTRX had zero devices enumerated
        throw std::runtime_error("No LitePCIe devices found!");
    }
    std::string path = args.at("path");
    _fd = open(path.c_str(), O_RDWR);
    if (_fd < 0)
        throw std::runtime_error("SoapyLiteXXTRX(): failed to open " + path);

    SoapySDR::logf(SOAPY_SDR_INFO, "Opened devnode %s, serial %s", path.c_str(), getLiteXXTRXSerial(_fd).c_str());
    // reset the LMS7002M
    litepcie_writel(_fd, CSR_LMS7002M_CONTROL_ADDR,
        1 * (1 << CSR_LMS7002M_CONTROL_RESET_OFFSET)
    );
    litepcie_writel(_fd, CSR_LMS7002M_CONTROL_ADDR,
        0 * (1 << CSR_LMS7002M_CONTROL_RESET_OFFSET)
    );

    // reset XTRX-specific LMS7002M controls
    litepcie_writel(_fd, CSR_LMS7002M_CONTROL_ADDR,
        0 * (1 << CSR_LMS7002M_CONTROL_POWER_DOWN_OFFSET) |
        1 * (1 << CSR_LMS7002M_CONTROL_TX_ENABLE_OFFSET)  |
        1 * (1 << CSR_LMS7002M_CONTROL_RX_ENABLE_OFFSET)  |
        0 * (1 << CSR_LMS7002M_CONTROL_TX_RX_LOOPBACK_ENABLE_OFFSET)
    );

    /* bypass synchro */
    litepcie_writel(_fd, CSR_PCIE_DMA0_SYNCHRONIZER_BYPASS_ADDR, 1);

    // reset other FPGA peripherals
#if 1
    // GGM Test
    writeSetting("FPGA_DMA_LOOPBACK_ENABLE", "FALSE");
    writeSetting("FPGA_TX_PATTERN", "0");
    writeSetting("FPGA_RX_PATTERN", "0");
#else
    writeSetting("FPGA_DMA_LOOPBACK_ENABLE", "FALSE");
    writeSetting("FPGA_TX_PATTERN", "0");
    writeSetting("FPGA_RX_PATTERN", "0");
#endif
    writeSetting("FPGA_RX_DELAY", "16");
    writeSetting("FPGA_TX_DELAY", "16");

    // setup LMS7002M
    _lms_spi = std::make_shared<LMS_SPI>(_fd);
    _lms2 = new lime::LMS7002M(_lms_spi);
    _lms2->SoftReset();
    _lms2->Modify_SPI_Reg_bits(LMS7param(SPIMODE), 1); // 4 Wire SPI Mode
    // configure data port directions and data clock rates
    for (auto &p: xtrx_default_cfg)
        _lms2->SPI_write(p.first, p.second);

    _lms2->SetReferenceClk_SX(lime::TRXDir::Rx, _refClockRate);
    _lms2->SetClockFreq(lime::LMS7002M::ClockID::CLK_REFERENCE, _refClockRate);

    // read info register
    uint16_t ver = _lms2->Get_SPI_Reg_bits(LMS7param(VER));
    uint16_t rev = _lms2->Get_SPI_Reg_bits(LMS7param(REV));
    SoapySDR::logf(SOAPY_SDR_INFO, "LMS7002M info: revision %d, version %d", rev, ver);

    // set clock to Internal Reference Clock
    this->setClockSource("internal");

    // enable components
    _lms2->EnableChannel(lime::TRXDir::Tx, 0, true);  // LMS_CHA
    _lms2->EnableChannel(lime::TRXDir::Tx, 1, true);  // LMS_CHB
    _lms2->EnableChannel(lime::TRXDir::Rx, 0, true);  // LMS_CHA
    _lms2->EnableChannel(lime::TRXDir::Rx, 1, true);  // LMS_CHB

    // turn the clocks on (tested frequencies: 61.44MHZ, 122.88MHZ)
    //this->setMasterClockRate(122.88e6);
    //this->setMasterClockRate(2*2*49.152e6);
    this->setMasterClockRate(64e6*2);

    // some defaults to avoid throwing
    _cachedSampleRates[SOAPY_SDR_RX] = 1e6;
    _cachedSampleRates[SOAPY_SDR_TX] = 1e6;
    for (size_t i = 0; i < 2; i++) {
        _cachedFreqValues[SOAPY_SDR_RX][i]["RF"] = 1e9;
        _cachedFreqValues[SOAPY_SDR_TX][i]["RF"] = 1e9;
        _cachedFreqValues[SOAPY_SDR_RX][i]["BB"] = 0;
        _cachedFreqValues[SOAPY_SDR_TX][i]["BB"] = 0;
        this->setAntenna(SOAPY_SDR_RX, i, "LNAW");
        this->setAntenna(SOAPY_SDR_TX, i, "BAND1");

        // Use the same default gains as LimeSDR
        // LimeSuiteGUI lists these as:
        //   RFE page:
        //     LNA: Gmax (maps to 30dB)
        //     Loopback: Gmax-40 (not listed here)
        //     TIA: Gmax-3 (maps to 9dB)
        //   RBB page:
        //     PGA Gain: 6dB
        //   TRF page:
        //     TXPAD gain control: 0
        this->setGain(SOAPY_SDR_RX, i, "LNA", 32.0);
        this->setGain(SOAPY_SDR_RX, i, "TIA", 9.0);
        this->setGain(SOAPY_SDR_RX, i, "PGA", 6.0);
        this->setGain(SOAPY_SDR_TX, i, "PAD", 0.0);

        _cachedFilterBws[SOAPY_SDR_RX][i] = 10e6;
        _cachedFilterBws[SOAPY_SDR_TX][i] = 10e6;
        this->setIQBalance(SOAPY_SDR_RX, i, std::polar(1.0, 0.0));
        this->setIQBalance(SOAPY_SDR_TX, i, std::polar(1.0, 0.0));
    }

    // set-up the DMA
    checked_ioctl(_fd, LITEPCIE_IOCTL_MMAP_DMA_INFO, &_dma_mmap_info);
    _dma_buf = NULL;

    // NOTE: if initialization misses a setting/register, try experimenting in
    //       LimeGUI and loading that register dump here
#ifdef TODO
    if (args.count("ini") != 0) {
        if (LMS7002M_load_ini(_lms, args.at("ini").c_str())){
            SoapySDR::log(SOAPY_SDR_ERROR, "SoapyLiteXXTRX configuration load failed");
            throw std::runtime_error("failed to load XTRX configuration");
        } else {
            SoapySDR::logf(SOAPY_SDR_INFO, "SoapyLiteXXTRX configuration loaded from: %s", args.at("ini").c_str());

        }
    }
#endif

    SoapySDR::log(SOAPY_SDR_INFO, "SoapyLiteXXTRX initialization complete");
}

SoapyLiteXXTRX::~SoapyLiteXXTRX(void) {
    SoapySDR::log(SOAPY_SDR_INFO, "Power down and cleanup");
    if (_rx_stream.opened) {
        litepcie_release_dma(_fd, 0, 1);

            munmap(_rx_stream.buf, _dma_mmap_info.dma_rx_buf_size *
                                    _dma_mmap_info.dma_rx_buf_count);
        _rx_stream.opened = false;
    }
    if (_tx_stream.opened) {
        // release the DMA engine
        litepcie_release_dma(_fd, 1, 0);

        munmap(_tx_stream.buf, _dma_mmap_info.dma_tx_buf_size *
                                   _dma_mmap_info.dma_tx_buf_count);
        _tx_stream.opened = false;
    }
    // power down and clean up
    // NOTE: disable if you want to inspect the configuration (e.g. in LimeGUI)
    //       or to validate the settings (e.g. using xtrx_litepcie_test)
    _lms2->EnableChannel(lime::TRXDir::Tx, 0, false);  // LMS_CHA
    _lms2->EnableChannel(lime::TRXDir::Tx, 1, false);  // LMS_CHB
    _lms2->EnableChannel(lime::TRXDir::Rx, 0, false);  // LMS_CHA
    _lms2->EnableChannel(lime::TRXDir::Rx, 1, false);  // LMS_CHB

    delete _lms2;
    close(_fd);
}


/***************************************************************************************************
 *                                  Identification API
 **************************************************************************************************/

SoapySDR::Kwargs SoapyLiteXXTRX::getHardwareInfo(void) const {
    SoapySDR::Kwargs args;
    args["identification"] = getLiteXXTRXIdentification(_fd);
    return args;
}

/***************************************************************************************************
 *                                     Antenna API
 **************************************************************************************************/

std::vector<std::string> SoapyLiteXXTRX::listAntennas(const int direction,
                                                 const size_t) const {
    std::vector<std::string> ants;
    if (direction == SOAPY_SDR_RX) {
        ants.push_back("LNAH");
        ants.push_back("LNAL");
        ants.push_back("LNAW");
        ants.push_back("LB1");
        ants.push_back("LB2");
    }
    if (direction == SOAPY_SDR_TX) {
        ants.push_back("BAND1");
        ants.push_back("BAND2");
    }
    return ants;
}

#define LMS7002M_RFE_NONE 0
#define LMS7002M_RFE_LNAH 1
#define LMS7002M_RFE_LNAL 2
#define LMS7002M_RFE_LNAW 3

#define LMS7002M_RFE_LB1 1
#define LMS7002M_RFE_LB2 2

void SoapyLiteXXTRX::setAntenna(const int direction, const size_t channel,
                           const std::string &name) {
    std::lock_guard<std::mutex> lock(_mutex);
    int rx_rf_switch = 0;
    if (direction == SOAPY_SDR_RX) {
        int path = LMS7002M_RFE_NONE;
        if (name == "LNAH") {
            path = LMS7002M_RFE_LNAH;
            rx_rf_switch = 2;
        }
        else if (name == "LNAL") {
            path = LMS7002M_RFE_LNAL;
            rx_rf_switch = 1;
        }
        else if (name == "LNAW") {
            path = LMS7002M_RFE_LNAW;
            rx_rf_switch = 0;
        }
        else if (name == "LB1")
            path = LMS7002M_RFE_LB1;
        else if (name == "LB2")
            path = LMS7002M_RFE_LB2;
        else
            throw std::runtime_error("SoapyLiteXXTRX::setAntenna(RX, " + name +
                                     ") - unknown antenna name");
        _lms2->SetPath(lime::TRXDir::Rx, channel % 2, path);
        litepcie_writel(_fd, CSR_RF_SWITCHES_RX_ADDR, rx_rf_switch);
    }
    if (direction == SOAPY_SDR_TX) {
        int tx_rf_switch = 0;
        int band = 0;
        if (name == "BAND1") {
            band = 1;
            tx_rf_switch = 1;
        }
        else if (name == "BAND2") {
            band = 2;
            tx_rf_switch = 0;
        }
        else
            throw std::runtime_error("SoapyLiteXXTRX::setAntenna(TX, " + name +
                                     ") - unknown antenna name");
        _lms2->SetPath(lime::TRXDir::Tx, channel % 2, band);
        litepcie_writel(_fd, CSR_RF_SWITCHES_TX_ADDR, tx_rf_switch);
    }
    _cachedAntValues[direction][channel] = name;
}

std::string SoapyLiteXXTRX::getAntenna(const int direction,
                                  const size_t channel) const {
    return _cachedAntValues.at(direction).at(channel);
}


/***************************************************************************************************
 *                                 Frontend corrections API
 **************************************************************************************************/

bool SoapyLiteXXTRX::hasDCOffsetMode(const int direction,
                                const size_t /*channel*/) const {
    if (direction == SOAPY_SDR_RX) {
        return true;
    } else {
        return false;
    }
}

void SoapyLiteXXTRX::setDCOffsetMode(const int direction, const size_t channel,
                                const bool automatic) {
    std::lock_guard<std::mutex> lock(_mutex);

    if (direction == SOAPY_SDR_RX) {
        /* FIXME:
         * With LMS7002M_rxtsp_set_dc_correction DCCORR_AVG[2:0] (register 0x404)
         * is also updated with a value between 0 and 7.
         */
        _lms2->Modify_SPI_Reg_bits(LMS7param(DC_BYP_RXTSP), automatic == 0, channel);
        _rxDCOffsetMode = automatic;
    } else {
        SoapySDR::Device::setDCOffsetMode(direction, channel, automatic);
    }
}

bool SoapyLiteXXTRX::getDCOffsetMode(const int direction,
                                const size_t channel) const {
    if (direction == SOAPY_SDR_RX) {
        return _rxDCOffsetMode;
    } else {
        return SoapySDR::Device::getDCOffsetMode(direction, channel);
    }
}

bool SoapyLiteXXTRX::hasDCOffset(const int direction,
                            const size_t /*channel*/) const {

    if (direction == SOAPY_SDR_TX) {
        return true;
    } else {
        return false;
    }
}

void SoapyLiteXXTRX::setDCOffset(const int direction, const size_t channel,
                            const std::complex<double> &offset) {
    std::lock_guard<std::mutex> lock(_mutex);

    const auto lmsDir = (direction == SOAPY_SDR_TX) ? lime::TRXDir::Tx : lime::TRXDir::Rx;
    const lime::LMS7002M::Channel chan = channel > 0 ? lime::LMS7002M::Channel::ChB : lime::LMS7002M::Channel::ChA;
    _lms2->SetActiveChannel(chan);
    _lms2->SetDCOffset(lmsDir, offset.real(), offset.imag());
    if (direction == SOAPY_SDR_TX) {
        _txDCOffset = offset;
    } else {
        SoapySDR::Device::setDCOffset(direction, channel, offset);
    }
}

std::complex<double> SoapyLiteXXTRX::getDCOffset(const int direction,
                                            const size_t channel) const {
    if (direction == SOAPY_SDR_TX) {
        return _txDCOffset;
    } else {
        return SoapySDR::Device::getDCOffset(direction, channel);
    }
}

void SoapyLiteXXTRX::setIQBalance(const int direction, const size_t channel,
                             const std::complex<double> &balance) {
    std::lock_guard<std::mutex> lock(_mutex);

//#define FIXME
#ifdef FIXME
    /* With this implementation (from LimeSuiteNG) RX signal has a small / near zero real
     * part and all the information on the imaginary part.
     * method "set_iq_correction" comes from obsolete LMS700M_Driver.
     */
    const auto lmsDir = (direction == SOAPY_SDR_TX) ? lime::TRXDir::Tx : lime::TRXDir::Rx;
    const lime::LMS7002M::Channel chan = channel > 0 ? lime::LMS7002M::Channel::ChB : lime::LMS7002M::Channel::ChA;
    const double gain = std::abs(balance);
    double gainI = 1.0;
    double gainQ = 1.0;
    if (gain < 1.0)
        gainI = gain;

    if (gain > 1.0)
        gainQ = 1.0 / gain;

    _lms2->SetActiveChannel(chan);
    _lms2->SetIQBalance(lmsDir, std::arg(balance), gainI, gainQ);

#else
    set_iq_correction(direction, channel, std::arg(balance), std::abs(balance));
#endif
    _cachedIqBalValues[direction][channel] = balance;
}

void SoapyLiteXXTRX::set_iq_correction(const int direction,
    const size_t channel,
    const double phase,
    const double gain)
{
    const lime::LMS7002M::Channel chan = channel > 0 ? lime::LMS7002M::Channel::ChB : lime::LMS7002M::Channel::ChA;
    _lms2->SetActiveChannel(chan);

    const bool bypassPhase = (phase == 0.0);
    const bool bypassGain = (gain == 1.0) || (gain == 0.0);

    const int gcorrq = (gain > 1.0) ? ((1.0/gain)*2047) : 2047;
    const int gcorri = (gain < 1.0) ? ((gain/1.0)*2047) : 2047;

    _lms2->Modify_SPI_Reg_bits((direction == SOAPY_SDR_TX) ? LMS7param(PH_BYP_TXTSP) : LMS7param(PH_BYP_RXTSP),
        bypassPhase ? 1 : 0);
    _lms2->Modify_SPI_Reg_bits((direction == SOAPY_SDR_TX) ? LMS7param(GC_BYP_TXTSP) : LMS7param(GC_BYP_RXTSP),
        bypassGain ? 1 : 0);
    _lms2->Modify_SPI_Reg_bits((direction == SOAPY_SDR_TX) ? LMS7param(IQCORR_TXTSP) : LMS7param(IQCORR_RXTSP),
        2047*(phase/(M_PI/2)));
    _lms2->Modify_SPI_Reg_bits((direction == SOAPY_SDR_TX) ? LMS7param(GCORRI_TXTSP) : LMS7param(GCORRI_RXTSP),
        gcorri);
    _lms2->Modify_SPI_Reg_bits((direction == SOAPY_SDR_TX) ? LMS7param(GCORRQ_TXTSP) : LMS7param(GCORRQ_RXTSP),
        gcorrq);
}

std::complex<double> SoapyLiteXXTRX::getIQBalance(const int direction,
                                             const size_t channel) const {
    return _cachedIqBalValues.at(direction).at(channel);
}


/***************************************************************************************************
 *                                           Gain API
 **************************************************************************************************/

std::vector<std::string> SoapyLiteXXTRX::listGains(const int direction,
                                              const size_t) const {
    std::vector<std::string> gains;
    if (direction == SOAPY_SDR_RX) {
        gains.push_back("LNA");
        gains.push_back("TIA");
        gains.push_back("PGA");
    }
    if (direction == SOAPY_SDR_TX) {
        gains.push_back("PAD");
    }
    return gains;
}

constexpr static int MAXIMUM_GAIN_VALUE = 74;
// clang-format off
// LNA table
constexpr static std::array<unsigned int, MAXIMUM_GAIN_VALUE> LNATable = {
    0,  0,  0,  1,  1,  1,  2,  2,  2,  3,  3,  3,  4,  4,  4,  5,
    5,  5,  6,  6,  6,  7,  7,  7,  8,  9,  10, 11, 11, 11, 11, 11,
    11, 11, 11, 11, 11, 11, 11, 11, 12, 13, 14, 14, 14, 14, 14, 14,
    14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14,
    14, 14, 14, 14, 14, 14, 14, 14, 14, 14
};
// PGA table
constexpr static std::array<unsigned int, MAXIMUM_GAIN_VALUE> PGATable = {
    0,  1,  2,  0,  1,  2,  0,  1,  2,  0,  1,  2,  0,  1,  2,  0,
    1,  2,  0,  1,  2,  0,  1,  2,  0,  0,  0,  0,  1,  2,  3,  4,
    5,  6,  7,  8,  9,  10, 11, 12, 12, 12, 12, 4,  5,  6,  7,  8,
    9,  10, 11, 12, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21,
    22, 23, 24, 25, 26, 27, 28, 29, 30, 31
};
// clang-format on

void SoapyLiteXXTRX::setGain(int direction, size_t channel, const double value) {
    std::lock_guard<std::mutex> lock(_mutex);
    double val = value;
    lime::LMS7002M::Channel chan = channel > 0 ? lime::LMS7002M::Channel::ChB : lime::LMS7002M::Channel::ChA;

    SoapySDR::logf(SOAPY_SDR_DEBUG, "SoapyLiteXXTRX::setGain(%s, ch%d, %f dB)",
                   dir2Str(direction), channel, value);

    if (SOAPY_SDR_TX == direction) {
        if (_lms2->SetTRFPAD_dB(value, chan) != lime::OpStatus::Success)
            throw std::runtime_error("SoapyLiteXXTRX::setGain(" +
                                     std::to_string(value) + " dB) failed - ");

        val -= _lms2->GetTRFPAD_dB(chan);
        if (_lms2->SetTBBIAMP_dB(val, chan) != lime::OpStatus::Success)
            throw std::runtime_error("SoapyLiteXXTRX::setGain(" +
                                     std::to_string(value) + " dB) failed - ");
    } else {
        _lms2->SetActiveChannel(chan);
        val = std::clamp(static_cast<int>(value + 12), 0, MAXIMUM_GAIN_VALUE - 1);

        unsigned int lna = LNATable.at(std::lround(val));
        unsigned int pga = PGATable.at(std::lround(val));

        unsigned int tia = 0;
        // TIA table
        if (val > 51)
            tia = 2;
        else if (val > 42)
            tia = 1;
        int rcc_ctl_pga_rbb = (430 * (pow(0.65, pga / 10.0)) - 110.35) / 20.4516 + 16; // From datasheet

        if ((_lms2->Modify_SPI_Reg_bits(LMS7param(G_LNA_RFE), lna + 1) != lime::OpStatus::Success) ||
            (_lms2->Modify_SPI_Reg_bits(LMS7param(G_TIA_RFE), tia + 1) != lime::OpStatus::Success) ||
            (_lms2->Modify_SPI_Reg_bits(LMS7param(G_PGA_RBB), pga) != lime::OpStatus::Success) ||
            (_lms2->Modify_SPI_Reg_bits(LMS7param(RCC_CTL_PGA_RBB), rcc_ctl_pga_rbb) != lime::OpStatus::Success))
        {
            throw std::runtime_error("SoapyLiteXXTRX::setGain(" +
                                     std::to_string(value) + " dB) failed - ");
        }

    }
}

void SoapyLiteXXTRX::setGain(const int direction, const size_t channel,
                        const std::string &name, const double value) {
    std::lock_guard<std::mutex> lock(_mutex);
    lime::LMS7002M::Channel chan = channel > 0 ? lime::LMS7002M::Channel::ChB : lime::LMS7002M::Channel::ChA;
    lime::OpStatus ret = lime::OpStatus::Success;

    SoapySDR::logf(SOAPY_SDR_DEBUG, "SoapyLiteXXTRX::setGain(%s, ch%d, %s, %f dB)",
                   dir2Str(direction), channel, name.c_str(), value);

    double &actualValue = _cachedGainValues[direction][channel][name];

    if (direction == SOAPY_SDR_RX and name == "LNA") {
        ret = _lms2->SetRFELNA_dB(value, chan);
        actualValue = _lms2->GetRFELNA_dB(chan);
    }

    if (direction == SOAPY_SDR_RX and name == "LB_LNA") {
        ret = _lms2->SetRFELoopbackLNA_dB(value, chan);
        actualValue = _lms2->GetRFELoopbackLNA_dB(chan);
    }

    if (direction == SOAPY_SDR_RX and name == "PGA") {
        ret = _lms2->SetRBBPGA_dB(value, chan);
        actualValue = _lms2->GetRBBPGA_dB(chan);
    }

    if (direction == SOAPY_SDR_RX and name == "TIA") {
        ret = _lms2->SetRFETIA_dB(value, chan);
        actualValue = _lms2->GetRFETIA_dB(chan);
    }

    if (direction == SOAPY_SDR_TX and name == "PAD") {
        ret = _lms2->SetTRFPAD_dB(value, chan);
        actualValue = _lms2->GetTRFPAD_dB(chan);
    }

    if (direction == SOAPY_SDR_TX and name == "LB_PAD") {
        ret = _lms2->SetTRFLoopbackPAD_dB(value, chan);
        actualValue = _lms2->GetTRFLoopbackPAD_dB(chan);
    }

    if (ret == lime::OpStatus::Error)
        throw std::runtime_error("SoapyLiteXXTRX::setGain(" +
                                 std::to_string(value) + " dB) failed - ");
}

double SoapyLiteXXTRX::getGain(const int direction, const size_t channel,
                          const std::string &name) const {
    return _cachedGainValues.at(direction).at(channel).at(name);
}

SoapySDR::Range SoapyLiteXXTRX::getGainRange(const int direction,
                                        const size_t channel,
                                        const std::string &name) const {
    if (direction == SOAPY_SDR_RX and name == "LNA")
        return SoapySDR::Range(0.0, 30.0);
    if (direction == SOAPY_SDR_RX and name == "LB_LNA")
        return SoapySDR::Range(0.0, 40.0);
    if (direction == SOAPY_SDR_RX and name == "TIA")
        return SoapySDR::Range(0.0, 12.0);
    if (direction == SOAPY_SDR_RX and name == "PGA")
        return SoapySDR::Range(-12.0, 19.0);
    if (direction == SOAPY_SDR_TX and name == "PAD")
        return SoapySDR::Range(-52.0, 0.0);
    if (direction == SOAPY_SDR_TX and name == "LB_PAD")
        return SoapySDR::Range(-4.3, 0.0);
    return SoapySDR::Device::getGainRange(direction, channel, name);
}


/***************************************************************************************************
 *                                     Frequency API
 **************************************************************************************************/

void SoapyLiteXXTRX::setFrequency(const int direction, const size_t channel,
                             const std::string &name, const double frequency,
                             const SoapySDR::Kwargs &/*args*/) {
    std::unique_lock<std::mutex> lock(_mutex);

    SoapySDR::logf(SOAPY_SDR_DEBUG,
                   "SoapyLiteXXTRX::setFrequency(%s, ch%d, %s, %f MHz)",
                   dir2Str(direction), channel, name.c_str(), frequency / 1e6);

    if (name == "RF") {
        double actualFreq = 0.0;
        lime::OpStatus ret = _lms2->SetFrequencySX(
            ((direction == SOAPY_SDR_RX)?lime::TRXDir::Rx:lime::TRXDir::Tx), frequency);
        if (ret != lime::OpStatus::Success)
            throw std::runtime_error("SoapyLiteXXTRX::setFrequency(" +
                                     std::to_string(frequency / 1e6) +
                                     " MHz) failed - "/* + std::to_string(ret)*/);
        _cachedFreqValues[direction][0][name] = actualFreq;
        _cachedFreqValues[direction][1][name] = actualFreq;
    }

#ifdef FIXME
    /* FIXME: read only with LimeSuiteNG */
    if (name == "BB") {
        const double baseRate = this->getTSPRate(direction);
        if (direction == SOAPY_SDR_RX)
            LMS7002M_rxtsp_set_freq(_lms, ch2LMS(channel),
                                    frequency / baseRate);
        if (direction == SOAPY_SDR_TX)
            LMS7002M_txtsp_set_freq(_lms, ch2LMS(channel),
                                    frequency / baseRate);
        _cachedFreqValues[direction][channel][name] = frequency;
    }
#endif
}

double SoapyLiteXXTRX::getFrequency(const int direction, const size_t channel,
                               const std::string &name) const {
    return _cachedFreqValues.at(direction).at(channel).at(name);
}

std::vector<std::string> SoapyLiteXXTRX::listFrequencies(const int /*direction*/,
                                                    const size_t /*channel*/) const {
    std::vector<std::string> opts;
    opts.push_back("RF");
    opts.push_back("BB");
    return opts;
}

SoapySDR::RangeList
SoapyLiteXXTRX::getFrequencyRange(const int direction, const size_t /*channel*/,
                             const std::string &name) const {
    SoapySDR::RangeList ranges;
    if (name == "RF") {
        ranges.push_back(SoapySDR::Range(100e3, 3.8e9));
    }
    if (name == "BB") {
        const double rate = this->getTSPRate(direction);
        ranges.push_back(SoapySDR::Range(-rate / 2, rate / 2));
    }
    return ranges;
}


/***************************************************************************************************
 *                                        Sample Rate API
 **************************************************************************************************/

bool SoapyLiteXXTRX::LMS1_SetSampleRate(double f_Hz, uint8_t rxDecimation, uint8_t txInterpolation)
{
    if (rxDecimation == 0)
        rxDecimation = 2;

    if (txInterpolation == 0)
        txInterpolation = 2;

    if (rxDecimation != 0 && txInterpolation / rxDecimation > 4) {
        char message[256];
        snprintf(message, 256, "TxInterpolation(%i)/RxDecimation(%i) should not be more than 4",
            txInterpolation, rxDecimation);
        throw std::logic_error(message);
    }
    uint8_t oversample = rxDecimation;
    const bool bypass = (oversample == 1) || (oversample == 0 && f_Hz > 62e6);
    uint8_t hbd_ovr = 7; // decimation ratio is 2^(1+hbd_ovr), HBD_OVR_RXTSP=7 - bypass
    uint8_t hbi_ovr = 7; // interpolation ratio is 2^(1+hbi_ovr), HBI_OVR_TXTSP=7 - bypass
    double cgenFreq = f_Hz * 4; // AI AQ BI BQ
    if (!bypass) {
        if (oversample == 0) {
            const int n = lime::LMS7002M::CGEN_MAX_FREQ / (cgenFreq);
            oversample = (n >= 32) ? 32 : (n >= 16) ? 16 : (n >= 8) ? 8 : (n >= 4) ? 4 : 2;
        }

        hbd_ovr = 4;
        if (oversample <= 16) {
            const int decTbl[] = { 0, 0, 0, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3 };
            hbd_ovr = decTbl[oversample];
            rxDecimation = pow(2, hbd_ovr + 1);
        }
        cgenFreq *= 2 << hbd_ovr;
        rxDecimation = 2 << hbd_ovr;

        if (txInterpolation == 0) {
            int txMultiplier = std::log2(lime::LMS7002M::CGEN_MAX_FREQ / cgenFreq);
            txInterpolation = rxDecimation << txMultiplier;
        }

        if (txInterpolation >= rxDecimation) {
            hbi_ovr = hbd_ovr + std::log2(txInterpolation / rxDecimation);
            txInterpolation = pow(2, hbi_ovr + 1);
        } else {
            SoapySDR::logf(
                SOAPY_SDR_ERROR,
                "Rx decimation(2^%i) > Tx interpolation(2^%i) currently not supported", hbd_ovr, hbi_ovr);
            return false;
        }
    }
    SoapySDR::logf(SOAPY_SDR_DEBUG, "Sampling rate set(%.3f MHz): CGEN:%.3f MHz, Decim: 2^%i, Interp: 2^%i",
        f_Hz / 1e6,
        cgenFreq / 1e6,
        1 + hbd_ovr,
        1 + hbi_ovr);
    _lms2->Modify_SPI_Reg_bits(LMS7param(EN_ADCCLKH_CLKGN), 0);
    if (rxDecimation != 0)
        _lms2->Modify_SPI_Reg_bits(LMS7param(CLKH_OV_CLKL_CGEN), 2 - std::log2(txInterpolation / rxDecimation));
    else
        _lms2->Modify_SPI_Reg_bits(LMS7param(CLKH_OV_CLKL_CGEN), 2);
    _lms2->Modify_SPI_Reg_bits(LMS7param(MAC), 2);
    _lms2->Modify_SPI_Reg_bits(LMS7param(HBD_OVR_RXTSP), hbd_ovr);
    _lms2->Modify_SPI_Reg_bits(LMS7param(HBI_OVR_TXTSP), hbi_ovr);
    _lms2->Modify_SPI_Reg_bits(LMS7param(MAC), 1);
    _lms2->Modify_SPI_Reg_bits(LMS7param(HBD_OVR_RXTSP), hbd_ovr);
    _lms2->Modify_SPI_Reg_bits(LMS7param(HBI_OVR_TXTSP), hbi_ovr);

    if (f_Hz >= 61.45e6) {
        // LimeLight & Pad
        _lms2->Modify_SPI_Reg_bits(LMS7param(DIQ2_DS), 1);
        _lms2->Modify_SPI_Reg_bits(LMS7param(LML1_SISODDR), 1);
        _lms2->Modify_SPI_Reg_bits(LMS7param(LML2_SISODDR), 1);
        // CDS
        _lms2->Modify_SPI_Reg_bits(LMS7param(CDSN_RXALML), 0);
        _lms2->Modify_SPI_Reg_bits(LMS7param(CDS_RXALML), 1);
        // LDO
        _lms2->Modify_SPI_Reg_bits(LMS7param(PD_LDO_DIGIp1), 0);
        _lms2->Modify_SPI_Reg_bits(LMS7param(PD_LDO_DIGIp2), 0);
        _lms2->Modify_SPI_Reg_bits(LMS7param(RDIV_DIGIp2), 140);
    }

    lime::OpStatus ret = _lms2->SetInterfaceFrequency(cgenFreq, hbi_ovr, hbd_ovr);
    return (ret == lime::OpStatus::Success) ? true : false;
}


void SoapyLiteXXTRX::setSampleRate(const int direction, const size_t channel,
                              const double rate) {
    std::lock_guard<std::mutex> lock(_mutex);
    std::string dirName ((direction == SOAPY_SDR_RX) ? "Rx" : "Tx");
    SoapySDR::logf(SOAPY_SDR_DEBUG, "setSampleRate(%s, %ld, %g MHz)", dirName, channel, rate / 1e6);

    try {
        LMS1_SetSampleRate(rate, 0, 0);
    } catch (const std::exception& e) {
        SoapySDR::logf(SOAPY_SDR_ERROR, "setSampleRate(%s, %ld, %g MHz) Failed", dirName, channel, rate / 1e6);
        throw std::runtime_error("SoapyLMS7::setSampleRate() failed with message " + std::string{ e.what() });
    }

    _cachedSampleRates[direction] = rate;
}

double SoapyLiteXXTRX::getSampleRate(const int direction, const size_t) const {
    return _cachedSampleRates.at(direction);
}

SoapySDR::RangeList SoapyLiteXXTRX::getSampleRateRange(const int ,
                                               const size_t) const {
    return {SoapySDR::Range(100e3, 61.44e6, 0)};
}

std::vector<std::string> SoapyLiteXXTRX::getStreamFormats(const int /*direction*/,
                                                     const size_t /*channel*/) const
{
    std::vector<std::string> formats;
    formats.push_back(SOAPY_SDR_CS16);
    formats.push_back(SOAPY_SDR_CF32);
    return formats;
}

/***************************************************************************************************
 *                                        BW filter API
 **************************************************************************************************/

void SoapyLiteXXTRX::setBandwidth(const int direction, const size_t channel,
                             const double bw) {
    if (bw == 0.0)
        return;
    double &actualBw = _cachedFilterBws[direction][channel];
    double lpf = bw;
    double newLPF;
    if (lpf < 0)
        lpf = actualBw;

    std::lock_guard<std::mutex> lock(_mutex);

    SoapySDR::logf(SOAPY_SDR_DEBUG, "SoapyLiteXXTRX::setBandwidth(%s, ch%d, %f MHz)",
                   dir2Str(direction), channel, bw / 1e6);

    lime::OpStatus ret = lime::OpStatus::Success;
    const lime::LMS7002M::Channel chan = channel > 0 ? lime::LMS7002M::Channel::ChB : lime::LMS7002M::Channel::ChA;
    _lms2->SetActiveChannel(chan);

    if (direction == SOAPY_SDR_RX)
        newLPF = std::clamp(lpf, 1.4001e6, 130e6);
    else
        newLPF = std::clamp(lpf, 5e6, 130e6);

    if (newLPF != lpf) {
        SoapySDR::logf(SOAPY_SDR_INFO,
            "SoapyLiteXXTRX::setBandwidth %cXLPF set to %.3f MHz (requested %0.3f MHz [out of range])",
            direction == SOAPY_SDR_TX ? 'T' : 'R', newLPF / 1e6, lpf / 1e6);
    }

    lpf = newLPF;

    if (direction == SOAPY_SDR_TX) {
        int gain = _lms2->GetTBBIAMP_dB(chan);
        ret  = _lms2->SetTxLPF(lpf);
        _lms2->SetTBBIAMP_dB(gain, chan);
    } else {
        ret = _lms2->SetRxLPF(lpf);
    }

    if (ret != lime::OpStatus::Success) {
        throw std::runtime_error("SoapyLiteXXTRX::setBandwidth(" +
                                 std::to_string(bw / 1e6) + " MHz) failed - " +
                                 "");
    }

    actualBw = lpf;
}

double SoapyLiteXXTRX::getBandwidth(const int direction,
                               const size_t channel) const {
    return _cachedFilterBws.at(direction).at(channel);
}

std::vector<double> SoapyLiteXXTRX::listBandwidths(const int direction,
                                              const size_t) const {
    std::vector<double> bws;

    if (direction == SOAPY_SDR_RX) {
        bws.push_back(1.4e6);
        bws.push_back(3.0e6);
        bws.push_back(5.0e6);
        bws.push_back(10.0e6);
        bws.push_back(15.0e6);
        bws.push_back(20.0e6);
        bws.push_back(37.0e6);
        bws.push_back(66.0e6);
        bws.push_back(108.0e6);
    }
    if (direction == SOAPY_SDR_TX) {
        bws.push_back(2.4e6);
        bws.push_back(2.74e6);
        bws.push_back(5.5e6);
        bws.push_back(8.2e6);
        bws.push_back(11.0e6);
        bws.push_back(18.5e6);
        bws.push_back(38.0e6);
        bws.push_back(54.0e6);
    }

    return bws;
}


/***************************************************************************************************
 *                                        Clocking API
 **************************************************************************************************/

double SoapyLiteXXTRX::getTSPRate(const int direction) const {
    return (direction == SOAPY_SDR_TX) ? _masterClockRate
                                       : _masterClockRate / 4;
}

void SoapyLiteXXTRX::setMasterClockRate(const double rate) {
    std::lock_guard<std::mutex> lock(_mutex);

    lime::LMS7002M::CGEN_details out;
    lime::OpStatus ret = _lms2->SetFrequencyCGEN(rate, true, &out);
    if (ret == lime::OpStatus::Success) {
        _masterClockRate = out.frequency;
    } else {
        SoapySDR::logf(SOAPY_SDR_ERROR, "LMS7002M_set_data_clock(%f MHz) -> %d",
                       rate / 1e6, ret);
        throw std::runtime_error("XTRX fail LMS7002M_set_data_clock()");
    }
    SoapySDR::logf(SOAPY_SDR_TRACE, "LMS7002M_set_data_clock(%f MHz) -> %f MHz",
                   rate / 1e6, _masterClockRate / 1e6);
}

double SoapyLiteXXTRX::getMasterClockRate(void) const { return _masterClockRate; }

/*!
 * Set the reference clock rate of the device.
 * \param rate the clock rate in Hz
 */
void SoapyLiteXXTRX::setReferenceClockRate(const double rate) {
    _refClockRate = rate;
}

/*!
 * Get the reference clock rate of the device.
 * \return the clock rate in Hz
 */
double SoapyLiteXXTRX::getReferenceClockRate(void) const { return _refClockRate; }

/*!
 * Get the range of available reference clock rates.
 * \return a list of clock rate ranges in Hz
 */
SoapySDR::RangeList SoapyLiteXXTRX::getReferenceClockRates(void) const {
    SoapySDR::RangeList ranges;
    // Really whatever you want to try...
    ranges.push_back(SoapySDR::Range(25e6, 27e6));
    return ranges;
}



/*!
 * Get the list of available clock sources.
 * \return a list of clock source names
 */
std::vector<std::string> SoapyLiteXXTRX::listClockSources(void) const {
    std::vector<std::string> sources;
    sources.push_back("internal");
    sources.push_back("external");
    return sources;
}

/*!
 * Set the clock source on the device
 * \param source the name of a clock source
 */
void SoapyLiteXXTRX::setClockSource(const std::string &source) {
    int control = litepcie_readl(_fd, CSR_VCTCXO_CONTROL_ADDR);
    control &= ~(1 << CSR_VCTCXO_CONTROL_SEL_OFFSET);

    if (source == "external") {
        control |= 1 << CSR_VCTCXO_CONTROL_SEL_OFFSET;
    } else if (source != "internal") {
        throw std::runtime_error("setClockSource(" + source + ") invalid");
    }
    litepcie_writel(_fd, CSR_VCTCXO_CONTROL_ADDR, control);
}

/*!
 * Get the clock source of the device
 * \return the name of a clock source
 */
std::string SoapyLiteXXTRX::getClockSource(void) const {
    int source = litepcie_readl(_fd, CSR_VCTCXO_CONTROL_ADDR) & (1 << CSR_VCTCXO_CONTROL_SEL_OFFSET);
    return source ? "external" : "internal";
}

/***************************************************************************************************
 *                                  Sensors API
 **************************************************************************************************/

std::vector<std::string> SoapyLiteXXTRX::listSensors(void) const {
    std::vector<std::string> sensors;
#ifdef CSR_XADC_BASE
    sensors.push_back("xadc_temp");
    sensors.push_back("xadc_vccint");
    sensors.push_back("xadc_vccaux");
    sensors.push_back("xadc_vccbram");
#endif
    return sensors;
}

SoapySDR::ArgInfo SoapyLiteXXTRX::getSensorInfo(const std::string &key) const {
    SoapySDR::ArgInfo info;

    std::size_t dash = key.find("_");
    if (dash < std::string::npos) {
        std::string deviceStr = key.substr(0, dash);
        std::string sensorStr = key.substr(dash + 1);

#ifdef CSR_XADC_BASE
        if (deviceStr == "xadc") {
            /* Temp */
            if (sensorStr == "temp") {
                info.key         = "temp";
                info.value       = "0.0";
                info.units       = "C";
                info.description = "FPGA temperature";
                info.type        = SoapySDR::ArgInfo::FLOAT;
            /* VCCINT */
            } else if (sensorStr == "vccint") {
                info.key         = "vccint";
                info.value       = "0.0";
                info.units       = "V";
                info.description = "FPGA internal supply voltage";
                info.type        = SoapySDR::ArgInfo::FLOAT;
            /* VCCAUX */
            } else if (sensorStr == "vccaux") {
                info.key         = "vccaux";
                info.value       = "0.0";
                info.units       = "V";
                info.description = "FPGA auxiliary supply voltage";
                info.type        = SoapySDR::ArgInfo::FLOAT;
            /* VCCBRAM */
            } else if (sensorStr == "vccbram") {
                info.key         = "vccbram";
                info.value       = "0.0";
                info.units       = "V";
                info.description = "FPGA supply voltage for block RAM memories";
                info.type        = SoapySDR::ArgInfo::FLOAT;
            } else {
                throw std::runtime_error("SoapyLiteXXTRX::getSensorInfo(" + key + ") unknown sensor");
            }
            return info;
        }
#endif
        throw std::runtime_error("SoapyLiteXXTRX::getSensorInfo(" + key + ") unknown device");
    }
    throw std::runtime_error("SoapyLiteXXTRX::getSensorInfo(" + key + ") unknown key");
}

std::string SoapyLiteXXTRX::readSensor(const std::string &key) const {
    std::string sensorValue;

    std::size_t dash = key.find("_");
    if (dash < std::string::npos) {
        std::string deviceStr = key.substr(0, dash);
        std::string sensorStr = key.substr(dash + 1);

#ifdef CSR_XADC_BASE
        if (deviceStr == "xadc") {
            /* Temp */
            if (sensorStr == "temp") {
                sensorValue = std::to_string(
                    (double)litepcie_readl(_fd, CSR_XADC_TEMPERATURE_ADDR) * 503.975 / 4096 - 273.15
                );
            /* VCCINT */
            } else if (sensorStr == "vccint") {
                sensorValue = std::to_string(
                    (double)litepcie_readl(_fd, CSR_XADC_VCCINT_ADDR) / 4096 * 3
                );
            /* VCCAUX */
            } else if (sensorStr == "vccaux") {
                sensorValue = std::to_string(
                    (double)litepcie_readl(_fd, CSR_XADC_VCCAUX_ADDR) / 4096 * 3
                );
            /* VCCBRAM */
            } else if (sensorStr == "vccbram") {
                sensorValue = std::to_string(
                    (double)litepcie_readl(_fd, CSR_XADC_VCCBRAM_ADDR) / 4096 * 3
                );
            } else {
                throw std::runtime_error("SoapyLiteXXTRX::getSensorInfo(" + key + ") unknown sensor");
            }
            return sensorValue;
        }
#endif
        throw std::runtime_error("SoapyLiteXXTRX::getSensorInfo(" + key + ") unknown device");
    }
    throw std::runtime_error("SoapyLiteXXTRX::getSensorInfo(" + key + ") unknown key");
}

/***************************************************************************************************
 *                                     Register API
 **************************************************************************************************/

std::vector<std::string> SoapyLiteXXTRX::listRegisterInterfaces(void) const {
    std::vector<std::string> interfaces;
    interfaces.push_back("LMS7002M");
    interfaces.push_back("LitePCIe");
    return interfaces;
}

void SoapyLiteXXTRX::writeRegister(const unsigned addr, const unsigned value) {
    _lms2->SPI_write(addr, value);
}

unsigned SoapyLiteXXTRX::readRegister(const unsigned addr) const {
    return _lms2->SPI_read(addr);
}

void SoapyLiteXXTRX::writeRegister(const std::string &name, const unsigned addr, const unsigned value) {
    if (name == "LMS7002M") {
        _lms2->SPI_write(addr, value);
    } else if (name == "LitePCIe") {
        litepcie_writel(_fd, addr, value);
    } else
        throw std::runtime_error("SoapyLiteXXTRX::writeRegister(" + name + ") unknown register");
}

unsigned SoapyLiteXXTRX::readRegister(const std::string &name, const unsigned addr) const {
    if (name == "LMS7002M") {
        return _lms2->SPI_read(addr);
    } else if (name == "LitePCIe") {
        return litepcie_readl(_fd, addr);
    } else
        throw std::runtime_error("SoapyLiteXXTRX::readRegister(" + name + ") unknown register");
}

/***************************************************************************************************
 *                                        Settings API
 **************************************************************************************************/

std::string SoapyLiteXXTRX::readSetting(const std::string &key) const
{
    SoapySDR::logf(SOAPY_SDR_DEBUG, "SoapyLiteXXTRX::readSetting(%s)", key.c_str());

    /* FPGA_TX_RX_LOOPBACK_ENABLE */
    if (key == "FPGA_TX_RX_LOOPBACK_ENABLE") {
        uint32_t control = litepcie_readl(_fd, CSR_LMS7002M_CONTROL_ADDR);
        control &= 1 << CSR_LMS7002M_CONTROL_TX_RX_LOOPBACK_ENABLE_OFFSET;
        return control ? "TRUE" : "FALSE";

    /* FPGA_TX_PATTERN */
    } else if (key == "FPGA_TX_PATTERN") {
        uint32_t control = litepcie_readl(_fd, CSR_LMS7002M_TX_PATTERN_CONTROL_ADDR);
        control &= 1 << CSR_LMS7002M_TX_PATTERN_CONTROL_ENABLE_OFFSET;
        return control ? "1" : "0";

    /* FPGA_TX_PATTERN */
    } else if (key == "FPGA_TX_PATTERN") {
        uint32_t control = litepcie_readl(_fd, CSR_LMS7002M_RX_PATTERN_CONTROL_ADDR);
        control &= 1 << CSR_LMS7002M_RX_PATTERN_CONTROL_ENABLE_OFFSET;
        return control ? "1" : "0";

    /* FPGA_RX_PATTERN_ERRORS */
    } else if (key == "FPGA_RX_PATTERN_ERRORS") {
        uint32_t errors = litepcie_readl(_fd, CSR_LMS7002M_RX_PATTERN_ERRORS_ADDR);
        return std::to_string(errors);

    /* FPGA_TX_DELAY */
    } else if (key == "FPGA_TX_DELAY") {
        uint32_t reg = litepcie_readl(_fd, CSR_LMS7002M_DELAY_ADDR);
        uint32_t mask = ((uint32_t)(1 << CSR_LMS7002M_DELAY_TX_DELAY_SIZE)-1);
        uint32_t delay = (reg >> CSR_LMS7002M_DELAY_TX_DELAY_OFFSET) & mask;
        return std::to_string(delay);

    /* FPGA_RX_DELAY */
    } else if (key == "FPGA_RX_DELAY") {
        uint32_t reg = litepcie_readl(_fd, CSR_LMS7002M_DELAY_ADDR);
        uint32_t mask = ((uint32_t)(1 << CSR_LMS7002M_DELAY_RX_DELAY_SIZE)-1);
        uint32_t delay = (reg >> CSR_LMS7002M_DELAY_RX_DELAY_OFFSET) & mask;
        return std::to_string(delay);

    /* DMA_BUFFERS */
    } else if (key == "DMA_BUFFERS") {
        return "RX hw count : "     + std::to_string(_rx_stream.hw_count)   +
                " RX sw count : "   + std::to_string(_rx_stream.sw_count)   +
                " RX user count : " + std::to_string(_rx_stream.user_count) +
                " TX hw count : "   + std::to_string(_tx_stream.hw_count)   +
                " TX sw count : "   + std::to_string(_tx_stream.sw_count)   +
                " TX user count : " + std::to_string(_tx_stream.user_count);
    } else
        throw std::runtime_error("SoapyLiteXXTRX::readSetting(" + key + ") unknown key");
}

void SoapyLiteXXTRX::writeSetting(const std::string &key, const std::string &value) {
    SoapySDR::logf(SOAPY_SDR_DEBUG, "SoapyLiteXXTRX::writeSetting(%s, %s)",
                   key.c_str(), value.c_str());

    std::lock_guard<std::mutex> lock(_mutex);

    /************/
    /* LMS7002M */
    /* **********/

    // undo any changes caused by one of the other keys with these enable calls
#ifdef TODO
    if (key == "RXTSP_ENABLE")
        LMS7002M_rxtsp_enable(_lms, LMS_CHAB, value == "TRUE");
    else if (key == "TXTSP_ENABLE")
        LMS7002M_txtsp_enable(_lms, LMS_CHAB, value == "TRUE");
    else if (key == "RBB_ENABLE")
        LMS7002M_rbb_enable(_lms, LMS_CHAB, value == "TRUE");
    else if (key == "TBB_ENABLE")
        LMS7002M_tbb_enable(_lms, LMS_CHAB, value == "TRUE");
    else if (key == "TRF_ENABLE_LOOPBACK")
        LMS7002M_trf_enable_loopback(_lms, LMS_CHAB, value == "TRUE");
    else if (key == "RXTSP_TSG_CONST") {
        const int ampl = std::stoi(value);
        LMS7002M_rxtsp_tsg_const(_lms, LMS_CHAB, ampl, 0);
    }
    else if (key == "TXTSP_TSG_CONST") {
        const int ampl = std::stoi(value);
        LMS7002M_txtsp_tsg_const(_lms, LMS_CHAB, ampl, 0);
    }
    else if (key == "TBB_ENABLE_LOOPBACK") {
        SoapySDR::log(SOAPY_SDR_DEBUG, "Setting TBB loopback");
        int path = 0;
        if (value == "LB_DISCONNECTED")
            path = LMS7002M_TBB_LB_DISCONNECTED;
        else if (value == "LB_DAC_CURRENT")
            path = LMS7002M_TBB_LB_DAC_CURRENT;
        else if (value == "LB_LB_LADDER")
            path = LMS7002M_TBB_LB_LB_LADDER;
        else if (value == "LB_MAIN_TBB")
            path = LMS7002M_TBB_LB_MAIN_TBB;
        else
            throw std::runtime_error("SoapyLiteXXTRX::writeSetting(" + key + ", " +
                                     value + ") unknown value");
        LMS7002M_tbb_enable_loopback(_lms, LMS_CHAB, path, false);
    }
    else if (key == "TBB_SET_PATH") {
        int path = 0;
        if (value == "TBB_BYP")
            path = LMS7002M_TBB_BYP;
        else if (value == "TBB_S5")
            path = LMS7002M_TBB_S5;
        else if (value == "TBB_LAD")
            path = LMS7002M_TBB_LAD;
        else if (value == "TBB_LBF")
            path = LMS7002M_TBB_LBF;
        else if (value == "TBB_HBF")
            path = LMS7002M_TBB_HBF;
        else
            throw std::runtime_error("SoapyLiteXXTRX::writeSetting(" + key + ", " +
                                     value + ") unknown value");
        LMS7002M_tbb_set_path(_lms, LMS_CHAB, path);
    }
    else if (key == "RBB_SET_PATH") {
        int path = 0;
        if (value == "BYP")
            path = LMS7002M_RBB_BYP;
        else if (value == "LBF")
            path = LMS7002M_RBB_LBF;
        else if (value == "HBF")
            path = LMS7002M_RBB_HBF;
        else if (value == "LB_BYP")
            path = LMS7002M_RBB_LB_BYP;
        else if (value == "LB_LBF")
            path = LMS7002M_RBB_LB_LBF;
        else if (value == "LB_HBF")
            path = LMS7002M_RBB_LB_HBF;
        else if (value == "PDET")
            path = LMS7002M_RBB_PDET;
        else
            throw std::runtime_error("SoapyLiteXXTRX::writeSetting(" + key + ", " +
                                     value + ") unknown value");
        LMS7002M_rbb_set_path(_lms, LMS_CHAB, path);
    }
    else if (key == "LOOPBACK_ENABLE") {
        SoapySDR::log(SOAPY_SDR_DEBUG, "Setting Digital Loopback");
        if (value == "TRUE") {
            LMS7002M_setup_digital_loopback(_lms);
        } else if (value == "FALSE") {
            LMS7002M_configure_lml_port(_lms, LMS_PORT2, LMS_TX, 1);
            LMS7002M_configure_lml_port(_lms, LMS_PORT1, LMS_RX, 1);
        } else
            throw std::runtime_error("SoapyLiteXXTRX::writeSetting(" + key + ", " +
                                     value + ") unknown value");
    }
    else if (key == "LOOPBACK_ENABLE_LFSR") {
        SoapySDR::log(SOAPY_SDR_DEBUG, "Setting LFSR Loopback");
        if (value == "TRUE") {
            LMS7002M_setup_digital_loopback_lfsr(_lms);
        } else if (value == "FALSE") {
            LMS7002M_configure_lml_port(_lms, LMS_PORT2, LMS_TX, 1);
            LMS7002M_configure_lml_port(_lms, LMS_PORT1, LMS_RX, 1);
        } else
            throw std::runtime_error("SoapyLiteXXTRX::writeSetting(" + key + ", " +
                                     value + ") unknown value");
    }
    else if (key == "TRF_ENABLE_LOOPBACK")
        LMS7002M_trf_enable_loopback(_lms, LMS_CHAB, value == "TRUE");
    else if (key == "RESET_RX_FIFO")
        LMS7002M_reset_lml_fifo(_lms, LMS_RX);
    else if (key == "DUMP_INI")
        LMS7002M_dump_ini(_lms, value.c_str());
    else if (key == "RXTSP_TONE")
        LMS7002M_rxtsp_tsg_tone_div(_lms, LMS_CHAB, std::stoi(value));
    else if (key == "TXTSP_TONE")
        LMS7002M_txtsp_tsg_tone_div(_lms, LMS_CHAB, std::stoi(value));
    else if (key == "RXTSP_ENABLE")
        LMS7002M_rxtsp_enable(_lms, LMS_CHAB, value == "TRUE");
    else if (key == "TXTSP_ENABLE")
        LMS7002M_txtsp_enable(_lms, LMS_CHAB, value == "TRUE");
#endif

    /********/
    /* FPGA */
    /********/

    /* FPGA_TX_RX_LOOPBACK_ENABLE */
    if (key == "FPGA_TX_RX_LOOPBACK_ENABLE") {
        SoapySDR::log(SOAPY_SDR_DEBUG, "Setting FPGA TX-RX Loopback");
        uint32_t control = litepcie_readl(_fd, CSR_LMS7002M_CONTROL_ADDR);
        control &= ~(1 << CSR_LMS7002M_CONTROL_TX_RX_LOOPBACK_ENABLE_OFFSET);
        if (value == "TRUE") {
            control |= (1 << CSR_LMS7002M_CONTROL_TX_RX_LOOPBACK_ENABLE_OFFSET);
        } else if (value != "FALSE")
            throw std::runtime_error("SoapyLiteXXTRX::writeSetting(" + key + ", " +
                                     value + ") unknown value");
        litepcie_writel(_fd, CSR_LMS7002M_CONTROL_ADDR, control);

    /* FPGA_DMA_LOOPBACK_ENABLE */
    } else if (key == "FPGA_DMA_LOOPBACK_ENABLE") {
        SoapySDR::log(SOAPY_SDR_DEBUG, "Setting FPGA DMA Loopback");
        if (value == "TRUE")
             dma_set_loopback(_fd, true);
        else if (value == "FALSE")
             dma_set_loopback(_fd, false);
        else
            throw std::runtime_error("SoapyLiteXXTRX::writeSetting(" + key + ", " + value + ") unknown value");

    /* FPGA_TX_PATTERN */
    } else if (key == "FPGA_TX_PATTERN") {
        SoapySDR::log(SOAPY_SDR_DEBUG, "Setting FPGA TX pattern");
        uint32_t control = litepcie_readl(_fd, CSR_LMS7002M_TX_PATTERN_CONTROL_ADDR);
        control &= ~(1 << CSR_LMS7002M_TX_PATTERN_CONTROL_ENABLE_OFFSET);
        if (value == "1") {
            control |= 1 << CSR_LMS7002M_TX_PATTERN_CONTROL_ENABLE_OFFSET;
        } else if (value != "0")
            throw std::runtime_error("SoapyLiteXXTRX::writeSetting(" + key + ", " + value + ") unknown value");
        litepcie_writel(_fd, CSR_LMS7002M_TX_PATTERN_CONTROL_ADDR, control);

    /* FPGA_RD_PATTERN */
    } else if (key == "FPGA_RX_PATTERN") {
        SoapySDR::log(SOAPY_SDR_DEBUG, "Setting FPGA RX pattern");
        uint32_t control = litepcie_readl(_fd, CSR_LMS7002M_RX_PATTERN_CONTROL_ADDR);
        control &= ~(1 << CSR_LMS7002M_RX_PATTERN_CONTROL_ENABLE_OFFSET);
        if (value == "1") {
            control |= 1 << CSR_LMS7002M_RX_PATTERN_CONTROL_ENABLE_OFFSET;
        } else if (value != "0")
            throw std::runtime_error("SoapyLiteXXTRX::writeSetting(" + key + ", " + value + ") unknown value");
        litepcie_writel(_fd, CSR_LMS7002M_RX_PATTERN_CONTROL_ADDR, control);

    /* FPGA_TX_DELAY */
    } else if (key == "FPGA_TX_DELAY") {
        int delay = std::stoi(value);
        if (delay < 0 || delay > 31)
            throw std::runtime_error("SoapyLiteXXTRX::writeSetting(" + key + ", " + value + ") invalid value");
        uint32_t reg = litepcie_readl(_fd, CSR_LMS7002M_DELAY_ADDR);
        uint32_t mask = ((uint32_t)(1 << CSR_LMS7002M_DELAY_TX_DELAY_SIZE)-1) << CSR_LMS7002M_DELAY_TX_DELAY_OFFSET;
        litepcie_writel(_fd, CSR_LMS7002M_DELAY_ADDR, (reg & ~mask) | (delay << CSR_LMS7002M_DELAY_TX_DELAY_OFFSET));

    /* FPGA_RX_DELAY */
    } else if (key == "FPGA_RX_DELAY") {
        int delay = std::stoi(value);
        if (delay < 0 || delay > 31)
            throw std::runtime_error("SoapyLiteXXTRX::writeSetting(" + key + ", " + value + ") invalid value");
        uint32_t reg = litepcie_readl(_fd, CSR_LMS7002M_DELAY_ADDR);
        uint32_t mask = ((uint32_t)(1 << CSR_LMS7002M_DELAY_RX_DELAY_SIZE)-1) << CSR_LMS7002M_DELAY_RX_DELAY_OFFSET;
        litepcie_writel(_fd, CSR_LMS7002M_DELAY_ADDR, (reg & ~mask) | (delay << CSR_LMS7002M_DELAY_RX_DELAY_OFFSET));

    } else
        throw std::runtime_error("SoapyLiteXXTRX::writeSetting(" + key + ", " + value + ") unknown key");
}

#ifdef FORNEXT
bool SoapyLiteXXTRX::tsg_tone(const int direction, const size_t channel, const bool enable,
        const bool fullscale, const bool div4, const bool dcMode)
{
    const ChannelConfig& ch = config;
    _lms2->Modify_SPI_Reg_bits((direction == SOAPY_SDR_RX) ? LMS7_INSEL_RXTSP : LMS7_INSEL_TXTSP,
        enable ? 1 : 0);
    if (!enable)
        return true;

    switch (direction) {
        case SOAPY_SDR_RX:
            //const ChannelConfig::Direction::TestSignal& signal = ch.rx.testSignal;
            _lms2->Modify_SPI_Reg_bits(LMS7_TSGFC_RXTSP, fullscale ? 1 : 0);
            _lms2->Modify_SPI_Reg_bits(LMS7_TSGFCW_RXTSP, div4 ? 2 : 1);
            _lms2->Modify_SPI_Reg_bits(LMS7_TSGMODE_RXTSP, dcMode ? 1 : 0);
            _lms2->SPI_write(0x040C, 0x01FF); // DC.. bypasss
            // LMS7_TSGMODE_RXTSP change resets DC values
            return chip->LoadDC_REG_IQ(TRXDir::Rx, signal.dcValue.real(), signal.dcValue.imag());
            break;
        case SOAPY_SDR_RX:
            //const ChannelConfig::Direction::TestSignal& signal = ch.tx.testSignal;
            _lms2->Modify_SPI_Reg_bits(LMS7_TSGFC_TXTSP, fullscale ? 1 : 0);
            _lms2->Modify_SPI_Reg_bits(LMS7_TSGFCW_TXTSP, div4 ? 2 : 1);
            _lms2->Modify_SPI_Reg_bits(LMS7_TSGMODE_TXTSP, signal.dcMode ? 1 : 0);
            _lms2->SPI_write(0x040C, 0x01FF); // DC.. bypasss
            // LMS7_TSGMODE_TXTSP change resets DC values
            return chip->LoadDC_REG_IQ(TRXDir::Tx, signal.dcValue.real(), signal.dcValue.imag());
        break;
    }
    return OpStatus::Success;

}
#endif
