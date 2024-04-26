//
// SoapySDR driver for the LMS7002M-based Fairwaves XTRX.
//
// Copyright (c) 2021 Julia Computing.
// Copyright (c) 2015-2015 Fairwaves, Inc.
// Copyright (c) 2015-2015 Rice University
// SPDX-License-Identifier: Apache-2.0
// http://www.apache.org/licenses/LICENSE-2.0
//

// TODO
//
// - much here is copied from the EVB7 driver, which is also LMS7002M-based,
//   but quite some functionality still needs to be adapted for the XTRX.
//   preferably by somebody who actually knows about SDRs.
//
// - sometimes (after a reboot?) the SoapySDR driver fails to initialize the
//   XTRX, esp. when using the loopback or pattern generator. executing
//   `litex_test record /dev/null 1024`, even when that hangs, fixes that.
//   what are we not properly initializing?

#include "XTRXDevice.hpp"
#include "litepcie_interface.h"
#include <SoapySDR/Registry.hpp>
#include <SoapySDR/Logger.hpp>
#include <LMS7002M/LMS7002M_logger.h>
#include <fstream>
#include <memory>
#include <sys/mman.h>

#include <lms7002mNG/IComms.h>
#include <lms7002mNG/OpStatus.h>
#include <lms7002mNG/LMS7002M_parameters.h>

#define USE_NG
#undef USE_NG
//#define USE_NG2
//#define USE_OLD

/* Default configuration
 * mainly for TX->PORT2 & RX->PORT1
 * FIXME: registers details
 */
std::map<uint32_t, uint32_t> xtrx_default_cfg = {
    {0x0023, 0x5542},
    {0x002a, 0x0192},
    {0x002b, 0x002c},
    {0x002c, 0xffff},
    {0x00ad, 0x03f3},
};

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
                    rd = litepcie_interface_transact(&_fd, wr, readback);
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

void customLogHandler(const LMS7_log_level_t level, const char *message) {
    switch (level) {
    case LMS7_FATAL:    SoapySDR::log(SOAPY_SDR_FATAL, message);    break;
    case LMS7_CRITICAL: SoapySDR::log(SOAPY_SDR_CRITICAL, message); break;
    case LMS7_ERROR:    SoapySDR::log(SOAPY_SDR_ERROR, message);    break;
    case LMS7_WARNING:  SoapySDR::log(SOAPY_SDR_WARNING, message);  break;
    case LMS7_NOTICE:   SoapySDR::log(SOAPY_SDR_NOTICE, message);   break;
    case LMS7_INFO:     SoapySDR::log(SOAPY_SDR_INFO, message);     break;
    case LMS7_DEBUG:    SoapySDR::log(SOAPY_SDR_DEBUG, message);    break;
    case LMS7_TRACE:    SoapySDR::log(SOAPY_SDR_TRACE, message);    break;
    }
}

// Forward declaration for usage in constructor.
std::string getXTRXSerial(int fd);


/***********************************************************************
 * Constructor
 **********************************************************************/

void dma_set_loopback(int fd, bool loopback_enable) {
    struct litepcie_ioctl_dma m;
    m.loopback_enable = loopback_enable ? 1 : 0;
    checked_ioctl(fd, LITEPCIE_IOCTL_DMA, &m);
}

//#define ENABLE_TEST_TONE

SoapyLiteXXTRX::SoapyLiteXXTRX(const SoapySDR::Kwargs &args)
    : _fd(-1), _lms(NULL), _masterClockRate(1.0e6), _refClockRate(26e6) {
    LMS7_set_log_handler(&customLogHandler);
    LMS7_set_log_level(LMS7_TRACE);
    SoapySDR::logf(SOAPY_SDR_INFO, "SoapyLiteXXTRX initializing...");
    setvbuf(stdout, NULL, _IOLBF, 0);

    // open LitePCIe descriptor
    if (args.count("path") == 0) {
        // if path is not present, then findXTRX had zero devices enumerated
        throw std::runtime_error("No LitePCIe devices found!");
    }
    std::string path = args.at("path");
    _fd = open(path.c_str(), O_RDWR);
    if (_fd < 0)
        throw std::runtime_error("SoapyLiteXXTRX(): failed to open " + path);

    SoapySDR::logf(SOAPY_SDR_INFO, "Opened devnode %s, serial %s", path.c_str(), getXTRXSerial(_fd).c_str());
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
    _lms = LMS7002M_create(litepcie_interface_transact, &_fd);
    if (_lms == NULL) {
        throw std::runtime_error(
            "SoapyLiteXXTRX(): failed to LMS7002M_create()");
    }
    /* hack */
#ifdef USE_OLD
    LMS7002M_reset(_lms);
#else
    _lms_spi = std::make_shared<LMS_SPI>(_fd);
    _lms2 = new lime::LMS7002M(_lms_spi);
    _lms2->SetReferenceClk_SX(lime::TRXDir::Rx, _refClockRate);
    _lms2->SetClockFreq(lime::LMS7002M::ClockID::CLK_REFERENCE, _refClockRate);
    _lms2->SoftReset();
#endif
    LMS7002M_set_spi_mode(_lms, 4); // nothing equivalent

    // read info register
#ifndef USE_OLD
    uint16_t ver = _lms2->Get_SPI_Reg_bits(LMS7param(VER));
    uint16_t rev = _lms2->Get_SPI_Reg_bits(LMS7param(REV));
#else
    LMS7002M_regs_spi_read(_lms, 0x002f);
    uint16_t ver = LMS7002M_regs(_lms)->reg_0x002f_ver;
    uint16_t rev = LMS7002M_regs(_lms)->reg_0x002f_rev;
#endif
    SoapySDR::logf(SOAPY_SDR_INFO, "LMS7002M info: revision %d, version %d", rev, ver);

    // set clock to Internal Reference Clock
    this->setClockSource("internal");

    // configure data port directions and data clock rates
#ifdef USE_OLD
    LMS7002M_configure_lml_port(_lms, LMS_PORT2, LMS_TX, 1);
    LMS7002M_configure_lml_port(_lms, LMS_PORT1, LMS_RX, 1);
#else
    for (auto &p: xtrx_default_cfg)
        _lms2->SPI_write(p.first, p.second);
#endif

    // enable components
#ifdef USE_NG
    _lms2->EnableChannel(lime::TRXDir::Tx, 0, true);  // LMS_CHA
    _lms2->EnableChannel(lime::TRXDir::Tx, 1, true);  // LMS_CHB
    _lms2->EnableChannel(lime::TRXDir::Rx, 0, true);  // LMS_CHA
    _lms2->EnableChannel(lime::TRXDir::Rx, 1, true);  // LMS_CHB
#else
    LMS7002M_afe_enable(_lms, LMS_TX, LMS_CHA, true);
    LMS7002M_afe_enable(_lms, LMS_TX, LMS_CHB, true);
    LMS7002M_afe_enable(_lms, LMS_RX, LMS_CHA, true);
    LMS7002M_afe_enable(_lms, LMS_RX, LMS_CHB, true);
    LMS7002M_rxtsp_enable(_lms, LMS_CHA, true);
    LMS7002M_rxtsp_enable(_lms, LMS_CHB, true);
    LMS7002M_txtsp_enable(_lms, LMS_CHA, true);
    LMS7002M_txtsp_enable(_lms, LMS_CHB, true);
    LMS7002M_rbb_enable(_lms, LMS_CHA, true);
    LMS7002M_rbb_enable(_lms, LMS_CHB, true);
    LMS7002M_tbb_enable(_lms, LMS_CHA, true);
    LMS7002M_tbb_enable(_lms, LMS_CHB, true);
    LMS7002M_rfe_enable(_lms, LMS_CHA, true);
    LMS7002M_rfe_enable(_lms, LMS_CHB, true);
    LMS7002M_trf_enable(_lms, LMS_CHA, true);
    LMS7002M_trf_enable(_lms, LMS_CHB, true);
    LMS7002M_sxx_enable(_lms, LMS_RX, true);
    LMS7002M_sxx_enable(_lms, LMS_TX, true);
#endif

    // XTRX-specific configuration
    LMS7002M_ldo_enable(_lms, true, LMS7002M_LDO_ALL);
    LMS7002M_xbuf_share_tx(_lms, true);

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
    if (args.count("ini") != 0) {
        if (LMS7002M_load_ini(_lms, args.at("ini").c_str())){
            SoapySDR::log(SOAPY_SDR_ERROR, "SoapyLiteXXTRX configuration load failed");
            throw std::runtime_error("failed to load XTRX configuration");
        } else {
            SoapySDR::logf(SOAPY_SDR_INFO, "SoapyLiteXXTRX configuration loaded from: %s", args.at("ini").c_str());

        }
    }

#ifdef ENABLE_TEST_TONE
    writeSetting("RXTSP_ENABLE", "TRUE");
    writeSetting("RXTSP_TONE", "8");
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
#ifdef USE_NG
    _lms2->EnableChannel(lime::TRXDir::Tx, 0, false);  // LMS_CHA
    _lms2->EnableChannel(lime::TRXDir::Tx, 1, false);  // LMS_CHB
    _lms2->EnableChannel(lime::TRXDir::Rx, 0, false);  // LMS_CHA
    _lms2->EnableChannel(lime::TRXDir::Rx, 1, false);  // LMS_CHB
    // LMS7002M_rxtsp_enable/LMS7002M_txtsp_enable
    // LMS7002M_rbb_enable/LMS7002M_tbb_enable
    // are partially covered by EnableChannel
    //
#else
    LMS7002M_afe_enable(_lms, LMS_TX, LMS_CHA, false);
    LMS7002M_afe_enable(_lms, LMS_TX, LMS_CHB, false);
    LMS7002M_afe_enable(_lms, LMS_RX, LMS_CHA, false);
    LMS7002M_afe_enable(_lms, LMS_RX, LMS_CHB, false);
    LMS7002M_rxtsp_enable(_lms, LMS_CHAB, false);
    LMS7002M_txtsp_enable(_lms, LMS_CHAB, false);
    LMS7002M_rbb_enable(_lms, LMS_CHAB, false);
    LMS7002M_tbb_enable(_lms, LMS_CHAB, false);
    LMS7002M_rfe_enable(_lms, LMS_CHAB, false);
    LMS7002M_trf_enable(_lms, LMS_CHAB, false);
    LMS7002M_sxx_enable(_lms, LMS_RX, false);
    LMS7002M_sxx_enable(_lms, LMS_TX, false);
#endif
    /* FIXME: nothing equivalent */
    LMS7002M_xbuf_share_tx(_lms, false);
    LMS7002M_ldo_enable(_lms, false, LMS7002M_LDO_ALL);
    LMS7002M_power_down(_lms);
    LMS7002M_destroy(_lms);

    delete _lms2;
    close(_fd);
}


/***********************************************************************
 * Identification API
 **********************************************************************/

SoapySDR::Kwargs SoapyLiteXXTRX::getHardwareInfo(void) const {
    SoapySDR::Kwargs args;

    char fpga_identification[256];
    for (int i = 0; i < 256; i++)
        fpga_identification[i] =
            litepcie_readl(_fd, CSR_IDENTIFIER_MEM_BASE + 4 * i);
    args["identification"] = std::string(fpga_identification);

    return args;
}


/*******************************************************************
 * Antenna API
 ******************************************************************/

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

#ifndef USE_OLD
#undef LMS7002M_RFE_NONE
#undef LMS7002M_RFE_LNAH
#undef LMS7002M_RFE_LNAL
#undef LMS7002M_RFE_LNAW
#undef LMS7002M_RFE_LB1
#undef LMS7002M_RFE_LB2
enum {
    LMS7002M_RFE_NONE = 0,
    LMS7002M_RFE_LNAH,
    LMS7002M_RFE_LNAL,
    LMS7002M_RFE_LNAW,
};
enum {
    LMS7002M_RFE_LB1 = 1,
    LMS7002M_RFE_LB2,
};

#endif

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
#ifndef USE_OLD
        _lms2->SetPath(lime::TRXDir::Rx, channel % 2, path);
#else
        LMS7002M_rfe_set_path(_lms, ch2LMS(channel), path);
#endif
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
#ifndef USE_OLD
        // FIXME: same ?
        _lms2->SetPath(lime::TRXDir::Tx, channel % 2, band);
#else
        LMS7002M_trf_select_band(_lms, ch2LMS(channel), band);
#endif
        litepcie_writel(_fd, CSR_RF_SWITCHES_TX_ADDR, tx_rf_switch);
    }
    _cachedAntValues[direction][channel] = name;
}

std::string SoapyLiteXXTRX::getAntenna(const int direction,
                                  const size_t channel) const {
    return _cachedAntValues.at(direction).at(channel);
}


/*******************************************************************
 * Frontend corrections API
 ******************************************************************/

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
#ifndef USE_OLD
    _lms2->Modify_SPI_Reg_bits(LMS7param(DC_BYP_RXTSP), automatic == 0, channel);
    /* FIXME: missing window */
#else
        LMS7002M_rxtsp_set_dc_correction(_lms, ch2LMS(channel), automatic,
                                         7 /*max*/);
#endif
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

#ifndef USE_OLD
    const auto lmsDir = (direction == SOAPY_SDR_TX) ? lime::TRXDir::Tx : lime::TRXDir::Rx;
    _lms2->Modify_SPI_Reg_bits(LMS7param(MAC), (channel % 2) + 1);
    _lms2->SetDCOffset(lmsDir, offset.real(), offset.imag());
#endif

    if (direction == SOAPY_SDR_TX) {
#ifdef USE_OLD
        LMS7002M_txtsp_set_dc_correction(_lms, ch2LMS(channel), offset.real(),
                                         offset.imag());
#endif
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

#ifdef FIXME
    const auto lmsDir = (direction == SOAPY_SDR_TX) ? lime::TRXDir::Tx : lime::TRXDir::Rx;
    const double gain = std::abs(balance);
    double gainI = 1.0;
    double gainQ = 1.0;
    if (gain < 1.0)
        gainI = gain;

    if (gain > 1.0)
        gainQ = 1.0 / gain;

    _lms2->Modify_SPI_Reg_bits(LMS7param(MAC), (channel % 2) + 1);
    _lms2->SetIQBalance(lmsDir, std::arg(balance), gainI, gainQ);

#else
    if (direction == SOAPY_SDR_TX) {
        LMS7002M_txtsp_set_iq_correction(_lms, ch2LMS(channel),
                                         std::arg(balance), std::abs(balance));
    } else {
        LMS7002M_rxtsp_set_iq_correction(_lms, ch2LMS(channel),
                                         std::arg(balance), std::abs(balance));
    }
#endif
    _cachedIqBalValues[direction][channel] = balance;
}

std::complex<double> SoapyLiteXXTRX::getIQBalance(const int direction,
                                             const size_t channel) const {
    return _cachedIqBalValues.at(direction).at(channel);
}


/*******************************************************************
 * Gain API
 ******************************************************************/

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

void SoapyLiteXXTRX::setGain(const int direction, const size_t channel,
                        const std::string &name, const double value) {
    std::lock_guard<std::mutex> lock(_mutex);
#ifndef USE_OLD
    lime::LMS7002M::Channel chan = channel > 0 ? lime::LMS7002M::Channel::ChB : lime::LMS7002M::Channel::ChA;
    lime::OpStatus ret = lime::OpStatus::Success;
#endif

    SoapySDR::logf(SOAPY_SDR_DEBUG, "SoapyLiteXXTRX::setGain(%s, ch%d, %s, %f dB)",
                   dir2Str(direction), channel, name.c_str(), value);

    double &actualValue = _cachedGainValues[direction][channel][name];

#ifndef USE_OLD
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
#else
    if (direction == SOAPY_SDR_RX and name == "LNA") {
        actualValue = LMS7002M_rfe_set_lna(_lms, ch2LMS(channel), value);
    }

    if (direction == SOAPY_SDR_RX and name == "LB_LNA") {
        actualValue =
            LMS7002M_rfe_set_loopback_lna(_lms, ch2LMS(channel), value);
    }

    if (direction == SOAPY_SDR_RX and name == "TIA") {
        actualValue = LMS7002M_rfe_set_tia(_lms, ch2LMS(channel), value);
    }

    if (direction == SOAPY_SDR_RX and name == "PGA") {
        actualValue = LMS7002M_rbb_set_pga(_lms, ch2LMS(channel), value);
    }

    if (direction == SOAPY_SDR_TX and name == "PAD") {
        actualValue = LMS7002M_trf_set_pad(_lms, ch2LMS(channel), value);
    }

    if (direction == SOAPY_SDR_TX and name == "LB_PAD") {
        actualValue =
            LMS7002M_trf_set_loopback_pad(_lms, ch2LMS(channel), value);
    }
#endif
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


/*******************************************************************
 * Frequency API
 ******************************************************************/

void SoapyLiteXXTRX::setFrequency(const int direction, const size_t channel,
                             const std::string &name, const double frequency,
                             const SoapySDR::Kwargs &/*args*/) {
    std::unique_lock<std::mutex> lock(_mutex);

    SoapySDR::logf(SOAPY_SDR_DEBUG,
                   "SoapyLiteXXTRX::setFrequency(%s, ch%d, %s, %f MHz)",
                   dir2Str(direction), channel, name.c_str(), frequency / 1e6);

    if (name == "RF") {
        double actualFreq = 0.0;
#ifdef USE_OLD
        int ret = LMS7002M_set_lo_freq(_lms, dir2LMS(direction), _refClockRate,
                                       frequency, &actualFreq);
        if (ret != 0)
#else
        lime::OpStatus ret = _lms2->SetFrequencySX(
            ((direction == SOAPY_SDR_RX)?lime::TRXDir::Rx:lime::TRXDir::Tx), frequency);
        if (ret != lime::OpStatus::Success)
#endif
            throw std::runtime_error("SoapyLiteXXTRX::setFrequency(" +
                                     std::to_string(frequency / 1e6) +
                                     " MHz) failed - "/* + std::to_string(ret)*/);
        _cachedFreqValues[direction][0][name] = actualFreq;
        _cachedFreqValues[direction][1][name] = actualFreq;
    }

#ifdef USE_OLD
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


/*******************************************************************
 * Sample Rate API
 ******************************************************************/


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
#ifndef USE_OLD
    std::string dirName ((direction == SOAPY_SDR_RX) ? "Rx" : "Tx");
    SoapySDR::logf(SOAPY_SDR_DEBUG, "setSampleRate(%s, %ld, %g MHz)", dirName, channel, rate / 1e6);

    try {
        LMS1_SetSampleRate(rate, 0, 0);
    } catch (const std::exception& e) {
        SoapySDR::logf(SOAPY_SDR_ERROR, "setSampleRate(%s, %ld, %g MHz) Failed", dirName, channel, rate / 1e6);
        throw std::runtime_error("SoapyLMS7::setSampleRate() failed with message " + std::string{ e.what() });
    }

    _cachedSampleRates[direction] = rate;
#else

    const double baseRate = this->getTSPRate(direction);
    const double factor = baseRate / rate;
    int intFactor = 1 << int((std::log(factor) / std::log(2.0)) + 0.5);
    SoapySDR::logf(
        SOAPY_SDR_INFO,
        "SoapyLiteXXTRX::setSampleRate(%s, %f MHz), baseRate %f MHz, factor %f",
        dir2Str(direction), rate / 1e6, baseRate / 1e6, factor);
    if (intFactor < 2)
        throw std::runtime_error("SoapyLiteXXTRX::setSampleRate() -- rate too high");
    intFactor = 1 << int((std::log(factor) / std::log(2.0)) + 0.5);
    printf("inFactor %d\n", intFactor);
    if (intFactor > 32)
        throw std::runtime_error("SoapyLiteXXTRX::setSampleRate() -- rate too low");

    if (std::abs(factor - intFactor) > 0.01)
        SoapySDR::logf(SOAPY_SDR_WARNING,
                       "SoapyLiteXXTRX::setSampleRate(): not a power of two factor: "
                       "TSP Rate = %f MHZ, Requested rate = %f MHz",
                       baseRate / 1e6, rate / 1e6);

    // apply the settings, both the interp/decim has to be matched with the lml
    // interface divider the lml interface needs a clock rate 2x the sample rate
    // for DDR TRX IQ mode
    if (direction == SOAPY_SDR_RX) {
        LMS7002M_rxtsp_set_decim(_lms, LMS_CHAB, intFactor);
        LMS7002M_configure_lml_port(_lms, LMS_PORT1, LMS_RX, intFactor / 2);
    }
    if (direction == SOAPY_SDR_TX) {
        LMS7002M_txtsp_set_interp(_lms, LMS_CHAB, intFactor);
        LMS7002M_configure_lml_port(_lms, LMS_PORT2, LMS_TX, intFactor / 2);
    }

    _cachedSampleRates[direction] = baseRate / intFactor;
#endif
}

double SoapyLiteXXTRX::getSampleRate(const int direction, const size_t) const {
    return _cachedSampleRates.at(direction);
}

std::vector<double> SoapyLiteXXTRX::listSampleRates(const int direction,
                                               const size_t) const {
    // FIXME: / 1000 fix round maybe of something related to magnitude
    const double baseRate = this->getTSPRate(direction)/1000;
    std::vector<double> rates;
    // from baseRate/32 to baseRate/2
    for (int i = 5; i >= 1; i--) {
        rates.push_back(1000*round(baseRate / (1 << i)));
    }
    return rates;
}

std::vector<std::string> SoapyLiteXXTRX::getStreamFormats(const int /*direction*/,
                                                     const size_t /*channel*/) const
{
    std::vector<std::string> formats;
    formats.push_back(SOAPY_SDR_CS16);
    formats.push_back(SOAPY_SDR_CF32);
    return formats;
}

/*******************************************************************
 * BW filter API
 ******************************************************************/

void SoapyLiteXXTRX::setBandwidth(const int direction, const size_t channel,
                             const double bw) {
    std::lock_guard<std::mutex> lock(_mutex);

    SoapySDR::logf(SOAPY_SDR_DEBUG, "SoapyLiteXXTRX::setBandwidth(%s, ch%d, %f MHz)",
                   dir2Str(direction), channel, bw / 1e6);

#ifndef USE_OLD
    lime::OpStatus ret = lime::OpStatus::Success;
    _lms2->Modify_SPI_Reg_bits(LMS7param(MAC), (channel % 2) + 1);
#else
    int ret = 0;
#endif
    double &actualBw = _cachedFilterBws[direction][channel];
    if (direction == SOAPY_SDR_RX) {
#ifndef USE_OLD
        ret = _lms2->CalibrateRx(bw, false);
        if (ret == lime::OpStatus::Success)
#else
        //ret = LMS7002M_rbb_set_filter_bw(_lms, ch2LMS(channel), bw, &actualBw);
        ret = LMS7002M_mcu_calibration_rx(_lms, _refClockRate, bw);
        if (ret == 0)
#endif
            actualBw = bw;
    }
    if (direction == SOAPY_SDR_TX) {
#ifndef USE_OLD
        ret = _lms2->CalibrateTx(bw, false);
        if (ret == lime::OpStatus::Success)
#else
        //ret = LMS7002M_tbb_set_filter_bw(_lms, ch2LMS(channel), bw, &actualBw);
        ret = LMS7002M_mcu_calibration_tx(_lms, _refClockRate, bw);
        if (ret == 0)
#endif
            actualBw = bw;
    }

#ifndef USE_OLD
    if (ret != lime::OpStatus::Success)
#else
    if (ret != 0)
#endif
        throw std::runtime_error("SoapyLiteXXTRX::setBandwidth(" +
                                 std::to_string(bw / 1e6) + " MHz) failed - " +
#ifndef USE_OLD
                                 "");
#else
                                 std::to_string(ret));
#endif
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


/*******************************************************************
 * Clocking API
 ******************************************************************/

double SoapyLiteXXTRX::getTSPRate(const int direction) const {
    return (direction == SOAPY_SDR_TX) ? _masterClockRate
                                       : _masterClockRate / 4;
}

void SoapyLiteXXTRX::setMasterClockRate(const double rate) {
    std::lock_guard<std::mutex> lock(_mutex);

#ifndef USE_OLD
    lime::LMS7002M::CGEN_details out;
    lime::OpStatus ret = _lms2->SetFrequencyCGEN(rate, true, &out);
    if (ret == lime::OpStatus::Success) {
        _masterClockRate = out.frequency;
    } else {
#else
    int ret =
        LMS7002M_set_data_clock(_lms, _refClockRate, rate, &_masterClockRate);
    if (ret != 0) {
#endif
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

/*******************************************************************
 * Clocking API
 ******************************************************************/

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
            if (sensorStr == "temp") {
                info.key = "temp";
                info.value = "0.0";
                info.units = "C";
                info.description = "FPGA temperature";
                info.type = SoapySDR::ArgInfo::FLOAT;
            } else if (sensorStr == "vccint") {
                info.key = "vccint";
                info.value = "0.0";
                info.units = "V";
                info.description = "FPGA internal supply voltage";
                info.type = SoapySDR::ArgInfo::FLOAT;
            } else if (sensorStr == "vccaux") {
                info.key = "vccaux";
                info.value = "0.0";
                info.units = "V";
                info.description = "FPGA auxiliary supply voltage";
                info.type = SoapySDR::ArgInfo::FLOAT;
            } else if (sensorStr == "vccbram") {
                info.key = "vccbram";
                info.value = "0.0";
                info.units = "V";
                info.description = "FPGA supply voltage for block RAM memories";
                info.type = SoapySDR::ArgInfo::FLOAT;
            } else {
                throw std::runtime_error("SoapyLiteXXTRX::getSensorInfo(" + key +
                                         ") unknown sensor");
            }
            return info;
        }
#endif
        throw std::runtime_error("SoapyLiteXXTRX::getSensorInfo(" + key +
                                 ") unknown device");
    }
    throw std::runtime_error("SoapyLiteXXTRX::getSensorInfo(" + key +
                             ") unknown key");
}

std::string SoapyLiteXXTRX::readSensor(const std::string &key) const {
    std::string sensorValue;

    std::size_t dash = key.find("_");
    if (dash < std::string::npos) {
        std::string deviceStr = key.substr(0, dash);
        std::string sensorStr = key.substr(dash + 1);

#ifdef CSR_XADC_BASE
        if (deviceStr == "xadc") {
            if (sensorStr == "temp") {
                sensorValue = std::to_string(
                    (double)litepcie_readl(_fd, CSR_XADC_TEMPERATURE_ADDR) *
                        503.975 / 4096 -
                    273.15);
            } else if (sensorStr == "vccint") {
                sensorValue = std::to_string(
                    (double)litepcie_readl(_fd, CSR_XADC_VCCINT_ADDR) / 4096 *
                    3);
            } else if (sensorStr == "vccaux") {
                sensorValue = std::to_string(
                    (double)litepcie_readl(_fd, CSR_XADC_VCCAUX_ADDR) / 4096 *
                    3);
            } else if (sensorStr == "vccbram") {
                sensorValue = std::to_string(
                    (double)litepcie_readl(_fd, CSR_XADC_VCCBRAM_ADDR) / 4096 *
                    3);
            } else {
                throw std::runtime_error("SoapyLiteXXTRX::getSensorInfo(" + key +
                                         ") unknown sensor");
            }
            return sensorValue;
        }
#endif
        throw std::runtime_error("SoapyLiteXXTRX::getSensorInfo(" + key +
                                 ") unknown device");
    }
    throw std::runtime_error("SoapyLiteXXTRX::getSensorInfo(" + key +
                             ") unknown key");
}


/*******************************************************************
 * Register API
 ******************************************************************/

std::vector<std::string> SoapyLiteXXTRX::listRegisterInterfaces(void) const {
    std::vector<std::string> interfaces;
    interfaces.push_back("LMS7002M");
    interfaces.push_back("LitePCI");
    return interfaces;
}


void SoapyLiteXXTRX::writeRegister(const unsigned addr, const unsigned value) {
    LMS7002M_spi_write(_lms, addr, value);
}

unsigned SoapyLiteXXTRX::readRegister(const unsigned addr) const {
    return LMS7002M_spi_read(_lms, addr);
}



void SoapyLiteXXTRX::writeRegister(const std::string &name, const unsigned addr, const unsigned value) {
    if (name == "LMS7002M") {
        LMS7002M_spi_write(_lms, addr, value);
    } else if (name == "LitePCI") {
        litepcie_writel(_fd, addr, value);
    } else
        throw std::runtime_error("SoapyLiteXXTRX::writeRegister(" + name + ") unknown register");
}

unsigned SoapyLiteXXTRX::readRegister(const std::string &name, const unsigned addr) const {
    if (name == "LMS7002M") {
        return LMS7002M_spi_read(_lms, addr);
    } else if (name == "LitePCI") {
        return litepcie_readl(_fd, addr);
    } else
        throw std::runtime_error("SoapyLiteXXTRX::readRegister(" + name + ") unknown register");
}



/*******************************************************************
 * Settings API
 ******************************************************************/

std::string SoapyLiteXXTRX::readSetting(const std::string &key) const
{
    SoapySDR::logf(SOAPY_SDR_DEBUG, "SoapyLiteXXTRX::readSetting(%s)", key.c_str());

    if (key == "FPGA_TX_RX_LOOPBACK_ENABLE") {
        uint32_t control = litepcie_readl(_fd, CSR_LMS7002M_CONTROL_ADDR);
        control &= 1 << CSR_LMS7002M_CONTROL_TX_RX_LOOPBACK_ENABLE_OFFSET;
        return control ? "TRUE" : "FALSE";
    } else if (key == "FPGA_TX_PATTERN") {
        uint32_t control = litepcie_readl(_fd, CSR_LMS7002M_TX_PATTERN_CONTROL_ADDR);
        control &= 1 << CSR_LMS7002M_TX_PATTERN_CONTROL_ENABLE_OFFSET;
        return control ? "1" : "0";
    } else if (key == "FPGA_RX_PATTERN") {
        uint32_t control = litepcie_readl(_fd, CSR_LMS7002M_RX_PATTERN_CONTROL_ADDR);
        control &= 1 << CSR_LMS7002M_RX_PATTERN_CONTROL_ENABLE_OFFSET;
        return control ? "1" : "0";
    } else if (key == "FPGA_RX_PATTERN_ERRORS") {
        uint32_t errors = litepcie_readl(_fd, CSR_LMS7002M_RX_PATTERN_ERRORS_ADDR);
        return std::to_string(errors);
    } else if (key == "FPGA_TX_DELAY") {
        uint32_t reg = litepcie_readl(_fd, CSR_LMS7002M_DELAY_ADDR);
        uint32_t mask = ((uint32_t)(1 << CSR_LMS7002M_DELAY_TX_DELAY_SIZE)-1);
        uint32_t delay = (reg >> CSR_LMS7002M_DELAY_TX_DELAY_OFFSET) & mask;
        return std::to_string(delay);
    } else if (key == "FPGA_RX_DELAY") {
        uint32_t reg = litepcie_readl(_fd, CSR_LMS7002M_DELAY_ADDR);
        uint32_t mask = ((uint32_t)(1 << CSR_LMS7002M_DELAY_RX_DELAY_SIZE)-1);
        uint32_t delay = (reg >> CSR_LMS7002M_DELAY_RX_DELAY_OFFSET) & mask;
        return std::to_string(delay);
    } else if (key == "DMA_BUFFERS") {
        return "RX hw count: " + std::to_string(_rx_stream.hw_count)
                + " RX sw count: " + std::to_string(_rx_stream.sw_count)
                + " RX user count: " + std::to_string(_rx_stream.user_count)
                + " TX hw count: " + std::to_string(_tx_stream.hw_count)
                + " TX sw count: " + std::to_string(_tx_stream.sw_count)
                + " TX user count: " + std::to_string(_tx_stream.user_count);
    } else
        throw std::runtime_error("SoapyLiteXXTRX::readSetting(" + key + ") unknown key");
}

void SoapyLiteXXTRX::writeSetting(const std::string &key, const std::string &value) {
    SoapySDR::logf(SOAPY_SDR_DEBUG, "SoapyLiteXXTRX::writeSetting(%s, %s)",
                   key.c_str(), value.c_str());

    std::lock_guard<std::mutex> lock(_mutex);

    // undo any changes caused by one of the other keys with these enable calls
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
    } else if (key == "TXTSP_TSG_CONST") {
        const int ampl = std::stoi(value);
        LMS7002M_txtsp_tsg_const(_lms, LMS_CHAB, ampl, 0);
    } else if (key == "TBB_ENABLE_LOOPBACK") {
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
    } else if (key == "TBB_SET_PATH") {
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
    } else if (key == "RBB_SET_PATH") {
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
    } else if (key == "LOOPBACK_ENABLE") {
        SoapySDR::log(SOAPY_SDR_DEBUG, "Setting Digital Loopback");
        if (value == "TRUE") {
            LMS7002M_setup_digital_loopback(_lms);
        } else if (value == "FALSE") {
            LMS7002M_configure_lml_port(_lms, LMS_PORT2, LMS_TX, 1);
            LMS7002M_configure_lml_port(_lms, LMS_PORT1, LMS_RX, 1);
        } else
            throw std::runtime_error("SoapyLiteXXTRX::writeSetting(" + key + ", " +
                                     value + ") unknown value");
    } else if (key == "LOOPBACK_ENABLE_LFSR") {
        SoapySDR::log(SOAPY_SDR_DEBUG, "Setting LFSR Loopback");
        if (value == "TRUE") {
            LMS7002M_setup_digital_loopback_lfsr(_lms);
        } else if (value == "FALSE") {
            LMS7002M_configure_lml_port(_lms, LMS_PORT2, LMS_TX, 1);
            LMS7002M_configure_lml_port(_lms, LMS_PORT1, LMS_RX, 1);
        } else
            throw std::runtime_error("SoapyLiteXXTRX::writeSetting(" + key + ", " +
                                     value + ") unknown value");
    } else if (key == "TRF_ENABLE_LOOPBACK") {
        LMS7002M_trf_enable_loopback(_lms, LMS_CHAB, value == "TRUE");
    } else if (key == "RESET_RX_FIFO") {
        LMS7002M_reset_lml_fifo(_lms, LMS_RX);
    } else if (key == "FPGA_TX_RX_LOOPBACK_ENABLE") {
        SoapySDR::log(SOAPY_SDR_DEBUG, "Setting FPGA TX-RX Loopback");
        uint32_t control = litepcie_readl(_fd, CSR_LMS7002M_CONTROL_ADDR);
        control &= ~(1 << CSR_LMS7002M_CONTROL_TX_RX_LOOPBACK_ENABLE_OFFSET);
        if (value == "TRUE") {
            control |= (1 << CSR_LMS7002M_CONTROL_TX_RX_LOOPBACK_ENABLE_OFFSET);
        } else if (value != "FALSE")
            throw std::runtime_error("SoapyLiteXXTRX::writeSetting(" + key + ", " +
                                     value + ") unknown value");
        litepcie_writel(_fd, CSR_LMS7002M_CONTROL_ADDR, control);
    } else if (key == "FPGA_DMA_LOOPBACK_ENABLE") {
        SoapySDR::log(SOAPY_SDR_DEBUG, "Setting FPGA DMA Loopback");
        if (value == "TRUE")
             dma_set_loopback(_fd, true);
        else if (value == "FALSE")
             dma_set_loopback(_fd, false);
        else
            throw std::runtime_error("SoapyLiteXXTRX::writeSetting(" + key + ", " +
                                     value + ") unknown value");

    } else if (key == "FPGA_TX_PATTERN") {
        SoapySDR::log(SOAPY_SDR_DEBUG, "Setting FPGA TX pattern");
        uint32_t control = litepcie_readl(_fd, CSR_LMS7002M_TX_PATTERN_CONTROL_ADDR);
        control &= ~(1 << CSR_LMS7002M_TX_PATTERN_CONTROL_ENABLE_OFFSET);
        if (value == "1") {
            control |= 1 << CSR_LMS7002M_TX_PATTERN_CONTROL_ENABLE_OFFSET;
        } else if (value != "0")
            throw std::runtime_error("SoapyLiteXXTRX::writeSetting(" + key + ", " +
                                     value + ") unknown value");
        litepcie_writel(_fd, CSR_LMS7002M_TX_PATTERN_CONTROL_ADDR, control);
    } else if (key == "FPGA_RX_PATTERN") {
        SoapySDR::log(SOAPY_SDR_DEBUG, "Setting FPGA RX pattern");
        uint32_t control = litepcie_readl(_fd, CSR_LMS7002M_RX_PATTERN_CONTROL_ADDR);
        control &= ~(1 << CSR_LMS7002M_RX_PATTERN_CONTROL_ENABLE_OFFSET);
        if (value == "1") {
            control |= 1 << CSR_LMS7002M_RX_PATTERN_CONTROL_ENABLE_OFFSET;
        } else if (value != "0")
            throw std::runtime_error("SoapyLiteXXTRX::writeSetting(" + key + ", " +
                                     value + ") unknown value");
        litepcie_writel(_fd, CSR_LMS7002M_RX_PATTERN_CONTROL_ADDR, control);
    } else if (key == "FPGA_TX_DELAY") {
        int delay = std::stoi(value);
        if (delay < 0 || delay > 31)
            throw std::runtime_error("SoapyLiteXXTRX::writeSetting(" + key + ", " +
                                     value + ") invalid value");
        uint32_t reg = litepcie_readl(_fd, CSR_LMS7002M_DELAY_ADDR);
        uint32_t mask = ((uint32_t)(1 << CSR_LMS7002M_DELAY_TX_DELAY_SIZE)-1) << CSR_LMS7002M_DELAY_TX_DELAY_OFFSET;
        litepcie_writel(_fd, CSR_LMS7002M_DELAY_ADDR,
                        (reg & ~mask) | (delay << CSR_LMS7002M_DELAY_TX_DELAY_OFFSET));
    } else if (key == "FPGA_RX_DELAY") {
        int delay = std::stoi(value);
        if (delay < 0 || delay > 31)
            throw std::runtime_error("SoapyLiteXXTRX::writeSetting(" + key + ", " +
                                     value + ") invalid value");
        uint32_t reg = litepcie_readl(_fd, CSR_LMS7002M_DELAY_ADDR);
        uint32_t mask = ((uint32_t)(1 << CSR_LMS7002M_DELAY_RX_DELAY_SIZE)-1) << CSR_LMS7002M_DELAY_RX_DELAY_OFFSET;
        litepcie_writel(_fd, CSR_LMS7002M_DELAY_ADDR,
                        (reg & ~mask) | (delay << CSR_LMS7002M_DELAY_RX_DELAY_OFFSET));
    } else if (key == "DUMP_INI") {
        LMS7002M_dump_ini(_lms, value.c_str());
    } else if (key == "RXTSP_TONE") {
        LMS7002M_rxtsp_tsg_tone_div(_lms, LMS_CHAB, std::stoi(value));
    } else if (key == "TXTSP_TONE") {
        LMS7002M_txtsp_tsg_tone_div(_lms, LMS_CHAB, std::stoi(value));
    } else if (key == "RXTSP_ENABLE") {
        LMS7002M_rxtsp_enable(_lms, LMS_CHAB, value == "TRUE");
    } else if (key == "TXTSP_ENABLE") {
        LMS7002M_txtsp_enable(_lms, LMS_CHAB, value == "TRUE");
    } else
        throw std::runtime_error("SoapyLiteXXTRX::writeSetting(" + key + ", " +
                                 value + ") unknown key");
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

/***********************************************************************
 * Find available devices
 **********************************************************************/

std::string getXTRXIdentification(int fd) {
    char fpga_identification[256];
    for (int i = 0; i < 256; i ++)
        fpga_identification[i] = litepcie_readl(fd, CSR_IDENTIFIER_MEM_BASE + 4 * i);
    return std::string(&fpga_identification[0]);
}

std::string getXTRXSerial(int fd) {
    char serial[32];
    snprintf(serial, 32, "%x%08x",
                litepcie_readl(fd, CSR_DNA_ID_ADDR + 4 * 0),
                litepcie_readl(fd, CSR_DNA_ID_ADDR + 4 * 1));
    return std::string(&serial[0]);
}

std::vector<SoapySDR::Kwargs> findXTRX(const SoapySDR::Kwargs &args) {
    std::vector<SoapySDR::Kwargs> discovered;
    if (args.count("path") != 0) {
        // respect user choice
        int fd = open(args.at("path").c_str(), O_RDWR);
        if (fd < 0)
            throw std::runtime_error("Invalid device path specified (should be an accessible device node)");

        // gather device info
        SoapySDR::Kwargs dev(args);
        dev["device"] = "LiteXXTRX";
        dev["serial"] = getXTRXSerial(fd);
        dev["identification"] = getXTRXIdentification(fd);
        dev["version"] = "1234";
        size_t ofs = 0;
        while (ofs < sizeof(dev["serial"]) and dev["serial"][ofs] == '0') ofs++;
        char label_str[256];
        sprintf(label_str, "%s %s %s %s", dev["device"].c_str(), args.at("path").c_str(),
                dev["serial"].c_str()+ofs, dev["identification"].c_str());
        dev["label"] = label_str;
        close(fd);

        discovered.push_back(dev);
    } else {
        // find all LitePCIe devices
        for (int i = 0; i < 10; i++) {
            std::string path = "/dev/litepcie" + std::to_string(i);
            int fd = open(path.c_str(), O_RDWR);
            if (fd < 0)
                continue;

            // check the FPGA identification to see if this is an XTRX
            std::string fpga_identification = getXTRXIdentification(fd);
            if (strstr(fpga_identification.c_str(), "LiteX SoC on Fairwaves XTRX") != NULL) {
                // gather device info
                SoapySDR::Kwargs dev(args);
                dev["device"] = "LiteXXTRX";
                dev["path"] = path;
                dev["serial"] = getXTRXSerial(fd);
                dev["identification"] = &fpga_identification[0];
                dev["version"] = "1234";
                size_t ofs = 0;
                while (ofs < sizeof(dev["serial"]) and dev["serial"][ofs] == '0') ofs++;
                char label_str[256];
                sprintf(label_str, "%s %s %s %s", dev["device"].c_str(), path.c_str(),
                    dev["serial"].c_str()+ofs, dev["identification"].c_str());
                dev["label"] = label_str;
                close(fd);

                // filter by serial if specified
                if (args.count("serial") != 0) {
                    // filter on serial number
                    if (args.at("serial") != dev["serial"])
                        continue;
                }

                discovered.push_back(dev);
            }
        }
    }

    return discovered;
}


/***********************************************************************
 * Make device instance
 **********************************************************************/

SoapySDR::Device *makeLiteXXTRX(const SoapySDR::Kwargs &args) {
    return new SoapyLiteXXTRX(args);
}


/***********************************************************************
 * Registration
 **********************************************************************/

static SoapySDR::Registry registerLiteXXTRX("LiteXXTRX", &findXTRX, &makeLiteXXTRX,
                                       SOAPY_SDR_ABI_VERSION);
