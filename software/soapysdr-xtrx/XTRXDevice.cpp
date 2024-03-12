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
#include <sys/mman.h>

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

void dma_init_cpu(int fd) {
    struct litepcie_ioctl_dma_init m;
    m.use_gpu = 0;
    checked_ioctl(fd, LITEPCIE_IOCTL_DMA_INIT, &m);
}

void dma_set_loopback(int fd, bool loopback_enable) {
    struct litepcie_ioctl_dma m;
    m.loopback_enable = loopback_enable ? 1 : 0;
    checked_ioctl(fd, LITEPCIE_IOCTL_DMA, &m);
}

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
    writeSetting("FPGA_TX_PATTERN", "1");
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
    if (_lms == NULL)
        throw std::runtime_error(
            "SoapyLiteXXTRX(): failed to LMS7002M_create()");
    LMS7002M_reset(_lms);
    LMS7002M_set_spi_mode(_lms, 4);

    // read info register
    LMS7002M_regs_spi_read(_lms, 0x002f);
    SoapySDR::logf(SOAPY_SDR_INFO, "LMS7002M info: revision %d, version %d",
                   LMS7002M_regs(_lms)->reg_0x002f_rev,
                   LMS7002M_regs(_lms)->reg_0x002f_ver);

    // set clock to Internal Reference Clock
    this->setClockSource("internal");

    // configure data port directions and data clock rates
    LMS7002M_configure_lml_port(_lms, LMS_PORT2, LMS_TX, 1);
    LMS7002M_configure_lml_port(_lms, LMS_PORT1, LMS_RX, 1);

    // enable components
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

    // XTRX-specific configuration
    LMS7002M_ldo_enable(_lms, true, LMS7002M_LDO_ALL);
    LMS7002M_xbuf_share_tx(_lms, true);

    // turn the clocks on (tested frequencies: 61.44MHZ, 122.88MHZ)
    this->setMasterClockRate(122.88e6);

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
        this->setGain(SOAPY_SDR_RX, i, "LNA", 30.0);
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
    dma_init_cpu(_fd);
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
    LMS7002M_xbuf_share_tx(_lms, false);
    LMS7002M_ldo_enable(_lms, false, LMS7002M_LDO_ALL);
    LMS7002M_power_down(_lms);
    LMS7002M_destroy(_lms);
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
        LMS7002M_rfe_set_path(_lms, ch2LMS(channel), path);
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
        LMS7002M_trf_select_band(_lms, ch2LMS(channel), band);
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
        LMS7002M_rxtsp_set_dc_correction(_lms, ch2LMS(channel), automatic,
                                         7 /*max*/);
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

    if (direction == SOAPY_SDR_TX) {
        LMS7002M_txtsp_set_dc_correction(_lms, ch2LMS(channel), offset.real(),
                                         offset.imag());
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

    if (direction == SOAPY_SDR_TX) {
        LMS7002M_txtsp_set_iq_correction(_lms, ch2LMS(channel),
                                         std::arg(balance), std::abs(balance));
    } else {
        LMS7002M_rxtsp_set_iq_correction(_lms, ch2LMS(channel),
                                         std::arg(balance), std::abs(balance));
    }
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

    SoapySDR::logf(SOAPY_SDR_DEBUG, "SoapyLiteXXTRX::setGain(%s, ch%d, %s, %f dB)",
                   dir2Str(direction), channel, name.c_str(), value);

    double &actualValue = _cachedGainValues[direction][channel][name];

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
        int ret = LMS7002M_set_lo_freq(_lms, dir2LMS(direction), _refClockRate,
                                       frequency, &actualFreq);
        if (ret != 0)
            throw std::runtime_error("SoapyLiteXXTRX::setFrequency(" +
                                     std::to_string(frequency / 1e6) +
                                     " MHz) failed - " + std::to_string(ret));
        _cachedFreqValues[direction][0][name] = actualFreq;
        _cachedFreqValues[direction][1][name] = actualFreq;
    }

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

void SoapyLiteXXTRX::setSampleRate(const int direction, const size_t,
                              const double rate) {
    std::lock_guard<std::mutex> lock(_mutex);

    const double baseRate = this->getTSPRate(direction);
    const double factor = baseRate / rate;
    SoapySDR::logf(
        SOAPY_SDR_DEBUG,
        "SoapyLiteXXTRX::setSampleRate(%s, %f MHz), baseRate %f MHz, factor %f",
        dir2Str(direction), rate / 1e6, baseRate / 1e6, factor);
    if (factor < 2.0)
        throw std::runtime_error("SoapyLiteXXTRX::setSampleRate() -- rate too high");
    int intFactor = 1 << int((std::log(factor) / std::log(2.0)) + 0.5);
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
}

double SoapyLiteXXTRX::getSampleRate(const int direction, const size_t) const {
    return _cachedSampleRates.at(direction);
}

std::vector<double> SoapyLiteXXTRX::listSampleRates(const int direction,
                                               const size_t) const {
    const double baseRate = this->getTSPRate(direction);
    std::vector<double> rates;
    // from baseRate/32 to baseRate/2
    for (int i = 5; i >= 1; i--) {
        rates.push_back(baseRate / (1 << i));
    }
    // GGM: hack
    rates.push_back(30720000/4);
    rates.push_back(30720000/2);
    rates.push_back(30720000);
    return rates;
}

std::vector<std::string> SoapyLiteXXTRX::getStreamFormats(const int /*direction*/,
                                                     const size_t /*channel*/) const
{
    std::vector<std::string> formats;
    formats.push_back(SOAPY_SDR_CS16);
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

    int ret = 0;
    double &actualBw = _cachedFilterBws[direction][channel];
    if (direction == SOAPY_SDR_RX) {
        //ret = LMS7002M_rbb_set_filter_bw(_lms, ch2LMS(channel), bw, &actualBw);
        ret = LMS7002M_mcu_calibration_rx(_lms, _refClockRate, bw);
        if (ret == 0)
            actualBw = bw;
    }
    if (direction == SOAPY_SDR_TX) {
        //ret = LMS7002M_tbb_set_filter_bw(_lms, ch2LMS(channel), bw, &actualBw);
        ret = LMS7002M_mcu_calibration_tx(_lms, _refClockRate, bw);
        if (ret == 0)
            actualBw = bw;
    }

    if (ret != 0)
        throw std::runtime_error("SoapyLiteXXTRX::setBandwidth(" +
                                 std::to_string(bw / 1e6) + " MHz) failed - " +
                                 std::to_string(ret));
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

    int ret =
        LMS7002M_set_data_clock(_lms, _refClockRate, rate, &_masterClockRate);
    if (ret != 0) {
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
        dev["serial"] = getXTRXSerial(fd);
        dev["identification"] = getXTRXIdentification(fd);
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
                dev["path"] = path;
                dev["serial"] = getXTRXSerial(fd);
                dev["identification"] = &fpga_identification[0];
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