#include <cmath>

#include <SoapySDR/Logger.hpp>

#include "LiteXXTRXDevice.hpp"

#include <lms7002mNG/OpStatus.h>
#include <lms7002mNG/LMS7002M_parameters.h>

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
