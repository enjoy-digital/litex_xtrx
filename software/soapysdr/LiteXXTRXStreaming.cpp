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

#include <chrono>
#include <cassert>
#include <thread>
#include <sys/mman.h>

#include "LiteXXTRXDevice.hpp"

/* Setup and configure a stream for RX or TX. */
SoapySDR::Stream *SoapyLiteXXTRX::setupStream(const int direction,
                                              const std::string &format,
                                              const std::vector<size_t> &channels,
                                              const SoapySDR::Kwargs &/*args*/) {
    std::lock_guard<std::mutex> lock(_mutex);

    if (direction == SOAPY_SDR_RX) {
        if (_rx_stream.opened) {
            throw std::runtime_error("RX stream already opened.");
        }

        /* Configure the file descriptor watcher. */
        _rx_stream.fds.fd = _fd;
        _rx_stream.fds.events = POLLIN;

        /* Initialize the DMA engine. */
        if ((litepcie_request_dma(_fd, 0, 1) == 0)) {
            throw std::runtime_error("DMA not available.");
        }

        /* Memory-map the DMA buffers. */
        _rx_stream.buf = mmap(NULL,
                              _dma_mmap_info.dma_rx_buf_count * _dma_mmap_info.dma_rx_buf_size,
                              PROT_READ | PROT_WRITE, MAP_SHARED, _fd,
                              _dma_mmap_info.dma_rx_buf_offset);
        if (_rx_stream.buf == MAP_FAILED) {
            throw std::runtime_error("MMAP failed.");
        }

        /* Ensure the DMA is disabled initially to avoid counters being in a bad state. */
        litepcie_dma_writer(_fd, 0, &_rx_stream.hw_count, &_rx_stream.sw_count);

        _rx_stream.opened = true;
        _rx_stream.format = format;

        /* Set channels to 0 and 1 if none are provided. */
        if (channels.empty()) {
            _rx_stream.channels = {0, 1};
        } else {
            _rx_stream.channels = channels;
        }

        return RX_STREAM;
    } else if (direction == SOAPY_SDR_TX) {
        if (_tx_stream.opened) {
            throw std::runtime_error("TX stream already opened.");
        }

        /* Configure the file descriptor watcher. */
        _tx_stream.fds.fd = _fd;
        _tx_stream.fds.events = POLLOUT;

        /* Initialize the DMA engine. */
        if ((litepcie_request_dma(_fd, 1, 0) == 0)) {
            throw std::runtime_error("DMA not available.");
        }

        /* Memory-map the DMA buffers. */
        _tx_stream.buf = mmap(NULL,
                              _dma_mmap_info.dma_tx_buf_count * _dma_mmap_info.dma_tx_buf_size,
                              PROT_WRITE, MAP_SHARED, _fd,
                              _dma_mmap_info.dma_tx_buf_offset);
        if (_tx_stream.buf == MAP_FAILED) {
            throw std::runtime_error("MMAP failed.");
        }

        /* Ensure the DMA is disabled initially to avoid counters being in a bad state. */
        litepcie_dma_reader(_fd, 0, &_tx_stream.hw_count, &_tx_stream.sw_count);

        _tx_stream.opened = true;
        _tx_stream.format = format;

        /* Set channels to 0 and 1 if none are provided. */
        if (channels.empty()) {
            _tx_stream.channels = {0, 1};
        } else {
            _tx_stream.channels = channels;
        }

        return TX_STREAM;
    } else {
        throw std::runtime_error("Invalid direction.");
    }
}

/* Close the specified stream and release associated resources. */
void SoapyLiteXXTRX::closeStream(SoapySDR::Stream *stream) {
    std::lock_guard<std::mutex> lock(_mutex);

    if (stream == RX_STREAM) {
        /* Release the DMA engine. */
        litepcie_release_dma(_fd, 0, 1);

        munmap(_rx_stream.buf,
               _dma_mmap_info.dma_rx_buf_size * _dma_mmap_info.dma_rx_buf_count);
        _rx_stream.opened = false;
    } else if (stream == TX_STREAM) {
        /* Release the DMA engine. */
        litepcie_release_dma(_fd, 1, 0);

        munmap(_tx_stream.buf,
               _dma_mmap_info.dma_tx_buf_size * _dma_mmap_info.dma_tx_buf_count);
        _tx_stream.opened = false;
    }
}

/* Activate the specified stream (enable DMA engine). */
int SoapyLiteXXTRX::activateStream(SoapySDR::Stream *stream, const int /*flags*/,
                                   const long long /*timeNs*/,
                                   const size_t /*numElems*/) {
    if (stream == RX_STREAM) {
        /* Enable the DMA engine for RX. */
        litepcie_dma_writer(_fd, 1, &_rx_stream.hw_count, &_rx_stream.sw_count);
        _rx_stream.user_count = 0;
    } else if (stream == TX_STREAM) {
        /* Enable the DMA engine for TX. */
        litepcie_dma_reader(_fd, 1, &_tx_stream.hw_count, &_tx_stream.sw_count);
        _tx_stream.user_count = 0;
    }

    return 0;
}

/* Deactivate the specified stream (disable DMA engine). */
int SoapyLiteXXTRX::deactivateStream(SoapySDR::Stream *stream, const int /*flags*/,
                                     const long long /*timeNs*/) {
    if (stream == RX_STREAM) {
        /* Disable the DMA engine for RX. */
        litepcie_dma_writer(_fd, 0, &_rx_stream.hw_count, &_rx_stream.sw_count);
    } else if (stream == TX_STREAM) {
        /* Disable the DMA engine for TX. */
        litepcie_dma_reader(_fd, 0, &_tx_stream.hw_count, &_tx_stream.sw_count);
    }
    return 0;
}

/*******************************************************************
 * Direct buffer API
 ******************************************************************/

/* Retrieve the maximum transmission unit (MTU) for a stream. */
size_t SoapyLiteXXTRX::getStreamMTU(SoapySDR::Stream *stream) const {
    if (stream == RX_STREAM) {
        /* Each sample is 2 * Complex{Int16}. */
        return _dma_mmap_info.dma_rx_buf_size / (2 * 2 * sizeof(int16_t));
    } else if (stream == TX_STREAM) {
        return _dma_mmap_info.dma_tx_buf_size / (2 * 2 * sizeof(int16_t));
    } else {
        throw std::runtime_error("SoapySDR::getStreamMTU(): Invalid stream.");
    }
}

/* Retrieve the number of direct access buffers available for a stream. */
size_t SoapyLiteXXTRX::getNumDirectAccessBuffers(SoapySDR::Stream *stream) {
    if (stream == RX_STREAM) {
        return _dma_mmap_info.dma_rx_buf_count;
    } else if (stream == TX_STREAM) {
        return _dma_mmap_info.dma_tx_buf_count;
    } else {
        throw std::runtime_error("SoapySDR::getNumDirectAccessBuffers(): Invalid stream.");
    }
}

/* Retrieve buffer addresses for a direct access buffer. */
int SoapyLiteXXTRX::getDirectAccessBufferAddrs(SoapySDR::Stream *stream,
                                               const size_t handle, void **buffs) {
    if (stream == RX_STREAM) {
        buffs[0] = (char *)_rx_stream.buf + handle * _dma_mmap_info.dma_rx_buf_size;
    } else if (stream == TX_STREAM) {
        buffs[0] = (char *)_tx_stream.buf + handle * _dma_mmap_info.dma_tx_buf_size;
    } else {
        throw std::runtime_error("SoapySDR::getDirectAccessBufferAddrs(): Invalid stream.");
    }
    return 0;
}

/***************************************************************************************************
 * DMA Buffer Management
 *
 * The DMA readers/writers utilize a zero-copy mechanism (i.e., a single buffer shared
 * with the kernel) and employ three counters to index that buffer:
 * - hw_count: Indicates the position where the hardware has read from or written to.
 * - sw_count: Indicates the position where userspace has read from or written to.
 * - user_count: Indicates the current position where userspace is reading from or
 *   writing to.
 *
 * The distinction between sw_count and user_count enables tracking of which buffers are
 * currently being processed. This feature is not directly supported by the LitePCIe DMA
 * library, so it is implemented separately.
 *
 * Separating user_count enables advancing read/write buffers without requiring a syscall
 * (interfacing with the kernel only when retiring buffers). However, this can result in
 * slower detection of overflows and underflows, so overflow/underflow detection is made
 * configurable.
 **************************************************************************************************/

#define DETECT_EVERY_OVERFLOW  true  /* Detect overflow every time it occurs. */
#define DETECT_EVERY_UNDERFLOW true  /* Detect underflow every time it occurs. */

/* Acquire a buffer for reading. */
int SoapyLiteXXTRX::acquireReadBuffer(SoapySDR::Stream *stream, size_t &handle,
                                      const void **buffs, int &flags,
                                      long long &/*timeNs*/, const long timeoutUs) {
    if (stream != RX_STREAM) {
        return SOAPY_SDR_STREAM_ERROR;
    }

    /* Check if there are buffers available. */
    int buffers_available = _rx_stream.hw_count - _rx_stream.user_count;
    assert(buffers_available >= 0);

    /* If not, check with the DMA engine. */
    if (buffers_available == 0 || DETECT_EVERY_OVERFLOW) {
        litepcie_dma_writer(_fd, 1, &_rx_stream.hw_count, &_rx_stream.sw_count);
        buffers_available = _rx_stream.hw_count - _rx_stream.user_count;
    }

    /* If no buffers available, wait for new buffers to arrive. */
    if (buffers_available == 0) {
        if (timeoutUs == 0) {
            return SOAPY_SDR_TIMEOUT;
        }
        int ret = poll(&_rx_stream.fds, 1, timeoutUs / 1000);
        if (ret < 0) {
            throw std::runtime_error("SoapyLiteXXTRX::acquireReadBuffer(): Poll failed, " +
                                     std::string(strerror(errno)) + ".");
        } else if (ret == 0) {
            return SOAPY_SDR_TIMEOUT;
        }

        /* Get new DMA counters. */
        litepcie_dma_writer(_fd, 1, &_rx_stream.hw_count, &_rx_stream.sw_count);
        buffers_available = _rx_stream.hw_count - _rx_stream.user_count;
        assert(buffers_available > 0);
    }

    /* Detect overflows of the underlying circular buffer. */
    if ((_rx_stream.hw_count - _rx_stream.sw_count) >
        ((int64_t)_dma_mmap_info.dma_rx_buf_count / 2)) {
        /* Drain all buffers to get out of the overflow quicker. */
        struct litepcie_ioctl_mmap_dma_update mmap_dma_update;
        mmap_dma_update.sw_count = _rx_stream.hw_count;
        checked_ioctl(_fd, LITEPCIE_IOCTL_MMAP_DMA_WRITER_UPDATE, &mmap_dma_update);
        _rx_stream.user_count = _rx_stream.hw_count;
        _rx_stream.sw_count = _rx_stream.hw_count;
        handle = -1;

        flags |= SOAPY_SDR_END_ABRUPT;
        return SOAPY_SDR_OVERFLOW;
    } else {
        /* Get the buffer. */
        int buf_offset = _rx_stream.user_count % _dma_mmap_info.dma_rx_buf_count;
        getDirectAccessBufferAddrs(stream, buf_offset, (void **)buffs);

        /* Update the DMA counters. */
        handle = _rx_stream.user_count;
        _rx_stream.user_count++;

        return getStreamMTU(stream);
    }
}

/* Release a read buffer after use. */
void SoapyLiteXXTRX::releaseReadBuffer(SoapySDR::Stream */*stream*/, size_t handle) {
    assert(handle != (size_t)-1 && "Attempt to release an invalid buffer (e.g., from an overflow).");

    /* Update the DMA counters. */
    struct litepcie_ioctl_mmap_dma_update mmap_dma_update;
    mmap_dma_update.sw_count = handle + 1;
    checked_ioctl(_fd, LITEPCIE_IOCTL_MMAP_DMA_WRITER_UPDATE, &mmap_dma_update);
}

/* Acquire a buffer for writing. */
int SoapyLiteXXTRX::acquireWriteBuffer(SoapySDR::Stream *stream, size_t &handle,
                                       void **buffs, const long timeoutUs) {
    if (stream != TX_STREAM) {
        return SOAPY_SDR_STREAM_ERROR;
    }

    /* Check if there are buffers available. */
    int buffers_pending = _tx_stream.user_count - _tx_stream.hw_count;
    assert(buffers_pending <= (int)_dma_mmap_info.dma_tx_buf_count);

    /* If not, check with the DMA engine. */
    if (buffers_pending == ((int64_t)_dma_mmap_info.dma_tx_buf_count) || DETECT_EVERY_UNDERFLOW) {
        litepcie_dma_reader(_fd, 1, &_tx_stream.hw_count, &_tx_stream.sw_count);
        buffers_pending = _tx_stream.user_count - _tx_stream.hw_count;
    }

    /* If no buffers available, wait for new buffers to become available. */
    if (buffers_pending == ((int64_t)_dma_mmap_info.dma_tx_buf_count)) {
        if (timeoutUs == 0) {
            return SOAPY_SDR_TIMEOUT;
        }
        int ret = poll(&_tx_stream.fds, 1, timeoutUs / 1000);
        if (ret < 0) {
            throw std::runtime_error("SoapyLiteXXTRX::acquireWriteBuffer(): Poll failed, " +
                                     std::string(strerror(errno)) + ".");
        } else if (ret == 0) {
            return SOAPY_SDR_TIMEOUT;
        }

        /* Get new DMA counters. */
        litepcie_dma_reader(_fd, 1, &_tx_stream.hw_count, &_tx_stream.sw_count);
        buffers_pending = _tx_stream.user_count - _tx_stream.hw_count;
        assert(buffers_pending < ((int64_t)_dma_mmap_info.dma_tx_buf_count));
    }

    /* Get the buffer. */
    int buf_offset = _tx_stream.user_count % _dma_mmap_info.dma_tx_buf_count;
    getDirectAccessBufferAddrs(stream, buf_offset, buffs);

    /* Update the DMA counters. */
    handle = _tx_stream.user_count;
    _tx_stream.user_count++;

    /* Detect underflows. */
    if (buffers_pending < 0) {
        return SOAPY_SDR_UNDERFLOW;
    } else {
        return getStreamMTU(stream);
    }
}

/* Release a write buffer after use. */
void SoapyLiteXXTRX::releaseWriteBuffer(SoapySDR::Stream */*stream*/, size_t handle,
                                        const size_t /*numElems*/, int &/*flags*/,
                                        const long long /*timeNs*/) {
    /* XXX: Inspect user-provided numElems and flags, and act upon them? */

    /* Update the DMA counters so that the engine can submit this buffer. */
    struct litepcie_ioctl_mmap_dma_update mmap_dma_update;
    mmap_dma_update.sw_count = handle + 1;
    checked_ioctl(_fd, LITEPCIE_IOCTL_MMAP_DMA_READER_UPDATE, &mmap_dma_update);
}

/* Interleave CS16 samples. */
void interleaveCS16(const int16_t *src, int8_t *dst, uint32_t len, size_t offset) {
    int16_t *dst_int16 = reinterpret_cast<int16_t*>(dst) + (offset * 4);
    const int16_t *samples_cs16 = src + (offset * BYTES_PER_SAMPLE);
    for (uint32_t i = 0; i < len; i++) {
        dst_int16[0] = samples_cs16[0]; /* I. */
        dst_int16[1] = samples_cs16[1]; /* Q. */
        samples_cs16 += 2;
        dst_int16 += 4;
    }
}

/* Interleave CF32 samples. */
void interleaveCF32(const float *src, int8_t *dst, uint32_t len, size_t offset) {
    int16_t *dst_int16 = reinterpret_cast<int16_t*>(dst) + (offset * 4);
    const float *samples_cf32 = src + (offset * BYTES_PER_SAMPLE);
    for (uint32_t i = 0; i < len; i++) {
        dst_int16[0] = static_cast<int16_t>(samples_cf32[0] * 2047.0); /* I. */
        dst_int16[1] = static_cast<int16_t>(samples_cf32[1] * 2047.0); /* Q. */
        samples_cf32 += 2;
        dst_int16 += 4;
    }
}

/* Generic interleave function. */
void interleave(const void *src, void *dst, uint32_t len, const std::string &format, size_t offset) {
    if (format == SOAPY_SDR_CS16) {
        interleaveCS16(reinterpret_cast<const int16_t*>(src), reinterpret_cast<int8_t*>(dst), len, offset);
    } else if (format == SOAPY_SDR_CF32) {
        interleaveCF32(reinterpret_cast<const float*>(src), reinterpret_cast<int8_t*>(dst), len, offset);
    } else {
        SoapySDR_logf(SOAPY_SDR_ERROR, "Unsupported format: %s.", format.c_str());
    }
}

/* Deinterleave CS16 samples. */
void deinterleaveCS16(const int8_t *src, int16_t *dst, uint32_t len, size_t offset) {
    int16_t *samples_cs16 = dst + (offset * BYTES_PER_SAMPLE);
    const int16_t *src_int16 = reinterpret_cast<const int16_t*>(src);
    for (uint32_t i = 0; i < len; i++) {
        samples_cs16[0] = src_int16[0]; /* I. */
        samples_cs16[1] = src_int16[1]; /* Q. */
        samples_cs16 += 2;
        src_int16 += 4;
    }
}

/* Deinterleave CF32 samples. */
void deinterleaveCF32(const int8_t *src, float *dst, uint32_t len, size_t offset) {
    float *samples_cf32 = dst + (offset * BYTES_PER_SAMPLE);
    const int16_t *src_int16 = reinterpret_cast<const int16_t*>(src);
    for (uint32_t i = 0; i < len; i++) {
        samples_cf32[0] = static_cast<float>(src_int16[0]) / 2047.0; /* I. */
        samples_cf32[1] = static_cast<float>(src_int16[1]) / 2047.0; /* Q. */
        samples_cf32 += 2;
        src_int16 += 4;
    }
}

/* Generic deinterleave function. */
void deinterleave(const void *src, void *dst, uint32_t len, const std::string &format, size_t offset) {
    if (format == SOAPY_SDR_CS16) {
        deinterleaveCS16(reinterpret_cast<const int8_t*>(src), reinterpret_cast<int16_t*>(dst), len, offset);
    } else if (format == SOAPY_SDR_CF32) {
        deinterleaveCF32(reinterpret_cast<const int8_t*>(src), reinterpret_cast<float*>(dst), len, offset);
    } else {
        SoapySDR_logf(SOAPY_SDR_ERROR, "Unsupported format: %s.", format.c_str());
    }
}

/* Read from the RX stream. */
int SoapyLiteXXTRX::readStream(SoapySDR::Stream *stream,
                               void *const *buffs,
                               const size_t numElems,
                               int &flags,
                               long long &timeNs,
                               const long timeoutUs) {
    if (stream != RX_STREAM) {
        return SOAPY_SDR_NOT_SUPPORTED;
    }

    /* Determine the number of samples to return, respecting the MTU. */
    size_t returnedElems = std::min(numElems, this->getStreamMTU(stream));
    const uint32_t cplx_size = BYTES_PER_SAMPLE * 2;

    size_t samp_avail = 0;

    /* If there's a remainder buffer from a previous read, process that first. */
    if (_rx_stream.remainderHandle >= 0) {
        const size_t n = std::min(_rx_stream.remainderSamps, returnedElems);
        const uint32_t remainderOffset = _rx_stream.remainderOffset * cplx_size * 2;

        if (n < returnedElems) {
            samp_avail = n;
        }

        /* Read out channels from the remainder buffer. */
        for (size_t i = 0; i < _rx_stream.channels.size(); i++) {
            const uint32_t chan = _rx_stream.channels[i];
            deinterleave(_rx_stream.remainderBuff + (remainderOffset + chan * cplx_size),
                         buffs[i], n, _rx_stream.format, 0);
        }
        _rx_stream.remainderSamps -= n;
        _rx_stream.remainderOffset += n;

        if (_rx_stream.remainderSamps == 0) {
            this->releaseReadBuffer(stream, _rx_stream.remainderHandle);
            _rx_stream.remainderHandle = -1;
            _rx_stream.remainderOffset = 0;
        }

        if (n == returnedElems) {
            return returnedElems;
        }
    }

    /* Acquire a new read buffer from the DMA engine. */
    size_t handle;
    int ret = this->acquireReadBuffer(stream, handle, (const void **)&_rx_stream.remainderBuff, flags, timeNs, timeoutUs);

    if (ret < 0) {
        if ((ret == SOAPY_SDR_TIMEOUT) && (samp_avail > 0)) {
            return samp_avail;
        }
        return ret;
    }

    _rx_stream.remainderHandle = handle;
    _rx_stream.remainderSamps = ret;

    const size_t n = std::min((returnedElems - samp_avail), _rx_stream.remainderSamps);

    /* Read out channels from the new buffer. */
    for (size_t i = 0; i < _rx_stream.channels.size(); i++) {
        const uint32_t chan = _rx_stream.channels[i];
        deinterleave(_rx_stream.remainderBuff + (chan * cplx_size),
                     buffs[i], n, _rx_stream.format, samp_avail);
    }
    _rx_stream.remainderSamps -= n;
    _rx_stream.remainderOffset += n;

    if (_rx_stream.remainderSamps == 0) {
        this->releaseReadBuffer(stream, _rx_stream.remainderHandle);
        _rx_stream.remainderHandle = -1;
        _rx_stream.remainderOffset = 0;
    }

    return returnedElems;
}

/* Write to the TX stream. */
int SoapyLiteXXTRX::writeStream(SoapySDR::Stream *stream,
                                const void *const *buffs,
                                const size_t numElems,
                                int &flags,
                                const long long timeNs,
                                const long timeoutUs) {
    if (stream != TX_STREAM) {
        return SOAPY_SDR_NOT_SUPPORTED;
    }

    /* Determine the number of samples to return, respecting the MTU. */
    size_t returnedElems = std::min(numElems, this->getStreamMTU(stream));
    const uint32_t cplx_size = BYTES_PER_SAMPLE * 2;

    size_t samp_avail = 0;

    /* If there's a remainder buffer from a previous write, process that first. */
    if (_tx_stream.remainderHandle >= 0) {
        const size_t n = std::min(_tx_stream.remainderSamps, returnedElems);
        const uint32_t remainderOffset = _tx_stream.remainderOffset * cplx_size * 2;

        if (n < returnedElems) {
            samp_avail = n;
        }

        /* Write out channels to the remainder buffer. */
        for (size_t i = 0; i < _tx_stream.channels.size(); i++) {
            interleave(buffs[i], _tx_stream.remainderBuff + remainderOffset + ((cplx_size * _tx_stream.channels[i])),
                       n, _tx_stream.format, 0);
        }
        _tx_stream.remainderSamps -= n;
        _tx_stream.remainderOffset += n;

        if (_tx_stream.remainderSamps == 0) {
            this->releaseWriteBuffer(stream, _tx_stream.remainderHandle, _tx_stream.remainderOffset, flags, timeNs);
            _tx_stream.remainderHandle = -1;
            _tx_stream.remainderOffset = 0;
        }

        if (n == returnedElems) {
            return returnedElems;
        }
    }

    /* Acquire a new write buffer from the DMA engine. */
    size_t handle;

    int ret = this->acquireWriteBuffer(stream, handle, (void **)&_tx_stream.remainderBuff, timeoutUs);
    if (ret < 0) {
        if ((ret == SOAPY_SDR_TIMEOUT) && (samp_avail > 0)) {
            return samp_avail;
        }
        return ret;
    }

    _tx_stream.remainderHandle = handle;
    _tx_stream.remainderSamps = ret;

    const size_t n = std::min((returnedElems - samp_avail), _tx_stream.remainderSamps);

    /* Write out channels to the new buffer. */
    for (size_t i = 0; i < _tx_stream.channels.size(); i++) {
        interleave(buffs[i], _tx_stream.remainderBuff + (cplx_size * _tx_stream.channels[i]), n, _tx_stream.format, samp_avail);
    }
    _tx_stream.remainderSamps -= n;
    _tx_stream.remainderOffset += n;

    if (_tx_stream.remainderSamps == 0) {
        this->releaseWriteBuffer(stream, _tx_stream.remainderHandle, _tx_stream.remainderOffset, flags, timeNs);
        _tx_stream.remainderHandle = -1;
        _tx_stream.remainderOffset = 0;
    }

    return returnedElems;
}
