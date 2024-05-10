#!/usr/bin/env python
"""Simple signal generator for testing transmit

Continuously output a carrier with single sideband sinusoid amplitude
modulation.

Terminate with cntl-C.
"""

import argparse
import math
import signal
import time

import numpy as np

import SoapySDR
from SoapySDR import * #SOAPY_SDR_ constants

import soapy_log_handle

def siggen_app(args, rate,
        filename   = "tx.bin",
        freq       = None,
        tx_bw      = None,
        tx_chan    = 0,
        tx_gain    = None,
        tx_ant     = None,
        clock_rate = None,
        wave_freq  = None
):
    """Generate signal until an interrupt signal is received."""

    if wave_freq is None:
        wave_freq = rate / 10

    sdr = SoapySDR.Device(args)
    #set clock rate first
    if clock_rate is not None:
        sdr.setMasterClockRate(clock_rate)

    #set sample rate
    sdr.setSampleRate(SOAPY_SDR_TX, tx_chan, rate)
    print("Actual Tx Rate %f Msps"%(sdr.getSampleRate(SOAPY_SDR_TX, tx_chan) / 1e6))

    #set bandwidth
    if tx_bw is not None:
        sdr.setBandwidth(SOAPY_SDR_TX, tx_chan, tx_bw)

    #set antenna
    print("Set the antenna")
    if tx_ant is not None:
        sdr.setAntenna(SOAPY_SDR_TX, tx_chan, tx_ant)

    #set overall gain
    print("Set the gain")
    if tx_gain is not None:
        sdr.setGain(SOAPY_SDR_TX, tx_chan, tx_gain)

    #tune frontends
    print("Tune the frontend")
    if freq is not None:
        sdr.setFrequency(SOAPY_SDR_TX, tx_chan, freq)

    print("Create Tx stream")
    tx_stream = sdr.setupStream(SOAPY_SDR_TX, SOAPY_SDR_CF32, [tx_chan])
    d_mtu = sdr.getStreamMTU(tx_stream);
    print("Activate Tx Stream")
    sdr.activateStream(tx_stream)
    samps_chan = np.fromfile(filename, dtype="int16").astype(np.float32).view(np.complex64)

    # normalize
    samps_chan = samps_chan - np.mean(samps_chan)
    samps_max  = np.max(np.absolute(samps_chan))
    samps_chan = samps_chan / samps_max

    time_last_print = time.time()
    total_samps = 0

    state = dict(running=True)

    def signal_handler(signum, _):
        print('Signal handler called with signal {}'.format(signum))
        state['running'] = False

    signal.signal(signal.SIGINT, signal_handler)

    while state['running']:
        status = sdr.writeStream(tx_stream, [samps_chan], samps_chan.size, timeoutUs=1000000)
        if status.ret == -7:
            print("sU")
            continue
        if status.ret != samps_chan.size:
            print(f"{samps_chan.size} {status.ret}")
            raise Exception("Expected writeStream() to consume all samples! %d" % status.ret)
        total_samps += status.ret

        if time.time() > time_last_print + 5.0:
            rate = total_samps / (time.time() - time_last_print) / 1e6
            print("Python siggen rate: %f Msps" % rate)
            total_samps = 0
            time_last_print = time.time()

    #cleanup streams
    print("Cleanup stream")
    sdr.deactivateStream(tx_stream)
    sdr.closeStream(tx_stream)
    print("Done!")

def main():
    """Parse command line arguments and start sig-gen."""
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)

    parser.add_argument("--args",       type=str,       help="device factor arguments",      default="")
    parser.add_argument("--rate",       type=float,     help="Tx and Rx sample rate",        default=5e6)
    parser.add_argument("--tx-ant",     type=str,       help="Optional Tx antenna",          default="BAND2")
    parser.add_argument("--tx-gain",    type=float,     help="Optional Tx gain (dB)",        default=40)
    parser.add_argument("--tx-chan",    type=int,       help="Transmitter channel (def=0)",  default=0)
    parser.add_argument("--freq",       type=float,     help="Optional Tx and Rx freq (Hz)", default=100e6)
    parser.add_argument("--tx-bw",      type=float,     help="Optional Tx filter bw (Hz)",   default=20e6)
    parser.add_argument("--wave-freq",  type=float,     help="Baseband waveform freq (Hz)")
    parser.add_argument("--clock-rate", type=float,     help="Optional clock rate (Hz)")
    parser.add_argument("--debug", action='store_true', help="Output debug messages")
    parser.add_argument("--filename",   type=str,       help="Lut to emit.",                 default="tx.bin")
    parser.add_argument(
        "--abort-on-error", action='store_true',
        help="Halts operations if the SDR logs an error")

    options = parser.parse_args()

    if options.abort_on_error:
        exception_level = SOAPY_SDR_WARNING
    else:
        exception_level = None
    soapy_log_handle.set_python_log_handler(exception_level=exception_level)
    if options.debug:
        SoapySDR.setLogLevel(SOAPY_SDR_DEBUG)

    siggen_app(
        args       = options.args,
        rate       = options.rate,
        filename   = options.filename,
        freq       = options.freq,
        tx_bw      = options.tx_bw,
        tx_ant     = options.tx_ant,
        tx_gain    = options.tx_gain,
        tx_chan    = options.tx_chan,
        clock_rate = options.clock_rate,
        wave_freq  = options.wave_freq,
    )

if __name__ == '__main__':
    main()
