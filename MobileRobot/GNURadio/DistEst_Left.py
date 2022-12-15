#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: Not titled yet
# Author: pi
# GNU Radio version: 3.8.2.0

from gnuradio import analog
from gnuradio import blocks
from gnuradio import filter
from gnuradio.filter import firdes
from gnuradio import gr
import sys
import signal
from argparse import ArgumentParser
from gnuradio.eng_arg import eng_float, intx
from gnuradio import eng_notation
from gnuradio import uhd
import time
import epy_block_0
import numpy


class DistEst_Left(gr.top_block):

    def __init__(self):
        gr.top_block.__init__(self, "Not titled yet")

        ##################################################
        # Variables
        ##################################################
        self.f_mod = f_mod = 2e9
        self.c = c = int(3e8)
        self.tx_pot_dB = tx_pot_dB = 38
        self.samp_rate = samp_rate = 2e6
        self.nsamples = nsamples = 40000
        self.lanbda = lanbda = c/f_mod
        self.RF_Gain = RF_Gain = 50

        ##################################################
        # Blocks
        ##################################################
        self.uhd_usrp_source_0 = uhd.usrp_source(
            ",".join(("serial=31BBEFE", '')),
            uhd.stream_args(
                cpu_format="fc32",
                args='',
                channels=list(range(0,1)),
            ),
        )
        self.uhd_usrp_source_0.set_center_freq(f_mod, 0)
        self.uhd_usrp_source_0.set_rx_agc(False, 0)
        self.uhd_usrp_source_0.set_gain(RF_Gain, 0)
        self.uhd_usrp_source_0.set_antenna("TX/RX", 0)
        self.uhd_usrp_source_0.set_bandwidth(samp_rate, 0)
        self.uhd_usrp_source_0.set_samp_rate(samp_rate)
        self.uhd_usrp_source_0.set_time_unknown_pps(uhd.time_spec())
        self.epy_block_0 = epy_block_0.blk(nsamples=nsamples)
        self.blocks_sub_xx_0 = blocks.sub_ff(1)
        self.blocks_null_sink_0 = blocks.null_sink(gr.sizeof_float*1)
        self.blocks_nlog10_ff_0_0_0_0 = blocks.nlog10_ff(10, 1, 0)
        self.blocks_nlog10_ff_0_0_0 = blocks.nlog10_ff(10, 1, 0)
        self.blocks_moving_average_xx_0 = blocks.moving_average_ff(nsamples, 1/nsamples, 4000, 1)
        self.blocks_complex_to_mag_squared_0 = blocks.complex_to_mag_squared(1)
        self.blocks_add_xx_0_0 = blocks.add_vff(1)
        self.band_pass_filter_0_1 = filter.fir_filter_ccf(
            1,
            firdes.band_pass(
                1,
                samp_rate,
                300e3,
                500e3,
                25e3,
                firdes.WIN_HAMMING,
                6.76))
        self.analog_const_source_x_0_0 = analog.sig_source_f(0, analog.GR_CONST_WAVE, 0, 0, tx_pot_dB)
        self.analog_const_source_x_0 = analog.sig_source_f(0, analog.GR_CONST_WAVE, 0, 0, (lanbda/(4*numpy.pi))**2)



        ##################################################
        # Connections
        ##################################################
        self.connect((self.analog_const_source_x_0, 0), (self.blocks_nlog10_ff_0_0_0_0, 0))
        self.connect((self.analog_const_source_x_0_0, 0), (self.blocks_sub_xx_0, 0))
        self.connect((self.band_pass_filter_0_1, 0), (self.blocks_complex_to_mag_squared_0, 0))
        self.connect((self.blocks_add_xx_0_0, 0), (self.epy_block_0, 0))
        self.connect((self.blocks_complex_to_mag_squared_0, 0), (self.blocks_moving_average_xx_0, 0))
        self.connect((self.blocks_moving_average_xx_0, 0), (self.blocks_nlog10_ff_0_0_0, 0))
        self.connect((self.blocks_nlog10_ff_0_0_0, 0), (self.blocks_sub_xx_0, 1))
        self.connect((self.blocks_nlog10_ff_0_0_0_0, 0), (self.blocks_add_xx_0_0, 0))
        self.connect((self.blocks_sub_xx_0, 0), (self.blocks_add_xx_0_0, 1))
        self.connect((self.epy_block_0, 0), (self.blocks_null_sink_0, 0))
        self.connect((self.uhd_usrp_source_0, 0), (self.band_pass_filter_0_1, 0))


    def get_f_mod(self):
        return self.f_mod

    def set_f_mod(self, f_mod):
        self.f_mod = f_mod
        self.set_lanbda(self.c/self.f_mod)
        self.uhd_usrp_source_0.set_center_freq(self.f_mod, 0)

    def get_c(self):
        return self.c

    def set_c(self, c):
        self.c = c
        self.set_lanbda(self.c/self.f_mod)

    def get_tx_pot_dB(self):
        return self.tx_pot_dB

    def set_tx_pot_dB(self, tx_pot_dB):
        self.tx_pot_dB = tx_pot_dB
        self.analog_const_source_x_0_0.set_offset(self.tx_pot_dB)

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.band_pass_filter_0_1.set_taps(firdes.band_pass(1, self.samp_rate, 300e3, 500e3, 25e3, firdes.WIN_HAMMING, 6.76))
        self.uhd_usrp_source_0.set_samp_rate(self.samp_rate)
        self.uhd_usrp_source_0.set_bandwidth(self.samp_rate, 0)

    def get_nsamples(self):
        return self.nsamples

    def set_nsamples(self, nsamples):
        self.nsamples = nsamples
        self.blocks_moving_average_xx_0.set_length_and_scale(self.nsamples, 1/self.nsamples)
        self.epy_block_0.nsamples = self.nsamples

    def get_lanbda(self):
        return self.lanbda

    def set_lanbda(self, lanbda):
        self.lanbda = lanbda
        self.analog_const_source_x_0.set_offset((self.lanbda/(4*numpy.pi))**2)

    def get_RF_Gain(self):
        return self.RF_Gain

    def set_RF_Gain(self, RF_Gain):
        self.RF_Gain = RF_Gain
        self.uhd_usrp_source_0.set_gain(self.RF_Gain, 0)





def main(top_block_cls=DistEst_Left, options=None):
    tb = top_block_cls()

    def sig_handler(sig=None, frame=None):
        tb.stop()
        tb.wait()

        sys.exit(0)

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    tb.start()

    try:
        input('Press Enter to quit: ')
    except EOFError:
        pass
    tb.stop()
    tb.wait()


if __name__ == '__main__':
    main()
