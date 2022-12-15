#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: transmite_tono
# Author: Susana
# GNU Radio version: 3.8.2.0

from distutils.version import StrictVersion

if __name__ == '__main__':
    import ctypes
    import sys
    if sys.platform.startswith('linux'):
        try:
            x11 = ctypes.cdll.LoadLibrary('libX11.so')
            x11.XInitThreads()
        except:
            print("Warning: failed to XInitThreads()")

from PyQt5 import Qt
from gnuradio import qtgui
from gnuradio.filter import firdes
import sip
from gnuradio import analog
from gnuradio import blocks
from gnuradio import gr
import sys
import signal
from argparse import ArgumentParser
from gnuradio.eng_arg import eng_float, intx
from gnuradio import eng_notation
from gnuradio.qtgui import Range, RangeWidget
import iio

from gnuradio import qtgui

class transmite_tono(gr.top_block, Qt.QWidget):

    def __init__(self):
        gr.top_block.__init__(self, "transmite_tono")
        Qt.QWidget.__init__(self)
        self.setWindowTitle("transmite_tono")
        qtgui.util.check_set_qss()
        try:
            self.setWindowIcon(Qt.QIcon.fromTheme('gnuradio-grc'))
        except:
            pass
        self.top_scroll_layout = Qt.QVBoxLayout()
        self.setLayout(self.top_scroll_layout)
        self.top_scroll = Qt.QScrollArea()
        self.top_scroll.setFrameStyle(Qt.QFrame.NoFrame)
        self.top_scroll_layout.addWidget(self.top_scroll)
        self.top_scroll.setWidgetResizable(True)
        self.top_widget = Qt.QWidget()
        self.top_scroll.setWidget(self.top_widget)
        self.top_layout = Qt.QVBoxLayout(self.top_widget)
        self.top_grid_layout = Qt.QGridLayout()
        self.top_layout.addLayout(self.top_grid_layout)

        self.settings = Qt.QSettings("GNU Radio", "transmite_tono")

        try:
            if StrictVersion(Qt.qVersion()) < StrictVersion("5.0.0"):
                self.restoreGeometry(self.settings.value("geometry").toByteArray())
            else:
                self.restoreGeometry(self.settings.value("geometry"))
        except:
            pass

        ##################################################
        # Variables
        ##################################################
        self.tx_freq = tx_freq = 2000000000
        self.samp_rate = samp_rate = 2e6
        self.nsamples = nsamples = 40000
        self.atenuation = atenuation = 0

        ##################################################
        # Blocks
        ##################################################
        self._tx_freq_range = Range(70e6, 6e9, 1e6, 2000000000, 200)
        self._tx_freq_win = RangeWidget(self._tx_freq_range, self.set_tx_freq, 'tx_freq', "counter_slider", float)
        self.top_grid_layout.addWidget(self._tx_freq_win)
        self._atenuation_range = Range(0, 90, 5, 0, 200)
        self._atenuation_win = RangeWidget(self._atenuation_range, self.set_atenuation, 'atenuation RF (dB)', "counter_slider", float)
        self.top_grid_layout.addWidget(self._atenuation_win)
        self.qtgui_number_sink_1_1 = qtgui.number_sink(
            gr.sizeof_float,
            0,
            qtgui.NUM_GRAPH_NONE,
            1
        )
        self.qtgui_number_sink_1_1.set_update_time(0.10)
        self.qtgui_number_sink_1_1.set_title("TX Signal Strength")

        labels = ['Signal strength', '', '', '', '',
            '', '', '', '', '']
        units = ['dB', '', '', '', '',
            '', '', '', '', '']
        colors = [("black", "black"), ("black", "black"), ("black", "black"), ("black", "black"), ("black", "black"),
            ("black", "black"), ("black", "black"), ("black", "black"), ("black", "black"), ("black", "black")]
        factor = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]

        for i in range(1):
            self.qtgui_number_sink_1_1.set_min(i, -1)
            self.qtgui_number_sink_1_1.set_max(i, 1)
            self.qtgui_number_sink_1_1.set_color(i, colors[i][0], colors[i][1])
            if len(labels[i]) == 0:
                self.qtgui_number_sink_1_1.set_label(i, "Data {0}".format(i))
            else:
                self.qtgui_number_sink_1_1.set_label(i, labels[i])
            self.qtgui_number_sink_1_1.set_unit(i, units[i])
            self.qtgui_number_sink_1_1.set_factor(i, factor[i])

        self.qtgui_number_sink_1_1.enable_autoscale(True)
        self._qtgui_number_sink_1_1_win = sip.wrapinstance(self.qtgui_number_sink_1_1.pyqwidget(), Qt.QWidget)
        self.top_grid_layout.addWidget(self._qtgui_number_sink_1_1_win)
        self.qtgui_freq_sink_x_0 = qtgui.freq_sink_c(
            1024, #size
            firdes.WIN_BLACKMAN_hARRIS, #wintype
            tx_freq, #fc
            samp_rate, #bw
            "", #name
            1
        )
        self.qtgui_freq_sink_x_0.set_update_time(0.10)
        self.qtgui_freq_sink_x_0.set_y_axis(-140, 10)
        self.qtgui_freq_sink_x_0.set_y_label('Relative Gain', 'dB')
        self.qtgui_freq_sink_x_0.set_trigger_mode(qtgui.TRIG_MODE_FREE, 0.0, 0, "")
        self.qtgui_freq_sink_x_0.enable_autoscale(False)
        self.qtgui_freq_sink_x_0.enable_grid(False)
        self.qtgui_freq_sink_x_0.set_fft_average(1.0)
        self.qtgui_freq_sink_x_0.enable_axis_labels(True)
        self.qtgui_freq_sink_x_0.enable_control_panel(True)



        labels = ['', '', '', '', '',
            '', '', '', '', '']
        widths = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        colors = ["blue", "red", "green", "black", "cyan",
            "magenta", "yellow", "dark red", "dark green", "dark blue"]
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
            1.0, 1.0, 1.0, 1.0, 1.0]

        for i in range(1):
            if len(labels[i]) == 0:
                self.qtgui_freq_sink_x_0.set_line_label(i, "Data {0}".format(i))
            else:
                self.qtgui_freq_sink_x_0.set_line_label(i, labels[i])
            self.qtgui_freq_sink_x_0.set_line_width(i, widths[i])
            self.qtgui_freq_sink_x_0.set_line_color(i, colors[i])
            self.qtgui_freq_sink_x_0.set_line_alpha(i, alphas[i])

        self._qtgui_freq_sink_x_0_win = sip.wrapinstance(self.qtgui_freq_sink_x_0.pyqwidget(), Qt.QWidget)
        self.top_grid_layout.addWidget(self._qtgui_freq_sink_x_0_win)
        self.iio_pluto_sink_0 = iio.pluto_sink('', int(tx_freq), int(samp_rate), int(samp_rate), 32768, False, atenuation, '', True)
        self.blocks_nlog10_ff_0 = blocks.nlog10_ff(1, 1, 0)
        self.blocks_moving_average_xx_0_0 = blocks.moving_average_ff(nsamples, 1/nsamples, 4000, 1)
        self.blocks_complex_to_mag_squared_0_0 = blocks.complex_to_mag_squared(1)
        self.analog_sig_source_x_0 = analog.sig_source_c(samp_rate, analog.GR_COS_WAVE, 400e3, 1, 0, 0)



        ##################################################
        # Connections
        ##################################################
        self.connect((self.analog_sig_source_x_0, 0), (self.blocks_complex_to_mag_squared_0_0, 0))
        self.connect((self.analog_sig_source_x_0, 0), (self.iio_pluto_sink_0, 0))
        self.connect((self.analog_sig_source_x_0, 0), (self.qtgui_freq_sink_x_0, 0))
        self.connect((self.blocks_complex_to_mag_squared_0_0, 0), (self.blocks_moving_average_xx_0_0, 0))
        self.connect((self.blocks_moving_average_xx_0_0, 0), (self.blocks_nlog10_ff_0, 0))
        self.connect((self.blocks_nlog10_ff_0, 0), (self.qtgui_number_sink_1_1, 0))


    def closeEvent(self, event):
        self.settings = Qt.QSettings("GNU Radio", "transmite_tono")
        self.settings.setValue("geometry", self.saveGeometry())
        event.accept()

    def get_tx_freq(self):
        return self.tx_freq

    def set_tx_freq(self, tx_freq):
        self.tx_freq = tx_freq
        self.iio_pluto_sink_0.set_params(int(self.tx_freq), int(self.samp_rate), int(self.samp_rate), self.atenuation, '', True)
        self.qtgui_freq_sink_x_0.set_frequency_range(self.tx_freq, self.samp_rate)

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.analog_sig_source_x_0.set_sampling_freq(self.samp_rate)
        self.iio_pluto_sink_0.set_params(int(self.tx_freq), int(self.samp_rate), int(self.samp_rate), self.atenuation, '', True)
        self.qtgui_freq_sink_x_0.set_frequency_range(self.tx_freq, self.samp_rate)

    def get_nsamples(self):
        return self.nsamples

    def set_nsamples(self, nsamples):
        self.nsamples = nsamples
        self.blocks_moving_average_xx_0_0.set_length_and_scale(self.nsamples, 1/self.nsamples)

    def get_atenuation(self):
        return self.atenuation

    def set_atenuation(self, atenuation):
        self.atenuation = atenuation
        self.iio_pluto_sink_0.set_params(int(self.tx_freq), int(self.samp_rate), int(self.samp_rate), self.atenuation, '', True)





def main(top_block_cls=transmite_tono, options=None):

    if StrictVersion("4.5.0") <= StrictVersion(Qt.qVersion()) < StrictVersion("5.0.0"):
        style = gr.prefs().get_string('qtgui', 'style', 'raster')
        Qt.QApplication.setGraphicsSystem(style)
    qapp = Qt.QApplication(sys.argv)

    tb = top_block_cls()

    tb.start()

    tb.show()

    def sig_handler(sig=None, frame=None):
        Qt.QApplication.quit()

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    timer = Qt.QTimer()
    timer.start(500)
    timer.timeout.connect(lambda: None)

    def quitting():
        tb.stop()
        tb.wait()

    qapp.aboutToQuit.connect(quitting)
    qapp.exec_()

if __name__ == '__main__':
    main()
