#!/usr/bin/env python3
"""
usrp_tx_noise.py

Modes:
  - continuous : continuous transmission (tone or noise)
  - burst      : gated bursts (tone or noise) using multiply_const gating

Use --noise to transmit broadband complex white noise instead of a single tone.

WARNING: Transmitting wideband noise on 2.4 GHz will interfere with Wi-Fi/Bluetooth and may be
illegal outside a controlled lab environment. Use low gain and obtain authorization.
"""

import sys
import time
import argparse
import numpy as np
from gnuradio import gr, blocks, uhd

def make_buffer(is_noise, samp_rate, tone_freq, amp, buf_dur=0.01):
    """Create a buffer of complex64 samples: either noise or a tone"""
    n = max(1, int(samp_rate * buf_dur))
    t = np.arange(n, dtype=np.float64) / float(samp_rate)
    if is_noise:
        # complex white Gaussian noise
        # real and imag ~ N(0, 1), scale by amp/sqrt(2) so total RMS ~ amp
        real = np.random.normal(scale=1.0, size=n)
        imag = np.random.normal(scale=1.0, size=n)
        noise = (real + 1j * imag).astype(np.complex64)
        # normalize to unit RMS then scale to requested amplitude
        rms = np.sqrt(np.mean(np.abs(noise)**2))
        if rms > 0:
            noise = noise / rms * float(amp)
        return noise
    else:
        tone = (float(amp) * np.exp(1j * 2 * np.pi * float(tone_freq) * t)).astype(np.complex64)
        return tone

class ContinuousTx(gr.top_block):
    def __init__(self, samp_rate=1e6, center_freq=2462e6, gain=40, tone_freq=100e3, amp=0.8, noise=False):
        gr.top_block.__init__(self, "Continuous TX")
        samp_rate = int(samp_rate)
        buf = make_buffer(noise, samp_rate, tone_freq, amp, buf_dur=0.02)
        self.src = blocks.vector_source_c(buf.tolist(), repeat=True, vlen=1)
        self.sink = uhd.usrp_sink(
            ",".join(["type=b200"]),
            uhd.stream_args(cpu_format="fc32", channels=range(1))
        )
        self.sink.set_samp_rate(samp_rate)
        self.sink.set_center_freq(float(center_freq))
        self.sink.set_gain(float(gain))
        try:
            self.sink.set_antenna("TX/RX", 0)
        except Exception:
            pass
        self.connect(self.src, self.sink)

class GatedBurstTx(gr.top_block):
    def __init__(self, samp_rate=1e6, center_freq=2462e6, gain=40, tone_freq=100e3, amp=0.8, noise=False):
        gr.top_block.__init__(self, "Gated Burst TX")
        samp_rate = int(samp_rate)
        buf = make_buffer(noise, samp_rate, tone_freq, amp, buf_dur=0.02)
        self.src = blocks.vector_source_c(buf.tolist(), repeat=True, vlen=1)
        self.mult = blocks.multiply_const_cc(0.0)  # start silent
        self.sink = uhd.usrp_sink(
            ",".join(["type=b200"]),
            uhd.stream_args(cpu_format="fc32", channels=range(1))
        )
        self.sink.set_samp_rate(samp_rate)
        self.sink.set_center_freq(float(center_freq))
        self.sink.set_gain(float(gain))
        try:
            self.sink.set_antenna("TX/RX", 0)
        except Exception:
            pass
        self.connect(self.src, self.mult, self.sink)

    def set_on(self):
        self.mult.set_k(1.0)

    def set_off(self):
        self.mult.set_k(0.0)

def parse_args():
    p = argparse.ArgumentParser(description="USRP TX test: tone or heavy noise (continuous or burst)")
    p.add_argument('mode', choices=['continuous', 'burst'], help='Operation mode')
    p.add_argument('--freq', type=float, default=2462e6, help='Center frequency in Hz (default: 2462e6 channel 6)')
    p.add_argument('--samp-rate', type=float, default=1e6, help='Sample rate in Sps (default: 1e6)')
    p.add_argument('--gain', type=float, default=40.0, help='TX gain (device-dependent; default: 40)')
    p.add_argument('--tone-freq', type=float, default=100e3, help='Tone frequency in Hz (ignored if --noise)')
    p.add_argument('--amp', type=float, default=0.8, help='Signal amplitude (0..1, default 0.8)')
    p.add_argument('--noise', action='store_true', help='Transmit broadband complex white noise instead of tone')
    # burst-specific
    p.add_argument('--on', type=float, default=0.001, help='Burst ON duration in seconds (default 0.001)')
    p.add_argument('--off', type=float, default=0.099, help='Burst OFF duration in seconds (default 0.099)')
    p.add_argument('--loop-delay', type=float, default=0.0, help='Extra delay after each loop iteration (default 0)')
    return p.parse_args()

def main():
    args = parse_args()

    if args.mode == 'continuous':
        tb = ContinuousTx(samp_rate=args.samp_rate,
                          center_freq=args.freq,
                          gain=args.gain,
                          tone_freq=args.tone_freq,
                          amp=args.amp,
                          noise=args.noise)
        mode_desc = "noise" if args.noise else "tone"
        print(f"Starting continuous {mode_desc} at {args.freq} Hz (samp_rate={args.samp_rate}).")
        print("WARNING: This transmits directly (no CSMA). Use low gain and legal authorization.")
        tb.start()
        try:
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            print("\nStopping...")
            tb.stop()
            tb.wait()
            print("Stopped.")
            sys.exit(0)

    elif args.mode == 'burst':
        tb = GatedBurstTx(samp_rate=args.samp_rate,
                          center_freq=args.freq,
                          gain=args.gain,
                          tone_freq=args.tone_freq,
                          amp=args.amp,
                          noise=args.noise)
        mode_desc = "noise" if args.noise else "tone"
        print(f"Starting gated burst ({mode_desc}) on {args.freq} Hz. ON={args.on}s OFF={args.off}s")
        print("WARNING: This transmits directly (no CSMA). Use low gain and legal authorization.")
        tb.start()
        try:
            while True:
                tb.set_on()
                time.sleep(max(0.0, args.on))
                tb.set_off()
                time.sleep(max(0.0, args.off + args.loop_delay))
        except KeyboardInterrupt:
            print("\nStopping...")
            tb.stop()
            tb.wait()
            print("Stopped.")
            sys.exit(0)

if __name__ == "__main__":
    main() 