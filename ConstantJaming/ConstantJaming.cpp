// usrp_tx_noise.cpp
// Usage: ./usrp_tx_noise <continuous|burst> [--noise] [--freq <Hz>] [--samp-rate <Sps>]
//                         [--gain <dB>] [--tone-freq <Hz>] [--amp <0..1>]
//                         [--on <s>] [--off <s>] [--buf-dur <s>] [--device-args "<args>"]
//
// Example: ./usrp_tx_noise continuous --noise --freq 2462e6 --samp-rate 1e6 --gain 40
//
// WARNING: This program transmits RF. Use only in authorized lab environment.

#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/types/metadata.hpp>
#include <uhd/utils/thread.hpp>

#include <iostream>
#include <vector>
#include <complex>
#include <cmath>
#include <random>
#include <chrono>
#include <thread>
#include <cstring>
#include <getopt.h>

using complexf = std::complex<float>;

struct Config {
    std::string mode = "continuous"; // "continuous" or "burst"
    double freq = 2462e6;
    double samp_rate = 1e6;
    double gain = 40.0;
    double tone_freq = 100e3;
    double amp = 0.8;
    bool noise = false;
    double on = 0.001;
    double off = 0.099;
    double buf_dur = 0.02; // seconds
    std::string device_args = "type=b200";
};
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

void usage(const char* prog) {
    std::cerr << "Usage: " << prog << " <continuous|burst> [options]\n"
              << "Options:\n"
              << "  --noise                 Transmit broadband complex white noise\n"
              << "  --freq <Hz>             Center frequency (default 2462e6)\n"
              << "  --samp-rate <Sps>       Sample rate (default 1e6)\n"
              << "  --gain <dB>             TX gain (default 40)\n"
              << "  --tone-freq <Hz>        Tone frequency (ignored if --noise)\n"
              << "  --amp <0..1>            Amplitude (default 0.8)\n"
              << "  --on <s>                Burst ON time (default 0.001)\n"
              << "  --off <s>               Burst OFF time (default 0.099)\n"
              << "  --buf-dur <s>           Buffer duration to generate (default 0.02)\n"
              << "  --device-args \"...\"     UHD device args (default \"type=b200\")\n";
}

std::vector<complexf> make_buffer(bool is_noise, double samp_rate, double tone_freq, double amp, double buf_dur) {
    size_t n = std::max<size_t>(1, static_cast<size_t>(std::round(samp_rate * buf_dur)));
    std::vector<complexf> buf(n);

    if (is_noise) {
        // complex white Gaussian noise with RMS ~= amp
        std::mt19937_64 rng((unsigned)std::chrono::high_resolution_clock::now().time_since_epoch().count());
        std::normal_distribution<float> dist(0.0f, 1.0f);
        double sum2 = 0.0;
        for (size_t i = 0; i < n; ++i) {
            float r = dist(rng);
            float im = dist(rng);
            buf[i] = complexf(r, im);
            sum2 += std::norm(buf[i]);
        }
        double rms = std::sqrt(sum2 / double(n));
        if (rms <= 0.0) rms = 1.0;
        double scale = amp / rms;
        for (size_t i = 0; i < n; ++i) buf[i] *= static_cast<float>(scale);
    } else {
        // complex tone: amp * exp(j*2*pi*f*t)
        for (size_t i = 0; i < n; ++i) {
            double t = double(i) / samp_rate;
            double phase = 2.0 * M_PI * tone_freq * t;
            buf[i] = complexf(static_cast<float>(amp * std::cos(phase)),
                              static_cast<float>(amp * std::sin(phase)));
        }
    }
    return buf;
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        usage(argv[0]);
        return EXIT_FAILURE;
    }

    Config cfg;
    cfg.mode = argv[1];

    static struct option long_opts[] = {
        {"noise", no_argument, nullptr, 'n'},
        {"freq", required_argument, nullptr, 'f'},
        {"samp-rate", required_argument, nullptr, 's'},
        {"gain", required_argument, nullptr, 'g'},
        {"tone-freq", required_argument, nullptr, 't'},
        {"amp", required_argument, nullptr, 'a'},
        {"on", required_argument, nullptr, 'o'},
        {"off", required_argument, nullptr, 'p'},
        {"buf-dur", required_argument, nullptr, 'b'},
        {"device-args", required_argument, nullptr, 'd'},
        {nullptr, 0, nullptr, 0}
    };

    int opt;
    int opt_index = 0;
    while ((opt = getopt_long(argc - 1, argv + 1, "nf:s:g:t:a:o:p:b:d:", long_opts, &opt_index)) != -1) {
        switch (opt) {
            case 'n': cfg.noise = true; break;
            case 'f': cfg.freq = std::stod(optarg); break;
            case 's': cfg.samp_rate = std::stod(optarg); break;
            case 'g': cfg.gain = std::stod(optarg); break;
            case 't': cfg.tone_freq = std::stod(optarg); break;
            case 'a': cfg.amp = std::stod(optarg); break;
            case 'o': cfg.on = std::stod(optarg); break;
            case 'p': cfg.off = std::stod(optarg); break;
            case 'b': cfg.buf_dur = std::stod(optarg); break;
            case 'd': cfg.device_args = std::string(optarg); break;
            case '?':
            default:
                usage(argv[0]);
                return EXIT_FAILURE;
        }
    }

    // Validate
    if (cfg.amp < 0.0 || cfg.amp > 1.0) {
        std::cerr << "amp must be in [0..1]\n";
        return EXIT_FAILURE;
    }
    if (cfg.mode != "continuous" && cfg.mode != "burst") {
        std::cerr << "mode must be 'continuous' or 'burst'\n";
        return EXIT_FAILURE;
    }

    try {
        std::cout << "Creating USRP with args: " << cfg.device_args << "\n";
        auto usrp = uhd::usrp::multi_usrp::make(cfg.device_args);

        // Set common parameters
        std::cout << "Setting sample rate: " << cfg.samp_rate << "\n";
        usrp->set_tx_rate(cfg.samp_rate);
        std::cout << "Setting center frequency: " << cfg.freq << "\n";
        uhd::tune_request_t tune_req(cfg.freq);
        usrp->set_tx_freq(tune_req);
        std::cout << "Setting gain: " << cfg.gain << "\n";
        usrp->set_tx_gain(cfg.gain);

        try {
            usrp->set_tx_antenna("TX/RX");
        } catch (...) {
            // ignore if antenna selection fails
        }

        // Prepare buffer
        std::vector<complexf> buf = make_buffer(cfg.noise, cfg.samp_rate, cfg.tone_freq, cfg.amp, cfg.buf_dur);

        // Create streamer
        uhd::stream_args_t stream_args("fc32"); // cpu format fc32
        stream_args.channels.push_back(0);
        auto tx_stream = usrp->get_tx_stream(stream_args);

        uhd::tx_metadata_t md;
        md.start_of_burst = false;
        md.end_of_burst = false;
        md.has_time_spec = false;

        std::cout << "Mode: " << cfg.mode << " -- " << (cfg.noise ? "noise" : "tone")
                  << "  freq=" << cfg.freq << " samp_rate=" << cfg.samp_rate << "\n";
        std::cout << "Buffer length: " << buf.size() << " samples (" << cfg.buf_dur << " s)\n";
        std::cout << "WARNING: This transmits directly (no CSMA). Use low gain and legal authorization.\n";

        if (cfg.mode == "continuous") {
            // Continuous loop: keep sending buffer repeatedly
            while (true) {
                size_t sent = tx_stream->send(&buf.front(), buf.size(), md, 1.0);
                if (sent != buf.size()) {
                    std::cerr << "Warning: sent " << sent << " / " << buf.size() << " samples\n";
                }
                // small sleep to let system breathe (not strictly necessary)
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        } else {
            // burst mode: send ON bursts and send zeros (or skip) during OFF
            // Precompute zero buffer for OFF
            std::vector<complexf> zero_buf(buf.size(), complexf(0.0f, 0.0f));

            std::cout << "Burst ON=" << cfg.on << " s OFF=" << cfg.off << " s\n";

            while (true) {
                // ON: send repeatedly for cfg.on seconds
                auto on_start = std::chrono::steady_clock::now();
                while (std::chrono::duration<double>(std::chrono::steady_clock::now() - on_start).count() < cfg.on) {
                    size_t sent = tx_stream->send(&buf.front(), buf.size(), md, 1.0);
                    if (sent != buf.size()) {
                        std::cerr << "Warning: sent " << sent << " / " << buf.size() << " samples\n";
                    }
                }

                // OFF: send zeros for cfg.off seconds (keeps streaming to prevent spurious transients)
                auto off_start = std::chrono::steady_clock::now();
                while (std::chrono::duration<double>(std::chrono::steady_clock::now() - off_start).count() < cfg.off) {
                    size_t sent = tx_stream->send(&zero_buf.front(), zero_buf.size(), md, 1.0);
                    (void)sent;
                    // tiny sleep
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
            }
        }

    } catch (const uhd::exception& e) {
        std::cerr << "UHD error: " << e.what() << "\n";
        return EXIT_FAILURE;
    } catch (const std::exception& e) {
        std::cerr << "STD error: " << e.what() << "\n";
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
