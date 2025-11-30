// usrp_tx_reactive_packet_level.cpp
// Packet-level reactive transmitter: sense packet edges with short windows,
// transmit only while external packet active, stop RX while transmitting,
// then resume RX. Includes verbose timestamped logging.
//
// Usage: ./usrp_tx_reactive_packet_level <continuous|burst> [options]
// Options:
//   --noise                 transmit noise instead of tone
//   --freq <Hz>             center frequency (default 2462e6)
//   --samp-rate <Sps>       sampling rate (default 20e6)
//   --gain <dB>             tx gain (default 30)
//   --tone-freq <Hz>        tone frequency (ignored if --noise)
//   --amp <0..1>            amplitude (default 0.6)
//   --on <s>                burst ON (used in burst mode) (default 0.002)
//   --off <s>               burst OFF (used in burst mode) (default 0.02)
//   --buf-dur <s>           tx buffer duration (default 0.001 -> fine control)
//   --sense-samps <N>       rx samples per measurement (default 64)
//   --sense-avg <N>         running average window (default 1)
//   --busy-thresh-db <dB>   busy threshold in dBFS (default -55)
//   --idle-diff-db <dB>     idle = busy - idle_diff (default 15)
//   --device-args "<args>"  UHD device args (default "type=b200")
//
// Example:
// ./usrp_tx_reactive_packet_level continuous --noise --freq 2462e6 --samp-rate 20e6 \
//    --sense-samps 64 --busy-thresh-db -55 --idle-diff-db 15 --buf-dur 0.001 --gain 30
//
// WARNING: This transmits RF. Use only in authorized environment.

#include <uhd/types/metadata.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/thread.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <complex>
#include <csignal>
#include <cstring>
#include <getopt.h>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>
#include <sstream>
#include <thread>
#include <vector>

using complexf = std::complex<float>;

// ---------------- timestamp helper ----------------
std::string now_str() {
    using namespace std::chrono;
    auto t = system_clock::now();
    auto tt = system_clock::to_time_t(t);
    auto tm = *std::localtime(&tt);
    auto us = duration_cast<microseconds>(t.time_since_epoch()).count() % 1000000;
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S") << "." << std::setw(6)
        << std::setfill('0') << us;
    return oss.str();
}

// ---------------------------------------------------

struct Config {
    std::string mode = "continuous"; // "continuous" or "burst"
    double freq = 2462e6;
    double samp_rate = 20e6; // default high for fine sensing
    double gain = 30.0;
    double tone_freq = 100e3;
    double amp = 0.6;
    bool noise = false;
    double on = 0.002;
    double off = 0.02;
    double buf_dur = 0.001; // small tx buffer for fine control (1 ms default)
    std::string device_args = "type=b200";

    // sensing params
    size_t sense_samps = 64;       // small window for microsecond resolution
    size_t sense_avg = 1;          // running average window
    double busy_thresh_db = -55.0; // threshold for "busy"
    double idle_diff_db = 15.0;    // idle threshold = busy_thresh - idle_diff
};

volatile std::sig_atomic_t stop_signal = 0;
void sigint_handler(int) { stop_signal = 1; }

void usage(const char *prog) {
    std::cerr << "Usage: " << prog << " <continuous|burst> [options]\n";
}

// create TX buffer (tone or noise)
std::vector<complexf>
make_buffer(bool is_noise, double samp_rate, double tone_freq, double amp, double buf_dur) {
    size_t n = std::max<size_t>(1, static_cast<size_t>(std::round(samp_rate * buf_dur)));
    std::vector<complexf> buf(n);

    if (is_noise) {
        std::mt19937_64 rng((unsigned)std::chrono::high_resolution_clock::now()
                                .time_since_epoch()
                                .count());
        std::normal_distribution<float> dist(0.0f, 1.0f);
        double sum2 = 0.0;
        for (size_t i = 0; i < n; ++i) {
            float r = dist(rng);
            float im = dist(rng);
            buf[i] = complexf(r, im);
            sum2 += std::norm(buf[i]);
        }
        double rms = std::sqrt(sum2 / double(n));
        if (rms <= 0.0)
            rms = 1.0;
        double scale = amp / rms;
        for (size_t i = 0; i < n; ++i)
            buf[i] *= static_cast<float>(scale);
    } else {
        for (size_t i = 0; i < n; ++i) {
            double t = double(i) / samp_rate;
            double phase = 2.0 * M_PI * tone_freq * t;
            buf[i] = complexf(static_cast<float>(amp * std::cos(phase)),
                              static_cast<float>(amp * std::sin(phase)));
        }
    }
    return buf;
}

// convert linear power to dBFS (10*log10(power)), avoid -inf
inline double power_dbfs(double power_linear) {
    constexpr double min_lin = 1e-20;
    double p = std::max(power_linear, min_lin);
    return 10.0 * std::log10(p);
}

int main(int argc, char *argv[]) {
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
        {"sense-samps", required_argument, nullptr, 'S'},
        {"sense-avg", required_argument, nullptr, 'A'},
        {"busy-thresh-db", required_argument, nullptr, 'B'},
        {"idle-diff-db", required_argument, nullptr, 'I'},
        {nullptr, 0, nullptr, 0}};

    int opt;
    int opt_index = 0;
    // parse args starting at argv+1 (we consumed mode)
    while (
        (opt = getopt_long(
             argc - 1, argv + 1, "nf:s:g:t:a:o:p:b:d:S:A:B:I:", long_opts, &opt_index)) !=
        -1) {
        switch (opt) {
        case 'n':
            cfg.noise = true;
            break;
        case 'f':
            cfg.freq = std::stod(optarg);
            break;
        case 's':
            cfg.samp_rate = std::stod(optarg);
            break;
        case 'g':
            cfg.gain = std::stod(optarg);
            break;
        case 't':
            cfg.tone_freq = std::stod(optarg);
            break;
        case 'a':
            cfg.amp = std::stod(optarg);
            break;
        case 'o':
            cfg.on = std::stod(optarg);
            break;
        case 'p':
            cfg.off = std::stod(optarg);
            break;
        case 'b':
            cfg.buf_dur = std::stod(optarg);
            break;
        case 'd':
            cfg.device_args = std::string(optarg);
            break;
        case 'S':
            cfg.sense_samps = static_cast<size_t>(std::stoul(optarg));
            break;
        case 'A':
            cfg.sense_avg = static_cast<size_t>(std::stoul(optarg));
            break;
        case 'B':
            cfg.busy_thresh_db = std::stod(optarg);
            break;
        case 'I':
            cfg.idle_diff_db = std::stod(optarg);
            break;
        case '?':
        default:
            usage(argv[0]);
            return EXIT_FAILURE;
        }
    }

    // Validate
    if (cfg.amp < 0.0 || cfg.amp > 1.0) {
        std::cerr << now_str() << " ERROR: amp must be in [0..1]\n";
        return EXIT_FAILURE;
    }
    if (cfg.mode != "continuous" && cfg.mode != "burst") {
        std::cerr << now_str() << " ERROR: mode must be 'continuous' or 'burst'\n";
        return EXIT_FAILURE;
    }
    if (cfg.sense_samps == 0)
        cfg.sense_samps = 64;
    if (cfg.sense_avg == 0)
        cfg.sense_avg = 1;
    if (cfg.buf_dur <= 0.0)
        cfg.buf_dur = 0.001;

    // derived thresholds
    double BUSY_THRESH_DB = cfg.busy_thresh_db;
    double IDLE_THRESH_DB = cfg.busy_thresh_db - cfg.idle_diff_db;

    std::signal(SIGINT, sigint_handler);

    try {
        std::cout << "[" << now_str() << "] Creating USRP with args: " << cfg.device_args
                  << "\n";
        auto usrp = uhd::usrp::multi_usrp::make(cfg.device_args);

        // Set rates and freqs
        std::cout << "[" << now_str() << "] Setting sample rate: " << cfg.samp_rate
                  << "\n";
        usrp->set_tx_rate(cfg.samp_rate);
        usrp->set_rx_rate(cfg.samp_rate);

        std::cout << "[" << now_str() << "] Setting center frequency: " << cfg.freq
                  << "\n";
        uhd::tune_request_t tune_req(cfg.freq);
        usrp->set_tx_freq(tune_req);
        usrp->set_rx_freq(tune_req);

        std::cout << "[" << now_str() << "] Setting TX gain: " << cfg.gain << "\n";
        usrp->set_tx_gain(cfg.gain);
        try {
            usrp->set_rx_gain(cfg.gain / 2.0);
        } catch (...) { /* ignore */
        }

        try {
            usrp->set_tx_antenna("TX/RX");
            usrp->set_rx_antenna("RX2");
        } catch (...) { /* ignore antenna selection errors */
        }

        // Build tx buffer
        std::vector<complexf> tx_buf =
            make_buffer(cfg.noise, cfg.samp_rate, cfg.tone_freq, cfg.amp, cfg.buf_dur);
        std::vector<complexf> zero_buf(std::max<size_t>(1, tx_buf.size() / 10),
                                       complexf(0.0f, 0.0f));

        // Create stream args with channel 0
        uhd::stream_args_t tx_stream_args("fc32");
        tx_stream_args.channels.push_back(0);
        auto tx_stream = usrp->get_tx_stream(tx_stream_args);

        uhd::stream_args_t rx_stream_args("fc32");
        rx_stream_args.channels.push_back(0);
        auto rx_stream = usrp->get_rx_stream(rx_stream_args);

        uhd::tx_metadata_t tx_md;
        tx_md.start_of_burst = false;
        tx_md.end_of_burst = false;
        tx_md.has_time_spec = false;

        uhd::rx_metadata_t rx_md;

        std::cout << "[" << now_str() << "] Mode: " << cfg.mode << " -- "
                  << (cfg.noise ? "noise" : "tone") << " freq=" << cfg.freq
                  << " samp_rate=" << cfg.samp_rate << "\n";
        std::cout << "[" << now_str() << "] TX buffer: " << tx_buf.size() << " samples ("
                  << cfg.buf_dur << " s)\n";
        std::cout << "[" << now_str() << "] Sensing: samp_window=" << cfg.sense_samps
                  << " samples, avg_window=" << cfg.sense_avg
                  << ", BUSY_THRESH=" << BUSY_THRESH_DB
                  << " dBFS, IDLE_THRESH=" << IDLE_THRESH_DB << " dBFS\n";
        std::cout << "[" << now_str()
                  << "] WARNING: This will transmit RF. Authorize before use.\n";

        // Control flags
        std::atomic<bool> running(true);
        std::atomic<bool> tx_active(false);
        std::atomic<bool> rx_streaming(false);
        std::atomic<bool> channel_busy(false);

        // RX start/stop helpers
        auto start_rx_stream = [&](void) -> void {
            if (rx_streaming.load())
                return;
            try {
                uhd::stream_cmd_t stream_cmd(
                    uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
                stream_cmd.stream_now = true;
                rx_stream->issue_stream_cmd(stream_cmd);
                rx_streaming.store(true);
                std::cout << "[" << now_str() << "] RX stream STARTED\n";
            } catch (const std::exception &e) {
                std::cerr << "[" << now_str()
                          << "] ERROR starting RX stream: " << e.what() << "\n";
                rx_streaming.store(false);
            }
        };
        auto stop_rx_stream = [&](void) -> void {
            if (!rx_streaming.load())
                return;
            try {
                uhd::stream_cmd_t stop_cmd(
                    uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
                rx_stream->issue_stream_cmd(stop_cmd);
                rx_streaming.store(false);
                std::cout << "[" << now_str() << "] RX stream STOPPED\n";
            } catch (const std::exception &e) {
                std::cerr << "[" << now_str()
                          << "] ERROR stopping RX stream: " << e.what() << "\n";
                rx_streaming.store(false);
            }
        };

        // RX measurement thread: short windows, hysteresis for packet edges
        std::thread rx_thread([&]() {
            std::vector<complexf> rx_buffer(cfg.sense_samps);
            std::vector<double> recent_powers;
            recent_powers.reserve(cfg.sense_avg);

            // start RX initially
            start_rx_stream();

            bool prev_busy = false;

            while (running.load() && !stop_signal) {
                // If we're transmitting, don't read RX (we stop RX before TX anyway)
                if (tx_active.load()) {
                    if (rx_streaming.load())
                        stop_rx_stream();
                    std::this_thread::sleep_for(std::chrono::microseconds(50));
                    continue;
                } else {
                    if (!rx_streaming.load())
                        start_rx_stream();
                }

                // receive with short timeout (responsive)
                size_t num_recv = 0;
                try {
                    num_recv = rx_stream->recv(
                        &rx_buffer.front(), rx_buffer.size(), rx_md, 0.05);
                } catch (const uhd::io_error &e) {
                    num_recv = 0;
                } catch (const std::exception &e) {
                    std::cerr << "[" << now_str() << "] RX recv error: " << e.what()
                              << "\n";
                    num_recv = 0;
                }

                double mean_power = 0.0;
                if (num_recv > 0) {
                    double sum2 = 0.0;
                    for (size_t i = 0; i < num_recv; ++i)
                        sum2 += std::norm(rx_buffer[i]);
                    mean_power = sum2 / double(num_recv);
                } else {
                    // treat as very small power (no samples) but don't immediately mark
                    // idle - rely on hysteresis
                    mean_power = 0.0;
                }

                recent_powers.push_back(mean_power);
                if (recent_powers.size() > cfg.sense_avg)
                    recent_powers.erase(recent_powers.begin());

                double avg_lin = 0.0;
                for (double p : recent_powers)
                    avg_lin += p;
                avg_lin /= std::max<size_t>(1, recent_powers.size());
                double avg_db = power_dbfs(avg_lin);

                // hysteresis: detect packet edges
                if (!prev_busy) {
                    if (avg_db >= BUSY_THRESH_DB) {
                        prev_busy = true;
                        channel_busy.store(true);
                        std::cout << "[" << now_str()
                                  << "] CHANNEL -> BUSY (power=" << avg_db << " dBFS)\n";
                    }
                } else {
                    if (avg_db <= IDLE_THRESH_DB) {
                        prev_busy = false;
                        channel_busy.store(false);
                        std::cout << "[" << now_str()
                                  << "] CHANNEL -> IDLE (power=" << avg_db << " dBFS)\n";
                    }
                }

                // tiny sleep to yield (recv timeout already controls cadence)
                std::this_thread::sleep_for(std::chrono::microseconds(0));
            }

            // clean up
            stop_rx_stream();
            std::cout << "[" << now_str() << "] RX thread exiting\n";
        });

        // Helper: send exactly N samples (in chunks) or until channel becomes idle
        auto send_samples_or_until_idle = [&](size_t samples_to_send) {
            if (samples_to_send == 0)
                return 0ULL;
            size_t buf_size = tx_buf.size();
            size_t sent = 0;
            bool first = true;

            // declare TX active and stop RX
            tx_active.store(true);
            if (rx_streaming.load())
                stop_rx_stream();

            std::cout << "[" << now_str()
                      << "] TX START (requested samples=" << samples_to_send << ")\n";

            while (sent < samples_to_send && !stop_signal) {
                // if channel turned idle while we were sending, stop early
                if (!channel_busy.load()) {
                    std::cout << "[" << now_str()
                              << "] TX STOPPING EARLY: channel became idle\n";
                    break;
                }

                size_t remaining = samples_to_send - sent;
                size_t to_send = std::min<size_t>(remaining, buf_size);

                tx_md.start_of_burst = first;
                tx_md.end_of_burst = false;
                tx_md.has_time_spec = false;

                size_t actually_sent = 0;
                try {
                    actually_sent = tx_stream->send(&tx_buf.front(), to_send, tx_md, 1.0);
                } catch (const std::exception &e) {
                    std::cerr << "[" << now_str() << "] TX send error: " << e.what()
                              << "\n";
                    break;
                }

                if (actually_sent == 0) {
                    std::cerr << "[" << now_str() << "] TX warning: sent 0 samples\n";
                    std::this_thread::sleep_for(std::chrono::microseconds(50));
                }

                sent += actually_sent;
                first = false;

                // quick yield so RX thread can update channel_busy
                std::this_thread::sleep_for(std::chrono::microseconds(0));
            }

            // send end-of-burst marker (small zero buffer)
            tx_md.start_of_burst = false;
            tx_md.end_of_burst = true;
            try {
                tx_stream->send(
                    &zero_buf.front(), std::min<size_t>(zero_buf.size(), 1), tx_md, 1.0);
            } catch (...) { /* ignore */
            }
            tx_md.end_of_burst = false;

            tx_active.store(false);

            // restart RX streaming
            if (!rx_streaming.load())
                start_rx_stream();

            std::cout << "[" << now_str() << "] TX END (sent samples=" << sent << ")\n";
            return static_cast<unsigned long long>(sent);
        };

        // Packet-level TX main loop
        if (cfg.mode == "continuous") {
            std::cout << "[" << now_str() << "] Entering continuous reactive mode\n";
            while (!stop_signal) {
                // Wait for channel busy (packet start)
                while (!channel_busy.load() && !stop_signal) {
                    std::this_thread::sleep_for(std::chrono::microseconds(50));
                }
                if (stop_signal)
                    break;

                // Packet started -> transmit while packet continues
                std::cout << "[" << now_str()
                          << "] PACKET DETECTED -> begin reactive TX\n";

                // compute samples per iteration (we will send in small chunks)
                double samp_rate = cfg.samp_rate;
                // aim to send in chunks of buf_dur each iteration, but we will break
                // early if idle
                size_t samples_per_iteration = static_cast<size_t>(std::max<size_t>(
                    1, static_cast<size_t>(std::round(cfg.buf_dur * samp_rate))));

                // Keep sending while channel busy
                while (channel_busy.load() && !stop_signal) {
                    send_samples_or_until_idle(samples_per_iteration);
                    // slight pause to allow RX update (channel_busy updated by RX thread)
                    std::this_thread::sleep_for(std::chrono::microseconds(0));
                }

                std::cout << "[" << now_str()
                          << "] Packet ended or stop signal; waiting for next packet\n";
            }

        } else {
            // Burst mode: when packet detected, transmit ON for cfg.on (but cut short if
            // packet ends), then OFF
            std::cout << "[" << now_str() << "] Entering burst reactive mode\n";
            while (!stop_signal) {
                while (!channel_busy.load() && !stop_signal)
                    std::this_thread::sleep_for(std::chrono::microseconds(50));
                if (stop_signal)
                    break;

                std::cout << "[" << now_str() << "] PACKET DETECTED -> burst ON\n";
                // send at most cfg.on seconds, but stop early if packet ends
                size_t max_samples =
                    static_cast<size_t>(std::ceil(cfg.on * cfg.samp_rate));
                send_samples_or_until_idle(max_samples);

                std::cout << "[" << now_str() << "] BURST OFF for " << cfg.off << " s\n";
                auto off_start = std::chrono::steady_clock::now();
                while (std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                                     off_start)
                               .count() < cfg.off &&
                       !stop_signal) {
                    // If channel busy again and we want immediate start, break
                    if (channel_busy.load())
                        break;
                    std::this_thread::sleep_for(std::chrono::microseconds(200));
                }
            }
        }

        // shutdown
        running.store(false);
        if (rx_thread.joinable())
            rx_thread.join();

    } catch (const uhd::exception &e) {
        std::cerr << "[" << now_str() << "] UHD ERROR: " << e.what() << "\n";
        return EXIT_FAILURE;
    } catch (const std::exception &e) {
        std::cerr << "[" << now_str() << "] STD ERROR: " << e.what() << "\n";
        return EXIT_FAILURE;
    }

    std::cout << "[" << now_str() << "] Exiting cleanly\n";
    return EXIT_SUCCESS;
}