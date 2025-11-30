#include <uhd/stream.hpp>
#include <uhd/types/tune_request.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/thread.hpp>

#include <chrono>
#include <cmath>
#include <complex>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

static bool stop_signal = false;
void sigint_handler(int) { stop_signal = true; }

std::string now_str() {
    using namespace std::chrono;
    auto now = system_clock::now();
    std::time_t tt = system_clock::to_time_t(now);
    auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;
    std::tm tm = *std::localtime(&tt);
    std::ostringstream ss;
    ss << std::put_time(&tm, "%F %T") << '.' << std::setw(3) << std::setfill('0') << ms.count();
    return ss.str();
}

int main(int argc, char *argv[]) {
    // Default parameters (change safely)
    double samp_rate = 1e6;         // 1 MS/s
    double center_freq = 2462e6;    // 2462 MHz (Wi-Fi channel 11)
    double gain = 30.0;             // RX gain (device-specific)
    std::string args = "type=b200"; // device args for B200/B210
    size_t buffer_len = 1024 * 16;  // samples per rx metadata read
    std::string out_file = "";      // optional: "capture.iq" to save raw
    double thresh_db = -50.0;       // detection threshold in dBFS (default)
    long debounce_ms = 200;         // minimum ms between detection prints
    std::string mode = "peak";      // "peak" | "rms" | "sample"

    // Simple arg parsing (very small)
    for (int i = 1; i < argc; i++) {
        std::string s = argv[i];
        if (s == "--samp-rate" && i + 1 < argc)
            samp_rate = atof(argv[++i]);
        else if (s == "--freq" && i + 1 < argc)
            center_freq = atof(argv[++i]);
        else if (s == "--gain" && i + 1 < argc)
            gain = atof(argv[++i]);
        else if (s == "--args" && i + 1 < argc)
            args = argv[++i];
        else if (s == "--out" && i + 1 < argc)
            out_file = argv[++i];
        else if (s == "--buf" && i + 1 < argc)
            buffer_len = (size_t)atoi(argv[++i]);
        else if (s == "--thresh-db" && i + 1 < argc)
            thresh_db = atof(argv[++i]);
        else if (s == "--debounce-ms" && i + 1 < argc)
            debounce_ms = atol(argv[++i]);
        else if (s == "--mode" && i + 1 < argc)
            mode = argv[++i];
        else if (s == "--help") {
            std::cout << "Usage: " << argv[0] << " [--samp-rate <sps>] [--freq <Hz>] [--gain <dB>] [--args <uhd args>]\n"
                      << "       [--out <file>] [--buf <samples>] [--thresh-db <dBFS>] [--debounce-ms <ms>] [--mode <peak|rms|sample>]\n";
            return 0;
        }
    }

    std::signal(SIGINT, &sigint_handler);

    try {
        // Create USRP device
        std::cout << "Making USRP device with args: " << args << "\n";
        auto usrp = uhd::usrp::multi_usrp::make(args);

        // Set sample rate, center freq, gain
        usrp->set_rx_rate(samp_rate);
        uhd::tune_request_t tune(center_freq);
        usrp->set_rx_freq(tune);
        usrp->set_rx_gain(gain);

        // Allow some settling time
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        std::cout << "USRP RX ready: samp_rate=" << samp_rate << " center_freq=" << center_freq << " gain=" << gain << "\n";
        std::cout << "Detection mode=" << mode << " thresh_db=" << thresh_db << " debounce_ms=" << debounce_ms << "\n";

        // Create a RX streamer
        uhd::stream_args_t stream_args("fc32"); // complex float32
        stream_args.channels = {0};
        auto rx_stream = usrp->get_rx_stream(stream_args);

        // Prepare buffer
        size_t samps_per_buff = buffer_len;
        std::vector<std::complex<float>> buff(samps_per_buff);

        // Optional file output
        std::ofstream fout;
        if (!out_file.empty()) {
            fout.open(out_file, std::ios::binary);
            if (!fout) {
                std::cerr << "Failed to open output file: " << out_file << "\n";
                out_file.clear();
            } else
                std::cout << "Writing captured IQ to: " << out_file << "\n";
        }

        // Start streaming (continuous)
        uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
        stream_cmd.stream_now = true;
        rx_stream->issue_stream_cmd(stream_cmd);

        size_t total_samps = 0;
        auto last_detect_time = std::chrono::steady_clock::time_point::min();

        std::cout << "Starting RX loop. Press Ctrl-C to stop.\n";

        // RX loop
        while (!stop_signal) {
            uhd::rx_metadata_t md;
            size_t num_rx = rx_stream->recv(&buff.front(), buff.size(), md, 1.0);
            if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE) {
                std::cerr << "RX metadata error: " << md.strerror() << "\n";
                if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT)
                    continue;
                else
                    break;
            }
            if (num_rx == 0)
                continue;

            // Optionally write raw IQ (float32 interleaved) to file
            if (fout.is_open()) {
                fout.write(reinterpret_cast<const char *>(buff.data()), num_rx * sizeof(std::complex<float>));
            }

            // Instantaneous detection based on mode
            double block_peak = 0.0;  // max magnitude
            double block_power = 0.0; // sum of powers for RMS
            for (size_t i = 0; i < num_rx; i++) {
                double re = buff[i].real();
                double im = buff[i].imag();
                double mag2 = re * re + im * im;
                double mag = std::sqrt(mag2);
                if (mag > block_peak)
                    block_peak = mag;
                block_power += mag2;

                if (mode == "sample") {
                    // per-sample immediate detection (could be noisy)
                    double db = 20.0 * std::log10(mag + 1e-12);
                    if (db >= thresh_db) {
                        auto now = std::chrono::steady_clock::now();
                        auto since = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_detect_time).count();
                        if (since >= debounce_ms) {
                            std::cout << "[" << now_str() << "] SAMPLE-DETECT: dbFS=" << db << " (sample index approx " << total_samps + i << ")\n";
                            last_detect_time = now;
                        }
                    }
                }
            }

            double mean_power = block_power / double(num_rx);
            double block_rms = std::sqrt(mean_power);
            double peak_db = 20.0 * std::log10(block_peak + 1e-12);
            double rms_db = 20.0 * std::log10(block_rms + 1e-12);

            // Decide detection using chosen mode
            bool detected = false;
            double detect_db = -INFINITY;
            if (mode == "peak") {
                detect_db = peak_db;
                detected = (detect_db >= thresh_db);
            } else if (mode == "rms") {
                detect_db = rms_db;
                detected = (detect_db >= thresh_db);
            } else if (mode == "sample") {
                // sample-mode already handled above per-sample; here also allow block peak print
                detect_db = peak_db;
                detected = (detect_db >= thresh_db);
            } else {
                // unknown mode -> default to peak
                detect_db = peak_db;
                detected = (detect_db >= thresh_db);
            }

            if (detected) {
                auto now = std::chrono::steady_clock::now();
                auto since = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_detect_time).count();
                if (since >= debounce_ms) {
                    std::cout << "[" << now_str() << "] DETECT: mode=" << mode
                              << " peak_dbFS=" << peak_db
                              << " rms_dbFS=" << rms_db
                              << " samples=" << num_rx
                              << " total_samples=" << total_samps + num_rx << "\n";
                    last_detect_time = now;
                }
            }

            total_samps += num_rx;
        }

        // Stop streaming
        uhd::stream_cmd_t stop_cmd(uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
        rx_stream->issue_stream_cmd(stop_cmd);
        if (fout.is_open())
            fout.close();
        std::cout << "Stopped. Total samples captured: " << total_samps << "\n";
    } catch (const std::exception &ex) {
        std::cerr << "Exception: " << ex.what() << "\n";
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
