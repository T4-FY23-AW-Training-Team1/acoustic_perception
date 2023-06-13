// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>
#include <cmath>
#include <string>
#include <fstream>
#include <sstream>
#include <thread>
#include <atomic>
#include <mutex>
#include <signal.h>
#include <alsa/asoundlib.h>
#include <Eigen/Dense>
#include <unsupported/Eigen/FFT>
#include <libusb-1.0/libusb.h>

#include "rclcpp/rclcpp.hpp"
#include <acoustics_msgs/msg/sound_source_direction.hpp>
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;
using namespace visualization_msgs;

class AlsaReader {
  public:
    AlsaReader(const char *DEVICE_NAME, snd_pcm_format_t FORMAT, int NUM_CHANNELS, unsigned int SAMPLE_RATE, int ONE_SHOT_RECORDING_DURATION_MS, const rclcpp::Logger &logger) : logger_(logger), is_running_(false) {
        // Open PCM device for recording.
        int rc = snd_pcm_open(&handle, DEVICE_NAME, SND_PCM_STREAM_CAPTURE, 0);
        if (rc < 0) {
          RCLCPP_ERROR(rclcpp::get_logger("PCM Device"), "Unable to open pcm device: %s", snd_strerror(rc));
          rclcpp::shutdown();
        }

        snd_pcm_hw_params_alloca(&params);
        snd_pcm_hw_params_any(handle, params);
        snd_pcm_hw_params_set_access(handle, params, SND_PCM_ACCESS_RW_INTERLEAVED);
        snd_pcm_hw_params_set_format(handle, params, FORMAT);
        snd_pcm_hw_params_set_channels(handle, params, NUM_CHANNELS);
        channels = NUM_CHANNELS;
        unsigned int rate = SAMPLE_RATE;
        snd_pcm_hw_params_set_rate_near(handle, params, &rate, &dir);

        frames = SAMPLE_RATE * ONE_SHOT_RECORDING_DURATION_MS / 1000;
        snd_pcm_hw_params_set_period_size_near(handle, params, &frames, &dir);

        // Use configuration for device
        rc = snd_pcm_hw_params(handle, params);
        if (rc < 0) {
              RCLCPP_ERROR(rclcpp::get_logger("PCM Device"), "Unable to set hw parameters: %s", snd_strerror(rc));
              rclcpp::shutdown();
        }
        //RCLCPP_INFO(rclcpp::get_logger("PCM DEVICE"), "Constructor Done");
    }

    ~AlsaReader() {
        stop();
        snd_pcm_drain(handle);
        snd_pcm_close(handle);
    }

    void start() {
        is_running_ = true;
        reader_thread_ = std::thread(&AlsaReader::readLoop, this);
        //RCLCPP_INFO(rclcpp::get_logger("PCM DEVICE"), "ALSA reader started");
    }

    void stop() {
        if (is_running_) {
            is_running_ = false;
            if (reader_thread_.joinable()) {
                reader_thread_.join();
            }
        }
    }

    std::vector<int16_t> record() {
      //RCLCPP_INFO(rclcpp::get_logger("PCM DEVICE"), "Start recording");
      std::lock_guard<std::mutex> lock(data_mutex_);
      return data_;
    }

    snd_pcm_uframes_t getExpectedFrames(){
      return frames;
    }

  private:
    void readLoop() {
        while (is_running_) {
          //RCLCPP_INFO(rclcpp::get_logger("PCM DEVICE"), "is_running_ = TRUE, frames = %ld", frames);
          std::vector<int16_t> new_data(frames*channels);
          //RCLCPP_INFO(rclcpp::get_logger("PCM DEVICE"), "start snd_pcm_readi");
          int rc = snd_pcm_readi(handle, new_data.data(), frames);
          if (rc == -EPIPE) {
              RCLCPP_ERROR(rclcpp::get_logger("PCM Device"), "overrun occurred");
              snd_pcm_prepare(handle);
          } else if (rc < 0) {
              RCLCPP_ERROR(rclcpp::get_logger("PCM Device"), "error from read: %s", snd_strerror(rc));
          } else if (rc != (int)frames) {
              RCLCPP_ERROR(rclcpp::get_logger("PCM Device"), "short read, read %d frames", rc);
          }
          //RCLCPP_INFO(rclcpp::get_logger("PCM DEVICE"), "move to data");
          {
              std::lock_guard<std::mutex> lock(data_mutex_);
              data_ = std::move(new_data);
          }
        }
    }

    snd_pcm_t *handle;
    snd_pcm_hw_params_t *params;
    snd_pcm_uframes_t frames;
    int channels;
    rclcpp::Logger logger_;
    int dir;

    std::atomic<bool> is_running_;
    std::thread reader_thread_;
    std::mutex data_mutex_;
    std::vector<int16_t> data_;
};


class AlsaHandle {
public:
    AlsaHandle(const char *DEVICE_NAME, snd_pcm_format_t FORMAT, int NUM_CHANNELS, unsigned int SAMPLE_RATE, int ONE_SHOT_RECORDING_DURATION_MS, const rclcpp::Logger &logger) : logger_(logger) {
        // Open PCM device for recording.
        int rc = snd_pcm_open(&handle, DEVICE_NAME, SND_PCM_STREAM_CAPTURE, 0);
        if (rc < 0) {
          RCLCPP_ERROR(rclcpp::get_logger("PCM Device"), "Unable to open pcm device: %s", snd_strerror(rc));
          rclcpp::shutdown();
        }

        snd_pcm_hw_params_alloca(&params);
        snd_pcm_hw_params_any(handle, params);
        snd_pcm_hw_params_set_access(handle, params, SND_PCM_ACCESS_RW_INTERLEAVED);
        snd_pcm_hw_params_set_format(handle, params, FORMAT);
        snd_pcm_hw_params_set_channels(handle, params, NUM_CHANNELS);
        unsigned int rate = SAMPLE_RATE;
        snd_pcm_hw_params_set_rate_near(handle, params, &rate, &dir);

        frames = SAMPLE_RATE * ONE_SHOT_RECORDING_DURATION_MS / 1000;
        snd_pcm_hw_params_set_period_size_near(handle, params, &frames, &dir);

        // Use configuration for device
        rc = snd_pcm_hw_params(handle, params);
        if (rc < 0) {
              RCLCPP_ERROR(rclcpp::get_logger("PCM Device"), "Unable to set hw parameters: %s", snd_strerror(rc));
              rclcpp::shutdown();
        }
    }

    ~AlsaHandle() {
        snd_pcm_drain(handle);
        snd_pcm_close(handle);
    }

    std::vector<int16_t> record(const int num_channels) {
    //void record(){
        std::vector<int16_t> data(frames * num_channels);
        //std::vector<int16_t> data(120);
        //RCLCPP_INFO(rclcpp::get_logger("PCM Device"), "readi start");
        int rc = snd_pcm_readi(handle, data.data(), frames);
        //RCLCPP_INFO(rclcpp::get_logger("PCM Device"), "readi end");
        if (rc == -EPIPE) {
            RCLCPP_ERROR(rclcpp::get_logger("PCM Device"), "overrun occurred");
            snd_pcm_prepare(handle);
        } else if (rc < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("PCM Device"), "error from read: %s", snd_strerror(rc));
        } else if (rc != (int)frames) {
            RCLCPP_ERROR(rclcpp::get_logger("PCM Device"), "short read, read %d frames", rc);
        }
        return data;
    }

    snd_pcm_uframes_t getExpectedFrames(){
      return frames;
    }

private:
    snd_pcm_t *handle;
    snd_pcm_hw_params_t *params;
    snd_pcm_uframes_t frames;
    rclcpp::Logger logger_;
    int dir;
};

class RespeakerPublisher : public rclcpp::Node
{
  public:

    RespeakerPublisher() : Node("micarray_driver")
    {
      active_probability = 0.01;
      duration_time_ = 0.0;
      printf("Started\n");
      
      // "recordings" group parameters
      this->declare_parameter<int>("recordings.sample_rate", 16000);
      this->declare_parameter<int>("recordings.bit_depth", 16);
      this->declare_parameter<int>("recordings.num_channels", 6);
      this->declare_parameter<std::vector<int64_t>>("recordings.target_channels", {1,2,3,4});
      this->declare_parameter<int>("recordings.format", 2); // This is the value of an enum factor"SND_PCM_FORMAT_S16_LE". Look enum _snd_pcm_format in pcm.h
      this->declare_parameter<std::string>("recordings.device_name", "hw:CARD=ArrayUAC10");
      this->declare_parameter<int>("recordings.recordings_duration_ms", 500);
      this->declare_parameter<int>("recordings.ros_timer_period_ms", 500);
      this->declare_parameter<double>("recordings.sound_speed", 343.0);

      // "MUSIC" group parameters
      this->declare_parameter<double>("MUSIC.min_frequency", 700);
      this->declare_parameter<double>("MUSIC.max_frequency", 1000);
      this->declare_parameter<double>("MUSIC.spectrum_threshold", 10.0);
      this->declare_parameter<double>("MUSIC.maintain_likelihood", 0.80);
      this->declare_parameter<double>("MUSIC.activity_threshold", 0.80);
      this->declare_parameter<double>("MUSIC.wait_time_length", 1.00);
      this->declare_parameter<int>("MUSIC.resolution_degree", 1);
      this->declare_parameter<int>("MUSIC.closeness_threshold", 30);
      this->declare_parameter<std::string>("MUSIC.microphone_arrangement", "");

      // get parameter values
      this->get_parameter("recordings.sample_rate", sample_rate);
      this->get_parameter("recordings.bit_depth", bit_depth);
      this->get_parameter("recordings.num_channels", num_channels);
      this->get_parameter("recordings.target_channels", target_channels);
      int format_index;
      this->get_parameter("recordings.format", format_index);
      format = static_cast<snd_pcm_format_t>(format_index);
      
      
      this->get_parameter("recordings.device_name", device_name);
      this->get_parameter("recordings.recordings_duration_ms", recordings_duration_ms);
      this->get_parameter("recordings.ros_timer_period_ms", ros_timer_period_ms);
      this->get_parameter("recordings.sound_speed", sound_speed);

      this->get_parameter("MUSIC.min_frequency", min_frequency);
      this->get_parameter("MUSIC.max_frequency", max_frequency);
      this->get_parameter("MUSIC.spectrum_threshold", spectrum_threshold);
      this->get_parameter("MUSIC.maintain_likelihood", maintain_likelihood);
      this->get_parameter("MUSIC.activity_threshold", activity_threshold);
      this->get_parameter("MUSIC.wait_time_length", wait_time_length);
      this->get_parameter("MUSIC.resolution_degree", resolution_degree);
      this->get_parameter("MUSIC.closeness_threshold", closeness_threshold);
      this->get_parameter("MUSIC.microphone_arrangement", micpos_csv);
      micpos = load_csv(micpos_csv);
      source_time_limit_ = wait_time_length;
      
      std::ostringstream oss;
      oss << "MicPos =\n" << micpos;

      RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());

      //alsa_handle_ = std::make_unique<AlsaHandle>(device_name.c_str(), format, num_channels, (unsigned int)sample_rate, recordings_duration_ms, this->get_logger());
      alsa_handle_ = std::make_unique<AlsaReader>(device_name.c_str(), format, num_channels, (unsigned int)sample_rate, recordings_duration_ms, this->get_logger());
      alsa_handle_->start();

      signal(SIGINT, RespeakerPublisher::signal_handler);
      signal(SIGTERM, RespeakerPublisher::signal_handler);

      direction_publisher_ = this->create_publisher<acoustics_msgs::msg::SoundSourceDirection>("~/output/sound_source_direction", 10);
      arrow_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("~/output/arrow", 10);
      timer_ = this->create_wall_timer(std::chrono::milliseconds(ros_timer_period_ms), std::bind(&RespeakerPublisher::timer_callback, this));
    }

    ~RespeakerPublisher() {
      alsa_handle_->stop();
      alsa_handle_.reset();
    }

    static void signal_handler(int signal){
      if (alsa_handle_){
        alsa_handle_->stop();
        alsa_handle_.reset();
      }
      exit(signal);
    }

  private:

    void timer_callback(){
      //RCLCPP_INFO(this->get_logger(), "Timer Callback");
      auto direction_message = acoustics_msgs::msg::SoundSourceDirection();
      auto arrow_message = visualization_msgs::msg::Marker();
      direction_message.header.stamp = this->now();
      direction_message.header.frame_id = "base_link";
      arrow_message.header = direction_message.header;
      arrow_message.ns = "direction_arrow";
      arrow_message.id = 0;
      arrow_message.type = visualization_msgs::msg::Marker::ARROW;
      arrow_message.action = visualization_msgs::msg::Marker::ADD;

      // Record data
      //const int num_frames = (int)alsa_handle_->getExpectedFrames();
      const int num_samples = (int)alsa_handle_->getExpectedFrames() * num_channels;
      std::vector<int16_t> data(num_samples);
      //std::vector<int16_t> data(20*num_channels);
      std::vector<std::vector<int16_t>> sep_data(num_channels);
      //std::string debugmsg = "Start recording";
      //RCLCPP_INFO(this->get_logger(), debugmsg.c_str());
      //data = alsa_handle_->record(num_channels);
      data = alsa_handle_->record();
      //RCLCPP_INFO(this->get_logger(), "End recording");
      //alsa_handle_->record();

      // Separate data
      //RCLCPP_INFO(this->get_logger(), "start separation (data_size %ld)", data.size());
      for (int i = 0; i < num_samples; i++){
        int chIdx = i % num_channels;
        sep_data[chIdx].push_back(data[i]);
      }

     // Get target channels
     //RCLCPP_INFO(this->get_logger(), "extract target channels (sep_data_size %ld)", sep_data[0].size());
      std::vector<std::vector<int16_t>> objChannels;
      for (int64_t index : target_channels){
              objChannels.push_back(sep_data[index]);
      }

      // Get STFT result
      //RCLCPP_INFO(this->get_logger(), "start stft (obj_chan_size: %ld %ld)", objChannels.size(), objChannels[0].size());
      std::vector<Eigen::MatrixXcd> stft_results;
      //RCLCPP_INFO(this->get_logger(), "target_channel size is %d",target_channels.size());
      for(int i = 0; i < (int)objChannels.size(); i++){
          stft_results.push_back(stft(objChannels[i], 512, 160));
      }

      // MUSIC
      /* Eigen::MatrixXd micpos(4,2);
      micpos << 1, 1,
                -1, 1,
                -1,-1,
                1, -1;
      micpos = micpos * 0.0457/2;
      int resolution = 1;*/
      // load micpos
      //std::ostringstream oss;
      //oss << "Start MUSIC:\n " << stft_results[0];
      //RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
      Eigen::VectorXd MUSICresult = MUSIC(stft_results, micpos, resolution_degree);

      //std::ostringstream oss;
      //oss << "Pmu: \n" << MUSICresult << std::endl;
      //RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());

      Eigen::VectorXd::Index MUSICIndex;
      double maxMUSICspcetrum = MUSICresult.maxCoeff(&MUSICIndex);
      direction_message.max_spectrum = maxMUSICspcetrum;
      
      int ssDir = MUSICIndex * resolution_degree;
      int retVAD = maxMUSICspcetrum > spectrum_threshold;

      if(ssDir >= 0){
        float temprad = float(ssDir)*M_PI/180;
        direction_message.unit_direction_x = cos(temprad);
        direction_message.unit_direction_y = sin(temprad);
      }else{
        direction_message.unit_direction_x = 0;
        direction_message.unit_direction_y = 0;
        RCLCPP_INFO(this->get_logger(), "Cannot get sound source direction");
      }

      direction_message.activity = retVAD;

      // Single Source Tracker
      double inner_product = old_direction.unit_direction_x * direction_message.unit_direction_x + old_direction.unit_direction_y * direction_message.unit_direction_y;
      //RCLCPP_INFO(this->get_logger(), "???=%d", closeness_threshold);
      if(direction_message.max_spectrum >= spectrum_threshold){
        if(duration_time_ <  (float)ros_timer_period_ms/1000/2 || old_direction.header.frame_id.empty()){
          RCLCPP_INFO(this->get_logger(), "Found new source!!");
          duration_time_ = (float)ros_timer_period_ms/1000;
          source_time_limit_ = wait_time_length;
          old_direction = direction_message;
        }else if(inner_product > cos(closeness_threshold * M_PI / 180)){  // The inner product of two unit directions will directly be the cos value of the angle between them
          RCLCPP_INFO(this->get_logger(), "Closeness confirmed (inpro=%lf, cos(thre)=%lf)", inner_product, cos(closeness_threshold * M_PI / 180));
          duration_time_ += (float)ros_timer_period_ms/1000;
          source_time_limit_ = wait_time_length;
          old_direction = direction_message;
        }else if(direction_message.max_spectrum > old_direction.max_spectrum){
          RCLCPP_INFO(this->get_logger(), "Something else?");
          duration_time_ = (float)ros_timer_period_ms/1000;
          source_time_limit_ = wait_time_length;
          old_direction = direction_message;
        }else{
          // Just caught some moise
          source_time_limit_ -= (float)ros_timer_period_ms/1000;
          if(source_time_limit_ <= 0.0){
            RCLCPP_INFO(this->get_logger(), "Time's over (Dead for %lf seconds)", wait_time_length);
            duration_time_ = 0.0;
          }else{
            RCLCPP_INFO(this->get_logger(), "%lf seconds remaining", source_time_limit_);
            duration_time_ += (float)ros_timer_period_ms/1000;
          }
        }
      }else{
        source_time_limit_ -= (float)ros_timer_period_ms/1000;
        if(source_time_limit_ <= 0.0){
          RCLCPP_INFO(this->get_logger(), "Time's over (Dead for %lf seconds)", wait_time_length);
          duration_time_ = 0.0;
        }else{
          RCLCPP_INFO(this->get_logger(), "%lf seconds remaining", source_time_limit_);
          duration_time_ += (float)ros_timer_period_ms/1000;
        }
      }      
      direction_message.duration_time = duration_time_;

      /*
      // Update probability and edit duration time
      if(retVAD == 1){
        active_probability = active_probability * maintain_likelihood / (active_probability * maintain_likelihood + (1-active_probability)*(1-maintain_likelihood));
      }else{
        active_probability = active_probability * (1-maintain_likelihood) / (active_probability * (1-maintain_likelihood) + (1-active_probability)*maintain_likelihood);
      }

      if(active_probability < 0.01)
        active_probability = 0.01;

      if(active_probability >= activity_threshold){
        duration_time_ += (float)ros_timer_period_ms/1000;
        direction_message.duration_time = duration_time_;
      }else{
        duration_time_ = 0;
        direction_message.duration_time = 0.0;
      }*/

      // Illustrate the estimated direction
      int r = 3; // ratio of the arrow size

      geometry_msgs::msg::Point start_point, end_point;
      start_point.x = 0.0;
      start_point.y = 0.0;
      start_point.z = 0.0;
      end_point.x = direction_message.unit_direction_x * r;
      end_point.y = direction_message.unit_direction_y * r;
      end_point.z = 0.0 * r;
      arrow_message.points.push_back(start_point);
      arrow_message.points.push_back(end_point);

      // Set arrow scale
      arrow_message.scale.x = 0.1*r; // shaft diameter
      arrow_message.scale.y = 0.2*r; // head diameter
      arrow_message.scale.z = 0.5*r; // head length

      // Set arrow color
      if(retVAD == 1){
        arrow_message.color.r = 1.0;
        arrow_message.color.g = 0.0;
        arrow_message.color.b = 0.0;
        arrow_message.color.a = 1.0;
      }else{
        arrow_message.color.r = 0.0;
        arrow_message.color.g = 1.0;
        arrow_message.color.b = 0.0;
        arrow_message.color.a = 0.5;
      }

      RCLCPP_INFO(this->get_logger(), "Publishing SoundSourceDirection: '%.2f, %.2f, (%d degrees), %lf, %d, %lf'", direction_message.unit_direction_x, direction_message.unit_direction_y,  ssDir, direction_message.max_spectrum, direction_message.activity, direction_message.duration_time);
      direction_publisher_->publish(direction_message);
      arrow_publisher_->publish(arrow_message);
    }

    Eigen::MatrixXd load_csv(const std::string& csvfile){
      std::vector<double> data;
      int rows = 0, cols = 0;
      std::ifstream file(csvfile);
      std::string line;
      if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", csvfile.c_str());
        return Eigen::MatrixXd();
      }
      while (std::getline(file, line)){
          std::stringstream ss(line);
          std::string cell;
          int current_cols = 0;
          while (std::getline(ss, cell, ',')){
              data.push_back(std::stod(cell));
              current_cols++;
          }
          if (rows == 0){
              cols = current_cols;
          }
          rows++;
      }

      return Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(data.data(), rows, cols);
    }

    Eigen::MatrixXcd stft(const std::vector<int16_t>& audio_data, size_t window_size, size_t step_size) {

      Eigen::FFT<double> fft;
      size_t num_windows = (audio_data.size() - window_size) / step_size + 1;

      Eigen::MatrixXcd result(window_size/2+1, num_windows);
      
      for (size_t i = 0; i < num_windows; ++i) {
          Eigen::VectorXd windowed_audio(window_size);
          
          for (size_t j = 0; j < window_size; ++j) {
              double hann_window = 0.5 * (1 - std::cos(2 * M_PI * j / (window_size - 1)));
              windowed_audio[j] = audio_data[i * step_size + j] * hann_window;
          }
          
          Eigen::VectorXcd fft_result(window_size);
          fft.fwd(fft_result, windowed_audio);
          Eigen::VectorXcd fft_result_pos = fft_result.topRows(fft_result.rows() / 2 + 1);
          result.col(i) = fft_result_pos;
      }
      return result;
    }

    Eigen::VectorXd MUSIC(const std::vector<Eigen::MatrixXcd>& stft_results, const Eigen::MatrixXd mic_position, const int resolution){
      int num_mic = stft_results.size();
      int num_bin = stft_results[0].rows();
      int num_frame = stft_results[0].cols();

      // Define size of output from resolution
      Eigen::VectorXd output = Eigen::VectorXd::Zero(360/resolution);

      // Get target frequency bin
      double unit_freq = (double)sample_rate/2/num_bin;
      double min_targ_freq = min_frequency;
      double max_targ_freq = max_frequency;
      int min_targ_freq_bin_index = (int)round(min_targ_freq / unit_freq);
      int max_targ_freq_bin_index = (int)round(max_targ_freq / unit_freq);

      for(int freq_bin = min_targ_freq_bin_index; freq_bin <= max_targ_freq_bin_index; freq_bin++){

          // Calculate correlation matrix
          Eigen::MatrixXcd R(num_mic, num_mic);
          Eigen::MatrixXcd X(num_mic, num_frame);

          for(int r = 0; r < num_mic; r++){
              for(int c = 0; c < num_frame; c++){
                  X(r,c) = stft_results[r](freq_bin,c);
              }
          }
          R = (X * X.adjoint()) / num_frame;

          Eigen::ComplexEigenSolver<Eigen::MatrixXcd> es(R);

          std::vector<std::pair<double, Eigen::VectorXcd>> eigens;

          for (int i = 0; i < es.eigenvalues().size(); ++i)
          {
              std::complex<double> lambda = es.eigenvalues()(i);
              Eigen::VectorXcd v = es.eigenvectors().col(i);
              eigens.push_back(std::make_pair(lambda.real(), v));
          }

          // sort eigenvectors by eigenvalues
          std::sort(eigens.begin(), eigens.end(), [](const std::pair<double, Eigen::VectorXcd> &a, const std::pair<double, Eigen::VectorXcd> &b) {
              return a.first < b.first;
          });

          int ns = 1;
          Eigen::MatrixXcd En(num_mic, num_mic-ns);
          for(int i = 0; i < num_mic-ns; i++){
              En.col(i) = eigens[i].second;
          }

          for(int i = 0; i < output.size(); i++){
              // Calculate sterring vector
              double targdeg = resolution*i * M_PI / 180; // in radians
              Eigen::VectorXcd steering_vector(num_mic);
              Eigen::VectorXd unit_vector(2);
              unit_vector << cos(targdeg), sin(targdeg);
              steering_vector = mic_position * unit_vector / (sound_speed) * 2*M_PI*unit_freq*freq_bin * std::complex<double>(0,1);
              steering_vector = steering_vector.array().exp();

              // Calculate MUSIC spectrum
              output[i] = output[i] + (num_mic / (En.adjoint() * steering_vector).eval().norm());
              Eigen::VectorXcd weight_vector = steering_vector / num_mic;
          }
      }
      output = output / (max_targ_freq_bin_index - min_targ_freq_bin_index + 1);

      return output;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<acoustics_msgs::msg::SoundSourceDirection>::SharedPtr direction_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr arrow_publisher_;
    //static std::unique_ptr<AlsaHandle> alsa_handle_;
    static std::unique_ptr<AlsaReader> alsa_handle_;

    // variaables defined in yaml
    int sample_rate, bit_depth, num_channels, recordings_duration_ms, ros_timer_period_ms, resolution_degree, closeness_threshold;
    snd_pcm_format_t format;
    std::vector<int64_t> target_channels;
    std::string device_name;
    std::string micpos_csv;
    Eigen::MatrixXd micpos;
    acoustics_msgs::msg::SoundSourceDirection old_direction;
    double sound_speed, min_frequency, max_frequency, spectrum_threshold, maintain_likelihood, activity_threshold, wait_time_length;
    // variables not defined in yaml
    double active_probability;
    double source_time_limit_;
    double duration_time_;
};

//std::unique_ptr<AlsaHandle> RespeakerPublisher::alsa_handle_ = nullptr;
std::unique_ptr<AlsaReader> RespeakerPublisher::alsa_handle_ = nullptr;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RespeakerPublisher>());
  rclcpp::shutdown();
  return 0;
}
