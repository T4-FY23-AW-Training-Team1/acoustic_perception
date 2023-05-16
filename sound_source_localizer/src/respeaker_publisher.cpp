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
#include <signal.h>
#include <alsa/asoundlib.h>
#include <Eigen/Dense>
#include <unsupported/Eigen/FFT>
#include <libusb-1.0/libusb.h>

#include "rclcpp/rclcpp.hpp"
#include "acoustics_msgs/msg/sound_source_direction.hpp"
#include "visualization_msgs/msg/marker.h"

#define VENDOR_ID 0x2886  // ReSpeakerのVendor ID
#define PRODUCT_ID 0x0018  // ReSpeakerのProduct ID

// オーディオ設定
const int SAMPLE_RATE = 16000;
const int BIT_DEPTH = 16;
const int NUM_CHANNELS = 6;
const snd_pcm_format_t FORMAT = SND_PCM_FORMAT_S16_LE;
//const char *DEVICE_NAME = "alsa_input.usb-SEEED_ReSpeaker_4_Mic_Array__UAC1.0_-00.multichannel-input";
const char *DEVICE_NAME = "hw:CARD=ArrayUAC10";
const int RECORDING_DURATION_MS = 500;
const int TIMER_PERIOD_MS = 500;
const double SOUND_SPEED = 343.0;


using namespace std::chrono_literals;

volatile bool stop_flag = false;
void signal_handler(int signum){
  stop_flag = true;
}

class AlsaHandle {
public:
    AlsaHandle() {
        // Open PCM device for recording.
        int rc = snd_pcm_open(&handle, DEVICE_NAME, SND_PCM_STREAM_CAPTURE, 0);
        if (rc < 0) {
            std::cerr << "unable to open pcm device: " << snd_strerror(rc) << std::endl;
            exit(1);
        }

        snd_pcm_hw_params_alloca(&params);
        snd_pcm_hw_params_any(handle, params);
        snd_pcm_hw_params_set_access(handle, params, SND_PCM_ACCESS_RW_INTERLEAVED);
        snd_pcm_hw_params_set_format(handle, params, FORMAT);
        snd_pcm_hw_params_set_channels(handle, params, NUM_CHANNELS);
        unsigned int rate = SAMPLE_RATE;
        snd_pcm_hw_params_set_rate_near(handle, params, &rate, &dir);

        frames = SAMPLE_RATE * RECORDING_DURATION_MS / 1000;
        snd_pcm_hw_params_set_period_size_near(handle, params, &frames, &dir);

        // Use configuration for device
        rc = snd_pcm_hw_params(handle, params);
        if (rc < 0) {
            std::cerr << "unable to set hw parameters: " << snd_strerror(rc) << std::endl;
            exit(1);
        }
    }

    ~AlsaHandle() {
        snd_pcm_drain(handle);
        snd_pcm_close(handle);
    }

    std::vector<int16_t> record(const int num_channels) {
        std::vector<int16_t> data(frames * num_channels);
        int rc = snd_pcm_readi(handle, data.data(), frames);
        if (rc == -EPIPE) {
            std::cerr << "overrun occurred" << std::endl;
            snd_pcm_prepare(handle);
        } else if (rc < 0) {
            std::cerr << "error from read: " << snd_strerror(rc) << std::endl;
        } else if (rc != (int)frames) {
            std::cerr << "short read, read " << rc << " frames" << std::endl;
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
    int dir;
};

class RespeakerPublisher : public rclcpp::Node
{
  public:
    RespeakerPublisher() : Node("respeaker_driver")
    {
      active_probability = 0.01;
      maintain_likelihood = 0.8;
      active_threshold = 0.8;
      duration_time_ = 0.0;
      alsa_handle_ = std::make_unique<AlsaHandle>();

      direction_publisher_ = this->create_publisher<acoustics_msgs::msg::SoundSourceDirection>("sound_source_direction", 10);
      arrow_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("sound_source_direction/arrow", 10);
      timer_ = this->create_wall_timer(std::chrono::milliseconds(TIMER_PERIOD_MS), std::bind(&RespeakerPublisher::timer_callback, this));
    }

  private:
    void timer_callback(){
      auto direction_message = acoustics_msgs::msg::SoundSourceDirection();
      auto arrow_message = visualization_msgs::msg::Marker();
      direction_message.header.stamp = this->now();
      direction_message.header.frame_id = "sensor_kit_base_link";
      arrow_message.header = direction_message.header;
      arrow_message.ns = "direction_arrow";
      arrow_message.id = 0;
      arrow_message.type = visualization_msgs::msg::Marker::ARROW;
      arrow_message.action = visualization_msgs::msg::Marker::ADD;

      // Record data
      //const int num_frames = (int)alsa_handle_->getExpectedFrames();
      const int num_samples = (int)alsa_handle_->getExpectedFrames() * NUM_CHANNELS;
      std::vector<int16_t> data(num_samples);
      std::string debugmsg = "Start recording";
      RCLCPP_INFO(this->get_logger(), debugmsg.c_str());
      data = alsa_handle_->record(NUM_CHANNELS);
      

     // Get Ch 1 to 4
      std::vector<std::vector<int16_t>> objChannels(4);

      int offset = 1;
      for (int i = 0; i < num_samples; i++){
          int chIdx = i % NUM_CHANNELS;
          if(chIdx >= offset && chIdx <= offset+3)
          {
              objChannels[chIdx-offset].push_back(data[i]);
          }
      }

      // Get STFT result
      std::vector<Eigen::MatrixXcd> stft_results;
      for(int i = 0; i < (int)objChannels.size(); i++){
          stft_results.push_back(stft(objChannels[i], 512, 160));
      }

      // MUSIC
      Eigen::MatrixXd micpos(4,2);
      micpos << 1, 1,
                -1, 1,
                -1,-1,
                1, -1;
      micpos = micpos * 0.0457/2;
      int resolution = 1;
      Eigen::VectorXd MUSICresult = MUSIC(stft_results, micpos, resolution);

      Eigen::VectorXd::Index MUSICIndex;
      double MUSIC_threshold = 10.0;
      double maxMUSICspcetrum = MUSICresult.maxCoeff(&MUSICIndex);
      
      int ssDir = MUSICIndex * resolution;
      int retVAD = maxMUSICspcetrum > MUSIC_threshold;

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

      // Update probability and edit duration time
      if(retVAD == 1){
        active_probability = active_probability * maintain_likelihood / (active_probability * maintain_likelihood + (1-active_probability)*(1-maintain_likelihood));
      }else{
        active_probability = active_probability * (1-maintain_likelihood) / (active_probability * (1-maintain_likelihood) + (1-active_probability)*maintain_likelihood);
      }

      if(active_probability < 0.01)
        active_probability = 0.01;

      if(active_probability >= active_threshold){
        duration_time_ += 0.02;
        direction_message.duration_time = duration_time_;
      }else{
        duration_time_ = 0;
        direction_message.duration_time = 0.0;
      }

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

      RCLCPP_INFO(this->get_logger(), "Publishing SoundSourceDirection: '%.2f, %.2f, %d, %f'", direction_message.unit_direction_x, direction_message.unit_direction_y, direction_message.activity, direction_message.duration_time);
      direction_publisher_->publish(direction_message);
      arrow_publisher_->publish(arrow_message);
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

      // Assuming 1 deg resolution
      Eigen::VectorXd output = Eigen::VectorXd::Zero(360/resolution);

      // Get target frequency bin
      double unit_freq = (double)8000/num_bin;
      double min_targ_freq = 750;
      double max_targ_freq = 920;
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

          // 固有値の昇順でソート
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
              steering_vector = mic_position * unit_vector / (SOUND_SPEED) * 2*M_PI*unit_freq*freq_bin * std::complex<double>(0,1);
              steering_vector = steering_vector.array().exp();

              // Calculate MUSIC spectrum
              output(i) = output(i) + (num_mic / (En.adjoint() * steering_vector).eval().norm());
              Eigen::VectorXcd weight_vector = steering_vector / num_mic;
          }
      }
      output = output / (max_targ_freq_bin_index - min_targ_freq_bin_index + 1);

      return output;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<acoustics_msgs::msg::SoundSourceDirection>::SharedPtr direction_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr arrow_publisher_;
    std::unique_ptr<AlsaHandle> alsa_handle_;
    double active_probability;
    double maintain_likelihood;
    double active_threshold;
    float duration_time_;
};

int main(int argc, char * argv[])
{
  signal(SIGINT, signal_handler);
  signal(SIGTSTP, signal_handler);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RespeakerPublisher>());
  rclcpp::shutdown();
  return 0;
}
