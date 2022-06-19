#include <signal.h>
#include <unistd.h>
#include <thread>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <image_transport/image_transport.h>

#include <librealsense2/rs.hpp>

#include "rs4se.hpp"
#include <vo_autoexpose/vo_autoexpose.h>
#include "../include/auto_exp/auto_exp.h"

#include <chrono>  
#include <math.h>

#define Vector3StampedMsg geometry_msgs::Vector3Stamped
#define ImuMsg sensor_msgs::Imu

using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;


ExpGainController controller; // Instance of exposure/gain automatic controller

exp_node::ExpNode aer_controller; 



// Signal handler
bool keep_running = true;
void signal_handler(int sig) {
  UNUSED(sig);
  keep_running = false;
}

std::string ros_node_name(int argc, char *argv[]) {
  for (int i = 1; i < argc; i++) {
    std::string arg(argv[i]);
    if (arg.find("__name:=") != std::string::npos) {
      return arg.substr(8);
    }
  }
  FATAL("Failed to find node name?");
}


  void set_exposure_gain(std_msgs::Header &header, cv::Mat &cv_frame, const rs2::video_frame &vf) {
    const auto sequence = vf.get_frame_metadata(RS2_FRAME_METADATA_FRAME_COUNTER);
    const auto actual_ir_exposure = vf.get_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE);
      const auto actual_ir_gain = vf.get_frame_metadata(RS2_FRAME_METADATA_GAIN_LEVEL);
      //std::cout << " vf frame gain " << vf_actual_ir_gain <<  std::endl;
      double actual_ir_gain_db = 20*log10(double(actual_ir_gain));
      //std::cout << " gain actual " <<  actual_ir_gain << std::endl;
      //std::cout << " exposure actual " <<  actual_ir_exposure << std::endl;
      //std::cout << " frame count " << sequence <<  std::endl;
      // Add new frame
      auto t1 = high_resolution_clock::now();
      controller.add_frame(0,cv_frame,float(actual_ir_exposure)/1000000,actual_ir_gain_db,header.seq,header.stamp.toSec());

      // Update the parameters
      auto t2 = high_resolution_clock::now();
      //ExposureParameters new_exposure_params = controller.update_params(GRADINFO_SCORE,EV_CORRECTIONS,IMU_ESTIM);
      ExposureParameters new_exposure_params = controller.update_params(GRADINFO_SCORE,GAMMA_CORRECTIONS,IMU_ESTIM);
      //ExposureParameters new_exposure_params = controller.update_params(GRADINFO_SCORE,GAMMA_CORRECTIONS,OPTIFLOW_ESTIM);

      auto t3 = high_resolution_clock::now();
      unsigned long long int next_ir_exposure = (unsigned long long int)(1000000*new_exposure_params.exposure);
      unsigned long long int next_ir_gain = (unsigned long long int)(pow(10,new_exposure_params.gain/20));
      //std::cout << " gain next " <<  next_ir_gain << std::endl;
      //std::cout << " exposure next " <<  next_ir_exposure << std::endl;
    if (sequence % 2 == 0) {

      if (sequence % 4 != 0 && next_ir_gain != actual_ir_gain) {
        rs2::sensor exposure_sensor = *rs2::sensor_from_frame(vf);

        if (next_ir_gain <16) exposure_sensor.set_option(RS2_OPTION_GAIN, 16);
        else if (next_ir_gain > 248) exposure_sensor.set_option(RS2_OPTION_GAIN, 248);
        else exposure_sensor.set_option(RS2_OPTION_GAIN, next_ir_gain);
        //std::cout << " gain " <<  next_ir_gain << std::endl;
      }
      else if (sequence % 4 == 0 && next_ir_exposure != actual_ir_exposure) {
        rs2::sensor exposure_sensor = *rs2::sensor_from_frame(vf);
        if (next_ir_exposure < 0) exposure_sensor.set_option(RS2_OPTION_EXPOSURE, 1);
        else if (next_ir_exposure > 200000) exposure_sensor.set_option(RS2_OPTION_EXPOSURE, 200000);
        else exposure_sensor.set_option(RS2_OPTION_EXPOSURE, next_ir_exposure);
        //std::cout << " exposure " <<  next_ir_exposure << std::endl;
      }
    }




    // Debug: Printing out processing times
    auto t4 = high_resolution_clock::now();
    /*     auto addframe_ms_int = duration_cast<milliseconds>(t2 - t1);
    auto updateparams_ms_int = duration_cast<milliseconds>(t3 - t2);
    auto sendparams_ms_int = duration_cast<milliseconds>(t4 - t3);
    std::cout << "Add frame: " << addframe_ms_int.count() << "ms" << " / Update: " << updateparams_ms_int.count() << "ms" << " / Send params: " << sendparams_ms_int.count() << "ms" << std::endl;

    std::cout << " new exposure " << next_ir_exposure << " new gain " << new_exposure_params.gain << " db" << std::endl; */
  }

    void set_exposure_gain_aer(std_msgs::Header &header, cv::Mat &cv_frame, const rs2::video_frame &vf) {
    const auto sequence = vf.get_frame_metadata(RS2_FRAME_METADATA_FRAME_COUNTER);
    const auto actual_ir_exposure = vf.get_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE);
      const auto actual_ir_gain = vf.get_frame_metadata(RS2_FRAME_METADATA_GAIN_LEVEL);
      //std::cout << " vf frame gain " << vf_actual_ir_gain <<  std::endl;
      //double actual_ir_gain_db = 20*log10(double(actual_ir_gain));
      double actual_ir_gain_db = 6*log2(double(actual_ir_gain/16));
      //std::cout << " gain actual " <<  actual_ir_gain << std::endl;
      //std::cout << " exposure actual " <<  actual_ir_exposure << std::endl;
      //std::cout << " frame count " << sequence <<  std::endl;
      // Add new frame

      exp_node::ExposureParametersAer new_exposure_params;
      auto t1 = high_resolution_clock::now();
      if (sequence % 2 == 0) {
        new_exposure_params = aer_controller.CameraCb(0,cv_frame,float(actual_ir_exposure)/1000000,actual_ir_gain_db,header.seq,header.stamp.toSec());
      }
      auto t2 = high_resolution_clock::now();
      //std::cout << " exposure main " <<  new_exposure_params.exposure << std::endl;
      //std::cout << " gain main " <<  new_exposure_params.gain << std::endl;

      
      unsigned long long int next_ir_exposure = (unsigned long long int)(new_exposure_params.exposure);
      unsigned long long int next_ir_gain = (unsigned long long int)(pow(2,new_exposure_params.gain/6))*16;
      //std::cout << " gain next " <<  next_ir_gain << std::endl;
      //std::cout << " exposure next " <<  next_ir_exposure << std::endl;
      if (sequence % 2 == 0) {
        //std::cout << " even " << std::endl;
        if (sequence % 4 != 0 && next_ir_gain != actual_ir_gain) {
          rs2::sensor exposure_sensor = *rs2::sensor_from_frame(vf);
          
          if (next_ir_gain <16) exposure_sensor.set_option(RS2_OPTION_GAIN, 16);
          else if (next_ir_gain > 248) exposure_sensor.set_option(RS2_OPTION_GAIN, 248);
          else exposure_sensor.set_option(RS2_OPTION_GAIN, next_ir_gain);
          //std::cout << " gain " <<  next_ir_gain << std::endl;
        }
        else if (sequence % 4 == 0 && next_ir_exposure != actual_ir_exposure) {
          rs2::sensor exposure_sensor = *rs2::sensor_from_frame(vf);
          if (next_ir_exposure < 0) exposure_sensor.set_option(RS2_OPTION_EXPOSURE, 1);
          else if (next_ir_exposure > 200000) exposure_sensor.set_option(RS2_OPTION_EXPOSURE, 200000);
          else exposure_sensor.set_option(RS2_OPTION_EXPOSURE, next_ir_exposure);
          //std::cout << " exposure " <<  next_ir_exposure << std::endl;
        }  
      }
      //else std::cout << " odd " << std::endl;

    auto t3 = high_resolution_clock::now();


    // Debug: Printing out processing times
    auto t4 = high_resolution_clock::now();
    auto addframe_ms_int = duration_cast<milliseconds>(t2 - t1);
    auto updateparams_ms_int = duration_cast<milliseconds>(t3 - t2);
    //auto sendparams_ms_int = duration_cast<milliseconds>(t4 - t3);
    //std::cout << "Update : " << addframe_ms_int.count() << "ms" << " / Send : " << updateparams_ms_int.count() << "ms" << std::endl;

    //std::cout << " new exposure " << next_ir_exposure << " new gain " << new_exposure_params.gain << " db" << std::endl; 
  }

static sensor_msgs::ImagePtr create_image_msg(const rs2::video_frame &vf,
                                              const std::string &frame_id,
                                              bool is_color,
                                              bool mid_exposure_ts,
                                              bool controller_add_frame,
                                              bool set_custom_ae) {
  // Form msg stamp
  const uint64_t ts_ns = vframe2ts(vf, mid_exposure_ts);
  ros::Time msg_stamp;
  msg_stamp.fromNSec(ts_ns);

  // Form msg header
  std_msgs::Header header;
  header.frame_id = frame_id;
  header.stamp = msg_stamp;

  // Image message
  const int width = vf.get_width();
  const int height = vf.get_height();
  if (is_color) {
    cv::Mat cv_frame = frame2cvmat(vf, width, height, CV_8UC3);
    return cv_bridge::CvImage(header, "rgb8", cv_frame).toImageMsg();
  }

  cv::Mat cv_frame = frame2cvmat(vf, width, height, CV_8UC1);

  if(controller_add_frame && set_custom_ae) set_exposure_gain_aer(header, cv_frame, vf); 
 
  return cv_bridge::CvImage(header, "mono8", cv_frame).toImageMsg();
}


static sensor_msgs::ImagePtr create_depth_msg(const rs2::depth_frame &df,
                                              const std::string &frame_id,
                                              const bool mid_exposure_ts) {
  // Form msg stamp
  const uint64_t ts_ns = vframe2ts(df, mid_exposure_ts);
  // should work fine since depth_frame is derived from video frame
  ros::Time msg_stamp;
  msg_stamp.fromNSec(ts_ns);

  // Form msg header
  std_msgs::Header header;
  header.frame_id = frame_id;
  header.stamp = msg_stamp;

  // Image message
  const int width = df.get_width();
  const int height = df.get_height();
  cv::Mat cv_frame = frame2cvmat(df, width, height, CV_16UC1);
  const auto msg = cv_bridge::CvImage(header, "mono16", cv_frame).toImageMsg();

  return msg;
}

static geometry_msgs::Vector3Stamped
create_vec3_msg(const rs2::motion_frame &f, const std::string &frame_id) {
  // Form msg stamp
  double ts_s = f.get_timestamp() * 1e-3;
  ros::Time stamp;
  stamp.fromSec(ts_s);
  // printf("[%s]: %.9f\n", frame_id.c_str(), ts_s);

  // Form msg header
  std_msgs::Header header;
  header.frame_id = frame_id;
  header.stamp = stamp;

  // Form msg
  const rs2_vector data = f.get_motion_data();
  geometry_msgs::Vector3Stamped msg;
  msg.header = header;
  msg.vector.x = data.x;
  msg.vector.y = data.y;
  msg.vector.z = data.z;

  return msg;
}

static sensor_msgs::Imu create_imu_msg(const double ts,
                                       const Eigen::Vector3d &gyro,
                                       const Eigen::Vector3d &accel,
                                       const std::string &frame_id) {
  sensor_msgs::Imu msg;

  msg.header.frame_id = frame_id;
  msg.header.stamp = ros::Time{ts};
  msg.angular_velocity.x = gyro(0);
  msg.angular_velocity.y = gyro(1);
  msg.angular_velocity.z = gyro(2);
  msg.linear_acceleration.x = accel(0);
  msg.linear_acceleration.y = accel(1);
  msg.linear_acceleration.z = accel(2);

  return msg;
}

struct intel_d435i_node_t {
  image_transport::Publisher rgb0_pub_;
  image_transport::Publisher ir0_pub_;
  image_transport::Publisher ir1_pub_;
  image_transport::Publisher depth0_pub_;
  ros::Publisher gyro0_pub_;
  ros::Publisher accel0_pub_;
  ros::Publisher imu0_pub_;

  rs_motion_module_config_t motion_config_;
  rs_rgbd_module_config_t rgbd_config_;

  intel_d435i_node_t(const std::string &nn) {
    ros::NodeHandle nh;

    // ROS params
    // -- RGBD
    // clang-format off
    ROS_PARAM(nh, nn + "/global_time", rgbd_config_.global_time);
    ROS_PARAM(nh, nn + "/correct_ts", rgbd_config_.correct_ts);
    ROS_PARAM(nh, nn + "/set_custom_ae", rgbd_config_.set_custom_ae);
    ROS_PARAM(nh, nn + "/enable_rgb", rgbd_config_.enable_rgb);
    ROS_PARAM(nh, nn + "/enable_ir", rgbd_config_.enable_ir);
    ROS_PARAM(nh, nn + "/enable_ir_left_only", rgbd_config_.enable_ir_left_only);
    ROS_PARAM(nh, nn + "/enable_depth", rgbd_config_.enable_depth);
    ROS_PARAM(nh, nn + "/enable_emitter", rgbd_config_.enable_emitter);
    ROS_PARAM(nh, nn + "/rgb_width", rgbd_config_.rgb_width);
    ROS_PARAM(nh, nn + "/rgb_height", rgbd_config_.rgb_height);
    ROS_PARAM(nh, nn + "/rgb_format", rgbd_config_.rgb_format);
    ROS_PARAM(nh, nn + "/rgb_frame_rate", rgbd_config_.rgb_frame_rate);
    ROS_PARAM(nh, nn + "/rgb_exposure", rgbd_config_.rgb_exposure);
    ROS_PARAM(nh, nn + "/ir_width", rgbd_config_.ir_width);
    ROS_PARAM(nh, nn + "/ir_height", rgbd_config_.ir_height);
    ROS_PARAM(nh, nn + "/ir_format", rgbd_config_.ir_format);
    ROS_PARAM(nh, nn + "/ir_frame_rate", rgbd_config_.ir_frame_rate);
    ROS_PARAM(nh, nn + "/ir_exposure", rgbd_config_.ir_exposure);
    ROS_PARAM(nh, nn + "/ir_gain", rgbd_config_.ir_gain);
    ROS_PARAM(nh, nn + "/depth_width", rgbd_config_.depth_width);
    ROS_PARAM(nh, nn + "/depth_height", rgbd_config_.depth_height);
    ROS_PARAM(nh, nn + "/depth_format", rgbd_config_.depth_format);
    ROS_PARAM(nh, nn + "/depth_frame_rate", rgbd_config_.depth_frame_rate);
    // -- Motion monule
    ROS_PARAM(nh, nn + "/global_time", motion_config_.global_time);
    ROS_PARAM(nh, nn + "/enable_motion", motion_config_.enable_motion);
    ROS_PARAM(nh, nn + "/accel_hz", motion_config_.accel_hz);
    ROS_PARAM(nh, nn + "/gyro_hz", motion_config_.gyro_hz);
    // clang-format on

    // Publishers
    const auto rgb0_topic = nn + "/rgb0/image";
    const auto ir0_topic = nn + "/ir0/image";
    const auto ir1_topic = nn + "/ir1/image";
    const auto depth0_topic = nn + "/depth0/image";
    const auto gyro0_topic = nn + "/gyro0/data";
    const auto accel0_topic = nn + "/accel0/data";
    const auto imu0_topic = nn + "/imu0/data";
    // -- RGB module
    if (rgbd_config_.enable_rgb) {
      image_transport::ImageTransport rgb_it(nh);
      rgb0_pub_ = rgb_it.advertise(rgb0_topic, 100);
    }
    // -- Stereo module
    if (rgbd_config_.enable_ir) {
      image_transport::ImageTransport stereo_it(nh);
      ir0_pub_ = stereo_it.advertise(ir0_topic, 100);
      if(!rgbd_config_.enable_ir_left_only)
      ir1_pub_ = stereo_it.advertise(ir1_topic, 100);
    }
    if (rgbd_config_.enable_depth) {
      image_transport::ImageTransport depth_it(nh);
      depth0_pub_ = depth_it.advertise(depth0_topic, 100);
    }
    // -- Motion module
    if (motion_config_.enable_motion) {
      gyro0_pub_ = nh.advertise<Vector3StampedMsg>(gyro0_topic, 100);
      accel0_pub_ = nh.advertise<Vector3StampedMsg>(accel0_topic, 100);
      imu0_pub_ = nh.advertise<ImuMsg>(imu0_topic, 100);
    }
  }

  void publish_ir_msgs(const rs2::video_frame &ir0,
                       const rs2::video_frame &ir1) {
    const auto correct_ts = rgbd_config_.correct_ts;
    const auto set_custom_ae = rgbd_config_.set_custom_ae;
    const auto cam0_msg = create_image_msg(ir0, "rs/ir0", false, correct_ts, true, set_custom_ae);
    const auto cam1_msg = create_image_msg(ir1, "rs/ir1", false, correct_ts, false, set_custom_ae);
    ir1_pub_.publish(cam1_msg);
    ir0_pub_.publish(cam0_msg);
  }


  void publish_ir0_msgs(const rs2::video_frame &ir0) {
    const auto correct_ts = rgbd_config_.correct_ts;
    const auto set_custom_ae = rgbd_config_.set_custom_ae;
    const auto cam0_msg = create_image_msg(ir0, "rs/ir0", false, correct_ts, true, set_custom_ae);
    ir0_pub_.publish(cam0_msg);
  }

  void publish_ir1_msgs(const rs2::video_frame &ir1) {
    const auto correct_ts = rgbd_config_.correct_ts;
    const auto set_custom_ae = rgbd_config_.set_custom_ae;
    const auto cam1_msg = create_image_msg(ir1, "rs/ir1", false, correct_ts, false, set_custom_ae);
    ir1_pub_.publish(cam1_msg);
  }

  void publish_rgb0_msg(const rs2::video_frame &rgb) {
    const auto correct_ts = rgbd_config_.correct_ts;
    const auto set_custom_ae = rgbd_config_.set_custom_ae;
    const auto msg = create_image_msg(rgb, "rs/rgb0", true, correct_ts, false, set_custom_ae);
    rgb0_pub_.publish(msg);
  }

  void publish_depth0_msg(const rs2::video_frame &depth) {
    const auto correct_ts = rgbd_config_.correct_ts;
    const auto msg = create_depth_msg(depth, "rs/depth0", correct_ts);
    depth0_pub_.publish(msg);
  }

  void publish_accel0_msg(const rs2::motion_frame &mf) {
    const auto msg = create_vec3_msg(mf, "rs/accel0");
    accel0_pub_.publish(msg);
  }

  void publish_gyro0_msg(const rs2::motion_frame &mf) {
    const auto msg = create_vec3_msg(mf, "rs/gyro0");
    gyro0_pub_.publish(msg);
  }

  void publish_imu0_msg(lerp_buf_t &buf) {
    while (buf.lerped_gyro_ts_.size()) {
      // Timestamp
      const auto ts = buf.lerped_gyro_ts_.front();
      buf.lerped_gyro_ts_.pop_front();
      buf.lerped_accel_ts_.pop_front();

      // Accel
      const auto accel = buf.lerped_accel_data_.front();
      buf.lerped_accel_data_.pop_front();

      // Gyro
      const auto gyro = buf.lerped_gyro_data_.front();
      buf.lerped_gyro_data_.pop_front();

      // Publish imu messages
      const auto msg = create_imu_msg(ts, gyro, accel, "rs/imu0");

      // set AE controller IMU data
      Eigen::Matrix<float,3,1> lin_accel; 
      lin_accel << accel(0), accel(1), accel(2);
      Eigen::Matrix<float,3,1> rot_vel; 
      rot_vel << gyro(0),gyro(1),gyro(2);
      controller.add_imu_meas(0,lin_accel,rot_vel,ros::Time{ts}.toSec());

      imu0_pub_.publish(msg);
    }

    buf.clear();
  }

  void stream() {
    // Callback
    lerp_buf_t lerp_buf;
    auto cb = [&](const rs2::frame &frame) {
      // Handle motion frame
      if (auto mf = frame.as<rs2::motion_frame>()) {
        if (mf && mf.get_profile().stream_type() == RS2_STREAM_ACCEL) {
          publish_accel0_msg(mf);

          double ts_s = mf.get_timestamp() * 1e-3;
          const rs2_vector data = mf.get_motion_data();
          lerp_buf.addAccel(ts_s, data.x, data.y, data.z);

        } else if (mf && mf.get_profile().stream_type() == RS2_STREAM_GYRO) {
          publish_gyro0_msg(mf);

          double ts_s = mf.get_timestamp() * 1e-3;
          const rs2_vector data = mf.get_motion_data();
          lerp_buf.addGyro(ts_s, data.x, data.y, data.z);
        }

        if (lerp_buf.ready()) {
          lerp_buf.interpolate();
          publish_imu0_msg(lerp_buf);
        }
        return;
      }

      // Handle camera frames
      if (const auto &fs = frame.as<rs2::frameset>()) {
        // IR0 & IR1
        const bool enable_ir = rgbd_config_.enable_ir;
        const bool enable_ir_left_only = rgbd_config_.enable_ir_left_only;
        const auto ir0 = fs.get_infrared_frame(1);
        if (enable_ir && ir0) {
          publish_ir0_msgs(ir0);
        }
        if(!enable_ir_left_only){
          const auto ir1 = fs.get_infrared_frame(2);
            if (enable_ir && ir1) 
              publish_ir1_msgs(ir1);
        
        }
        
        //if (enable_ir && ir0 && ir1) {
        //  publish_ir_msgs(ir0, ir1);
        //}




        // Align depth to rgb
        rs2::align align_to_color(RS2_STREAM_COLOR);
        const auto fs_aligned = align_to_color.process(fs);

        // RGB
        const bool enable_rgb = rgbd_config_.enable_rgb;
        const auto rgb = fs_aligned.get_color_frame();
        if (enable_rgb && rgb) {
          publish_rgb0_msg(rgb);
        }

        // Depth image
        const bool enable_depth = rgbd_config_.enable_depth;
        const auto depth = fs_aligned.get_depth_frame();
        if (enable_depth && depth) {
          publish_depth0_msg(depth);
        }
      }
    };

    // Connect and stream
    rs2::device device = rs2_connect();
    intel_d435i_t sensor(device, rgbd_config_, motion_config_, cb);

    /*---------------------------------------------
    * INITIALIZATION OF EXPOSURE/GAIN CONTROLLER
    -----------------------------------------------*/

    // TODO: Replace with a single init function that loads from YAML
    Vector5f calib;
    calib << 0.0, 0.0, 0.0, 0.0, 0.0 ;
    Eigen::Matrix<float,4,4> T_imu_2_cam;
    T_imu_2_cam << 0.99992724 , -0.01054832,
       0.00585135, 0.00322414,
       0.01048953, 0.99989509,
       0.00998853, -0.01212793,
       -0.0059561, -0.00992643,
       0.99993299, -0.02028277, 0., 0., 0., 1.       ;
    VectorNcorrect gamma_values;
    gamma_values << 1.0/1.9, 1.0/1.5, 1.0/1.2, 1.0, 1.2, 1.5, 1.9;

    controller.add_camera("left",640,480,2.0,389.75244464,387.96067921,calib);
    controller.add_imu("imu",T_imu_2_cam);
    controller.set_gamma_values(gamma_values);
    controller.init();


    aer_controller.init();

    // Pipelines are threads so we need a blocking loop
    signal(SIGINT, signal_handler);
    while (keep_running) {
      sleep(1);
    }
  }
};

int main(int argc, char **argv) {
  // ROS init
  const std::string node_name = ros_node_name(argc, argv);
  if (ros::isInitialized() == false) {
    ros::init(argc, argv, node_name, ros::init_options::NoSigintHandler);
  }

  // Start ROS node
  intel_d435i_node_t node(node_name);
  node.stream();

  return 0;
}
