#include <signal.h>
#include <unistd.h>
#include <thread>

#include <ros/ros.h>
#include "ros/time.h"
#include "ros/serialization.h"
#include <ros/console.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/TimeReference.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <image_transport/image_transport.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>

#include <librealsense2/rs.hpp>

#include "rs4se.hpp"
#include "../include/auto_exp/auto_exp.h"

#include <chrono>  
#include <math.h>

#define Vector3StampedMsg geometry_msgs::Vector3Stamped
#define ImuMsg sensor_msgs::Imu

using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;


exp_node::ExpNode aer_controller_ir; 

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

int last_frame = 0;

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
  //auto t1 = high_resolution_clock::now();
  if (sequence % 2 == 0) {
    new_exposure_params = aer_controller_ir.CameraCb(0,cv_frame,float(actual_ir_exposure)/1000000,actual_ir_gain_db,header.seq,header.stamp.toSec());
  }
  //auto t2 = high_resolution_clock::now();
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

//auto t3 = high_resolution_clock::now();


// Debug: Printing out processing times
//auto t4 = high_resolution_clock::now();
//auto addframe_ms_int = duration_cast<milliseconds>(t2 - t1);
//auto updateparams_ms_int = duration_cast<milliseconds>(t3 - t2);
//auto sendparams_ms_int = duration_cast<milliseconds>(t4 - t3);
//std::cout << "Update : " << addframe_ms_int.count() << "ms" << " / Send : " << updateparams_ms_int.count() << "ms" << std::endl;

//std::cout << " new exposure " << next_ir_exposure << " new gain " << new_exposure_params.gain << " db" << std::endl; 
    //std::cout << " new exposure " << next_ir_exposure << " new gain " << new_exposure_params.gain << " db" << std::endl; 
//std::cout << " new exposure " << next_ir_exposure << " new gain " << new_exposure_params.gain << " db" << std::endl; 
}

pcl_ptr points_to_pcl(const rs2::points& points)
{
    pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());
    auto ptr = points.get_vertices();
    for (auto& p : cloud->points)
    {
        p.x = ptr->x;
        p.y = ptr->y;
        p.z = ptr->z;
        ptr++;
    }

    return cloud;
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
  const auto msg = cv_bridge::CvImage(header, "16UC1", cv_frame).toImageMsg();

  return msg;
}

 static PointCloud::Ptr create_pointcloud_msg(const rs2::depth_frame &df,
                                              const rs2::video_frame &vf,
                                              const std::string &frame_id,
                                              const bool mid_exposure_ts) {

  // Declare pointcloud object, for calculating pointclouds and texture mappings
  rs2::pointcloud pc;
  // We want the points object to be persistent so we can display the last cloud when a frame drops
  rs2::points points;

  // Tell pointcloud object to map to this color frame
  pc.map_to(vf);
  // Generate the pointcloud and texture mappings
  points = pc.calculate(df);

  auto pcl_points = points_to_pcl(points);


  pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  // Form msg header
  std_msgs::Header header;
  header.frame_id = frame_id;
  
  // Image message
  const int width = df.get_width();
  const int height = df.get_height();

  PointCloud::Ptr msg (new PointCloud);
  msg->header.frame_id = frame_id;

  msg->height = height;
  msg->width = width;

  msg->points.resize(points.size());
  auto ptr = points.get_vertices();
  for (auto& p : msg->points)
  {

      msg->points.push_back (pcl::PointXYZ(ptr->x, ptr->y, ptr->z));
      ptr++;
  }
  //
  
  msg->is_dense = false;
  pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
  
  return msg;
} 

static sensor_msgs::CameraInfoPtr create_depth_camera_info_msg(const rs2::depth_frame &df,
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

  sensor_msgs::CameraInfoPtr msg;
  msg.reset(new sensor_msgs::CameraInfo);
  msg->header = header;

  // Image message
  const int width = df.get_width();
  const int height = df.get_height();
  
  msg->height = height;
  msg->width = width;
  msg->distortion_model = "plumb_bob";
// \"plumb_bob\", the 5 parameters are: (k1, k2, t1, t2, k3).\n\

// Intrinsic camera matrix for the raw (distorted) images.\n\
//     [fx  0 cx]
// K = [ 0 fy cy]
//     [ 0  0  1]

// Projection/camera matrix
//     [fx'  0  cx' Tx]
// P = [ 0  fy' cy' Ty]
//     [ 0   0   1   0]


  msg->D.resize(5); // All 0, no distortion
  msg->K[0] = 389.75244464;
  msg->K[2] = 331.49985938;
  msg->K[4] = 387.96067921;
  msg->K[5] = 229.15638747;
  msg->K[8] = 1.0;
  msg->R[0] = 1.0;
  msg->R[4] = 1.0;
  msg->R[8] = 1.0;
  msg->P[0] = 389.75244464;
  msg->P[2] = 331.49985938;
  msg->P[5] = 387.96067921;
  msg->P[6] = 229.15638747;
  msg->P[10] = 1.0;

  return msg;

}

static sensor_msgs::TimeReference create_external_trigger_msg(const rs2::video_frame &vf, const std::string &frame_id) {
  // Form msg stamp
  const auto ts_ms = vf.get_timestamp();
  const auto ts_ns = str2ts(std::to_string(ts_ms));
  // should work fine since depth_frame is derived from video frame
  ros::Time msg_stamp;
  msg_stamp.fromNSec(ts_ns);

  // Form msg header
  std_msgs::Header header;
  header.frame_id = frame_id;
  header.stamp = msg_stamp;   

  sensor_msgs::TimeReference msg;
  msg.header = header;   
  msg.time_ref = msg_stamp;
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
  ros::Publisher depthinfo_pub_;
  ros::Publisher external_trigger_pub_;
  ros::Publisher pointcloud_pub_;

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
    ROS_PARAM(nh, nn + "/rgb_set_ae", rgbd_config_.rgb_set_ae);
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
    ROS_PARAM(nh, nn + "/ir_sync", rgbd_config_.ir_sync);
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
    const auto pointcloud_topic = nn + "/pointcloud/data";
    const auto depth_info_topic = nn + "/depth0/camera_info";
    const auto trigger_topic = nn + "/external_trigger";
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
      depthinfo_pub_ = nh.advertise<sensor_msgs::CameraInfo>(depth_info_topic, 100);
      //pointcloud_pub_ = nh.advertise<PointCloud> (pointcloud_topic, 100);
    }
    if (rgbd_config_.ir_sync > 0){
      external_trigger_pub_ = nh.advertise<sensor_msgs::TimeReference>(trigger_topic, 100);
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
    const auto msg = create_image_msg(rgb, "rs/rgb0", true, correct_ts, false, false);
    rgb0_pub_.publish(msg);
  }

  void publish_depth0_msg(const rs2::video_frame &depth) {
    const auto correct_ts = rgbd_config_.correct_ts;
    const auto msg = create_depth_msg(depth, "rs/depth0", correct_ts);
    depth0_pub_.publish(msg);
  }

  void publish_depth_info_msg(const rs2::video_frame &depth) {
    const auto correct_ts = rgbd_config_.correct_ts;
    const auto msg = create_depth_camera_info_msg(depth, "rs/depthinfo", correct_ts);
    depthinfo_pub_.publish(msg);
  }

  void publish_external_trigger_msg(const rs2::video_frame &ir0) {
    const auto correct_ts = rgbd_config_.correct_ts;
    const auto msg = create_external_trigger_msg(ir0,"/external_trigger");
    external_trigger_pub_.publish(msg);
  } 

  void publish_pointcloud_msg(const rs2::video_frame &depth , const rs2::video_frame &rgb) {
    const auto correct_ts = rgbd_config_.correct_ts;
    const auto msg = create_pointcloud_msg(depth, rgb, "rs/pointcloud", correct_ts);
    pointcloud_pub_.publish (msg);
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
        const int ir_sync = rgbd_config_.ir_sync;
        const auto ir0 = fs.get_infrared_frame(1);
        if (enable_ir && ir0) {
          const auto sequence = ir0.get_frame_metadata(RS2_FRAME_METADATA_FRAME_COUNTER);
          //const auto gpio = ir0.get_frame_metadata(RS2_FRAME_METADATA_GPIO_INPUT_DATA);
          //std::cout << "gpio " << gpio <<   std::endl;
          
          // skip repeated frames
          if (sequence == last_frame){
            //std::cout << "repeated frame " << last_frame <<   std::endl;
            return;
          }
          last_frame = sequence;
          publish_ir0_msgs(ir0);
          //std::cout << "published sequence " << sequence <<   std::endl;
        }
        if(!enable_ir_left_only){
          const auto ir1 = fs.get_infrared_frame(2);
            if (enable_ir && ir1) 
              publish_ir1_msgs(ir1);
        
        }
        if (enable_ir && ir0) {
          rs2_timestamp_domain ir_timestamp_domain;
          ir_timestamp_domain = fs.get_frame_timestamp_domain();
          //std::cout << " ir_timestamp_domain " << ir_timestamp_domain <<  std::endl;
          if (ir_sync > 0) publish_external_trigger_msg(ir0);
        }
        // Align depth to rgb
        rs2::align align_to_color(RS2_STREAM_COLOR);
        //const auto fs_aligned = align_to_color.process(fs);

        // RGB
        const bool enable_rgb = rgbd_config_.enable_rgb;
        //const auto rgb = fs_aligned.get_color_frame();
        const auto rgb = fs.get_color_frame();
        if (enable_rgb && rgb) {
          publish_rgb0_msg(rgb);
        }

        // Depth image
        const bool enable_depth = rgbd_config_.enable_depth;
        //const auto depth = fs_aligned.get_depth_frame();
        const auto depth = fs.get_depth_frame(); //unaligned to color
        if (enable_depth && depth) {
          publish_depth0_msg(depth);
          publish_depth_info_msg(depth);
        }
        /* D'ont use it's not ready and is slooooow
        if (rgb && depth){
            publish_pointcloud_msg(depth, rgb);  
          } 
          */
      }
    };
    // Connect and stream
    rs2::device device = rs2_connect();
    intel_d435i_t sensor(device, rgbd_config_, motion_config_, cb);
    if (rgbd_config_.set_custom_ae) aer_controller_ir.init();
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
