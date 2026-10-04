/******************************************************************************
 * This file is part of lslidar_driver.
 *
 * Copyright 2022 LeiShen Intelligent Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "lslidar_driver/lslidar_x10_driver.hpp"

namespace lslidar_driver {
    LslidarX10Driver::LslidarX10Driver(rclcpp::Node::SharedPtr node) : LslidarDriver(node),
                                                                       last_azimuth(0),
                                                                       sweep_end_time(0.0),
                                                                       is_first_sweep(true),
                                                                       current_packet_time(0.0),
                                                                       last_packet_time(0.0),
                                                                       multiecho_scan_(new sensor_msgs::msg::MultiEchoLaserScan()),
                                                                       point_cloud_xyzi_(new pcl::PointCloud<pcl::PointXYZI>()) {
        // Publish complete frames in acquisition order.
        thread_pool_ = std::make_unique<ThreadPool>(1);
    }

    bool LslidarX10Driver::loadParameters() {
        node_->declare_parameter<std::string>("pcap", "");
        node_->declare_parameter<double>("packet_rate", 188.0);
        node_->declare_parameter<std::string>("frame_id", "laser_link");
        node_->declare_parameter<bool>("add_multicast", false);
        node_->declare_parameter<std::string>("group_ip", "224.1.1.2");
        node_->declare_parameter<bool>("use_time_service", false);
        node_->declare_parameter<std::string>("device_ip", "");
        node_->declare_parameter<int>("msop_port", (int) MSOP_DATA_PORT_NUMBER);
        node_->declare_parameter<int>("difop_port", (int) DIFOP_DATA_PORT_NUMBER);
        node_->declare_parameter<std::string>("pointcloud_topic", "lslidar_point_cloud");
        node_->declare_parameter<bool>("use_first_point_time", false);
        node_->declare_parameter<double>("min_range", 0.3);
        node_->declare_parameter<double>("max_range", 150.0);
        node_->declare_parameter<bool>("is_pretreatment", false);
        node_->declare_parameter<double>("x_offset", 0.0);
        node_->declare_parameter<double>("y_offset", 0.0);
        node_->declare_parameter<double>("z_offset", 0.0);
        node_->declare_parameter<double>("roll", 0.0);
        node_->declare_parameter<double>("pitch", 0.0);
        node_->declare_parameter<double>("yaw", 0.0);
        node_->declare_parameter<bool>("read_once", false);
        node_->declare_parameter<bool>("read_fast", false);
        node_->declare_parameter<double>("repeat_delay", 0.0);
        
        node_->declare_parameter<int>("angle_disable_min", 0);
        node_->declare_parameter<int>("angle_disable_max", 0);
        node_->declare_parameter<int>("N10Plus_hz", 10);
        node_->declare_parameter<bool>("publish_scan", false);
        node_->declare_parameter<bool>("use_high_precision", false);
        node_->declare_parameter<bool>("invert_azimuth", false);
        node_->declare_parameter<bool>("publish_multiecholaserscan", false);
        node_->declare_parameter<std::string>("laserscan_topic", "scan");
        node_->declare_parameter<std::string>("lidar_model", "");
        node_->declare_parameter<std::string>("serial_port", "");
        node_->declare_parameter<float>("n301_protocol", 7.0);
        node_->declare_parameter<bool>("enable_noise_filter", false);
        
		
        node_->declare_parameter<bool>("is_MatrixTransformation", false);
        node_->declare_parameter<std::vector<double>>("transform_main", std::vector<double>());
        node_->declare_parameter<std::vector<double>>("transform_imu", std::vector<double>());

        node_->get_parameter("pcap", dump_file);
        node_->get_parameter("packet_rate", packet_rate);
        node_->get_parameter("frame_id", frame_id);
        node_->get_parameter("use_time_service", use_time_service);
        node_->get_parameter("device_ip", lidar_ip_string);
        node_->get_parameter("msop_port", msop_udp_port);
        // node_->get_parameter("difop_port", difop_udp_port);
        node_->get_parameter("pointcloud_topic", pointcloud_topic);
        node_->get_parameter("use_first_point_time", use_first_point_time);
        node_->get_parameter("min_range", min_range);
        node_->get_parameter("max_range", max_range);
        node_->get_parameter("is_pretreatment", is_pretreatment);
        node_->get_parameter("x_offset", x_offset);
        node_->get_parameter("y_offset", y_offset);
        node_->get_parameter("z_offset", z_offset);
        node_->get_parameter("roll", roll);
        node_->get_parameter("pitch", pitch);
        node_->get_parameter("yaw", yaw);

        node_->get_parameter("angle_disable_min", angle_disable_min);
        node_->get_parameter("angle_disable_max", angle_disable_max);
        node_->get_parameter("N10Plus_hz", N10Plus_hz);
        node_->get_parameter("publish_scan", publish_scan);
        node_->get_parameter("use_high_precision", use_high_precision);
        node_->get_parameter("invert_azimuth", invert_azimuth);
        node_->get_parameter("publish_multiecholaserscan", publish_multiecholaserscan);
        node_->get_parameter("laserscan_topic", san_topic_);
        node_->get_parameter("lidar_model", lidar_model);
        node_->get_parameter("serial_port", serial_port_);
        node_->get_parameter("n301_protocol", n301_protocol);
        node_->get_parameter("enable_noise_filter", enable_noise_filter);
        
		// 获取转换矩阵
        node_->get_parameter("is_MatrixTransformation", is_MatrixTransformation);
        auto tTransform_main = node_->get_parameter("transform_main").as_double_array();
        auto tTransform_imu = node_->get_parameter("transform_imu").as_double_array();

        if ((!tTransform_main.empty() && tTransform_main.size() != 16) ||
            (!tTransform_imu.empty() && tTransform_imu.size() != 16)) {
            RCLCPP_ERROR(node_->get_logger(), "Transform matrices must contain exactly 16 values");
            return false;
        }
        Eigen::Matrix4f MatrixTransform_main = Eigen::Matrix4f::Identity();
        Eigen::Matrix4f MatrixTransform_imu = Eigen::Matrix4f::Identity();

        for (int i = 0; i < 16; ++i) {
            if (!tTransform_main.empty()) MatrixTransform_main(i / 4, i % 4) = tTransform_main[i];
            if (!tTransform_imu.empty()) MatrixTransform_imu(i / 4, i % 4) = tTransform_imu[i];
        }

        if ((!tTransform_main.empty()) && (!tTransform_imu.empty()))
        {
            MatrixTransform_result = MatrixTransform_main * MatrixTransform_imu;
        }
        else if ((tTransform_main.empty()) && (!tTransform_imu.empty()))
        {
            MatrixTransform_result = MatrixTransform_imu;
        }
        else
        {
            MatrixTransform_result = MatrixTransform_main;
        }

        return true;
    }

    void LslidarX10Driver::outputParameters() {
        LS_PARAM << "pcap file: " << dump_file << LS_END;
        LS_PARAM << "packet rate: " << packet_rate << LS_END;
        LS_PARAM << "frame id: " << frame_id << LS_END;
        LS_PARAM << "pointcloud topic: " << pointcloud_topic << LS_END;
        LS_PARAM << "use time service: " << std::boolalpha << use_time_service << LS_END;
        LS_PARAM << "use first point time: " << std::boolalpha << use_first_point_time << LS_END;
        LS_PARAM << "publish scan: " << std::boolalpha << publish_scan << LS_END;
        LS_PARAM << "use high precision mode: " << std::boolalpha << use_high_precision << LS_END;
        LS_PARAM << "invert azimuth: " << std::boolalpha << invert_azimuth << LS_END;
        LS_PARAM << "publish multiecholaserscan: " << std::boolalpha << publish_multiecholaserscan << LS_END;
        LS_PARAM << "enable noise filter: " << std::boolalpha << enable_noise_filter << LS_END;
        LS_PARAM << "min range: " << min_range << LS_END;
        LS_PARAM << "max range: " << max_range << LS_END;
        LS_PARAM << "angle disable min: " << angle_disable_min << LS_END;
        LS_PARAM << "angle disable max: " << angle_disable_max << LS_END;
        LS_PARAM << "N10Plus hz: " << N10Plus_hz << LS_END;

        if (is_pretreatment) {
            LS_MSG << "x offset: " << x_offset << LS_END;
            LS_MSG << "y offset: " << y_offset << LS_END;
            LS_MSG << "z offset: " << z_offset << LS_END;
            LS_MSG << "roll: "  << roll << LS_END;
            LS_MSG << "pitch: " << pitch << LS_END;
            LS_MSG << "yaw: "   << yaw << LS_END;
        }

        if (!serial_port_.empty()) { 
            LS_SOCKET << "Only accepting serial data from: " << serial_port_.c_str() << LS_END;
        } else {
            LS_SOCKET << "Only accepting packets from IP address: " << lidar_ip_string.c_str() << LS_END;
            LS_SOCKET << "Opening UDP socket msop port: "  << msop_udp_port << LS_END;
            // LS_SOCKET << "Opening UDP socket difop port: " << difop_udp_port << LS_END;
        }
    }

    bool LslidarX10Driver::createRosIO() {
        pointcloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_topic, 10); 
        if (publish_scan) laserscan_pub_ = node_->create_publisher<sensor_msgs::msg::LaserScan>(san_topic_, 10);
        if (publish_multiecholaserscan && lidar_model == "N10Plus") multiecho_scan_pub_ = node_->create_publisher<sensor_msgs::msg::MultiEchoLaserScan>("multiecho_scan", 10);
        time_pub_ = node_->create_publisher<std_msgs::msg::Float64>("time_topic", 10);

        motor_control_sub_ = node_->create_subscription<std_msgs::msg::Int8>("motor_control",1,
            [this](std_msgs::msg::Int8::SharedPtr msg) {
                this->motorControl(msg);
            });

        if (!serial_port_.empty()) {
            serial_input_ = std::make_shared<lslidar_driver::LSIOSR>(serial_port_, baud_rate);
            data_acquisition_strategy_ = std::make_shared<SerialStrategy>(serial_input_, packet_length, lidar_model);
        } else {
            if (!dump_file.empty()) {
                msop_input_.reset(new lslidar_driver::InputPCAP(node_, msop_udp_port, 1206, packet_rate, dump_file));
            } else {
                msop_input_.reset(new lslidar_driver::InputSocket(node_, msop_udp_port, 1206));
            }
            data_acquisition_strategy_ = std::make_shared<NetworkStrategy>(msop_input_);
        }

        return true;
    }

    void LslidarX10Driver::initTimeStamp() {
        for (int i = 0; i < 10; i++) {
            this->packetTimeStamp[i] = 0;
        }
        this->packet_time_s = 0;
        this->packet_time_ns = 0;
        this->timeStamp = rclcpp::Time(0.0);

        point_time_offset = use_first_point_time ? 1 : 0;
    }

    bool LslidarX10Driver::initAngleConfig() {
        while (angle_disable_min < 0) angle_disable_min += 36000;
        while (angle_disable_max < 0) angle_disable_max += 36000;            
        while (angle_disable_min > 36000) angle_disable_min -= 36000;            
        while (angle_disable_max > 36000) angle_disable_max -= 36000;

        if (angle_disable_max == angle_disable_min) {
            angle_able_min = 0;
            angle_able_max = 36000;
        } else {
            if (angle_disable_min < angle_disable_max && angle_disable_min != 0) {
                angle_able_min = angle_disable_max;
                angle_able_max = angle_disable_min + 36000;
            } else if (angle_disable_min < angle_disable_max && angle_disable_min == 0) {
                angle_able_min = angle_disable_max;
                angle_able_max = 36000;
            } else if (angle_disable_min > angle_disable_max) {
                angle_able_min = angle_disable_max;
                angle_able_max = angle_disable_min;
            }
        }

        // create the sin and cos table for different azimuth values
        for (int j = 0; j < 36000; ++j) {
            float angle = static_cast<float>(j * M_PI / 18000.0);
            sin_azimuth_table[j] = (invert_azimuth ? -1.0f : 1.0f) * sinf(angle);
            cos_azimuth_table[j] = cosf(angle);
        }

        return true;
    }

    bool LslidarX10Driver::initialize() {
        if (!loadParameters()) {
            LS_WARN << "Cannot load all required parameters" << LS_END;
            return false;
        }

        this->outputParameters();

        if(!configureParameters()) {
            LS_WARN << "lidar model error,please check Lidar model." << LS_END;
            return false;
        }
        
        if (!createRosIO()) {
            LS_WARN << "Failed to create ROS2 interfaces" << LS_END;
            return false;
        }

        if (lidar_model == "N301") {
            if(!determineN301Model()) {
                LS_ERROR << "N301 LiDAR protocol error." << LS_END;
                return false;
            }
        }
        
        if (!initAngleConfig()) {
            LS_WARN << "Failed to initialize angle configuration." << LS_END;
            return false;
        }
        
        this->initTimeStamp();

        if (is_pretreatment) {
            pointcloud_transform_.setTransform(x_offset, y_offset, z_offset, roll, pitch, yaw);
        }

        point_cloud_xyzi_->header.frame_id = frame_id;
        point_cloud_xyzi_->height = 1;

        if(publish_multiecholaserscan && lidar_model == "N10Plus") {
            resetMultiEchoLaserScan();
        }

        return true;
    }


    void LslidarX10Driver::publishLiadrData(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                                           double frame_time, double frame_duration) {
        if(cloud->width < 50) return;
        const rclcpp::Time stamp(static_cast<int64_t>(frame_time * 1000000000.0));
        
        if (publish_scan) {
            sensor_msgs::msg::PointCloud2 raw_pc_msg;
            auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
            pcl::toROSMsg(*cloud, raw_pc_msg);
            pointcloudToLaserscan(raw_pc_msg, *scan_msg);
            scan_msg->header.stamp = stamp;
            scan_msg->scan_time = static_cast<float>(std::max(0.0, frame_duration));
            laserscan_pub_->publish(std::move(scan_msg));
        }

        auto pc_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        if (is_pretreatment) pointcloud_transform_.applyTransform(*cloud);
		if (is_MatrixTransformation) pointcloud_transform_.applyTransform_2(*cloud, MatrixTransform_result);
            
        pcl::toROSMsg(*cloud, *pc_msg);

        pc_msg->header.stamp = stamp;
        pointcloud_pub_->publish(*pc_msg); 

        auto time_msg = std::make_shared<std_msgs::msg::Float64>();
        time_msg->data = frame_time;
        time_pub_->publish(*time_msg);

        return;
    }

    void LslidarX10Driver::publishMultiEchoLaserScan(
        std::shared_ptr<sensor_msgs::msg::MultiEchoLaserScan> scan,
        double frame_time, double frame_duration) {
        scan->header.stamp = rclcpp::Time(static_cast<int64_t>(frame_time * 1000000000.0));
        scan->scan_time = static_cast<float>(std::max(0.0, frame_duration));
        multiecho_scan_pub_->publish(*scan);
    }

    void LslidarX10Driver::resetMultiEchoLaserScan() {
        multiecho_scan_ = std::make_unique<sensor_msgs::msg::MultiEchoLaserScan>();
        multiecho_scan_->header.frame_id = frame_id;
        multiecho_scan_->angle_min = -M_PI;
        multiecho_scan_->angle_increment = 2.0 * M_PI / points_size;
        multiecho_scan_->angle_max = multiecho_scan_->angle_min +
            multiecho_scan_->angle_increment * (points_size - 1);
        multiecho_scan_->time_increment = 0.0; 
        multiecho_scan_->range_min = min_range;
        multiecho_scan_->range_max = max_range;

        multiecho_scan_->ranges.resize(points_size);
        multiecho_scan_->intensities.resize(points_size);
        for (int i = 0; i < points_size; ++i) {
            multiecho_scan_->ranges[i].echoes = {INITIAL_RANGE, INITIAL_RANGE};
            multiecho_scan_->intensities[i].echoes = {INITIAL_INTENSITIE, INITIAL_INTENSITIE};
        }
    }

    void LslidarX10Driver::pointcloudToLaserscan(const sensor_msgs::msg::PointCloud2 &cloud_msg,
                                             sensor_msgs::msg::LaserScan &output_scan) {
        output_scan.header = cloud_msg.header;
        output_scan.header.frame_id = cloud_msg.header.frame_id;
        output_scan.angle_min = -M_PI;
        output_scan.time_increment = 0.0; 
        output_scan.range_min = min_range; 
        output_scan.range_max = max_range; 
        if (points_size < 2) throw std::invalid_argument("LaserScan requires at least two bins");
        const size_t ranges_size = static_cast<size_t>(points_size) * (use_high_precision ? 10 : 1);
        const double bin_width = 2.0 * M_PI / ranges_size;
        output_scan.angle_increment = static_cast<float>(bin_width);
        output_scan.angle_max = output_scan.angle_min +
            output_scan.angle_increment * static_cast<float>(ranges_size - 1);
        output_scan.ranges.assign(ranges_size, INITIAL_RANGE); 
        output_scan.intensities.assign(ranges_size, INITIAL_INTENSITIE);

        for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_msg, "x"), iter_y(cloud_msg, "y"),
                iter_z(cloud_msg, "z"), iter_intensity(cloud_msg, "intensity");
            iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_intensity) {

            if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) {
                RCLCPP_DEBUG(node_->get_logger(), "rejected for nan in point(%f, %f, %f)\n", *iter_x, *iter_y, *iter_z);
                continue;
            }

            double range = hypot(*iter_x, *iter_y);
            if (range < min_range || range > max_range) {
                RCLCPP_DEBUG(node_->get_logger(), "rejected for range %f not in range [%f, %f]. Point: (%f, %f, %f)",
                             range, min_range, max_range, *iter_x, *iter_y, *iter_z);
                continue;
            }

            double angle = atan2(*iter_y, *iter_x);
            if (angle >= M_PI) angle -= 2.0 * M_PI;
            const size_t index = std::min(ranges_size - 1,
                static_cast<size_t>(std::floor((angle + M_PI) / bin_width)));
            if (range < output_scan.ranges[index]) {
                output_scan.ranges[index] = range;
                output_scan.intensities[index] = *iter_intensity;
            }
        }
    }

    LslidarX10Driver::~LslidarX10Driver() {
        // Finish queued publications while the X10 members are still alive.
        thread_pool_.reset();
    }

    bool LslidarX10Driver::poll() {
        if (!motor_running.load()) {
            usleep(2000000);
            return true; 
        }
        lslidar_msgs::msg::LslidarPacket::UniquePtr packet(new lslidar_msgs::msg::LslidarPacket());
        int packet_size = -1;
        while (rclcpp::ok()) {
            packet_size = data_acquisition_strategy_->getPacket(packet);
            if (packet_size >= packet_length) break;
            if (packet_size == 0) continue; 
            if (packet_size < 0) return false;
        }

        if (!rclcpp::ok()) return false;
        // Discard damaged packets; a transport failure is reported separately.
        if (!checkPacketValidity(packet, packet_size)) return true;

        if (lidar_model == "M10P") {
            if (serial_port_.empty()) {
                packet_size = (packet->data[2] << 8) + packet->data[3];
                packet_points_max = static_cast<int>(packet_size - 20) * 0.5;
                
                if (packet_size > 180) {
                    // normal packet size: 158-168     normal motor speed: 3460-3490
                    motor_speed = (packet->data[6] << 8) + packet->data[7];
                    LS_WARN << "Abnormal lidar data!  packet_size: " << packet_size << "  motor_speed: " << motor_speed << LS_END;
                    return false;
                }
            } else {
                packet_points_max = static_cast<int>(packet_size - 20) * 0.5;
            }
        }

        decodePacket(packet);
        if (actual_points == 0) return true;

        size_t new_sweep_start = 0;
        do {
            // if (last_azimuth < 18000 && points[new_sweep_start].azimuth >= 18000) {  // 180°为一帧起始
            if (abs(points[new_sweep_start].azimuth - last_azimuth) > 35000) {          // 0°为一帧起始
                break;
            } else {
                last_azimuth = points[new_sweep_start].azimuth;
                ++new_sweep_start;
            }
        } while (new_sweep_start < actual_points);
        
        if (use_time_service && (lidar_model == "M10P" || lidar_model == "M10GPS")) {
            memset(&cur_time, 0, sizeof(cur_time));
            cur_time.tm_year = packet->data[packet_size - 12] + 2000 - 1900;
            cur_time.tm_mon  = packet->data[packet_size - 11] - 1;
            cur_time.tm_mday = packet->data[packet_size - 10];
            cur_time.tm_hour = packet->data[packet_size - 9];
            cur_time.tm_min  = packet->data[packet_size - 8];
            cur_time.tm_sec  = packet->data[packet_size - 7];
            packet_time_s = static_cast<uint64_t>(timegm(&cur_time)); // s
            packet_time_ns = ((packet->data[packet_size - 6] << 8) + packet->data[packet_size - 5]) * 1000000 + 
                             ((packet->data[packet_size - 4] << 8) + packet->data[packet_size - 3]) * 1000;

            timeStamp = rclcpp::Time(packet_time_s, packet_time_ns);
            packet->stamp = timeStamp;
            current_packet_time = timeStamp.seconds();
        } else if (use_time_service && lidar_model == "N301") {
            memset(&cur_time, 0, sizeof(cur_time));
            cur_time.tm_year = packet->data[YEAR] + 2000 - 1900;
            cur_time.tm_mon  = packet->data[MONTH] - 1;
            cur_time.tm_mday = packet->data[DAY];
            cur_time.tm_hour = packet->data[1197];
            cur_time.tm_min  = packet->data[1198];
            cur_time.tm_sec  = packet->data[1199];

            packet_time_s = static_cast<uint64_t>(timegm(&cur_time)); //s
            packet_time_ns = (packet->data[1200] +
                             (packet->data[1201] << 8) +
                             (packet->data[1202] << 16) +
                             (packet->data[1203] << 24)) * 1e3; //ns
            timeStamp = rclcpp::Time(packet_time_s, packet_time_ns % 1000000000);
            packet->stamp = timeStamp;
            current_packet_time = timeStamp.seconds();
        } else {
            packet->stamp = node_->get_clock()->now();
            current_packet_time = node_->get_clock()->now().seconds();
        } 

        size_t start_fir_idx = 0;
        size_t end_fir_idx = new_sweep_start;
        if (is_first_sweep && new_sweep_start == actual_points) {
            return true;
        } else {
            if (is_first_sweep) {
                is_first_sweep = false;
                start_fir_idx = new_sweep_start;
                end_fir_idx = actual_points;
            }
        }

        for (size_t fir_idx = start_fir_idx; fir_idx < end_fir_idx; ++fir_idx) {
            //check if the point is valid
            is_valid_point = isPointValid(fir_idx) && 
                         !(enable_noise_filter && isNoisePoint(fir_idx, end_fir_idx));

            //convert the point to xyz coordinate
            size_t table_idx = points[fir_idx].azimuth;
            float cos_azimuth = cos_azimuth_table[table_idx];
            float sin_azimuth = sin_azimuth_table[table_idx];

            float x_coord = points[fir_idx].distance * cos_azimuth;
            float y_coord = points[fir_idx].distance * sin_azimuth;

            if (is_valid_point) {
                //add point
                pcl::PointXYZI point_xyzi;
                point_xyzi.x = x_coord;
                point_xyzi.y = y_coord;
                point_xyzi.z = 0.0;
                point_xyzi.intensity = points[fir_idx].intensity;
                point_cloud_xyzi_->points.push_back(point_xyzi);
                ++point_cloud_xyzi_->width;
            }

            if (publish_multiecholaserscan && lidar_model == "N10Plus") {
                if (fir_idx == start_fir_idx || points[fir_idx].azimuth != points[fir_idx - 1].azimuth) {
                    double angle = atan2(y_coord, x_coord) + M_PI;
                    int index = std::min(static_cast<int>(angle / multiecho_scan_->angle_increment), 
                                         static_cast<int>(multiecho_scan_->ranges.size()) - 1);

                    sensor_msgs::msg::LaserEcho range_echo;
                    sensor_msgs::msg::LaserEcho intensity_echo;

                    // 第一回波
                    if (is_valid_point) {
                        range_echo.echoes.push_back(points[fir_idx].distance);
                        intensity_echo.echoes.push_back(points[fir_idx].intensity);
                    } else {
                        range_echo.echoes.push_back(INITIAL_RANGE);
                        intensity_echo.echoes.push_back(INITIAL_INTENSITIE);
                    }

                    // 第二回波
                    if (fir_idx + 1 < end_fir_idx && points[fir_idx].azimuth == points[fir_idx + 1].azimuth) {
                        if (isPointValid(fir_idx + 1)) {
                            range_echo.echoes.push_back(points[fir_idx + 1].distance);
                            intensity_echo.echoes.push_back(points[fir_idx + 1].intensity);
                        } else {
                            range_echo.echoes.push_back(INITIAL_RANGE);
                            intensity_echo.echoes.push_back(INITIAL_INTENSITIE);
                        }
                    } else {
                        range_echo.echoes.push_back(INITIAL_RANGE);
                        intensity_echo.echoes.push_back(INITIAL_INTENSITIE);
                    }

                    multiecho_scan_->ranges[index] = range_echo;
                    multiecho_scan_->intensities[index] = intensity_echo;
                }
            }
        }
        // 当前包有下一圈数据
        if (end_fir_idx != actual_points) {
            if (last_packet_time > 1e-6) {
                sweep_end_time = current_packet_time - (current_packet_time - last_packet_time) * 
                                (actual_points - (end_fir_idx + point_time_offset)) / actual_points;
            } else {
                sweep_end_time = current_packet_time;
            }

            sweep_end_time = sweep_end_time > 0 ? sweep_end_time : 0;

            const double frame_duration = last_point_cloud_time > 0.0 ?
                sweep_end_time - last_point_cloud_time : 0.0;
            point_cloud_time = use_first_point_time && last_point_cloud_time > 0.0 ?
                last_point_cloud_time : sweep_end_time;
            last_point_cloud_time = sweep_end_time;
            auto frame = std::move(point_cloud_xyzi_);
            const double frame_time = point_cloud_time;
            thread_pool_->enqueue([this, frame, frame_time, frame_duration]() {
                publishLiadrData(frame, frame_time, frame_duration);
            });
            if(publish_multiecholaserscan && lidar_model == "N10Plus") {
                auto echoes = std::shared_ptr<sensor_msgs::msg::MultiEchoLaserScan>(std::move(multiecho_scan_));
                thread_pool_->enqueue([this, echoes, frame_time, frame_duration]() {
                    publishMultiEchoLaserScan(echoes, frame_time, frame_duration);
                });
                resetMultiEchoLaserScan();
            }

            point_cloud_xyzi_.reset(new pcl::PointCloud<pcl::PointXYZI>());
            point_cloud_xyzi_->header.frame_id = frame_id;
            point_cloud_xyzi_->height = 1;

            last_azimuth = points[actual_points - 1].azimuth;
            start_fir_idx = end_fir_idx;
            end_fir_idx = actual_points;
            for (size_t fir_idx = start_fir_idx; fir_idx < end_fir_idx; ++fir_idx) {
                //check if the point is valid
                is_valid_point = isPointValid(fir_idx) && 
                         !(enable_noise_filter && isNoisePoint(fir_idx, end_fir_idx));
                
                //convert the point to xyz coordinate
                size_t table_idx = points[fir_idx].azimuth;
                float cos_azimuth = cos_azimuth_table[table_idx];
                float sin_azimuth = sin_azimuth_table[table_idx];

                float x_coord = points[fir_idx].distance * cos_azimuth;
                float y_coord = points[fir_idx].distance * sin_azimuth;

                if (is_valid_point) {
                    //add point
                    pcl::PointXYZI point_xyzi;
                    point_xyzi.x = x_coord;
                    point_xyzi.y = y_coord;
                    point_xyzi.z = 0.0;
                    point_xyzi.intensity = points[fir_idx].intensity;
                    point_cloud_xyzi_->points.push_back(point_xyzi);
                    ++point_cloud_xyzi_->width;
                }

                if (publish_multiecholaserscan && lidar_model == "N10Plus") {
                    if (fir_idx == start_fir_idx || points[fir_idx].azimuth != points[fir_idx - 1].azimuth) {
                        double angle = atan2(y_coord, x_coord) + M_PI;
                        int index = std::min(static_cast<int>(angle / multiecho_scan_->angle_increment), 
                                             static_cast<int>(multiecho_scan_->ranges.size()) - 1);

                        sensor_msgs::msg::LaserEcho range_echo;
                        sensor_msgs::msg::LaserEcho intensity_echo;

                        // 第一回波
                        if (is_valid_point) {
                            range_echo.echoes.push_back(points[fir_idx].distance);
                            intensity_echo.echoes.push_back(points[fir_idx].intensity);
                        } else {
                            range_echo.echoes.push_back(INITIAL_RANGE);
                            intensity_echo.echoes.push_back(INITIAL_INTENSITIE);
                        }

                        // 第二回波
                        if (fir_idx + 1 < end_fir_idx && points[fir_idx].azimuth == points[fir_idx + 1].azimuth) {
                            if (isPointValid(fir_idx + 1)) {
                                range_echo.echoes.push_back(points[fir_idx + 1].distance);
                                intensity_echo.echoes.push_back(points[fir_idx + 1].intensity);
                            } else {
                                range_echo.echoes.push_back(INITIAL_RANGE);
                                intensity_echo.echoes.push_back(INITIAL_INTENSITIE);
                            }
                        } else {
                            range_echo.echoes.push_back(INITIAL_RANGE);
                            intensity_echo.echoes.push_back(INITIAL_INTENSITIE);
                        }

                        multiecho_scan_->ranges[index] = range_echo;
                        multiecho_scan_->intensities[index] = intensity_echo;
                    }
                }
            }
        }

        last_packet_time = current_packet_time;

        return true;
    }

    bool LslidarX10Driver::isPointValid(const int fir_idx) const {
        if (!(points[fir_idx].distance >= min_range && points[fir_idx].distance <= max_range)) return false;

        if (angle_able_max > 36000) {
            if ((points[fir_idx].azimuth > (angle_able_max - 36000)) && points[fir_idx].azimuth < angle_able_min)  return false;
        } else {
            if ((points[fir_idx].azimuth > angle_able_max) || points[fir_idx].azimuth < angle_able_min) return false;
        }

        return true;
    }

    bool LslidarX10Driver::isNoisePoint(size_t curr_idx, size_t end_fir_idx) const {
        if (lidar_model == "N10Plus") return false; 
        if (curr_idx == 0 || curr_idx >= end_fir_idx - 1) {
            return false; 
        }

        const auto& curr = points[curr_idx];
        const auto& prev = points[curr_idx - 1];
        const auto& next = points[curr_idx + 1];

        float dynamic_threshold = 0.015f + 0.001f * curr.distance;
        float diff_prev = fabs(curr.distance - prev.distance) / prev.distance;
        float diff_next = fabs(curr.distance - next.distance) / next.distance;

        return (diff_prev > dynamic_threshold) && (diff_next > dynamic_threshold);
    }

    bool LslidarX10Driver::checkPacketValidityM10(const lslidar_msgs::msg::LslidarPacket::UniquePtr &packet, int packet_size) const {
        if (packet->data[0] != 0xA5 || packet->data[1] != 0x5A) {
            return false;
        }
        
        if (packet_size == 1206) {
            return true;
        } 

        return (packet->data[packet_size - 2] == 0xFA && packet->data[packet_size - 1] == 0xFB);
    }

    bool LslidarX10Driver::checkPacketValidityN10(const lslidar_msgs::msg::LslidarPacket::UniquePtr &packet, int packet_size) const {
        if (!packet || packet_size < 3 || packet_size > static_cast<int>(packet->data.size()) ||
            packet_length < 3 || packet_length > static_cast<int>(packet->data.size())) return false;
        if (packet->data[0] != 0xA5 || packet->data[1] != 0x5A) {
            return false;
        }

        if (packet_size == 1206) {
            packet_size = packet_length;
        } 
        if (packet_size != packet_length) return false;

        uint8_t computed_crc = 0;
        for (int i = 0; i < packet_size - 1; ++i) {
            computed_crc += packet->data[i];
        }
        
        if (computed_crc != packet->data[packet_size - 1]) {
            LS_WARN << "Checksum mismatch: calculated 0x" << std::hex << std::setw(2) << std::setfill('0')
                    << static_cast<unsigned int>(computed_crc) << ", received 0x" << std::setw(2)
                    << static_cast<unsigned int>(packet->data[packet_size - 1])
                    << std::dec << std::setfill(' ') << LS_END;
            LS_WARN << "Abandon the current data packet." << LS_END;

            return false;
        }

        return true;
    }

    bool LslidarX10Driver::checkPacketValidityN301(const lslidar_msgs::msg::LslidarPacket::UniquePtr &packet, int packet_size) const {
        for (size_t blk_idx = 0; blk_idx < BLOCKS_PER_PACKET_N301; ++blk_idx) {
            if (packet->data[blk_idx * SIZE_BLOCK_N301] != 0xff || packet->data[blk_idx * SIZE_BLOCK_N301 + 1] != 0xee) {
                return false;
            }
        }

        return true;
    }

    void LslidarX10Driver::decodePacketM10(lslidar_msgs::msg::LslidarPacket::UniquePtr &packet) {
        std::vector<FiringX10> raw_points;
        raw_points.reserve(packet_points_max);

        for (size_t point_idx = 0; point_idx < POINT_LEN_M10 * packet_points_max; point_idx += POINT_LEN_M10) {
            uint16_t raw_distance = (packet->data[point_idx + data_bits_start] << 8) +
                                     packet->data[point_idx + data_bits_start + 1];
            if (raw_distance == 0xFFFF) continue;
        
            FiringX10 point;
            point.distance = static_cast<float>(raw_distance) * DISTANCE_RESOLUTION_X10;
            point.intensity = 0;
            
            raw_points.push_back(point);
        }

        actual_points = raw_points.size();

        points.clear();
        points.reserve(actual_points);

        int angle_CodedDisc = ((packet->data[angle_bits_start] << 8) + packet->data[angle_bits_start + 1]) % 36000; // 误差: 0.01°以内
        for (size_t valid_idx = 0; valid_idx < raw_points.size(); valid_idx++) {
            FiringX10 valid_point = raw_points[valid_idx];
            valid_point.azimuth = angle_CodedDisc + (ANGLE_INTERVAL_M10 / actual_points * valid_idx) * 100;
            valid_point.azimuth = valid_point.azimuth % 36000;
            points.push_back(valid_point);
        }
    }

    void LslidarX10Driver::decodePacketN10(lslidar_msgs::msg::LslidarPacket::UniquePtr &packet) {
        int start_angle = ((packet->data[angle_bits_start] << 8) + packet->data[angle_bits_start + 1]) % 36000;
        int end_angle = ((packet->data[end_angle_bits_start] << 8) + packet->data[end_angle_bits_start + 1]) % 36000;

        int ANGLE_INTERVAL_N10;
        if (start_angle > end_angle) {
            ANGLE_INTERVAL_N10 = end_angle + 36000 - start_angle;
        } else {
            ANGLE_INTERVAL_N10 = end_angle - start_angle;
        }

        actual_points = packet_points_max;

        points.clear();
        points.reserve(actual_points);
				
        for (size_t point_idx = 0, point_num = 0; point_idx < POINT_LEN_N10 * actual_points; point_idx += POINT_LEN_N10, ++point_num) {
            FiringX10 point;
            point.azimuth = (start_angle + (ANGLE_INTERVAL_N10 / (actual_points - 1) * point_num)) % 36000; // 误差: 0.01°以内
            point.distance = static_cast<float >((packet->data[point_idx + data_bits_start] << 8) + 
                                                  packet->data[point_idx + data_bits_start + 1]) * DISTANCE_RESOLUTION_X10;
            point.intensity = static_cast<float>(packet->data[point_idx + data_bits_start + 2]);
            points.push_back(point);
        }

        return;
    }

    void LslidarX10Driver::decodePacketN10Plus(lslidar_msgs::msg::LslidarPacket::UniquePtr &packet) {
        int start_angle = ((packet->data[angle_bits_start] << 8) + packet->data[angle_bits_start + 1]) % 36000;
        int end_angle = ((packet->data[end_angle_bits_start] << 8) + packet->data[end_angle_bits_start + 1]) % 36000;

        int ANGLE_INTERVAL_N10;
        if (start_angle > end_angle) {
            ANGLE_INTERVAL_N10 = end_angle + 36000 - start_angle;
        } else {
            ANGLE_INTERVAL_N10 = end_angle - start_angle;
        }

        actual_points = packet_points_max * 2;
        points.clear();
        points.reserve(actual_points);

        int angle_groups = packet_points_max;
        float angle_increment = static_cast<float>(ANGLE_INTERVAL_N10) / (angle_groups - 1);
                                                    // 16
        for (size_t group_idx = 0; group_idx < angle_groups; ++group_idx) {
            int current_angle = (start_angle + static_cast<int>(angle_increment * group_idx)) % 36000;
            
            for (int echo_idx = 0; echo_idx < 2; ++echo_idx) {
                FiringX10 point;
                point.azimuth = current_angle;
            
                size_t data_offset = data_bits_start + group_idx * 6 + echo_idx * 3;

                point.distance = static_cast<float>((packet->data[data_offset] << 8) + packet->data[data_offset + 1]) * DISTANCE_RESOLUTION_X10;
                point.intensity = static_cast<float>(packet->data[data_offset + 2]);
                
                points.push_back(point);
            }
        }

        return;
    }

    void LslidarX10Driver::decodePacketN301_1_6(lslidar_msgs::msg::LslidarPacket::UniquePtr &packet) {
        uint16_t firing_azimuth[BLOCKS_PER_PACKET_N301];

        for (size_t b_idx = 0; b_idx < BLOCKS_PER_PACKET_N301; ++b_idx) {
            firing_azimuth[b_idx] = packet->data[b_idx * SIZE_BLOCK_N301 + 2] + (packet->data[b_idx * SIZE_BLOCK_N301 + 3] << 8);
        }

        actual_points = packet_points_max;

        points.clear();
        points.reserve(actual_points);

        for (size_t block_idx = 0; block_idx < BLOCKS_PER_PACKET_N301; ++block_idx) {
            int32_t azimuth_diff_b = 0; // 相邻两个方位角的差值
            if (block_idx < BLOCKS_PER_PACKET_N301 - 1) {
                azimuth_diff_b = firing_azimuth[block_idx + 1] - firing_azimuth[block_idx];
            } else {
                azimuth_diff_b = firing_azimuth[block_idx] - firing_azimuth[block_idx - 1];
            }
            azimuth_diff_b = azimuth_diff_b < 0 ? azimuth_diff_b + 36000 : azimuth_diff_b;

            // 第一个点
            FiringX10 point1;
            point1.azimuth = firing_azimuth[block_idx] % 36000;
            point1.distance = static_cast<float>((packet->data[block_idx * SIZE_BLOCK_N301 + 4] + 
                                                 (packet->data[block_idx * SIZE_BLOCK_N301 + 5] << 8)) * DISTANCE_RESOLUTION_N301_1_6);
            point1.intensity = static_cast<float>(packet->data[block_idx * SIZE_BLOCK_N301 + 6]);
            points.push_back(point1);

            // 第二个点
            FiringX10 point2;
            point2.azimuth = firing_azimuth[block_idx] + azimuth_diff_b * 0.5;
            point2.azimuth = point2.azimuth % 36000;
            point2.distance = static_cast<float>((packet->data[block_idx * SIZE_BLOCK_N301 + 52] + 
                                                 (packet->data[block_idx * SIZE_BLOCK_N301 + 53] << 8)) * DISTANCE_RESOLUTION_N301_1_6);
            point2.intensity = static_cast<float>(packet->data[block_idx * SIZE_BLOCK_N301 + 54]);
            points.push_back(point2);
        }
    }

    void LslidarX10Driver::decodePacketN301_1_7(lslidar_msgs::msg::LslidarPacket::UniquePtr &packet) {
        uint16_t firing_azimuth[BLOCKS_PER_PACKET_N301];

        for (size_t b_idx = 0; b_idx < BLOCKS_PER_PACKET_N301; ++b_idx) {
            firing_azimuth[b_idx] = (packet->data[b_idx * SIZE_BLOCK_N301 + 2] + (packet->data[b_idx * SIZE_BLOCK_N301 + 3] << 8)) % 36000;
        }

        actual_points = packet_points_max;

        points.clear();
        points.reserve(actual_points);

        // 计算每个点的角度、距离和强度
        for (size_t block_idx = 0; block_idx < BLOCKS_PER_PACKET_N301; ++block_idx) {
            int32_t azimuth_diff_b = 0;
            if (block_idx < BLOCKS_PER_PACKET_N301 - 1) {
                azimuth_diff_b = firing_azimuth[block_idx + 1] - firing_azimuth[block_idx];
            } else {
                azimuth_diff_b = firing_azimuth[block_idx] - firing_azimuth[block_idx - 1];
            }
            azimuth_diff_b = azimuth_diff_b < 0 ? azimuth_diff_b + 36000 : azimuth_diff_b;

            // 每一块30个点
            for (size_t scan_fir_idx = 0; scan_fir_idx < SCANS_PER_BLOCK_N301_1_7; ++scan_fir_idx) {
                if (scan_fir_idx == 15) continue;   // 跳过非点信息 每块第 49 50 51 byte为 年 月 日
                size_t byte_idx = RAW_SCAN_SIZE_N301 * scan_fir_idx;
                size_t idx = scan_fir_idx;
                if (scan_fir_idx > 15) idx -= 1;

                FiringX10 point;
                point.azimuth = firing_azimuth[block_idx] + idx * azimuth_diff_b / FIRING_TOFFSET_N301_1_7;
                point.azimuth = point.azimuth % 36000;
                point.distance = static_cast<float>((packet->data[block_idx * SIZE_BLOCK_N301 + byte_idx + 4] + 
                                                    (packet->data[block_idx * SIZE_BLOCK_N301 + byte_idx + 5] << 8)) * DISTANCE_RESOLUTION_N301_1_7);
                point.intensity = static_cast<float>(packet->data[block_idx * SIZE_BLOCK_N301 + byte_idx + 6]);

                points.push_back(point);
            }
        }
    }

    bool LslidarX10Driver::judgmentProtocol(lslidar_msgs::msg::LslidarPacket::UniquePtr &packet) {
        for (size_t idx = 0; idx < BLOCKS_PER_PACKET_N301; ++idx) {
            // 1.6协议 每个数据块前 7 个字节为FF EE 角度 第一个点
            for (size_t i = 7; i < SIZE_BLOCK_N301; ++i) {
                if (i == 52 || i == 53 || i == 54) continue;    //  第二个点
                if (packet->data[idx * SIZE_BLOCK_N301 + i] != 0x00) {
                    return true;    // 1.7 协议
                }
            }
        }

        return false; // 1.6 协议
    }

    bool LslidarX10Driver::determineN301Model() {
        if (n301_protocol > 3.0) {    // launch不指定协议，启用自动判断
            lslidar_msgs::msg::LslidarPacket::UniquePtr pkt(new lslidar_msgs::msg::LslidarPacket());
            int packet_size;
            while (rclcpp::ok()) {
                packet_size = msop_input_->getPacket(pkt);
                if (packet_size == packet_length) {
                    if (!checkPacketValidity(pkt, packet_size)) continue;
                    
                    break;
                } 
                if (packet_size == 0) continue; 
                if (packet_size < 0) return false;
            }

            if (judgmentProtocol(pkt)) {
                n301_protocol = 1.7;
            } else {
                n301_protocol = 1.6;
            }
        }

        const float EPSILON = 0.0001;
        if (std::fabs(n301_protocol - 1.6) < EPSILON) {
            YEAR  = 1194;
            MONTH = 1195;
            DAY   = 1196;
            packet_points_max = BLOCKS_PER_PACKET_N301 * FIRING_TOFFSET_N301_1_6;
            decodePacket = std::bind(&LslidarX10Driver::decodePacketN301_1_6, this, std::placeholders::_1);
        } else if (n301_protocol > 1.6) {
            YEAR  = 1149;
            MONTH = 1150;
            DAY   = 1151;
            packet_points_max = BLOCKS_PER_PACKET_N301 * FIRING_TOFFSET_N301_1_7;
            decodePacket = std::bind(&LslidarX10Driver::decodePacketN301_1_7, this, std::placeholders::_1);
        }

        LS_INFO << "Protocol" << n301_protocol << LS_END;

        return true;
    }

    bool LslidarX10Driver::configureParameters() {
        if (lidar_model == "M10" ) {
            baud_rate = BaudRate::BAUD_460800;
            packet_length = 92;
            packet_points_max = 42;
            angle_bits_start = 2;
            data_bits_start = 6;
            points_size = ceil(M10_PULSE_FREQUENCY / M10_MOTOR_FREQUENCY);
            decodePacket = std::bind(&LslidarX10Driver::decodePacketM10, this, std::placeholders::_1);
            checkPacketValidity = std::bind(&LslidarX10Driver::checkPacketValidityM10, this, std::placeholders::_1, std::placeholders::_2);
        } else if (lidar_model == "M10GPS") {
            baud_rate = BaudRate::BAUD_460800;
            packet_length = 102;
            packet_points_max = 42;
            angle_bits_start = 2;
            data_bits_start = 6;
            points_size = ceil(M10_PULSE_FREQUENCY / M10_MOTOR_FREQUENCY);
            decodePacket = std::bind(&LslidarX10Driver::decodePacketM10, this, std::placeholders::_1);
            checkPacketValidity = std::bind(&LslidarX10Driver::checkPacketValidityM10, this, std::placeholders::_1, std::placeholders::_2);
        } else if (lidar_model == "M10P") {
            baud_rate = BaudRate::BAUD_500000;
            packet_length = 140;
            angle_bits_start = 4;
            data_bits_start = 8;
            points_size = ceil(M10P_PULSE_FREQUENCY / M10P_MOTOR_FREQUENCY);
            decodePacket = std::bind(&LslidarX10Driver::decodePacketM10, this, std::placeholders::_1);
            checkPacketValidity = std::bind(&LslidarX10Driver::checkPacketValidityM10, this, std::placeholders::_1, std::placeholders::_2);
        } else if (lidar_model == "N10") {
            baud_rate = BaudRate::BAUD_230400;
            packet_length = 58;
			packet_points_max = 16;
            angle_bits_start = 5;
            end_angle_bits_start = 55;
			data_bits_start = 7;
            points_size = ceil(N10_PULSE_FREQUENCY / N10_MOTOR_FREQUENCY);
            decodePacket = std::bind(&LslidarX10Driver::decodePacketN10, this, std::placeholders::_1);
            checkPacketValidity = std::bind(&LslidarX10Driver::checkPacketValidityN10, this, std::placeholders::_1, std::placeholders::_2);
		} else if (lidar_model == "N10Plus") {
            if (N10Plus_hz < 6 || N10Plus_hz > 12) {
                RCLCPP_ERROR(node_->get_logger(), "N10Plus_hz must be in [6, 12]");
                return false;
            }
            baud_rate = BaudRate::BAUD_460800;
            packet_length = 108;
			packet_points_max = 16;
			angle_bits_start = 5;
			end_angle_bits_start = 105;
            data_bits_start = 7;
            points_size = ceil(N10PLUS_PULSE_FREQUENCY / N10Plus_hz);
            decodePacket = std::bind(&LslidarX10Driver::decodePacketN10Plus, this, std::placeholders::_1);
            checkPacketValidity = std::bind(&LslidarX10Driver::checkPacketValidityN10, this, std::placeholders::_1, std::placeholders::_2);
		} else if (lidar_model == "N301") {
            packet_length = 1206;
            points_size = ceil(N301_PULSE_FREQUENCY / N301_MOTOR_FREQUENCY);
            checkPacketValidity = std::bind(&LslidarX10Driver::checkPacketValidityN301, this, std::placeholders::_1, std::placeholders::_2);
        } else {
            return false;
        }

        LS_INFO << "Lidar model: " << lidar_model.c_str() << LS_END;

        return true;
    }

    void LslidarX10Driver::motorControl(const std_msgs::msg::Int8::SharedPtr msg) {
        int motor_command = msg->data;
        if (motor_command != 0 && motor_command != 1) {
            LS_WARN << "Input parameter error.  option 0 or 1" << LS_END;
            return;
        }

        unsigned char data[188]= {0x00};
        data[0] = 0xA5;
        data[1] = 0x5A;
        data[2] = 0x55;
        data[186] = 0xFA;
        data[187] = 0xFB;

        if (motor_command == 0) {           // 停止且不发数据
            data[184] = 0x03;
            data[185] = char(motor_command);
        } else if (motor_command == 1) {    // 旋转
            data[184] = 0x01;
            data[185] = char(motor_command);
        }

        std::string status = (motor_command == 1) ? "rotation" : "stopped";
        ssize_t rc = 0;
        if (!serial_port_.empty()) {
            rc = serial_input_->send((const char *)data, 188);
        } else {
            rc = msop_input_->sendPacket(data, 188);
        }

        if (rc < 0) {
            LS_ERROR << "Serial port data failed" << LS_END;
        } else if (rc == 188) {
            LS_MSG << "Set successfully! LiDAR " << status << "." << LS_END;
            LS_MSG << "" << LS_END;
        }

        if (motor_command == 1) {
            motor_running.store(true);
        } else if (motor_command == 0) {
            motor_running.store(false);
        }
    }

}  // namespace lslidar_driver
