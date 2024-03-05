/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "netft_rdt_driver.h"
#include <cstdint>
#include <exception>

using boost::asio::ip::udp;

namespace netft_rdt_driver {
    struct RDTRecord {
        uint32_t rdt_sequence_;
        uint32_t ft_sequence_;
        uint32_t status_;
        int32_t fx_;
        int32_t fy_;
        int32_t fz_;
        int32_t tx_;
        int32_t ty_;
        int32_t tz_;

        enum { RDT_RECORD_SIZE = 36 };
        void unpack(const uint8_t* buffer);
        static uint32_t unpack32(const uint8_t* buffer);
    };

    uint32_t RDTRecord::unpack32(const uint8_t* buffer) {
        return
            (static_cast<uint32_t>(buffer[0]) << 24) |
            (static_cast<uint32_t>(buffer[1]) << 16) |
            (static_cast<uint32_t>(buffer[2]) << 8) |
            (static_cast<uint32_t>(buffer[3]) << 0);
    }

    void RDTRecord::unpack(const uint8_t* buffer) {
        rdt_sequence_ = unpack32(buffer + 0);
        ft_sequence_ = unpack32(buffer + 4);
        status_ = unpack32(buffer + 8);
        fx_ = static_cast<int32_t>(unpack32(buffer + 12));
        fy_ = static_cast<int32_t>(unpack32(buffer + 16));
        fz_ = static_cast<int32_t>(unpack32(buffer + 20));
        tx_ = static_cast<int32_t>(unpack32(buffer + 24));
        ty_ = static_cast<int32_t>(unpack32(buffer + 28));
        tz_ = static_cast<int32_t>(unpack32(buffer + 32));
    }


    struct RDTCommand {
        uint16_t command_header_{};
        uint16_t command_{};
        uint32_t sample_count_{};

        RDTCommand() : command_header_(HEADER) {
            // empty
        }

        enum { HEADER = 0x1234 };

        // Possible values for command_
        enum {
            CMD_STOP_STREAMING = 0,
            CMD_START_HIGH_SPEED_STREAMING = 2,
            // More command values are available but are not used by this driver
        };

        // Special values for sample count
        enum { INFINITE_SAMPLES = 0 };

        enum { RDT_COMMAND_SIZE = 8 };

        //!Packet structure into buffer for network transport
        //  Buffer should be RDT_COMMAND_SIZE
        void pack(uint8_t* buffer) const;
    };

    void RDTCommand::pack(uint8_t* buffer) const {
        // Data is big-endian
        buffer[0] = (command_header_ >> 8) & 0xFF;
        buffer[1] = (command_header_ >> 0) & 0xFF;
        buffer[2] = (command_ >> 8) & 0xFF;
        buffer[3] = (command_ >> 0) & 0xFF;
        buffer[4] = (sample_count_ >> 8) & 0xFF;
        buffer[5] = (sample_count_ >> 0) & 0xFF;
        buffer[6] = (sample_count_ >> 8) & 0xFF;
        buffer[7] = (sample_count_ >> 0) & 0xFF;
    }

    struct CalibrationInfoResponse {
        uint16_t header_;
        uint8_t force_units_;
        uint8_t torque_units_;
        uint32_t counts_per_force_;
        uint32_t counts_per_torque_;
        uint16_t scale_factors_[6];

        // Possible values for force units
        enum {
            FORCE_UNIT_POUND = 1,
            FORCE_UNIT_NEWTON = 2,
            FORCE_UNIT_KILOPOUND = 3,
            FORCE_UNIT_KILONEWTON = 4,
            FORCE_UNIT_KILOGRAM = 5,
            FORCE_UNIT_GRAM = 6,
        };

        // Possible values for torque units
        enum {
            TORQUE_UNIT_POUND_INCH = 1,
            TORQUE_UNIT_POUND_FOOT = 2,
            TORQUE_UNIT_NEWTON_METER = 3,
            TORQUE_UNIT_NEWTON_MILLIMETER = 4,
            TORQUE_UNIT_KILOGRAM_CENTIMETER = 5,
            TORQUE_UNIT_KILONEWTON_METER = 6,
        };

        enum { CALIB_INFO_RESPONSE_SIZE = 24 };
        void unpack(const uint8_t* buffer);
        static uint32_t unpack32(const uint8_t* buffer);
        static uint16_t unpack16(const uint8_t* buffer);
        static uint8_t unpack8(const uint8_t* buffer);
        std::string getForceUnitString();
        std::string getTorqueUnitString();
    };

    uint32_t CalibrationInfoResponse::unpack32(const uint8_t* buffer) {
        return
            (static_cast<uint32_t>(buffer[0]) << 24) |
            (static_cast<uint32_t>(buffer[1]) << 16) |
            (static_cast<uint32_t>(buffer[2]) << 8) |
            (static_cast<uint32_t>(buffer[3]) << 0);
    }

    uint16_t CalibrationInfoResponse::unpack16(const uint8_t* buffer) {
        return
            (static_cast<uint16_t>(buffer[0]) << 8) |
            (static_cast<uint16_t>(buffer[1]) << 0);
    }

    uint8_t CalibrationInfoResponse::unpack8(const uint8_t* buffer) {
        return
            (static_cast<uint8_t>(buffer[0]) << 0);
    }

    void CalibrationInfoResponse::unpack(const uint8_t* buffer) {
        header_ = unpack16(buffer + 0);
        force_units_ = unpack8(buffer + 2);
        torque_units_ = unpack8(buffer + 3);
        counts_per_force_ = unpack32(buffer + 4);
        counts_per_torque_ = unpack32(buffer + 8);
        scale_factors_[0] = unpack16(buffer + 12);
        scale_factors_[1] = unpack16(buffer + 14);
        scale_factors_[2] = unpack16(buffer + 16);
        scale_factors_[3] = unpack16(buffer + 18);
        scale_factors_[4] = unpack16(buffer + 20);
        scale_factors_[5] = unpack16(buffer + 22);
    }

    std::string CalibrationInfoResponse::getForceUnitString()
    {
        switch (force_units_)
        {
        case FORCE_UNIT_POUND:
            return "Pounds";
        case FORCE_UNIT_NEWTON:
            return "Newtons";
        case FORCE_UNIT_KILOPOUND:
            return "Kilopounds";
        case FORCE_UNIT_KILONEWTON:
            return "Kilonewtons";
        case FORCE_UNIT_KILOGRAM:
            return "Kilograms";
        case FORCE_UNIT_GRAM:
            return "Grams";
        default:
            return "Unknown units";
        }
    }

    std::string CalibrationInfoResponse::getTorqueUnitString()
    {
        switch (torque_units_)
        {
        case TORQUE_UNIT_POUND_INCH:
            return "Inch-Pounds";
        case TORQUE_UNIT_POUND_FOOT:
            return "Pound-Feet";
        case TORQUE_UNIT_NEWTON_METER:
            return "Newton-Meters";
        case TORQUE_UNIT_NEWTON_MILLIMETER:
            return "Newton-Millimeters";
        case TORQUE_UNIT_KILOGRAM_CENTIMETER:
            return "Kilogram-Centimeters";
        case TORQUE_UNIT_KILONEWTON_METER:
            return "Kilonewton-Meters";
        default:
            return "Unknown units";
        }
    }

    struct CalibrationInfoCommand {
        uint8_t command_{};
        uint8_t reserved_[19] = {0};

        CalibrationInfoCommand() : command_(READCALINFO) {
        }

        // Possible values for command_
        enum {
            READFT = 0,
            READCALINFO = 1,
            WRITETRANSFORM = 2,
            WRITECONDITION = 3,
        };

        enum { CALIB_INFO_COMMAND_SIZE = 20 };

        //!Packet structure into buffer for network transport
        //  Buffer should be CALIB_INFO_COMMAND_SIZE
        void pack(uint8_t* buffer) const;
    };

    void CalibrationInfoCommand::pack(uint8_t* buffer) const {
        buffer[0] = command_;
        for (size_t i=1;i < CALIB_INFO_COMMAND_SIZE ; ++i)
        {
            buffer[i] = 0x0;
        }
    }

    NetFTRDTDriver::NetFTRDTDriver(const std::string& address) : Node("netft_rdt_driver"),
                                                                 address_(address),
                                                                 socket_(io_service_),
                                                                 stop_recv_thread_(false),
                                                                 recv_thread_running_(false),
                                                                 packet_count_(0),
                                                                 lost_packets_(0),
                                                                 out_of_order_count_(0),
                                                                 seq_counter_(0),
                                                                 diag_packet_count_(0),
                                                                 last_diag_pub_time_(rclcpp::Clock().now()),
                                                                 last_rdt_sequence_(0),
                                                                 system_status_(0) {
        // Construct UDP socket
        const udp::endpoint netft_endpoint(boost::asio::ip::address_v4::from_string(address), RDT_PORT);
        socket_.open(udp::v4());
        socket_.connect(netft_endpoint);

        // Read the calibration information
        RCLCPP_INFO(get_logger(), "Reading NetFT Calibration");
        if (!readCalibrationInformation(address))
        {
            throw std::runtime_error("TCP Request for calibration information failed");
        }

        // Start receive thread
        recv_thread_ = boost::thread(&NetFTRDTDriver::recvThreadFunc, this);

        // Since start steaming command is sent with UDP packet,
        // the packet could be lost, retry startup 10 times before giving up
        for (int i = 0; i < 10; ++i) {
            startStreaming();
            if (waitForNewData())
                break;
        }
        {
            boost::unique_lock<boost::mutex> lock(mutex_);
            if (packet_count_ == 0) {
                throw std::runtime_error("No data received from NetFT device");
            }
        }

        // Create Publishers for making data available to other nodes.
        const rclcpp::QoS qos(10);
        ready_pub = this->create_publisher<std_msgs::msg::Bool>("netft_ready", qos);
        geo_pub = this->create_publisher<geometry_msgs::msg::WrenchStamped>("netft_data", 100);
    }

    NetFTRDTDriver::~NetFTRDTDriver() {
        // TODO stop transmission,
        // stop thread
        stop_recv_thread_ = true;
        if (!recv_thread_.timed_join(boost::posix_time::time_duration(0, 0, 1, 0))) {
            RCLCPP_WARN(get_logger(), "Interrupting recv thread");
            recv_thread_.interrupt();
            if (!recv_thread_.timed_join(boost::posix_time::time_duration(0, 0, 1, 0))) {
                RCLCPP_WARN(get_logger(), "Failed second join to recv thread");
            }
        }
        socket_.close();
    }


    bool NetFTRDTDriver::waitForNewData() {
        // Wait upto 100ms for new data
        bool got_new_data;
        {
            boost::mutex::scoped_lock lock(mutex_);
            const unsigned current_packet_count = packet_count_;
            condition_.timed_wait(lock, boost::posix_time::milliseconds(100));
            got_new_data = packet_count_ != current_packet_count;
        }
        return got_new_data;
    }

    bool NetFTRDTDriver::readCalibrationInformation(const std::string & address)
    {
        // Connect to socket
        using boost::asio::ip::tcp;
        tcp::socket calibration_socket(io_service_);
        calibration_socket.connect( tcp::endpoint( boost::asio::ip::address::from_string(address), TCP_PORT ));

        // Set up request
        CalibrationInfoCommand info_request;
        info_request.command_ = CalibrationInfoCommand::READCALINFO;
        uint8_t buffer[CalibrationInfoCommand::CALIB_INFO_COMMAND_SIZE];
        info_request.pack(buffer);

        // Send request
        boost::system::error_code error;
        boost::asio::write( calibration_socket, boost::asio::buffer(buffer), error );
        if( error ) {
            RCLCPP_ERROR_STREAM(get_logger(), "Failed to send calibration info request: " << error.message());
            return false;
        }

        uint8_t read_buffer[CalibrationInfoResponse::CALIB_INFO_RESPONSE_SIZE + 1];
        const size_t len = calibration_socket.read_some(boost::asio::buffer(read_buffer, CalibrationInfoResponse::CALIB_INFO_RESPONSE_SIZE + 1));
        if (len != CalibrationInfoResponse::CALIB_INFO_RESPONSE_SIZE)
        {
            RCLCPP_ERROR_STREAM(get_logger(), "Calibration response was length: " << len << ", but expected " << CalibrationInfoResponse::CALIB_INFO_RESPONSE_SIZE);
            return false;
        }
        
        // Unpack response
        CalibrationInfoResponse calibration_info;
        calibration_info.unpack(read_buffer);

        // Check the info and use it
        force_scale_ = 1.0 / calibration_info.counts_per_force_;
        torque_scale_ = 1.0 / calibration_info.counts_per_torque_;

        if (calibration_info.force_units_ == CalibrationInfoResponse::FORCE_UNIT_NEWTON)
        {
            RCLCPP_INFO_STREAM(get_logger(), "Read " << calibration_info.counts_per_force_ << " counts per force, with units Newtons");
        } else {
            RCLCPP_WARN_STREAM(get_logger(), "Expected calibration to have units Newtons, got: " << calibration_info.getForceUnitString() << ". Publishing non-SI units");
        }

        if (calibration_info.torque_units_ == CalibrationInfoResponse::TORQUE_UNIT_NEWTON_METER)
        {
            RCLCPP_INFO_STREAM(get_logger(), "Read " << calibration_info.counts_per_torque_ << " counts per torque, with units Newton-Meters");
        } else {
            RCLCPP_WARN_STREAM(get_logger(), "Expected calibration to have units Newton-Meters, got: " << calibration_info.getTorqueUnitString() << ". Publishing non-SI units");
        }

        return true;
    }

    void NetFTRDTDriver::startStreaming(void) {
        // Command NetFT to start data transmission
        RDTCommand start_transmission;
        start_transmission.command_ = RDTCommand::CMD_START_HIGH_SPEED_STREAMING;
        start_transmission.sample_count_ = RDTCommand::INFINITE_SAMPLES;
        // TODO change buffer into boost::array
        uint8_t buffer[RDTCommand::RDT_COMMAND_SIZE];
        start_transmission.pack(buffer);
        socket_.send(boost::asio::buffer(buffer, RDTCommand::RDT_COMMAND_SIZE));
    }


    void NetFTRDTDriver::recvThreadFunc() {
        try {
            recv_thread_running_ = true;
            RDTRecord rdt_record{};
            geometry_msgs::msg::WrenchStamped tmp_data;
            uint8_t buffer[RDTRecord::RDT_RECORD_SIZE + 1];
            while (!stop_recv_thread_) {
                size_t len = socket_.receive(boost::asio::buffer(buffer, RDTRecord::RDT_RECORD_SIZE + 1));
                if (len != RDTRecord::RDT_RECORD_SIZE) {
                    RCLCPP_WARN(get_logger(), "Receive size of %d bytes does not match expected size of %d",
                                static_cast<int>(len), static_cast<int>(RDTRecord::RDT_RECORD_SIZE));
                }
                else {
                    rdt_record.unpack(buffer);
                    if (rdt_record.status_ != 0) {
                        // Latch any system status error code
                        boost::unique_lock<boost::mutex> lock(mutex_);
                        system_status_ = rdt_record.status_;
                    }
                    const auto seqdiff = static_cast<int32_t>(rdt_record.rdt_sequence_ - last_rdt_sequence_);
                    last_rdt_sequence_ = rdt_record.rdt_sequence_;
                    if (seqdiff < 1) {
                        boost::unique_lock<boost::mutex> lock(mutex_);
                        // Don't use data that is old
                        ++out_of_order_count_;
                    }
                    else {
                        tmp_data.header.stamp = rclcpp::Clock().now();
                        tmp_data.header.frame_id = "base_link";
                        tmp_data.wrench.force.x = static_cast<double>(rdt_record.fx_) * force_scale_;
                        tmp_data.wrench.force.y = static_cast<double>(rdt_record.fy_) * force_scale_;
                        tmp_data.wrench.force.z = static_cast<double>(rdt_record.fz_) * force_scale_;
                        tmp_data.wrench.torque.x = static_cast<double>(rdt_record.tx_) * torque_scale_;
                        tmp_data.wrench.torque.y = static_cast<double>(rdt_record.ty_) * torque_scale_;
                        tmp_data.wrench.torque.z = static_cast<double>(rdt_record.tz_) * torque_scale_;
                        {
                            boost::unique_lock<boost::mutex> lock(mutex_);
                            new_data_ = tmp_data;
                            lost_packets_ += (seqdiff - 1);
                            ++packet_count_;
                            condition_.notify_all();
                        }
                    }
                }
            } // end while
        }
        catch (std::exception& e) {
            recv_thread_running_ = false;
            {
                boost::unique_lock<boost::mutex> lock(mutex_);
                recv_thread_error_msg_ = e.what();
            }
        }
    }


    void NetFTRDTDriver::getData(geometry_msgs::msg::WrenchStamped& data) {
        {
            boost::unique_lock<boost::mutex> lock(mutex_);
            data = new_data_;
        }
    }


    // TODO: diagnostics not implemented for ros2 yet
    // void NetFTRDTDriver::diagnostics(diagnostic_updater::DiagnosticStatusWrapper &d)
    // {
    //   // Publish diagnostics
    //   d.name = "NetFT RDT Driver : " + address_;
    //
    //   d.summary(d.OK, "OK");
    //   d.hardware_id = "0";
    //
    //   if (diag_packet_count_ == packet_count_)
    //   {
    //     d.mergeSummary(d.ERROR, "No new data in last second");
    //   }
    //
    //   if (!recv_thread_running_)
    //   {
    //     d.mergeSummaryf(d.ERROR, "Receive thread has stopped : %s", recv_thread_error_msg_.c_str());
    //   }
    //
    //   if (system_status_ != 0)
    //   {
    //     d.mergeSummaryf(d.ERROR, "NetFT reports error 0x%08x", system_status_);
    //   }
    //
    //   rclcpp::Clock clock;
    //   rclcpp::Time current_time = clock.now();
    //   double recv_rate = double(int32_t(packet_count_ - diag_packet_count_)) / (current_time - last_diag_pub_time_).seconds();
    //
    //   d.clear();
    //   d.addf("IP Address", "%s", address_.c_str());
    //   d.addf("System status", "0x%08x", system_status_);
    //   d.addf("Good packets", "%u", packet_count_);
    //   d.addf("Lost packets", "%u", lost_packets_);
    //   d.addf("Out-of-order packets", "%u", out_of_order_count_);
    //   d.addf("Recv rate (pkt/sec)", "%.2f", recv_rate);
    //   d.addf("Force scale (N/bit)", "%f", force_scale_);
    //   d.addf("Torque scale (Nm/bit)", "%f", torque_scale_);
    //
    //   geometry_msgs::msg::WrenchStamped data;
    //   getData(data);
    //   d.addf("Force X (N)",   "%f", data.wrench.force.x);
    //   d.addf("Force Y (N)",   "%f", data.wrench.force.y);
    //   d.addf("Force Z (N)",   "%f", data.wrench.force.z);
    //   d.addf("Torque X (Nm)", "%f", data.wrench.torque.x);
    //   d.addf("Torque Y (Nm)", "%f", data.wrench.torque.y);
    //   d.addf("Torque Z (Nm)", "%f", data.wrench.torque.z);
    //
    //   last_diag_pub_time_ = current_time;
    //   diag_packet_count_ = packet_count_;
    // }
} // end namespace netft_rdt_driver
