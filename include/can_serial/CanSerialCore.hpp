/*
 * Copyright 2025 handshow
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CAN_SERIAL_CORE_HPP
#define CAN_SERIAL_CORE_HPP

#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <linux/can.h>
#include <linux/can/netlink.h>
#include <linux/can/raw.h>
#include <linux/sockios.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <bits/stdc++.h>
#include <thread>

class CanSerial {
public:
    using FrameCallback = std::function<void(const can_frame&)>;
    
    CanSerial(const std::string& interface = "can0");
    ~CanSerial();
    
    void init();
    void async_read();
    void start_io_service();
    void send_frame(const can_frame& frame);
    void set_frame_callback(FrameCallback callback);
    std::thread io_thread_;

private:
    void handle_received(const boost::system::error_code& ec, size_t bytes);

    std::string can_interface_;
    int sock_ = -1;
    boost::asio::io_service io_service_;
    boost::asio::posix::basic_stream_descriptor<> stream_;
    can_frame recv_frame_;
    FrameCallback frame_callback_;

    
};

#endif