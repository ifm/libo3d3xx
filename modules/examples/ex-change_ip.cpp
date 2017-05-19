/*
 * Copyright (C) 2017 ifm syntron gmbh
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distribted on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

//
// ex-change_ip.cpp
//
// Change the IPv4 address of the camera
//

#include <iostream>
#include <memory>
#include "o3d3xx_camera.h"
#include "o3d3xx_framegrabber.h"

int main(int argc, const char **argv)
{
    o3d3xx::Logging::Init();

    o3d3xx::Camera::Ptr cam = std::make_shared<o3d3xx::Camera>();
    cam->RequestSession();
    cam->SetOperatingMode(o3d3xx::Camera::operating_mode::EDIT);

    o3d3xx::NetConfig::Ptr net = cam->GetNetConfig();
    bool has_changed = true;
    net->SetStaticIPv4Address("192.168.0.70");
    cam->SetNetConfig(net.get(), &has_changed);
    cam->SaveNet();

    return 0;
}
