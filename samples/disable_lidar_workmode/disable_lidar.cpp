//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "livox_lidar_def.h"
#include "livox_lidar_api.h"

#include <unistd.h>
#include <stdio.h>
#include <thread>
#include <chrono>
#include <iostream>
#include <string>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <map>
#include <unistd.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <csignal>

//std::condition_variable quit_condition;
std::condition_variable cv;
std::mutex mtx;

uint8_t test = 1;
     
void WorkModeCallback(livox_status status, uint32_t handle,LivoxLidarAsyncControlResponse *response, void *client_data) {
  if (response == nullptr) {
    return;
  }
  printf("--------------------------------------------------------------------------------\n");
  printf("WorkModeCallack, status:%u, handle:%u, ret_code:%u, error_key:%u",
      status, handle, response->ret_code, response->error_key);
  printf("--------------------------------------------------------------------------------\n");
  cv.notify_one();
}

void DebugPointCloudCallback(livox_status status, uint32_t handle, LivoxLidarLoggerResponse* response, void* client_data) {
  printf("--------------------------------------------------------------------------------\n");
  printf("livox_status = %d, Lidar: %u response is %d\n", status, handle, response->ret_code);
  printf("--------------------------------------------------------------------------------\n");
}

void LidarInfoChangeCallback(const uint32_t handle, const LivoxLidarInfo* info, void* client_data) {
  if (info == nullptr) {
    printf("lidar info change callback failed, the info is nullptr.\n");
    return;
  } 

  printf("LidarInfoChangeCallback Lidar handle: %u SN: %s\n", handle, info->sn);
  sleep(2);
  SetLivoxLidarWorkMode(handle, kLivoxLidarStandBy, WorkModeCallback, nullptr);

  //SetLivoxLidarDebugPointCloud(handle, true, DebugPointCloudCallback, nullptr);
  // sleep(2);
  //SetLivoxLidarWorkMode(handle, kLivoxLidarSleep, WorkModeCallback, nullptr);
  // SetLivoxLidarDebugPointCloud(handle, false, DebugPointCloudCallback, nullptr);
}

void LivoxLidarPushMsgCallback(const uint32_t handle, const uint8_t dev_type, const char* info, void* client_data) {
  struct in_addr tmp_addr;
  tmp_addr.s_addr = handle;  
  std::cout << "handle: " << handle << ", ip: " << inet_ntoa(tmp_addr) << ", push msg info: " << std::endl;
  std::cout << info << std::endl;
  cv.notify_one();
  return;
}

void Stop(int signal) {
  cv.notify_all();
}

int main(int argc, const char *argv[]) {
  if (argc != 2) {
    printf("Params Invalid, must input config path.\n");
    return -1;
  }

  // This is the path of the json file.
  const std::string path = argv[1];
  printf("path:%s.\n", path.c_str());

  if (!LivoxLidarSdkInit(path.c_str())) {
    printf("Livox Init Failed\n");
    LivoxLidarSdkUninit();
    return -1;
  }

  SetLivoxLidarInfoCallback(LivoxLidarPushMsgCallback, nullptr);
  SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallback, nullptr);
    
  // capture Ctrl + C signal.
  //std::signal(SIGINT, Stop);
  while(test <= 1.0) {
    {
      std::unique_lock<std::mutex> lock(mtx);
      cv.wait(lock);
    }
    //UpgradeLivoxLidars(handles.data(), handles.size());
    --test;
    break;
  }

  printf("Deivice Logger exit.\n");

  sleep(3000000); // 3sec?
  LivoxLidarSdkUninit();
	printf("Livox End!\n");
  return 0;
}

