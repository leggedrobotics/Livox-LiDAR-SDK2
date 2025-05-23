//
// Copyright(c) 2015 Gabi Melman.
// Distributed under the MIT License (http://opensource.org/licenses/MIT)
//

#pragma once

// async logger implementation
// uses a thread pool to perform the actual logging

#include "spdlog/details/thread_pool.h"

#include <chrono>
#include <memory>
#include <string>

template <typename It>
inline spdlog::async_logger::async_logger(std::string logger_name, It begin, It end,
                                          std::weak_ptr<details::thread_pool> tp, async_overflow_policy overflow_policy)
  : logger(std::move(logger_name), begin, end), thread_pool_(std::move(tp)), overflow_policy_(overflow_policy)
{
}

inline spdlog::async_logger::async_logger(std::string logger_name, sinks_init_list sinks_list,
                                          std::weak_ptr<details::thread_pool> tp, async_overflow_policy overflow_policy)
  : async_logger(std::move(logger_name), sinks_list.begin(), sinks_list.end(), std::move(tp), overflow_policy)
{
}

inline spdlog::async_logger::async_logger(std::string logger_name, sink_ptr single_sink,
                                          std::weak_ptr<details::thread_pool> tp, async_overflow_policy overflow_policy)
  : async_logger(std::move(logger_name), { std::move(single_sink) }, std::move(tp), overflow_policy)
{
}

// send the log message to the thread pool
inline void spdlog::async_logger::sink_it_(details::log_msg& msg)
{
#if defined(SPDLOG_ENABLE_MESSAGE_COUNTER)
  incr_msg_counter_(msg);
#endif
  if (auto pool_ptr = thread_pool_.lock())
  {
    pool_ptr->post_log(shared_from_this(), msg, overflow_policy_);
  }
  else
  {
    throw spdlog_ex("async log: thread pool doesn't exist anymore");
  }
}

// send flush request to the thread pool
inline void spdlog::async_logger::flush_()
{
  if (auto pool_ptr = thread_pool_.lock())
  {
    pool_ptr->post_flush(shared_from_this(), overflow_policy_);
  }
  else
  {
    throw spdlog_ex("async flush: thread pool doesn't exist anymore");
  }
}

//
// backend functions - called from the thread pool to do the actual job
//
inline void spdlog::async_logger::backend_log_(const details::log_msg& incoming_log_msg)
{
  try
  {
    for (auto& s : sinks_)
    {
      if (s->should_log(incoming_log_msg.level))
      {
        s->log(incoming_log_msg);
      }
    }
  }
  SPDLOG_CATCH_AND_HANDLE

  if (should_flush_(incoming_log_msg))
  {
    backend_flush_();
  }
}

inline void spdlog::async_logger::backend_flush_()
{
  try
  {
    for (auto& sink : sinks_)
    {
      sink->flush();
    }
  }
  SPDLOG_CATCH_AND_HANDLE
}

inline std::shared_ptr<spdlog::logger> spdlog::async_logger::clone(std::string new_name)
{
  auto cloned = std::make_shared<spdlog::async_logger>(std::move(new_name), sinks_.begin(), sinks_.end(), thread_pool_,
                                                       overflow_policy_);

  cloned->set_level(this->level());
  cloned->flush_on(this->flush_level());
  cloned->set_error_handler(this->error_handler());
  return std::move(cloned);
}
