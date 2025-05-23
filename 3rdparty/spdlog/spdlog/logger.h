//
// Copyright(c) 2015-2108 Gabi Melman.
// Distributed under the MIT License (http://opensource.org/licenses/MIT)
//

#pragma once

// Thread safe logger (except for set_pattern(..), set_formatter(..) and
// set_error_handler())
// Has name, log level, vector of std::shared sink pointers and formatter
// Upon each log write the logger:
// 1. Checks if its log level is enough to log the message and if yes:
// 2. Call the underlying sinks to do the job.
// 3. Each sink use its own private copy of a formatter to format the message
// and send to its destination.
//
// The use of private formatter per sink provides the opportunity to cache some
// formatted data,
// and support customize format per each sink.

#include "spdlog/common.h"
#include "spdlog/formatter.h"
#include "spdlog/sinks/sink.h"

#include <memory>
#include <string>
#include <vector>

namespace spdlog
{
class logger
{
public:
  logger(std::string name, sink_ptr single_sink);
  logger(std::string name, sinks_init_list sinks);

  template <typename It>
  logger(std::string name, It begin, It end);

  virtual ~logger();

  logger(const logger&) = delete;
  logger& operator=(const logger&) = delete;

  template <typename... Args>
  void log(level::level_enum lvl, const char* fmt, const Args&... args);

  template <typename... Args>
  void log(source_loc loc, level::level_enum lvl, const char* fmt, const Args&... args);

  void log(level::level_enum lvl, const char* msg);

  void log(source_loc loc, level::level_enum lvl, const char* msg);

  template <typename... Args>
  void trace(const char* fmt, const Args&... args);

  template <typename... Args>
  void debug(const char* fmt, const Args&... args);

  template <typename... Args>
  void info(const char* fmt, const Args&... args);

  template <typename... Args>
  void warn(const char* fmt, const Args&... args);

  template <typename... Args>
  void error(const char* fmt, const Args&... args);

  template <typename... Args>
  void critical(const char* fmt, const Args&... args);

#ifdef SPDLOG_WCHAR_TO_UTF8_SUPPORT
#ifndef _WIN32
#error SPDLOG_WCHAR_TO_UTF8_SUPPORT only supported on windows
#else
  template <typename... Args>
  void log(level::level_enum lvl, const wchar_t* fmt, const Args&... args);

  template <typename... Args>
  void log(source_loc source, level::level_enum lvl, const wchar_t* fmt, const Args&... args);

  template <typename... Args>
  void trace(const wchar_t* fmt, const Args&... args);

  template <typename... Args>
  void debug(const wchar_t* fmt, const Args&... args);

  template <typename... Args>
  void info(const wchar_t* fmt, const Args&... args);

  template <typename... Args>
  void warn(const wchar_t* fmt, const Args&... args);

  template <typename... Args>
  void error(const wchar_t* fmt, const Args&... args);

  template <typename... Args>
  void critical(const wchar_t* fmt, const Args&... args);
#endif  // _WIN32
#endif  // SPDLOG_WCHAR_TO_UTF8_SUPPORT

  template <class T>
  void log(level::level_enum lvl, const T&);

  // T can be statically converted to string_view
  template <class T, typename std::enable_if<std::is_convertible<T, spdlog::string_view_t>::value, T>::type* = nullptr>
  void log(source_loc loc, level::level_enum lvl, const T&);

  // T cannot be statically converted to string_view
  template <class T, typename std::enable_if<!std::is_convertible<T, spdlog::string_view_t>::value, T>::type* = nullptr>
  void log(source_loc loc, level::level_enum lvl, const T&);

  template <typename T>
  void trace(const T& msg);

  template <typename T>
  void debug(const T& msg);

  template <typename T>
  void info(const T& msg);

  template <typename T>
  void warn(const T& msg);

  template <typename T>
  void error(const T& msg);

  template <typename T>
  void critical(const T& msg);

  bool should_log(level::level_enum msg_level) const;
  void set_level(level::level_enum log_level);

  static level::level_enum default_level();
  level::level_enum level() const;
  const std::string& name() const;

  // set formatting for the sinks in this logger.
  // each sink will get a seperate instance of the formatter object.
  void set_formatter(std::unique_ptr<formatter> formatter);
  void set_pattern(std::string pattern, pattern_time_type time_type = pattern_time_type::local);

  // flush functions
  void flush();
  void flush_on(level::level_enum log_level);
  level::level_enum flush_level() const;

  // sinks
  const std::vector<sink_ptr>& sinks() const;
  std::vector<sink_ptr>& sinks();

  // error handler
  void set_error_handler(log_err_handler err_handler);
  log_err_handler error_handler() const;

  // create new logger with same sinks and configuration.
  virtual std::shared_ptr<logger> clone(std::string logger_name);

protected:
  virtual void sink_it_(details::log_msg& msg);
  virtual void flush_();

  bool should_flush_(const details::log_msg& msg);

  // default error handler.
  // print the error to stderr with the max rate of 1 message/minute.
  void default_err_handler_(const std::string& msg);

  // increment the message count (only if defined(SPDLOG_ENABLE_MESSAGE_COUNTER))
  void incr_msg_counter_(details::log_msg& msg);

  const std::string name_;
  std::vector<sink_ptr> sinks_;
  spdlog::level_t level_{ spdlog::logger::default_level() };
  spdlog::level_t flush_level_{ level::off };
  log_err_handler err_handler_{ [this](const std::string& msg) { this->default_err_handler_(msg); } };
  std::atomic<time_t> last_err_time_{ 0 };
  std::atomic<size_t> msg_counter_{ 1 };
};
}  // namespace spdlog

#include "details/logger_impl.h"
