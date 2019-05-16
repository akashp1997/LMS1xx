#ifndef ASVMQ_H_
#define ASVMQ_H_

#include <pybind11/pybind11.h>

#include <cstdarg>
#include <cstdio>
namespace py = pybind11;

void logDebug(const char *format, ...) {
  va_list vl;
  va_start(vl, format);
  char buff[1000];
  if(vsprintf(buff, format, vl)>0) {
    //py::print("[DEBUG]", buff);
  }
  va_end(vl);
}

void logWarn(const char *format, ...) {
  va_list vl;
  va_start(vl, format);
  char buff[1000];
  if(vsprintf(buff, format, vl)>0) {
    py::print("[WARN]", buff);
  }
  va_end(vl);
}

void logInfo(const char *format, ...) {
  va_list vl;
  va_start(vl, format);
  char buff[1000];
  if(vsprintf(buff, format, vl)>0) {
    py::print("[INFO]", buff);
  }
  va_end(vl);
}

void logFatal(const char *format, ...) {
  va_list vl;
  va_start(vl, format);
  char buff[1000];
  if(vsprintf(buff, format, vl)>0) {
    py::print("[FATAL]", buff);
  }
  va_end(vl);
}


/*void logDebug(const char *format, ...) {
  va_list vl;
  va_start(vl, format);
  py::object asvmq = py::module::import("asvmq");
  py::object log_debug = asvmq.attr("log_debug");

  char *buff;
  if(vsprintf(buff, format, vl)>0) {
    log_debug(buff);
  }
  va_end(vl);
}

void logWarn(const char *format, ...) {
  va_list vl;
  va_start(vl, format);

  py::object asvmq = py::module::import("asvmq");
  py::object log_warn = asvmq.attr("log_warn");

  char *buff;
  if(vsprintf(buff, format, vl)>0) {
    log_warn(buff);
  }
  va_end(vl);
}

void logInfo(const char *format, ...) {
  va_list vl;
  va_start(vl, format);

  py::object asvmq = py::module::import("asvmq");
  py::object log_info = asvmq.attr("log_info");

  char *buff;
  if(vsprintf(buff, format, vl)>0) {
    log_info(buff);
  }
  va_end(vl);
}

void logFatal(const char *format, ...) {
  va_list vl;
  va_start(vl, format);

  py::object asvmq = py::module::import("asvmq");
  py::object log_fatal = asvmq.attr("log_fatal");

  char *buff;
  if(vsprintf(buff, format, vl)>0) {
    log_fatal(buff);
  }
  va_end(vl);
}*/
#endif // ASVMQ_H_
