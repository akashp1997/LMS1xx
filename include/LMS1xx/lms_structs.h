/*
 * lms_structs.h
 *
 *  Author: Konrad Banachowicz
 *          Mike Purvis <mpurvis@clearpathrobotics.com>
 ***************************************************************************
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Lesser General Public            *
 *   License as published by the Free Software Foundation; either          *
 *   version 2.1 of the License, or (at your option) any later version.    *
 *                                                                         *
 *   This library is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU     *
 *   Lesser General Public License for more details.                       *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this library; if not, write to the Free Software   *
 *   Foundation, Inc., 59 Temple Place,                                    *
 *   Suite 330, Boston, MA  02111-1307  USA                                *
 *                                                                         *
 ***************************************************************************/

#ifndef LMS1XX_LMS_STRUCTS_H_
#define LMS1XX_LMS_STRUCTS_H_

#include <stdint.h>
#include <vector>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

namespace py = pybind11;

/*!
* @class scanCfg
* @brief Structure containing scan configuration.
*
* @author Konrad Banachowicz
*/
struct scanCfg
{
  /*!
   * @brief Scanning frequency.
   * 1/100 Hz
   */
  int scaningFrequency;

  /*!
   * @brief Scanning resolution.
   * 1/10000 degree
   */
  int angleResolution;

  /*!
   * @brief Start angle.
   * 1/10000 degree
   */
  int startAngle;

  /*!
   * @brief Stop angle.
   * 1/10000 degree
   */
  int stopAngle;
};

/*!
* @class scanDataCfg
* @brief Structure containing scan data configuration.
*
* @author Konrad Banachowicz
*/
struct scanDataCfg
{

  /*!
   * @brief Output channels.
   * Defines which output channel is activated.
   */
  int outputChannel;

  /*!
   * @brief Remission.
   * Defines whether remission values are output.
   */
  bool remission;

  /*!
   * @brief Remission resolution.
   * Defines whether the remission values are output with 8-bit or 16bit resolution.
   */
  int resolution;

  /*!
   * @brief Encoders channels.
   * Defines which output channel is activated.
   */
  int encoder;

  /*!
   * @brief Position.
   * Defines whether position values are output.
   */
  bool position;

  /*!
   * @brief Device name.
   * Determines whether the device name is to be output.
   */
  bool deviceName;

  bool timestamp;

  /*!
   * @brief Output interval.
   * Defines which scan is output.
   *
   * 01 every scan\n
   * 02 every 2nd scan\n
   * ...\n
   * 50000 every 50000th scan
   */
  int outputInterval;
};

/*!
* @class outputRange
* @brief Structure containing scan output range configuration
*
* @author wpd
*/
struct scanOutputRange
{
  /*!
   * @brief Scanning resolution.
   * 1/10000 degree
   */
  int angleResolution;

  /*!
   * @brief Start angle.
   * 1/10000 degree
   */
  int startAngle;

  /*!
   * @brief Stop angle.
   * 1/10000 degree
   */
  int stopAngle;
};

/*!
* @class scanData
* @brief Structure containing single scan message.
*
* @author Konrad Banachowicz
*/
struct scanData
{

  /*!
   * @brief Number of samples in dist1.
   *
   */
  int dist_len1;

  /*!
   * @brief Radial distance for the first reflected pulse
   *
   */
  uint16_t dist1[1082];

  py::list getDist1() {
    std::vector <uint16_t> dist1_(dist1, dist1+1082);
    return py::cast(dist1_);
  }

  void setDist1(py::list dist1_) {
    for(auto i=0;i<dist1_.size();i++) {
      dist1[i] = dist1_[i].cast<uint16_t>();
    }
  }

  /*!
   * @brief Number of samples in dist2.
   *
   */
  int dist_len2;

  /*!
   * @brief Radial distance for the second reflected pulse
   *
   */
  uint16_t dist2[1082];

  py::list getDist2() {
    std::vector <uint16_t> dist2_(dist2, dist2+1082);
    return py::cast(dist2_);
  }

  void setDist2(py::list dist2_) {
    for(auto i=0;i<dist2_.size();i++) {
      dist2[i] = dist2_[i].cast<uint16_t>();
    }
  }

  /*!
   * @brief Number of samples in rssi1.
   *
   */
  int rssi_len1;

  /*!
   * @brief Remission values for the first reflected pulse
   *
   */
  uint16_t rssi1[1082];

  py::list getRssi1() {
    std::vector <uint16_t> rssi1_(rssi1, rssi1+1082);
    return py::cast(rssi1_);
  }

  void setRssi1(py::list rssi1_) {
    for(auto i=0;i<rssi1_.size();i++) {
      rssi1[i] = rssi1_[i].cast<uint16_t>();
    }
  }

  /*!
   * @brief Number of samples in rssi2.
   *
   */
  int rssi_len2;

  /*!
   * @brief Remission values for the second reflected pulse
   *
   */
  uint16_t rssi2[1082];

  py::list getRssi2() {
    std::vector <uint16_t> rssi2_(rssi2, rssi2+1082);
    return py::cast(rssi2_);
  }

  void setRssi2(py::list rssi2_) {
    for(auto i=0;i<rssi2_.size();i++) {
      rssi2[i] = rssi2_[i].cast<uint16_t>();
    }
  }
};

#endif  // LMS1XX_LMS_STRUCTS_H_
