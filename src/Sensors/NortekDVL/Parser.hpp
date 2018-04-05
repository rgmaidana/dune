//***************************************************************************
// Copyright 2018 OceanScan - Marine Systems & Technology, Lda.             *
//***************************************************************************
// This file is part of DUNE: Unified Navigation Environment.               *
//                                                                          *
// Commercial Licence Usage                                                 *
// Licencees holding valid commercial DUNE licences may use this file in    *
// accordance with the commercial licence agreement provided with the       *
// Software or, alternatively, in accordance with the terms contained in a  *
// written agreement between you and OceanScan - Marine Systems &           *
// Technology, Lda. For licensing terms, conditions, and further            *
// information contact info@oceanscan-mst.com.                              *
//                                                                          *
// Modified European Union Public Licence - EUPL v.1.1 Usage                *
// Alternatively, this file may be used under the terms of the Modified     *
// EUPL, Version 1.1 only (the "Licence"), appearing in the file LICENCE.md *
// included in the packaging of this file. You may not use this work        *
// except in compliance with the Licence. Unless required by applicable     *
// law or agreed to in writing, software distributed under the Licence is   *
// distributed on an "AS IS" basis, WITHOUT WARRANTIES OR CONDITIONS OF     *
// ANY KIND, either express or implied. See the Licence for the specific    *
// language governing permissions and limitations at                        *
// https://github.com/LSTS/dune/blob/master/LICENCE.md and                  *
// http://ec.europa.eu/idabc/eupl.html.                                     *
//***************************************************************************
// Author: José Braga                                                       *
// Author: Ricardo Martins                                                  *
//***************************************************************************

#ifndef SENSORS_NORTEK_DVL_PARSER_HPP_INCLUDED_
#define SENSORS_NORTEK_DVL_PARSER_HPP_INCLUDED_

// ISO C++ 98 headers.
#include <string>

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Sensors
{
  namespace NortekDVL
  {
    using DUNE_NAMESPACES;

    //! Number of distance measurements.
    static const unsigned c_beam_count = 4;
    //! Beam width.
    static const float c_beam_width = 1.0f;
    //! Beam offset.
    static const float c_beam_offset = 0.045f;
    //! Beam angle.
    static const float c_beam_angle = 25.0f;
    //! Log file prefix.
    static const char* c_log_prefix = "Data_";
    //! Log file suffix.
    static const char* c_log_suffix = ".ad2cp";

    //! Parser class to interpret Nortek DVL's incoming data.
    class Parser
    {
    public:
      //! Constructor.
      //! @param[in] task parent task.
      //! @param[in] handle io handle.
      //! @param[in] pos device position.
      //! @param[in] ang device orientation.
      //! @param[in] entities beam entities.
      //! @param[in] entity filtered dvl entity.
      Parser(Tasks::Task* task, IO::Handle* handle, std::vector<float>& pos,
             std::vector<float>& ang, std::vector<unsigned>& entities, unsigned entity):
        m_task(task),
        m_handle(handle),
        m_filter(NULL),
        m_state(ST_SYNC),
        m_timestamp(-1),
        m_index(0),
        m_data_size(0),
        m_checksum(0),
        m_status(0),
        m_water(false),
        m_type(RT_NONE)
      {
        m_filter = new BeamFilter(m_task, c_beam_count, c_beam_width, c_beam_offset,
                                  c_beam_angle, pos, ang, BeamFilter::ANTICLOCKWISE);

        m_filter->setSourceEntities(entities);
        m_filt_distance.setSourceEntity(entity);
        m_bfr.resize(c_max_size);
        m_hdr.resize(0);
      }

      //! Destructor.
      ~Parser(void)
      {
        Memory::clear(m_filter);
      }

      //! Read data.
      //! @param[in] timeout polling timeout.
      //! @return true if valid data was parsed, false otherwise.
      bool
      readData(double timeout = 1.0)
      {
        uint8_t bfr[c_max_size];
        Counter<double> timer(timeout);

        while (!timer.overflow())
        {
          if (Poll::poll(*m_handle, timer.getRemaining()))
          {
            size_t rv = m_handle->read(bfr, sizeof(bfr));

            m_raw_data.value.assign(bfr, bfr + rv);
            m_task->dispatch(m_raw_data);

            for (size_t i = 0; i < rv; ++i)
            {
              if (!parse(bfr[i]))
                continue;

              return true;
            }
          }
        }

        return false;
      }

      //! Indicate to the parser if device is in a water environment.
      //! @param[in] status true if device is in water, false otherwise.
      void
      setWater(bool status)
      {
        m_water = status;
      }

      //! Open new log file.
      //! @param[in] path file system path.
      void
      openLog(const FileSystem::Path& path)
      {
        if (path == m_log_root)
          return;

        closeLog();
        m_log_root = path;
      }

      //! Close log file.
      void
      closeLog(void)
      {
        if (m_log_file.is_open())
        {
          m_log_file.close();

          if (m_log_path.size() == 0)
            m_log_path.remove();
        }

        m_log_path.clear();
      }

    private:
      //! Get log path using current time.
      //! @param[in] root log folder.
      //! @return log path.
      static Path
      getLogPath(const Path& root)
      {
        double now = Clock::getSinceEpoch();

        while (true)
        {
          std::string log_name(c_log_prefix);
          log_name.append(Format::getDateSafe(now) + Format::getTimeSafe(now));
          log_name.append(c_log_suffix);

          Path path = root / log_name;
          if (!path.exists())
            return path;

          now += 1.0;
        }
      }

      //! Parse one byte of data.
      //! @param[in] byte data byte.
      //! @return true if a message was parsed, false otherwise.
      bool
      parse(uint8_t byte)
      {
        switch (m_state)
        {
          case ST_SYNC:
            if (byte == c_sync)
            {
              m_timestamp = Clock::getSinceEpoch();
              m_index = 0;
              m_bfr[m_index++] = byte;
              m_state = ST_HEADER;
              m_type = RT_NONE;
            }
            break;

          case ST_HEADER:
            m_bfr[m_index++] = byte;
            if (byte == c_hdr_size)
              m_state = ST_ID;
            else
              m_state = ST_SYNC;

            break;

          case ST_ID:
            m_bfr[m_index++] = byte;
            m_state = ST_FAMILY;
            m_type = RT_NONE;

            if (byte == c_bt_type)
              m_type = RT_BOTTOM_TRACK;
            else if (byte == c_wt_type)
              m_type = RT_WATER_TRACK;
            else if (byte == c_cp_type)
              m_type = RT_CURRENT_PROFILE;
            else if (byte == c_rc_type)
              m_type = RT_STRING_DATA;

            if (m_type == RT_NONE)
            {
              m_task->err(DTR("unexpected type"));
              m_state = ST_SYNC;
            }
            break;

          case ST_FAMILY:
            m_bfr[m_index++] = byte;
            if (byte == c_family)
              m_state = ST_DATA_SIZE_LSB;
            else
              m_state = ST_SYNC;

            break;

          case ST_DATA_SIZE_LSB:
            m_bfr[m_index++] = byte;
            m_data_size = (uint16_t)byte;
            m_state = ST_DATA_SIZE_MSB;
            break;
          case ST_DATA_SIZE_MSB:
            m_bfr[m_index++] = byte;
            m_data_size |= byte << 8;

            // increase buffer if necessary.
            if (m_data_size + c_hdr_size > (int)m_bfr.size())
              m_bfr.resize(m_data_size + c_hdr_size);

            m_state = ST_DATA_CSUM_LSB;
            break;
          case ST_DATA_CSUM_LSB:
            m_bfr[m_index++] = byte;
            m_checksum = (uint16_t)byte;
            m_state = ST_DATA_CSUM_MSB;
            break;
          case ST_DATA_CSUM_MSB:
            m_bfr[m_index++] = byte;
            m_checksum |= byte << 8;
            m_state = ST_HDR_CSUM_LSB;
            break;
          case ST_HDR_CSUM_LSB:
            m_bfr[m_index++] = byte;
            m_state = ST_HDR_CSUM_MSB;
            break;
          case ST_HDR_CSUM_MSB:
            m_bfr[m_index++] = byte;
            m_state = ST_DATA;
            break;
          case ST_DATA:
            m_bfr[m_index++] = byte;

            if (m_index == (unsigned)(m_data_size + c_hdr_size))
            {
              m_state = ST_SYNC;

              // failed checksum.
              if (checksumFailed())
                return false;

              if (m_type == RT_STRING_DATA)
                decodeHeader();
              else if (m_type == RT_CURRENT_PROFILE)
                decodeCurrentProfile();
              else
                decodeBottomTrack();

              return true;
            }

          default:
            break;
        }

        return false;
      }

      //! Check if checksum matches.
      //! @return true if checksum does not match, false otherwise.
      bool
      checksumFailed(void)
      {
        uint16_t checksum = c_csum_start;
        uint16_t size = m_data_size;
        uint16_t datasize = size >> 1;

        for (unsigned i = 0; i < datasize; i++)
        {
          uint16_t nb;
          std::memcpy(&nb, &m_bfr[c_hdr_size + i * 2], 2);
          checksum += nb;
          size -= 2;
        }

        if (size > 0)
          checksum += ((uint16_t)m_bfr[c_hdr_size + datasize * 2]) << 8;

        if (checksum == m_checksum)
          return false;

        m_task->debug("invalid checksum: %04X (%04X)", checksum, m_checksum);
        return true;
      }

      //! Interpret bottom track data received from device.
      void
      decodeBottomTrack(void)
      {
        processStatus();
        processDistance();
        processVelocity();
      }

      //! Interpret string data header.
      void
      decodeHeader(void)
      {
        m_task->trace("received header");
        m_hdr.assign(m_bfr.begin(), m_bfr.begin() + m_data_size + c_hdr_size);

        // we'll reopen if needed.
        closeLog();
      }

      //! Interpret current profile data received from device.
      void
      decodeCurrentProfile(void)
      {
        if (!m_log_file.is_open())
        {
          if (m_hdr.size() == 0)
            return;

          m_log_path = getLogPath(m_log_root);
          m_task->trace("generating file: %s", m_log_path.c_str());
          m_log_file.open(m_log_path.c_str(), std::ofstream::app | std::ios::binary);
          m_log_file.write((const char*)&m_hdr[0], m_hdr.size());
        }

        m_log_file.write((const char*)&m_bfr[0], m_data_size + c_hdr_size);
        m_task->trace("parsed current profile data");
      }

      //! Parse status and sensor data.
      void
      processStatus(void)
      {
        uint8_t bits_old = (uint8_t)((c_wake_mask & m_status) >> SB_WAKE_ST);

        float pres;
        IMC::Temperature temp;

        std::memcpy(&m_status, &m_bfr[c_hdr_size + IND_STATUS], 4);
        std::memcpy(&temp.value, &m_bfr[c_hdr_size + IND_TEMPER], 4);
        std::memcpy(&pres, &m_bfr[c_hdr_size + IND_PRESSU], 4);

        // check if pressure data is valid.
        if (pres > 0.0)
        {
          IMC::Pressure pre;
          IMC::Depth dep;
          pre.value = pres * c_pascal_per_bar;
          dep.value = ((pre.value - Math::c_sea_level_pressure) /
                       (Math::c_gravity * c_seawater_density));

          pre.setTimeStamp(m_timestamp);
          dep.setTimeStamp(m_timestamp);
          m_task->dispatch(pre, DF_KEEP_TIME);
          m_task->dispatch(dep, DF_KEEP_TIME);

          m_task->spew("pressure: %0.1f, depth: %0.1f", pre.value, dep.value);
        }

        temp.setTimeStamp(m_timestamp);
        m_task->dispatch(temp, DF_KEEP_TIME);

        m_task->spew("status bits: %08X |  temperature: %0.1f",
                     m_status, temp.value);

        // verify wake up status bits.
        uint8_t bits_new = (uint8_t)((c_wake_mask & m_status) >> SB_WAKE_ST);

        if (bits_old != bits_new)
          processBits(bits_new);
      }

      //! Process Wake Up Bits
      //! @param[in] bits status bitmask.
      void
      processBits(uint8_t bits)
      {
        switch (bits)
        {
          case (WS_BAD_POWER):
            m_task->err(DTR("power is not good"));
            break;
          case (WS_POWER_APP):
            break;
          case (WS_BREAK):
            m_task->trace("break");
            break;
          case (WS_RTC_ALARM):
            m_task->err(DTR("RTC alarm"));
          default:
            break;
        }
      }

      //! Parse distance data.
      void
      processDistance(void)
      {
        // Water track data seems to carry no valid distance measurements.
        if (m_type != RT_BOTTOM_TRACK)
          return;

        for (unsigned i = 0; i < c_beam_count; i++)
        {
          float altitude;
          std::memcpy(&altitude, &m_bfr[c_hdr_size + IND_BEAM_D + i * 4], 4);
          m_filter->update(i, altitude);

          if (m_status & (1 << (SB_VAL_DIS + i)))
            m_filter->setValidity(i, IMC::Distance::DV_VALID);
          else
            m_filter->setValidity(i, IMC::Distance::DV_INVALID);

          m_task->spew("beam #%d: %0.1f (m)", i, altitude);
        }

        m_filter->dispatch(m_timestamp);

        // get filtered distance.
        float alt = m_filter->get();
        m_filt_distance.setTimeStamp(m_timestamp);
        m_filt_distance.value = alt;
        m_filt_distance.validity = (alt > 0.0 ? IMC::Distance::DV_VALID :
                                    IMC::Distance::DV_INVALID);
        m_task->dispatch(m_filt_distance, DF_KEEP_TIME);
      }

      //! Parse velocity data.
      void
      processVelocity(void)
      {
        // z1 comes from beams 1 and 3, z2 comes from beams 2 and 4.
        float x, y, z, z1, z2;
        std::memcpy(&x, &m_bfr[c_hdr_size + IND_VEL_XX], 4);
        std::memcpy(&y, &m_bfr[c_hdr_size + IND_VEL_YY], 4);
        std::memcpy(&z1, &m_bfr[c_hdr_size + IND_VEL_Z1], 4);
        std::memcpy(&z2, &m_bfr[c_hdr_size + IND_VEL_Z2], 4);

        // Verify validity of z-axis velocities.
        bool z1valid = m_status & (1 << (SB_VAL_VEL + 2));
        bool z2valid = m_status & (1 << (SB_VAL_VEL + 3));

        // Average if both valid/invalid, or just use valid velocity.
        if ((z1valid && z2valid) || (!z1valid && !z2valid))
          z = (z1 + z2) / 2;
        else if (z1valid)
          z = z1;
        else
          z = z2;

        m_gvel.x = x;
        m_gvel.y = y;
        m_gvel.z = z;
        m_wvel.x = x;
        m_wvel.y = y;
        m_wvel.z = z;

        // Validity flags.
        m_gvel.validity = 0;
        m_wvel.validity = 0;

        if (m_status & (1 << SB_VAL_VEL))
        {
          m_gvel.validity |= IMC::GroundVelocity::VAL_VEL_X;
          m_wvel.validity |= IMC::WaterVelocity::VAL_VEL_X;
        }

        if (m_status & (1 << (SB_VAL_VEL + 1)))
        {
          m_gvel.validity |= IMC::GroundVelocity::VAL_VEL_Y;
          m_wvel.validity |= IMC::WaterVelocity::VAL_VEL_Y;
        }

        if (z1valid || z2valid)
        {
          m_gvel.validity |= IMC::GroundVelocity::VAL_VEL_Z;
          m_wvel.validity |= IMC::WaterVelocity::VAL_VEL_Z;
        }

        m_gvel.setTimeStamp(m_timestamp);
        m_wvel.setTimeStamp(m_timestamp);

        if (m_type == RT_BOTTOM_TRACK)
          m_task->dispatch(m_gvel, DF_KEEP_TIME);
        else if (m_type == RT_WATER_TRACK)
          m_task->dispatch(m_wvel, DF_KEEP_TIME);

        m_task->spew("speed: %0.1f, %0.1f, %0.1f (m/s)", x, y, z);
      }

      //! Parser state machine.
      enum State
      {
        ST_SYNC,
        ST_HEADER,
        ST_ID,
        ST_FAMILY,
        ST_DATA_SIZE_LSB,
        ST_DATA_SIZE_MSB,
        ST_DATA_CSUM_LSB,
        ST_DATA_CSUM_MSB,
        ST_HDR_CSUM_LSB,
        ST_HDR_CSUM_MSB,
        ST_DATA
      };

      //! Data format indices.
      enum Index
      {
        // Status index.
        IND_STATUS = 20,
        // Sound speed.
        IND_SSPEED = 24,
        // Temperature.
        IND_TEMPER = 28,
        // Pressure.
        IND_PRESSU = 32,
        // Beam's distances.
        IND_BEAM_D = 52,
        // Velocity in the x-axis.
        IND_VEL_XX = 132,
        // Velocity in the y-axis.
        IND_VEL_YY = 136,
        // Velocity in the z-axis.
        IND_VEL_Z1 = 140,
        // Velocity in the z-axis.
        IND_VEL_Z2 = 144
      };

      //! Status bits
      enum StatusBits
      {
        SB_VAL_DIS = 4,
        SB_VAL_VEL = 12,
        SB_WAKE_ST = 28
      };

      //! Wake Up State
      enum WakeUpState
      {
        WS_BAD_POWER = 0x00,
        WS_POWER_APP = 0x01,
        WS_BREAK = 0x10,
        WS_RTC_ALARM = 0x11
      };

      //! Return type
      enum ReturnType
      {
        RT_NONE,
        RT_BOTTOM_TRACK,
        RT_WATER_TRACK,
        RT_CURRENT_PROFILE,
        RT_STRING_DATA
      };

      //! Maximum packet size.
      static const unsigned c_max_size = 256;
      //! Sync number.
      static const uint8_t c_sync = 0xa5;
      //! Size of header.
      static const uint8_t c_hdr_size = 10;
      //! Data record current profile identifier.
      static const uint8_t c_cp_type = 0x16;
      //! Data record bottom track identifier.
      static const uint8_t c_bt_type = 0x1b;
      //! Data record water track identifier.
      static const uint8_t c_wt_type = 0x1d;
      //! String data record identifier.
      static const uint8_t c_rc_type = 0xa0;
      //! Instrument family.
      static const uint8_t c_family = 0x10;
      //! Wake Up status bitmask.
      static const uint32_t c_wake_mask = 0xf0000000;
      //! Initial checksum mask.
      static const uint16_t c_csum_start = 0xb58c;
      //! Current profile number of cells bitmask.
      static const uint16_t c_ncells_mask = 0x03ff;
      //! Parent task.
      Tasks::Task* m_task;
      //! IO Handle.
      IO::Handle* m_handle;
      //! Beam Filter.
      Navigation::BeamFilter* m_filter;
      //! Read buffer.
      std::vector<uint8_t> m_bfr;
      //! Header buffer.
      std::vector<uint8_t> m_hdr;
      //! Ground velocity message.
      IMC::GroundVelocity m_gvel;
      //! Water velocity message.
      IMC::WaterVelocity m_wvel;
      //! Filtered distance.
      IMC::Distance m_filt_distance;
      //! Raw messages.
      IMC::DevDataBinary m_raw_data;
      //! Parser state.
      State m_state;
      //! Timestamp of received data.
      double m_timestamp;
      //! Received data index.
      unsigned m_index;
      //! Data size.
      uint16_t m_data_size;
      //! Data checksum.
      uint16_t m_checksum;
      //! Status bits.
      uint32_t m_status;
      //! Device is in water.
      bool m_water;
      //! Return data type.
      ReturnType m_type;
      //! Log file.
      std::ofstream m_log_file;
      //! Log root.
      FileSystem::Path m_log_root;
      //! Log path.
      FileSystem::Path m_log_path;
    };
  }
}

#endif
