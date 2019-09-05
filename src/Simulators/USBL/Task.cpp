//***************************************************************************
// Copyright 2007-2019 Universidade do Porto - Faculdade de Engenharia      *
// Laboratório de Sistemas e Tecnologia Subaquática (LSTS)                  *
//***************************************************************************
// This file is part of DUNE: Unified Navigation Environment.               *
//                                                                          *
// Commercial Licence Usage                                                 *
// Licencees holding valid commercial DUNE licences may use this file in    *
// accordance with the commercial licence agreement provided with the       *
// Software or, alternatively, in accordance with the terms contained in a  *
// written agreement between you and Faculdade de Engenharia da             *
// Universidade do Porto. For licensing terms, conditions, and further      *
// information contact lsts@fe.up.pt.                                       *
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
// Authors: Pedro Calado / Renan Maidana / Luis Venancio                    *
//***************************************************************************

// ISO C++ 98 headers.
#include <cstring>

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Simulators
{
  namespace USBL
  {
    using DUNE_NAMESPACES;

    struct Arguments
    {
      //! USBL latitude coordinate
      double usbl_lat;
      //! USBL longitude coordinate
      double usbl_lon;
      //! Heading angle of the USBL mount
      double usbl_heading;
      //! Depth of the USBL transducer
      double usbl_depth;
      //! USBL Slant range accuracy
      double usbl_slant_acc;
      //! USBL Bearing Resolution
      double usbl_bearing_res;
      //! Transmission delay
      double trans_delay;
    };

    //! Struct for the field value in DeviceData Binary message
    struct USBLMessage
    {
      //! range in meters
      double range;
      //! bearing in radians
      double bearing;
      //! elevation in radians
      double elevation;
    };

    struct Task: public Tasks::Periodic
    {
      //! Entity state message.
      IMC::EntityState m_ent;
      //! Current position.
      IMC::SimulatedState m_sstate;
      //! North offset of the USBL acoustic transducer
      double m_usbl_off_n;
      //! East offset of the USBL acoustic transducer
      double m_usbl_off_e;
      //! Task Arguments.
      Arguments m_args;

      //! Vector holding distances reported by the acoustic modem 
      std::vector<double> usbl_distances;
      //! Debug string to see who sent the range messages back
      std::string destRange;

      Task(const std::string& name, Tasks::Context& ctx):
        Tasks::Periodic(name, ctx)
      {
        param("Latitude", m_args.usbl_lat)
        .defaultValue("0.0")
        .units(Units::Degree)
        .description("Latitude coordinate of the USBL transducer location");

        param("Longitude", m_args.usbl_lon)
        .defaultValue("0.0")
        .units(Units::Degree)
        .description("Longitude coordinate of the USBL transducer location");

        param("Mounted Heading", m_args.usbl_heading)
        .defaultValue("0.0")
        .units(Units::Degree)
        .description("Heading angle at which the transducer was mounted");

        param("Mounted Depth", m_args.usbl_depth)
        .defaultValue("2.0")
        .units(Units::Meter)
        .description("Depth at which the transducer was mounted");

        param("Slant Range Accuracy", m_args.usbl_slant_acc)
        .defaultValue("0.01")
        .units(Units::Meter)
        .description("Sensor's slant range accuracy");

        param("Bearing Resolution", m_args.usbl_bearing_res)
        .defaultValue("0.1")
        .units(Units::Degree)
        .description("Sensor's bearing resolution");

        param("Transmission Delay", m_args.trans_delay)
        .defaultValue("0.5")
        .units(Units::Second)
        .description("Delay of the transmission");

        setEntityState(IMC::EntityState::ESTA_BOOT, Status::CODE_WAIT_GPS_FIX);

        bind<IMC::GpsFix>(this);
        bind<IMC::UamRxRange>(this);
      }

      void
      onUpdateParameters(void)
      {
        if (paramChanged(m_args.usbl_lat))
          m_args.usbl_lat = Angles::radians(m_args.usbl_lat);

        if (paramChanged(m_args.usbl_lon))
          m_args.usbl_lon = Angles::radians(m_args.usbl_lon);

        if (paramChanged(m_args.usbl_heading))
          m_args.usbl_heading = Angles::normalizeRadian(Angles::radians(m_args.usbl_heading));

        if (paramChanged(m_args.usbl_bearing_res))
          m_args.usbl_bearing_res = Angles::radians(m_args.usbl_bearing_res);

        if (1 / getFrequency() <= m_args.trans_delay)
        {
          std::string msg = "Transmission delay must be shorter than task's period";
          err("%s", msg.c_str());
          throw std::runtime_error(msg);
        }
      }

      void
      consume(const IMC::GpsFix* msg)
      {
        if (msg->type != IMC::GpsFix::GFT_MANUAL_INPUT)
          return;

        WGS84::displacement(msg->lat, msg->lon, 0,
                            m_args.usbl_lat, m_args.usbl_lon, 0,
                            &m_usbl_off_n, &m_usbl_off_e);

        trace("offsets to navigational reference | %0.2f %0.2f", m_usbl_off_n, m_usbl_off_e);
        
        // Here we see the displacement between the modem's gps position and the USBL sensor
        inf("Offsets to navigational reference - North: %0.2f | East: %0.2f", m_usbl_off_n, m_usbl_off_e);

        setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
      }

      // After we send a "ping" through the AcousticModem, we receive and consume a UamRxRange,
      // dispatched by the AcousticModem itself. With the distance reported, we can do some stuff (TBD)
      void
      consume(const IMC::UamRxRange* msg){
          inf("Added range %f to the distance vector", msg->value);
          usbl_distances.push_back(msg->value);
          if (usbl_distances.size() == 10)
            inf("Finished obtaining ranges from %s", msg->sys.c_str());
      }

      void task(void){
        if (getEntityState() != IMC::EntityState::ESTA_NORMAL)
            return;   // Did not get GpsFix message

        // As we are sending UamTxFrame messages, the AcousticModem won't send new messages if it
        // is in the "busy" state, so we don't have to worry about waiting to transmit a new message
        // If the last range has been received, send another "ping" to some vehicle
        // Don't send if the distances vector is complete
        if (usbl_distances.size() < 10){
          IMC::UamTxFrame uamMsg;
          uamMsg.sys_dst = "lauv-noptilus-2";
          uamMsg.flags = IMC::UamTxFrame::UTF_ACK;
          dispatch(uamMsg);
        }
      }
    };
  }
}

DUNE_TASK
