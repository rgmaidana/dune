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
// Author: Eduardo Marques                                                  *
// Author: Luis Venancio                                                    *
//***************************************************************************

// ISO C++ 98 headers.
#include <cstdlib>
#include <cmath>
#include <map>
#include <iomanip>

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Simulators
{
  namespace AcousticModem
  {
    using DUNE_NAMESPACES;

    struct Arguments
    {
      //! Multicast Address
      Address udp_maddr;
      //! UDP port
      uint16_t udp_port;

      //! Location of static device
      std::vector<double> location;

      //! Modem type.
      std::string mtype;
      //! Trasmission speed.
      uint16_t tx_speed;

      //! Standard deviation for distance probability.
      fp32_t dst_peak_width;
      //! Standard deviation for data size probability.
      fp32_t dsize_peak_width;
      //! PRNG type.
      std::string prng_type;
      //! PRNG seed.
      int prng_seed;
    };

    static const double c_sound_speed = 1500;
    static const double c_max_range = 3000;

    struct Task: public Tasks::Task
    {
      //! Task arguments.
      Arguments m_args;

      //! Static device flag.
      bool m_fixed_location;

      //! UDP socket.
      UDPSocket* m_sock;
      //! UDP message buffer.
      uint8_t m_buf[1024];

      //! Delivery queue.
      //! <time of delivery, message to release*>
      typedef std::map<double, IMC::UamRxFrame*> ReceiveMap;
      ReceiveMap m_queue;
      //! Local simulated state.
      IMC::SimulatedState m_lstate;      
      //! PRNG handle.
      Random::Generator* m_prng;

      Task(const std::string& name, Tasks::Context& ctx):
        Tasks::Task(name, ctx),
        m_fixed_location(false),
        m_sock(0),
        m_prng(NULL)
      {
        param("UDP Communications -- Multicast Address", m_args.udp_maddr)
        .defaultValue("225.0.2.1")
        .description("UDP multicast address for communications");

        param("UDP Communications -- Port", m_args.udp_port)
        .defaultValue("8021")
        .description("UDP port for communications");

        param("Fixed Location", m_args.location)
        .defaultValue("")
        .description("WGS84 latitude and longitude for node with fixed position");

        param("Modem Type", m_args.mtype)
        .description("Vehicle modem type (Ex. Evologics, Seatrac)");

        param("Transmission Speed", m_args.tx_speed)
        .description("Modem transmission speed (bps)");

        param("Distance Standard Deviation", m_args.dst_peak_width)
        .defaultValue("1000");

        param("Size Standard Deviation", m_args.dsize_peak_width)
        .defaultValue("21");

        param("PRNG Type", m_args.prng_type)
        .defaultValue(Random::Factory::c_default);

        param("PRNG Seed", m_args.prng_seed)
        .defaultValue("-1");

        // Register consumers.
        bind<IMC::SimulatedState>(this);
        bind<IMC::UamTxFrame>(this);
      }

      ~Task(void)
      {
        onResourceRelease();
      }

      void
      onUpdateParameters(void)
      {
        if (m_args.location.size() == 2)
        {
          m_fixed_location = true;
          m_lstate.clear();
          m_lstate.lat = Angles::radians(m_args.location[0]);
          m_lstate.lon = Angles::radians(m_args.location[1]);
          m_lstate.z = 0;
        }
      }

      void
      onResourceAcquisition(void)
      {
        //Initialize UDP socket in multicast
        m_sock = new DUNE::Network::UDPSocket();
        m_sock->setMulticastTTL(1);
        m_sock->setMulticastLoop(true);
        m_sock->joinMulticastGroup(m_args.udp_maddr);
        m_sock->bind(m_args.udp_port);

        //Initialize random number generator
        m_prng = Random::Factory::create(m_args.prng_type, m_args.prng_seed);

        //Deactivate until SimulatedState message is received
        requestDeactivation();
      }

      void
      onResourceRelease(void)
      {
        if (m_sock)
        {
          delete m_sock;
          m_sock = 0;
        }

        Memory::clear(m_prng);
        ReceiveMap::iterator it = m_queue.begin();
        for(; it != m_queue.end(); ++it)
          Memory::clear(it->second);
      }

      void
      share(const IMC::Message* msg)
      {
        // Share message over the UDP multicast socket.
        int n = msg->getSerializationSize();
        IMC::Packet::serialize(msg, m_buf, n);
        m_sock->write(m_buf, n, m_args.udp_maddr, m_args.udp_port);
      }

      // Compute distance to source vehicle
      double
      distance(IMC::SAMessage* src_state)
      {
        //Convert to absolute coordinates
        fp64_t llat, llon;
        Coordinates::toWGS84(m_lstate, llat, llon);

        return WGS84::distance(llat, llon, m_lstate.z,
                               src_state->lat, src_state->lon, src_state->depth);
      }

      // Simulate delivery failures gausian distribution of distance
      bool
      deliverySucceeds(fp64_t distance, uint16_t data_size)
      {
        // Out of range
        if(distance > c_max_range)
          return 0;
        
        // Gaussian profiles
        fp32_t dist_prob = exp(-1 * (distance*distance)/(2 * m_args.dst_peak_width * m_args.dst_peak_width));
        fp32_t size_prob = exp(-1 * (fp32_t)(data_size*data_size)/
                                (2 * m_args.dsize_peak_width * m_args.dsize_peak_width));

        return m_prng->uniform() <= dist_prob*size_prob ? 1 : 0;
      }

      // Parse SAMessage into UamRxFrame and add to queue
      void
      parseRx(IMC::SAMessage* amsg, fp64_t d)
      {
        // Create UamRxFrame
        IMC::UamRxFrame* rx = new UamRxFrame;
        rx->sys_src = resolveSystemId(amsg->getSource());
        rx->sys_dst = getSystemName();
        rx->flags = amsg->flags;
        rx->data = amsg->data;

        // Simulate time of delivery
        fp64_t tod = amsg->getTimeStamp()                                         //Start sending
                    + amsg->data.size()*8/amsg->tspeed   //Time to send last bit
                    + d/c_sound_speed;                                            //Travel time

        // If t.o.d. has passed: dispatch; otherwise: queue
        if(tod <= Clock::getSinceEpoch())
          dispatch(rx);
        else
          m_queue.insert(std::pair<double, UamRxFrame*>(tod, rx));
      }

      void
      checkMessages()
      {
        // Check all messages
        ReceiveMap::iterator it = m_queue.begin();
        for(; it != m_queue.end(); ++it)
          if(it->first <= Clock::getSinceEpoch())
          {
            // Set time and dispatch
            IMC::UamRxFrame* msg = it->second;
            msg->setTimeStamp();
            dispatch(msg);

            // Erase message
            delete msg;
            m_queue.erase(it);
          }
      }

      void
      consume(const IMC::UamTxFrame* msg)
      {
        // Only use local UamTxFrame.
        if (msg->getSource() != getSystemId())
          return;

        // Construct simulated acoustic message metadata
        IMC::SAMessage amsg;
        Coordinates::toWGS84(m_lstate, amsg.lat, amsg.lon);
        amsg.depth = m_lstate.z;
        amsg.mtype = m_args.mtype;
        amsg.tspeed = m_args.tx_speed;

        // Copy UamTxFrame data
        amsg.seq = msg->seq;
        amsg.sys_dst = msg->sys_dst;
        amsg.flags = msg->flags;
        amsg.data = msg->data;

        // Send
        amsg.setSource(getSystemId());
        amsg.setTimeStamp();
        share(&amsg);
      }

      void
      consume(const IMC::SimulatedState* msg)
      {
        if(m_fixed_location)
          return;

        m_lstate = *msg;

        if(!isActive())
          requestActivation();
      }

      void
      onMain(void)
      {
        while (!stopping())
        {
          // Check for messages in queue
          if (!m_queue.empty())
            checkMessages();

          checkIncomingData();
          waitForMessages(0.1);
        }
      }

      void
      checkIncomingData(void)
      {
        Address dummy;

        try
        {
          if (Poll::poll(*m_sock, 0.01))
          {
            // Retrieve message
            size_t n = m_sock->read(m_buf, sizeof(m_buf), &dummy);
            IMC::Message* m = IMC::Packet::deserialize(m_buf, n);

            // Check if msg is simulated acoustic msg
            if(m->getId() == DUNE_IMC_SAMESSAGE)
            {
              IMC::SAMessage* amsg = static_cast<IMC::SAMessage*>(m);

              // Check destination and modem compatibility
              bool dst_check = resolveSystemName(amsg->sys_dst) == getSystemId();                    //Specific destination
              dst_check |= ((amsg->sys_dst == "broadcast") & (amsg->getSource() != getSystemId()));  //All
              if(dst_check && amsg->mtype == m_args.mtype)
              {
                // Check range
                fp64_t d = distance(amsg);
                if(deliverySucceeds(d, amsg->data.size()))
                  parseRx(amsg, d);
              }
            }
            else
            {
              err(DTR("unexpected simulation message: %s"), m->getName());
            }
            delete m;
          }
        }
        catch (std::runtime_error& e)
        {
          err(DTR("read error: %s"), e.what());
        }
      }
    };
  }
}

DUNE_TASK
