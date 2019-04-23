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

    // Environmental constants
    static const double c_sound_speed = 1500;
    static const double c_max_range = 3000;

    enum Codes
    {
      CODE_RANGE    = 0x01,
      CODE_REPLY    = 0x07
    };

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
      int tx_speed;

      //! Standard deviation for distance probability.
      float dst_peak_width;
      //! Standard deviation for data size probability.
      float dsize_peak_width;
      //! PRNG type.
      std::string prng_type;
      //! PRNG seed.
      int prng_seed;
    };

    struct Task: public Tasks::Task
    {
      //! Task arguments.
      Arguments m_args;

      //! UDP socket.
      UDPSocket* m_sock;
      //! UDP message buffer.
      uint8_t m_buf[1024];

      //! Delivery queue.
      //! <time of delivery, message*>
      typedef std::map<double, const IMC::SAMessage*> ReceiveMap;
      ReceiveMap m_queue;

      //! Local simulated state.
      IMC::SimulatedState m_lstate;
      //! PRNG handle.
      Random::Generator* m_prng;

      Task(const std::string& name, Tasks::Context& ctx):
        Tasks::Task(name, ctx),
        m_sock(0),
        m_prng(NULL)
      {
        param("UDP Communications -- Multicast Address", m_args.udp_maddr)
        .defaultValue("225.0.2.1")
        .description("UDP multicast address for communications");

        param("UDP Communications -- Port", m_args.udp_port)
        .defaultValue("8021")
        .description("UDP port for communications");

        param("Modem Type", m_args.mtype)
        .description("Vehicle modem type (Ex. Evologics, Seatrac)");

        param("Transmission Speed", m_args.tx_speed)
        .description("Modem transmission speed (bps)");

        param("Distance Standard Deviation", m_args.dst_peak_width)
        .defaultValue("750");

        param("Size Standard Deviation", m_args.dsize_peak_width)
        .defaultValue("21");

        param("PRNG Type", m_args.prng_type)
        .defaultValue(Random::Factory::c_default);

        param("PRNG Seed", m_args.prng_seed)
        .defaultValue("-1");

        // Register consumers.
        bind<IMC::GpsFix>(this);
        bind<IMC::SimulatedState>(this);
        bind<IMC::UamTxFrame>(this);
      }

      ~Task(void)
      {
        onResourceRelease();
      }

      void
      onResourceAcquisition(void)
      {
        // Initialize UDP socket in multicast
        m_sock = new DUNE::Network::UDPSocket();
        m_sock->setMulticastTTL(1);
        m_sock->setMulticastLoop(true);
        m_sock->joinMulticastGroup(m_args.udp_maddr);
        m_sock->bind(m_args.udp_port);

        // Initialize random number generator
        m_prng = Random::Factory::create(m_args.prng_type, m_args.prng_seed);

        // Deactivate until SimulatedState message is received
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
        for (; it != m_queue.end(); ++it)
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

      void
      buildSAMessage(IMC::SAMessage &amsg, const IMC::UamTxFrame* msg)
      {
        // Construct simulated acoustic message from TxFrame
        buildSAMessage(amsg, msg->seq, msg->sys_dst, msg->flags, msg->data);
      }

      void
      buildSAMessage(IMC::SAMessage &amsg, uint16_t seq, 
                      std::string sys_dst, uint8_t flags, std::vector<char> data)
      {
        // Construct simulated acoustic message metadata
        Coordinates::toWGS84(m_lstate, amsg.lat, amsg.lon);
        amsg.depth = m_lstate.z;
        amsg.mtype = m_args.mtype;
        amsg.txtime = data.size()*8/m_args.tx_speed;

        // Copy UamTxFrame data
        amsg.seq = seq;
        amsg.sys_dst = sys_dst;
        amsg.flags = flags;
        amsg.data = data;

        // Set header
        amsg.setSource(getSystemId());
        amsg.setTimeStamp();
      }

      // Parse SAMessage into UamRxFrame
      void
      buildRxFrame(IMC::UamRxFrame &rx, const IMC::SAMessage* amsg)
      {
        rx.sys_src = resolveSystemId(amsg->getSource());
        rx.sys_dst = getSystemName();
        rx.flags = amsg->flags;
        rx.data = amsg->data;
        rx.setTimeStamp();
      }

      // Parse SAMessage into UamRxFrame
      void
      buildRxRange(IMC::UamRxRange &range, const IMC::SAMessage* amsg)
      {
        range.sys = resolveSystemId(amsg->getSource());
        range.seq = amsg->seq;
        range.value = distance(amsg);
        range.setTimeStamp();
      }

        return 0;
      }

      // Build and send reply to range request
      void
      sendRangeReply(const IMC::SAMessage* amsg)
      {
        IMC::SAMessage range_reply;

        // Message never gets to UAN as RxFrame, CRC8 not needed
        std::vector<char> data = amsg->data;
        data[1] = CODE_REPLY;
        buildSAMessage(range_reply, amsg->seq, 
                        resolveSystemId(amsg->getSource()), 
                        0x00,
                        data);

        // Send reply
        share(&range_reply);
      }

      // Compute distance to source vehicle
      double
      distance(const IMC::SAMessage* src_state)
      {
        // Convert to absolute coordinates
        double llat, llon;
        Coordinates::toWGS84(m_lstate, llat, llon);

        return WGS84::distance(llat, llon, m_lstate.z,
                               src_state->lat, src_state->lon, src_state->depth);
      }

      // Check if there are messages to deliver
      void
      checkMessages()
      {
        // Check all messages
        ReceiveMap::iterator it = m_queue.begin();
        for (; it != m_queue.end(); ++it)
          if (it->first <= Clock::getSinceEpoch())
          {
            const IMC::SAMessage* amsg = it->second;
            if (amsg->data[1] != CODE_REPLY)
            {
              // Set time and dispatch
              IMC::UamRxFrame msg;
              buildRxFrame(msg, amsg);
              dispatch(msg);

              // Range request
              if (amsg->data[1] == CODE_RANGE)
              {
                sendRangeReply(amsg);
              }
            }
            else
            {
              // Construct UamRxRange
              IMC::UamRxRange range;
              buildRxRange(range, amsg);
              dispatch(range);
            }

            // Erase message
            delete amsg;
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
        buildSAMessage(amsg, msg);

        // Send
        amsg.setSource(getSystemId());
        amsg.setTimeStamp();
        share(&amsg);

        //Send to bus (Logging)
        // Send to bus (Logging)
        dispatch(amsg);
      }

      void
      consume(const IMC::GpsFix* msg)
      {
        if (msg->type != IMC::GpsFix::GFT_MANUAL_INPUT)
          return;

        if (!isActive())
          requestActivation();

        // Define vehicle origin.
        m_lstate.lat = msg->lat;
        m_lstate.lon = msg->lon;
        m_lstate.height = msg->height;
        m_lstate.x = 0;
        m_lstate.y = 0;
        m_lstate.z = 0;
      }

      void
      consume(const IMC::SimulatedState* msg)
      {
        if (!isActive())
          requestActivation();

        m_lstate = *msg;
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

      // Check if message should be parsed
      bool
      toParse(IMC::SAMessage* amsg)
      {
        // Check destination
        bool check;
        check = resolveSystemName(amsg->sys_dst) == getSystemId();                          // Specific destination
        check |= ((amsg->sys_dst == "broadcast") & (amsg->getSource() != getSystemId()));   // or All

        // Check modem compatibility
        check &= amsg->mtype == m_args.mtype;

        return check;
      }

      // Simulate delivery failures gausian distribution of distance
      bool
      deliverySucceeds(double distance, uint16_t data_size)
      {
        // Out of range
        if (distance > c_max_range)
          return 0;
        
        // Gaussian profiles
        float dist_prob = exp(-1 * (distance*distance)/(2 * m_args.dst_peak_width * m_args.dst_peak_width));
        float size_prob = exp(-1 * (float)(data_size*data_size)/
                                (2 * m_args.dsize_peak_width * m_args.dsize_peak_width));

        return m_prng->uniform() <= dist_prob*size_prob;
      }

      // Check collisions
      bool
      checkCollisions(double toa, double tod)
      {
        // Check all messages for collisions
        ReceiveMap::iterator it = m_queue.begin();
        for (; it != m_queue.end(); ++it)
        {
          // If a new message arrives during the reception 
          // time interval [toa tod] of another message
          double q_tod = it->first;
          double q_txtime = it->second->txtime;
          double q_toa = q_tod - q_txtime;
          if (!(toa > q_tod || tod < q_toa))
          {
            // Erase message
            delete it->second;
            m_queue.erase(it);
            return 1;
          }
        }

        return 0;
      }

      // Send SAMessage to queue
      void
      sendToQueue(IMC::SAMessage* amsg, double d)
      {
        // Clone message
        IMC::SAMessage* msg = amsg->clone();

        // Time of arrival
        double toa = amsg->getTimeStamp()   // Start sending
                    + d/c_sound_speed;      // Travel time
        // Time of delivery
        double tod = toa + amsg->txtime;    // Time to send last bit

        if (!checkCollisions(toa, tod))
          m_queue.insert(std::pair<double, SAMessage*>(tod, msg));
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
            if (m->getId() == DUNE_IMC_SAMESSAGE)
            {
              IMC::SAMessage* amsg = static_cast<IMC::SAMessage*>(m);

              if (toParse(amsg))
              {
                // Check range
                double d = distance(amsg);
                if (deliverySucceeds(d, amsg->data.size()))
                  sendToQueue(amsg, d);
              }
            }
            else
            {
              err(DTR("unexpected simulation message: %s"), m->getName());
            }
            delete m;
          }
        }
        catch(std::runtime_error& e)
        {
          err(DTR("read error: %s"), e.what());
        }
      }
    };
  }
}

DUNE_TASK
