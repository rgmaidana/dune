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
// Author: Luis Venancio                                                    *
//***************************************************************************

//TODO: Safe share -> same as transmit data

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
    static const double c_timeout = 5.0;

    struct Ticket
    {
      //! IMC source address.
      uint16_t imc_sid;
      //! IMC source entity.
      uint8_t imc_eid;
      //! Sequence number.
      uint16_t seq;
      //! Wait for ack.
      bool ack;

      Ticket():
      ack(0)
      {}
    };

    struct Operation
    {
      //! Ticket for transmission
      Ticket* ticket;
      //! Time of operation start
      double start_time;
      //! Time of operation end
      double end_time;
      //! Message to handle
      IMC::SAMessage msg;

      Operation():
      ticket(NULL)
      {}

      bool
      isTx()
      {
        return ticket != NULL ? 1 : 0;
      }
    };

    struct Arguments
    {
      //! Multicast Address
      Address udp_maddr;
      //! UDP port
      uint16_t udp_port;
      //! Modem type.
      std::string mtype;
      //! Trasmission speed.
      float tx_speed;
      //! Standard deviation for distance probability.
      fp32_t dst_peak_width;
      //! Standard deviation for data size probability.
      fp32_t dsize_peak_width;
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
      //! Operation queue.
      typedef std::vector<Operation*> OpQueue;
      OpQueue m_queue;
      //! Current operation being handled.
      Operation* m_current_op;
      //! Current transmission ticket.
      Ticket* m_ticket;
      //! Timeout counter.
      Time::Counter<double> m_timeout;
      //! Local simulated state.
      IMC::SimulatedState m_lstate;
      //! PRNG handle.
      Random::Generator* m_prng;

      Task(const std::string& name, Tasks::Context& ctx):
        Tasks::Task(name, ctx),
        m_sock(0),
        m_current_op(NULL),
        m_ticket(NULL),
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
        OpQueue::iterator it=m_queue.begin();
        for (; it != m_queue.end(); ++it)
          delete (*it);
        delete m_current_op;
        m_current_op = NULL;
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
      clearTicket(IMC::UamTxStatus::ValueEnum reason, const std::string& error = "")
      {
        if (m_ticket != NULL)
        {
          sendTxStatus(*m_ticket, reason, error);
          delete m_ticket;
          m_ticket = NULL;
        }
      }

      void
      replaceTicket(const Ticket* ticket)
      {
        clearTicket(IMC::UamTxStatus::UTS_CANCELED);
        m_ticket = new Ticket(*ticket);
      }

      // Send status
      void
      sendTxStatus(const Ticket& ticket, IMC::UamTxStatus::ValueEnum value,
                    const std::string& error = "")
      {
        IMC::UamTxStatus status;
        status.setDestination(ticket.imc_sid);
        status.setDestinationEntity(ticket.imc_eid);
        status.seq = ticket.seq;
        status.value = value;
        status.error = error;
        dispatch(status);
      }

      void
      buildSAMessage(IMC::SAMessage& amsg, const IMC::UamTxFrame* msg)
      {
        // Construct simulated acoustic message from TxFrame
        buildSAMessage(amsg, msg->seq, msg->sys_dst, msg->flags, msg->data);
      }

      void
      buildSAMessage(IMC::SAMessage& amsg, uint16_t seq,
                      std::string sys_dst, uint8_t flags, std::vector<char> data)
      {
        // Construct simulated acoustic message metadata
        Coordinates::toWGS84(m_lstate, amsg.lat, amsg.lon);
        amsg.depth = m_lstate.z;
        amsg.mtype = m_args.mtype;
        amsg.txtime = (float)data.size()*8/m_args.tx_speed;

        // Copy UamTxFrame data
        amsg.seq = seq;
        amsg.sys_dst = sys_dst;
        amsg.flags = flags;
        amsg.data = data;

        // Set header
        amsg.setSource(getSystemId());
        amsg.setTimeStamp();
      }

      // Parse SAMessage into UamRxFrame and send
      void
      rcvRxFrame(const IMC::SAMessage* amsg)
      {
        IMC::UamRxFrame rx;
        rx.sys_src = resolveSystemId(amsg->getSource());
        rx.sys_dst = getSystemName();
        rx.flags = amsg->flags;
        rx.data = amsg->data;
        rx.setTimeStamp();

        dispatch(rx);
        inf("Received message");
      }

      // Parse SAMessage into UamRxFrame
      void
      rcvRxRange(const IMC::SAMessage* amsg)
      {
        IMC::UamRxRange range;
        range.sys = resolveSystemId(amsg->getSource());
        range.seq = amsg->seq;
        range.value = distance(amsg);
        range.setTimeStamp();

        dispatch(range);
        clearTicket(IMC::UamTxStatus::UTS_DONE);
      }

      // Build and send reply to range request
      void
      sendRangeReply(const IMC::SAMessage* range_request)
      {
        Ticket dummy;
        IMC::SAMessage range_reply;

        std::vector<char> data{'R','E','P','L','Y'};
        buildSAMessage(range_reply,
                        range_request->seq,
                        resolveSystemId(range_request->getSource()),
                        IMC::SAMessage::UTF_RPL,
                        data);

        toQueue(&range_reply, &dummy);
      }

      void
      processRx(const IMC::SAMessage* amsg)
      {
        if (amsg->flags == IMC::SAMessage::UTF_RPL)
        {
          rcvRxRange(amsg);
        }
        else
        {
          rcvRxFrame(amsg);

          // Range request
          if (amsg->flags == IMC::SAMessage::UTF_ACK)
            sendRangeReply(amsg);
        }
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

      void
      consume(const IMC::UamTxFrame* msg)
      {
        // Only use local UamTxFrame.
        if (msg->getSource() != getSystemId())
          return;

        // Create and fill new ticket.
        Ticket ticket;
        ticket.imc_sid = msg->getSource();
        ticket.imc_eid = msg->getSourceEntity();
        ticket.seq = msg->seq;
        ticket.ack = (msg->flags & IMC::UamTxFrame::UTF_ACK) != 0;

        if (msg->sys_dst == getSystemName())
        {
          sendTxStatus(ticket, IMC::UamTxStatus::UTS_INV_ADDR);
          return;
        }

        inf("Consume TxFrame");
        IMC::SAMessage amsg;
        buildSAMessage(amsg, msg);
        toQueue(&amsg, &ticket);
      }

      void
      consume(const IMC::GpsFix* msg)
      {
        if (msg->type != IMC::GpsFix::GFT_MANUAL_INPUT)
          return;

        if(!isActive())
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
        if(!isActive())
          requestActivation();

        m_lstate = *msg;
      }

      void
      toQueue(const IMC::SAMessage* amsg, Ticket* tck = NULL)
      {
        if(tck)
        {
          Operation* op = new Operation;

          replaceTicket(tck);
          op->ticket = m_ticket;
          op->start_time = Clock::getSinceEpoch();
          op->end_time = op->start_time + amsg->txtime;
          op->msg = *amsg;

          if (m_ticket->ack)
            m_timeout.setTop(c_timeout);

          m_queue.push_back(op);
          inf("Tx went to Queue");
        }
        else
        {
          // Check range
          double d = distance(amsg);
          if (deliverySucceeds(d, amsg->data.size()))
          {
            Operation *op = new Operation;

            op->ticket = NULL;
            op->start_time = amsg->getTimeStamp()
                            + d/c_sound_speed;
            op->end_time = op->start_time + amsg->txtime;
            op->msg = *amsg;

            m_queue.push_back(op);
            inf("Rx went to queue");
          }
        }
      }

      void
      doOperation()
      {
        if (!m_current_op)
          return;

        if (m_current_op->end_time <= Clock::getSinceEpoch())
        {
          if (m_current_op->isTx())
          {
            share(&m_current_op->msg);

            if (!m_ticket->ack)
              clearTicket(IMC::UamTxStatus::UTS_DONE);
              
            inf("Tx Operation finished");
          }
          else
          {
            processRx(&m_current_op->msg);
            inf("Rx operation finished");
          }

          delete m_current_op;
          m_current_op = NULL;
        }
      }

      void
      checkQueue()
      {
        if (m_queue.empty())
          return;

        OpQueue::iterator it = m_queue.begin();
        while(it != m_queue.end())
        {
          Operation* op = (*it);
          if (op->start_time <= Clock::getSinceEpoch())
          {
            if (isBusy())
            {
              // If a message is received during an operation delete both
              // Collision map (1: keep; 0: destroy):
              // Tx/Tx -> 1/0
              // Tx/Rx -> 0/0
              // Rx/Tx -> 1/0
              // Rx/Rx -> 0/0
              if (op->isTx())
              {
                sendTxStatus(*m_ticket, IMC::UamTxStatus::UTS_BUSY);
              }
              else
              {
                delete m_current_op;
                m_current_op = NULL;

                if (m_current_op->isTx())
                  clearTicket(IMC::UamTxStatus::UTS_FAILED);
              }
            }
            else
            {
              m_current_op = new Operation(*op);

              if (m_current_op->isTx())
                sendTxStatus(*m_ticket, IMC::UamTxStatus::UTS_IP);

              inf("Operation start");
            }

            delete op;
            m_queue.erase(it);
          }
          else
          {            
            ++it;
          }
        }
      }

      void
      checkTimeout()
      {
        if (!m_ticket)
          return;

        if (m_ticket->ack && m_timeout.overflow()) 
        {
          inf("Operation failed");
          clearTicket(IMC::UamTxStatus::UTS_FAILED);
          inf("Operation failed");
        }
          
      }

      bool
      isBusy()
      {
        return m_current_op != NULL ? 1 : 0;
      }

      // Check if message should be parsed
      bool
      toParse(const IMC::SAMessage* amsg)
      {
        // Check destination
        bool check;
        check = resolveSystemName(amsg->sys_dst) == getSystemId();                          // Specific destination
        check |= ((amsg->sys_dst == "broadcast") & (amsg->getSource() != getSystemId()));   // or All

        // Check modem compatibility
        check &= amsg->mtype == m_args.mtype;

        return check;
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
                toQueue(amsg);
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

      void
      onMain(void)
      {
        while (!stopping())
        {
          checkIncomingData();
          checkQueue();
          doOperation();
          checkTimeout();

          waitForMessages(0.1);
        }
      }
    };
  }
}

DUNE_TASK
