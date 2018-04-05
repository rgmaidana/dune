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
//***************************************************************************

// ISO C++ 98 headers.
#include <cmath>
#include <cstdio>
#include <string>
#include <limits>

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Sensors
{
  namespace Optode4831
  {
    using DUNE_NAMESPACES;

    //! Salinity compensation upper temperature constant.
    static const float c_temp_0 = 298.15f;
    //! Salinity compensation lower temperature constant.
    static const float c_temp_1 = 273.15f;
    //! Salinity compensation constant B0.
    static const float c_b0 = -6.24097e-3;
    //! Salinity compensation constant B1.
    static const float c_b1 = -6.93498e-3;
    //! Salinity compensation constant B2.
    static const float c_b2 = -6.90358e-3;
    //! Salinity compensation constant B3.
    static const float c_b3 = -4.29155e-3;
    //! Salinity compensation constant C0.
    static const float c_c0 = -3.11680e-7;

    //! Dissolved oxygen depth compensation factor.
    static const float c_depth_factor = 3.2e-05;
    //! Data input timeout.
    static const float c_timeout = 2.0f;
    //! List of configuration commands.
    static const char* c_cmds[] = {"set passkey(1000)", "set flow control(none)", "set enable text(no)",
                                   "set salinity(0)", "set passkey(1)", "set enable sleep(yes)",
                                   "set enable rawdata(yes)", "set enable airsaturation(yes)",
                                   "set enable temperature(yes)", "set enable decimalformat(yes)",
                                   "set enable polled mode(no)"};
    //! Number of setup commands.
    static const unsigned c_cmds_size = 10;
    //! Reply acknowledgement.
    static const char* c_ack = "#\r\n";
    struct Arguments
    {
      //! Serial port device.
      std::string uart_dev;
      //! Serial port baud rate.
      unsigned uart_baud;
      //! Sampling period.
      double period;
      //! Measurement command string identifier.
      std::string cmd;
      //! Name of device's power channel.
      std::string power_channel;
      //! True to enable automatic activation/deactivation based on medium.
      bool auto_activation;
      //! Temperature entity label.
      std::string elabel_temp;
    };

    struct Task: public Hardware::BasicDeviceDriver
    {
      //! Last compensated depth.
      double m_depth;
      //! Last defined salinity.
      double m_salinity;
      //! Last sensed temperature.
      double m_temperature;
      //! Serial port handle.
      SerialPort* m_uart;
      //! Watchdog.
      Counter<double> m_wdog;
      //! Save device raw data.
      IMC::DevDataText m_dev_data;
      //! Received data line.
      std::string m_line;
      //! Temperature entity id.
      unsigned m_temp_eid;
      //! Task arguments.
      Arguments m_args;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        Hardware::BasicDeviceDriver(name, ctx),
        m_depth(0.0),
        m_salinity(0.0),
        m_temperature(0.0),
        m_uart(NULL)
      {
        paramActive(Tasks::Parameter::SCOPE_IDLE,
                    Tasks::Parameter::VISIBILITY_USER);

        param("Serial Port - Device", m_args.uart_dev)
        .defaultValue("")
        .description("Serial port device used to communicate with the sensor");

        param("Serial Port - Baud Rate", m_args.uart_baud)
        .defaultValue("9600")
        .description("Serial port baud rate");

        param("Sampling Interval", m_args.period)
        .defaultValue("1.0")
        .minimumValue("1.0")
        .maximumValue("1400")
        .units(Units::Second)
        .description("Amount of seconds between samplings");

        param("Measurement String Identifier", m_args.cmd)
        .defaultValue("4831F")
        .description("Measurement command string identifier");

        param("Power Channel", m_args.power_channel)
        .defaultValue("Oxygen Sensor")
        .description("Name of device's power channel");

        param(DTR_RT("Automatic Activation"), m_args.auto_activation)
        .defaultValue("true")
        .visibility(Tasks::Parameter::VISIBILITY_USER)
        .scope(Tasks::Parameter::SCOPE_IDLE)
        .description("Operator is able to control device");

        param("Entity Label - Temperature", m_args.elabel_temp)
        .defaultValue("Depth Sensor")
        .description("Entity label of the IMU");

        bind<IMC::Salinity>(this);
        bind<IMC::Temperature>(this);
        bind<IMC::VehicleMedium>(this);
      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
        if (paramChanged(m_args.power_channel))
        {
          clearPowerChannelNames();
          addPowerChannelName(m_args.power_channel);
        }

        if (isActive() && paramChanged(m_args.period))
        {
          if (stop())
          {
            setPeriod(m_args.period);
            start();
          }
        }
      }

      void
      onEntityResolution(void)
      {
        try
        {
          m_temp_eid = resolveEntity(m_args.elabel_temp);
        }
        catch (...)
        {
          m_temp_eid = std::numeric_limits<unsigned>::max();
        }
      }

      //! Acquire resources.
      bool
      onConnect(void)
      {
        try
        {
          m_uart = new SerialPort(m_args.uart_dev, m_args.uart_baud);
          m_uart->setCanonicalInput(true);
          m_uart->flush();
        }
        catch (std::runtime_error& e)
        {
          throw RestartNeeded(e.what(), 30);
        }

        return true;
      }

      //! Initialize resources.
      void
      onInitializeDevice(void)
      {
        stop();
        getVersion();

        for (unsigned i = 0; i < c_cmds_size; ++i)
        {
          if (!sendCommand(c_cmds[i]))
            return;
        }

        if (!setPeriod(m_args.period))
          return;

        if (start())
          m_wdog.reset();
      }

      //! Release resources.
      void
      onDisconnect(void)
      {
        stop();
        Memory::clear(m_uart);
      }

      void
      onEstimatedState(const IMC::EstimatedState& msg)
      {
        if (msg.getSource() != getSystemId())
          return;

        m_depth = msg.depth;
      }

      void
      consume(const IMC::Salinity* msg)
      {
        if (msg->getSource() != getSystemId())
          return;

        if (msg->value < 0)
          return;

        m_salinity = msg->value;
      }

      void
      consume(const IMC::Temperature* msg)
      {
        if (msg->getSource() != getSystemId())
          return;

        if (msg->getSourceEntity() != m_temp_eid)
          return;

        m_temperature = msg->value;
      }

      void
      consume(const IMC::VehicleMedium* msg)
      {
        if (!m_args.auto_activation)
          return;

        // Request deactivation.
        if (msg->medium == IMC::VehicleMedium::VM_GROUND)
        {
          if (isActive())
            requestDeactivation();
        }
        else
        {
          if (!isActive() && !isActivating())
            requestActivation();
        }
      }

      //! Wake Up device.
      void
      wakeUp(void)
      {
        if (m_uart != NULL)
        {
          std::string bfr(";\r\n");
          m_uart->write(bfr.c_str(), bfr.size());
        }
      }

      //! Stop sampling.
      //! @return true if device was stopped, false otherwise.
      bool
      stop(void)
      {
        if (m_uart == NULL)
          return false;

        wakeUp();
        if (!sendCommand("stop"))
          return false;

        return true;
      }

      //! Start sampling.
      //! @return true if device was started, false otherwise.
      bool
      start(void)
      {
        if (!sendCommand("start"))
          return false;

        return true;
      }

      //! Request device's firmware version.
      void
      getVersion(void)
      {
        sendCommand("get sw version");
        listen();
      }

      //! Set device's sampling rate interval.
      //! @param[in] period desired sampling rate interval.
      //! @return true if command was acknowledged, false otherwise.
      bool
      setPeriod(double period)
      {
        // Watchdog will be twice the desired sampling rate interval.
        m_wdog.setTop(m_args.period * 10);

        return sendCommand(String::str("set interval(%0.1f)", period));
      }

      //! Process incoming sentence.
      //! @param[in] msg sentence.
      void
      process(const std::string& msg)
      {
        double tstamp = Clock::getSinceEpoch();
        m_dev_data.value.assign(sanitize(msg));
        m_dev_data.setTimeStamp(tstamp);
        dispatch(m_dev_data, DF_KEEP_TIME);

        if (String::startsWith(msg, m_args.cmd))
          parse(msg, tstamp);
        else if (String::startsWith(msg, "SW Version"))
          onVersion(msg);
      }

      //! Parse and dispatch incoming data.
      //! @param[in] msg sentence.
      //! @param[in] tstamp current timestamp.
      void
      parse(const std::string& msg, double tstamp)
      {
        IMC::Temperature temp;
        IMC::AirSaturation air;
        IMC::DissolvedOxygen oxy;

        std::sscanf(msg.c_str(),
                    "%*s\t%*u\t%f\t%f\t%f%*[^\n]\n",
                    &oxy.value, &air.value, &temp.value);

        // correct for depth.
        oxy.value *= (1 + c_depth_factor * m_depth);

        // correct for salinity.
        double Ts = std::log((c_temp_0 - m_temperature) / (c_temp_1 + m_temperature));
        double f1 = m_salinity * (c_b0 + c_b1 * Ts + c_b2 * Ts * Ts + c_b3 * Ts * Ts * Ts);
        oxy.value *= std::exp(f1 + c_c0 * m_salinity * m_salinity);

        temp.setTimeStamp(tstamp);
        air.setTimeStamp(tstamp);
        oxy.setTimeStamp(tstamp);

        dispatch(temp, DF_KEEP_TIME);
        dispatch(air, DF_KEEP_TIME);
        dispatch(oxy, DF_KEEP_TIME);

        trace("temperature: %0.1f | saturation: %0.1f | oxygen: %0.2f",
              temp.value, air.value, oxy.value);
        setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
        m_wdog.reset();
      }

      //! Process and announce firmware version
      //! @param[in] msg sentence.
      void
      onVersion(const std::string& msg)
      {
        unsigned major, minor, patch;
        std::sscanf(msg.c_str(), "%*[^\t]\t%*s\t%*u\t%u\t%u\t%u%*[^\n]\n",
                    &major, &minor, &patch);

        inf(DTR("firmware version %u.%u.%u"), major, minor, patch);
      }

      //! Send command to device.
      //! @param[in] cmd command to send.
      //! @return true if command was successful, false otherwise.
      bool
      sendCommand(const std::string& cmd)
      {
        if (m_uart == NULL)
          return false;

        m_dev_data.value.assign(sanitize(cmd));
        dispatch(m_dev_data);

        std::string bfr(cmd + "\r\n");
        m_uart->write(bfr.c_str(), bfr.size());
        spew("sent: '%s'", sanitize(bfr).c_str());
        return readUntil(c_ack, c_timeout);
      }

      //! Check serial port for incoming transmissions.
      void
      listen(void)
      {
        char bfr[256];

        if (!Poll::poll(*m_uart, 1.0))
          return;

        size_t rv = m_uart->readString(bfr, sizeof(bfr));

        for (size_t i = 0; i < rv; ++i)
        {
          m_line.push_back(bfr[i]);

          // Detect line termination.
          if (bfr[i] == '\n')
          {
            spew("recv: '%s'", sanitize(m_line).c_str());
            process(m_line);
            m_line.clear();
          }
        }
      }

      //! Read input until a given sequence is received. Note that
      //! data after the sequence might be discarded.
      //! @param[in] reply sequence to search.
      //! @param[in] timeout timeout in second.
      //! @return true if command was acknowledged, false otherwise.
      bool
      readUntil(const std::string& reply, float timeout)
      {
        Counter<float> timer(timeout);

        while (!timer.overflow())
        {
          if (!Poll::poll(*m_uart, timer.getRemaining()))
            break;

          char bfr[256] = {0};
          m_uart->read(bfr, sizeof(bfr));

          spew("reply: '%s'", sanitize(bfr).c_str());

          if (String::endsWith(bfr, reply))
            return true;
        }

        return false;
      }

      //! Main loop.
      bool
      onReadData(void)
      {
        listen();

        if (m_wdog.overflow())
        {
          setEntityState(IMC::EntityState::ESTA_ERROR, Status::CODE_COM_ERROR);
          throw RestartNeeded(DTR(Status::getString(CODE_COM_ERROR)), 5);
        }

        return true;
      }
    };
  }
}

DUNE_TASK
