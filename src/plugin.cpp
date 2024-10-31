#include "plugin.h"
#include <mc_control/GlobalPluginMacros.h>
#include <mc_rtc/log/FlatLog.h>

namespace mc_plugin
{

mc_joystick_plugin::~mc_joystick_plugin() = default;

void mc_joystick_plugin::init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config)
{
  if(controller.controller().config().has("JoystickPlugin"))
  {
    configure(controller.controller().config()("JoystickPlugin"));
  }
  else
  {
    configure(config);
  }
  if(!joystick_.isFound())
  {
    joystickConnected_ = false;
    mc_rtc::log::warning("[mc_joystick_plugin] NO JOYPAD DETECTED");
  }
  if(!controller.controller().datastore().has("Joystick::connected"))
  {
    controller.controller().datastore().make<bool>("Joystick::connected");
  }
  controller.controller().datastore().make_call(
      "Joystick::Button", [this](joystickButtonInputs button) -> bool { return get_inputs(button) == 1; });
  controller.controller().datastore().make_call(
      "Joystick::ButtonEvent", [this](joystickButtonInputs button) -> bool { return get_events(button) == 1.; });
  controller.controller().datastore().make_call(
      "Joystick::Trigger", [this](joystickAnalogicInputs trigger) -> double { return get_inputs(trigger); });
  controller.controller().datastore().make_call(
      "Joystick::Stick", [this](joystickAnalogicInputs stick) -> Eigen::Vector2d { return get_stick_value(stick); });
  if(!controller.controller().datastore().has("Joystick::UpPad"))
  {
    controller.controller().datastore().make<bool>("Joystick::UpPad");
  }
  if(!controller.controller().datastore().has("Joystick::DownPad"))
  {
    controller.controller().datastore().make<bool>("Joystick::DownPad");
  }
  if(!controller.controller().datastore().has("Joystick::LeftPad"))
  {
    controller.controller().datastore().make<bool>("Joystick::LeftPad");
  }
  if(!controller.controller().datastore().has("Joystick::RightPad"))
  {
    controller.controller().datastore().make<bool>("Joystick::RightPad");
  }

  joystick_button_state_.setZero();
  joystick_analogical_state_.setZero();

  controller.controller().gui()->addElement(
      {"JoystickPlugin", "State"}, mc_rtc::gui::Button("Reset", [this, &controller]() { reset(controller); }),
      mc_rtc::gui::Checkbox(
          "A", [this]() -> bool { return get_inputs(joystickButtonInputs::A) == 1; }, [this]() {}),
      mc_rtc::gui::Checkbox(
          "B", [this]() -> bool { return get_inputs(joystickButtonInputs::B) == 1; }, [this]() {}),
      mc_rtc::gui::Checkbox(
          "X", [this]() -> bool { return get_inputs(joystickButtonInputs::X) == 1; }, [this]() {}),
      mc_rtc::gui::Checkbox(
          "Y", [this]() -> bool { return get_inputs(joystickButtonInputs::Y) == 1; }, [this]() {}),
      mc_rtc::gui::ArrayLabel("Right Stick", {"x", "y"},
                              [this]() -> Eigen::Vector2d { return get_stick_value(joystickAnalogicInputs::R_STICK); }),
      mc_rtc::gui::ArrayLabel("Left Stick", {"x", "y"},
                              [this]() -> Eigen::Vector2d { return get_stick_value(joystickAnalogicInputs::L_STICK); }),
      mc_rtc::gui::ArrayLabel(
          "Trigger", {"LT", "RT"},
          [this]() -> Eigen::Vector2d {
            return Eigen::Vector2d{get_inputs(joystickAnalogicInputs::LT), get_inputs(joystickAnalogicInputs::RT)};
          }),
      mc_rtc::gui::Checkbox(
          "Start", [this]() -> bool { return get_inputs(joystickButtonInputs::START) == 1; }, [this]() {}),
      mc_rtc::gui::Checkbox(
          "LB", [this]() -> bool { return get_inputs(joystickButtonInputs::LB) == 1; }, [this]() {}),
      mc_rtc::gui::Checkbox(
          "RB", [this]() -> bool { return get_inputs(joystickButtonInputs::RB) == 1; }, [this]() {}));

  auto & logger = controller.controller().logger();
  logger.addLogEntries(
      this, "joystick_plugin_button_sate", [this]() -> const Eigen::Matrix<double, joystickButtonInputs::N_button_inputs, 1> & { return joystick_button_state_; },
      "joystick_plugin_button_event", [this]() -> const Eigen::Matrix<double, joystickButtonInputs::N_button_inputs, 1> & { return joystick_button_event_; },
      "joystick_plugin_analogical_state_0", [this]() -> const Eigen::Matrix<double, joystickAnalogicInputs::N_analogic_inputs, 1> { return joystick_analogical_state_.col(0); },
      "joystick_plugin_analogical_state_1", [this]() -> const Eigen::Matrix<double, joystickAnalogicInputs::N_analogic_inputs, 1> { return joystick_analogical_state_.col(1); }
      
      );
}

double mc_joystick_plugin::get_inputs(joystickButtonInputs in)
{
  return joystick_button_state_(in);
}
double mc_joystick_plugin::get_events(joystickButtonInputs in)
{
  return joystick_button_event_(in);
}
double mc_joystick_plugin::get_inputs(joystickAnalogicInputs in)
{

  return joystick_analogical_state_(in, 0);
}
Eigen::Vector2d mc_joystick_plugin::get_stick_value(joystickAnalogicInputs in)
{
  return Eigen::Vector2d{joystick_analogical_state_(in, 0), joystick_analogical_state_(in, 1)};
}

void mc_joystick_plugin::reset(mc_control::MCGlobalController &)
{
  mc_rtc::log::info("mc_joystick_plugin::reset called");
  joystickConnected_ = false;
  joystick_.reset();
}

void mc_joystick_plugin::before(mc_control::MCGlobalController & controller)
{
  joystickConnected_ = joystick_.isFound();
  joystick_button_event_.setZero();
  controller.controller().datastore().assign<bool>("Joystick::connected", joystickConnected_);
  if(controller.controller().datastore().has("Replay::Log"))
  {
    const auto log = controller.controller().datastore().get<std::shared_ptr<mc_rtc::log::FlatLog>>("Replay::Log");
    if(log->has("joystick_plugin_button_sate"))
    {
      joystick_button_state_ = log->get<Eigen::VectorXd>("joystick_plugin_button_sate", t_indx, joystick_button_state_);
      joystick_button_event_ =
          log->get<Eigen::VectorXd>("joystick_plugin_button_event", t_indx, joystick_button_event_);
      joystick_analogical_state_.col(0) =
          log->get<Eigen::VectorXd>("joystick_plugin_analogical_state_0", t_indx, Eigen::VectorXd::Zero(0));
      joystick_analogical_state_.col(1) =
          log->get<Eigen::VectorXd>("joystick_plugin_analogical_state_1", t_indx, Eigen::VectorXd::Zero(0));
    }
    t_indx += 1;
    
  }
  else if(joystickConnected_)
  {
    if(joystick_.sample(&event_))
    {

      if(event_.isButton())
      {
        if(event_.number < joystickButtonInputs::N_button_inputs)
        {
          joystick_button_state_(static_cast<joystickButtonInputs>(event_.number)) = event_.value;
          joystick_button_event_(static_cast<joystickButtonInputs>(event_.number)) = 1;
        }
      }

      if(event_.isAxis())
      {
        double value = -((event_.value + static_cast<double>(JoystickEvent::MIN_AXES_VALUE))
                         / (static_cast<double>(JoystickEvent::MAX_AXES_VALUE - JoystickEvent::MIN_AXES_VALUE)));

        if(event_.number == 0)
        {
          joystick_analogical_state_(joystickAnalogicInputs::L_STICK, 1) = value;
        }
        if(event_.number == 1)
        {
          joystick_analogical_state_(joystickAnalogicInputs::L_STICK, 0) = value;
        }
        if(event_.number == 2)
        {
          joystick_analogical_state_(joystickAnalogicInputs::LT, 0) = value;
        }
        if(event_.number == 3)
        {
          joystick_analogical_state_(joystickAnalogicInputs::R_STICK, 1) = value;
        }
        if(event_.number == 4)
        {
          joystick_analogical_state_(joystickAnalogicInputs::R_STICK, 0) = value;
        }
        if(event_.number == 5)
        {
          joystick_analogical_state_(joystickAnalogicInputs::RT, 0) = value;
        }
        if(event_.number == 6)
        {
          if(static_cast<double>(event_.value) == 32767)
          {
            // std::cout << "right pad" << std::endl;
            joystick_analogical_state_(joystickAnalogicInputs::RIGHT_PAD, 0) = value;
            controller.controller().datastore().assign<bool>("Joystick::RightPad", true);
            controller.controller().datastore().assign<bool>("Joystick::LeftPad", false);
          }
          else if(static_cast<double>(event_.value) == -32767)
          {
            // std::cout << "left pad" << std::endl;
            joystick_analogical_state_(joystickAnalogicInputs::LEFT_PAD, 0) = value;
            controller.controller().datastore().assign<bool>("Joystick::RightPad", false);
            controller.controller().datastore().assign<bool>("Joystick::LeftPad", true);
          }
          else
          {
            controller.controller().datastore().assign<bool>("Joystick::RightPad", false);
            controller.controller().datastore().assign<bool>("Joystick::LeftPad", false);
          }
        }
        if(event_.number == 7)
        {
          if(static_cast<double>(event_.value) == 32767)
          {
            // std::cout << "bottom pad" << std::endl;
            joystick_analogical_state_(joystickAnalogicInputs::DOWN_PAD, 0) = value;
            controller.controller().datastore().assign<bool>("Joystick::DownPad", true);
            controller.controller().datastore().assign<bool>("Joystick::UpPad", false);
          }
          else if(static_cast<double>(event_.value) == -32767)
          {
            // std::cout << "up pad" << std::endl;
            joystick_analogical_state_(joystickAnalogicInputs::UP_PAD, 0) = value;
            controller.controller().datastore().assign<bool>("Joystick::DownPad", false);
            controller.controller().datastore().assign<bool>("Joystick::UpPad", true);
          }
          else
          {
            controller.controller().datastore().assign<bool>("Joystick::DownPad", false);
            controller.controller().datastore().assign<bool>("Joystick::UpPad", false);
          }
        }
      }
    }
  }

  else
  {
    joystick_.reset();
  }
}

void mc_joystick_plugin::after(mc_control::MCGlobalController &)
{
  mc_rtc::log::info("mc_joystick_plugin::after");
}

mc_control::GlobalPlugin::GlobalPluginConfiguration mc_joystick_plugin::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = true;
  out.should_run_after = false;
  out.should_always_run = false;
  return out;
}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("mc_joystick_plugin", mc_plugin::mc_joystick_plugin)
