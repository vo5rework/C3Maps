#include <string>
#include <iostream>
#include <sstream>
#include <cassert>

#include "tsettings.hpp"
#include "tcontrols.hpp"
#include "num_param.h"
#include "str_param.h"
#include "txt_param.h"

#define TINKLA_TOGGLE 1
#define TINKLA_FLOAT 2
#define TINKLA_STRING 3
#define TINKLA_TEXT 4

// ---------------------------------------------
// TeslaTogglesPanel
// ---------------------------------------------
TeslaTogglesPanel::TeslaTogglesPanel(SettingsWindow *parent) : ListWidget(parent) {

  std::vector<std::tuple<QString, QString, QString, QString, QString, QString, QString, float,float,float,float,int>> tinkla_toggles{

    {"TinklaAdjustAccWithSpeedLimit",
    "Adjust ACC max with speed limit",
    "Adjust cruise control speed limit when legal speed limit for the road changes.",
    "../assets/offroad/icon_speed_limit.png",
    "","","",0.0,0.0,0.0,0.0, TINKLA_TOGGLE
    },

    {"TinklaTurnSlowdownEnabled",
     "Slow Down in Turns",
     "Enable automatic cruise speed reduction for sharp turns based on curvature.",
     "../assets/offroad/icon_speed_limit.png",
     "","","",0.0,0.0,0.0,0.0,TINKLA_TOGGLE
    },

    {"TinklaTurnSlowdownFactor",
     "Turn Slowdown Factor",
     "Scales the cruise speed reduction when approaching turns based on curvature. 1.0 = default betaC3 behavior, <1 = less aggressive, >1 = more aggressive.",
     "../assets/offroad/icon_speed_limit.png",
     "Turn Slowdown Factor:",
     "Multiplier for turn-based slowdown",
     "",
     1.0, 0.5, 2.0, 0.05, TINKLA_FLOAT
    },

    {"TinklaSpeedLimitUseRelative",
    "Use relative offset",
    "Use a relative offset (percentage of speed limit).",
    "../assets/offroad/icon_speed_limit.png",
    "","","",0.0,0.0,0.0,0.0, TINKLA_TOGGLE
    },
    {"TinklaSpeedLimitOffset",
      "Speed Limit Offset",
      "The speed offset vs. the legal speed limit you want ACC to apply when automatically changing with speed limit (in your car's UOM or percentage if using relative offset).",
      "../assets/offroad/icon_speed_limit.png",
      "Speed Limit Offset:",
      "Enter offset in your car's UOM",
      "",
      0.0,-5.0,20.0,1.0,TINKLA_FLOAT
    },
    {"TinklaBrakeFactor",
      "Braking Factor",
      "The multiplier used to compute the Tesla braking power. 0.5 is less and 1.5 is more.",
      "../assets/offroad/icon_speed_limit.png",
      "Enter the braking multiplier:",
      "Enter the braking multiplier:",
      "",
      1.0,0.5,1.5,0.01,TINKLA_FLOAT
    },
    {"TinklaAccelProfile",
      "Acceleration Profile",
      "The profile to be used for acceleration: 1-Chill, 2-Standard, 3-MadMax",
      "../assets/offroad/icon_speed_limit.png",
      "Acceleration Profile:","Enter profile #.",
      "",
      2.0,1.0,3.0,1.0,TINKLA_FLOAT
    },
    {"TinklaTeslaRadarIgnoreSGUError",
    "Ignore Radar Errors",
    "Ignore Tesla Radar errors about calibration. ",
    "../assets/offroad/icon_settings.png",
    "","","",0.0,0.0,0.0,0.0, TINKLA_TOGGLE
    },
    {"TinklaAutopilotDisabled",
    "Autopilot feature disabled",
    "Use when car has the autopilot feature disabled.",
    "../assets/offroad/icon_settings.png",
    "","","",0.0,0.0,0.0,0.0, TINKLA_TOGGLE
    },
    {"TinklaDisableStartStopSounds",
    "Disable Main Sounds",
    "Disables the device from playing the Engagement and Disengagement sounds. To be used when the car will generate these sounds by itself. Prompt and Warning sounds will still be played.",
    "../assets/offroad/icon_settings.png",
    "","","",0.0,0.0,0.0,0.0, TINKLA_TOGGLE
    },
    {"TinklaDisablePromptSounds",
    "Disable Prompt Sounds",
    "Disables the device from playing the Prompt sounds. To be used when the car will generate these sounds by itself.  Engagement/Disengagement and Warning sounds will still be played.",
    "../assets/offroad/icon_settings.png",
    "","","",0.0,0.0,0.0,0.0, TINKLA_TOGGLE
    },

  };

  Params params;
  for (auto &[param, title, desc, icon, edit_title,edit_desc, edit_uom, val_default,val_min,val_max,val_step, field_type] : tinkla_toggles) {
    if (field_type == TINKLA_TOGGLE) {
      auto toggle = new TinklaParamControl(param, title, desc, icon, this);
      bool locked = params.getBool((param + "Lock").toStdString());
      toggle->setEnabled(!locked);
      if (!locked) {
        connect(uiState(), &UIState::offroadTransition, toggle, &ParamControl::setEnabled);
      }
      addItem(toggle);
    }
    if (field_type == TINKLA_FLOAT) {
      addItem(new NumParamControl(title, desc, edit_title,edit_desc, edit_uom, param,val_default,val_min,val_max,val_step, icon));
    }
    if (field_type == TINKLA_STRING) {
      addItem(new StrParamControl(title, desc, edit_title,edit_desc, param, edit_uom, QString::fromStdString(""), icon));
    }
  };
}
