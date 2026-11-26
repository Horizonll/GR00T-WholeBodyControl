#pragma once

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <optional>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>

// Called only by SONIC's LowCmd writer; every tick starts with a fresh disabled frame.
class Dex1Grippers {
  static constexpr float open = -5.2f, closed = 0.0f;
  static constexpr float kp = 5.0f, kd = 0.05f, max_lead = 0.15f, torque_limit = 0.6f;
  struct Hand {
    std::optional<bool> target;
    bool stopped = false;
    std::array<float, 3> history{};
    int count = 0;

    float smooth(float q) {
      if (!count || history[2] != q) {
        history = {history[1], history[2], q};
        count = std::min(count + 1, 3);
      }
      return count < 3 ? q : 0.2f * history[0] + 0.3f * history[1] + 0.5f * history[2];
    }
  };
  std::array<Hand, 2> hands_{};

  static bool finite_feedback(float value) {
    // Release builds use -ffast-math, so inspect the IEEE-754 exponent directly.
    return (std::bit_cast<std::uint32_t>(value) & 0x7f800000u) != 0x7f800000u;
  }

public:
  void reset() { hands_ = {}; }

  void write(unitree_hg::msg::dds_::LowCmd_& frame,
             const unitree_hg::msg::dds_::LowState_& state, std::array<bool, 2> close) {
    constexpr std::array<int, 2> slots{31, 33};
    for (size_t side = 0; side < slots.size(); ++side) {
      auto& hand = hands_[side];
      if (hand.target != close[side]) {
        hand = {};
        hand.target = close[side];
      }
      if (hand.stopped) continue;
      const auto& motor = state.motor_state()[slots[side]];
      const float q = motor.q();
      if (motor.motorstate() || !finite_feedback(q) || !finite_feedback(motor.dq()) ||
          !finite_feedback(motor.tau_est())) {
        hand.stopped = true;
        continue;
      }
      const float goal = close[side] ? closed : open;
      const float remaining = close[side] ? goal - q : q - goal;
      if (remaining <= 0.025f) {
        hand.stopped = close[side] || hand.count > 0;
        continue;
      }
      const float margin = (torque_limit - kd * std::abs(motor.dq())) / kp;
      const float low = std::max(open, q - margin), high = std::min(closed, q + margin);
      if (low > high) continue;  // Recheck the command budget on the next frame.
      auto& cmd = frame.motor_cmd()[slots[side]];
      cmd.mode(1);
      cmd.q(std::clamp(hand.smooth(std::clamp(goal, q - max_lead, q + max_lead)), low, high));
      cmd.dq(0);
      cmd.tau(0);
      cmd.kp(kp);
      cmd.kd(kd);
    }
  }
};
