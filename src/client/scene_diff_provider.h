#ifndef MARLON_CLIENT_SCENE_DIFF_PROVIDER_H
#define MARLON_CLIENT_SCENE_DIFF_PROVIDER_H

#include "../graphics/graphics.h"

namespace marlon {
  namespace client {
    class Scene_diff_provider {
    public:
      virtual ~Scene_diff_provider() = default;

      virtual graphics::Scene_diff *get_scene_diff() const noexcept = 0;
    };
  }
}

#endif