#pragma once

#include <atomic>

namespace rviz_global_relocalize
{

/**
 * Shared state to coordinate TF publishing between tools.
 * When MockNaviTool is publishing TF, SetBaselinkTool should stop.
 */
class TfPublisherState
{
public:
  static TfPublisherState & instance()
  {
    static TfPublisherState state;
    return state;
  }
  
  void setMockNaviActive(bool active)
  {
    mock_navi_active_.store(active);
  }
  
  bool isMockNaviActive() const
  {
    return mock_navi_active_.load();
  }

private:
  TfPublisherState() : mock_navi_active_(false) {}
  std::atomic<bool> mock_navi_active_;
};

} // namespace rviz_global_relocalize
