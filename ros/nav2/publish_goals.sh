#!/bin/bash
ros2 action send_goal /navigate_through_poses nav2_msgs/action/NavigateThroughPoses "{
  poses: [
    {
      header: { frame_id: 'map' },
      pose: {
        position: { x: -1.5, y: 1.0, z: 0.0 },
        orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
      }
    },
    {
      header: { frame_id: 'map' },
      pose: {
        position: { x: 0.5, y: -2.0, z: 0.0 },
        orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
      }
    },
    {
      header: { frame_id: 'map' },
      pose: {
        position: { x: 2.0, y: 2.5, z: 0.0 },
        orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
      }
    }
  ]
}"

