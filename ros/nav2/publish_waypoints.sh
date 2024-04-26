#!/bin/bash

ros2 action send_goal /follow_waypoints nav2_msgs/action/FollowWaypoints "{
  poses: [
    {
      header: { frame_id: 'map' },
      pose: {
        position: { x: -2.0, y: 1.5, z: 0.0 },
        orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
      }
    },
    {
      header: { frame_id: 'map' },
      pose: {
        position: { x: 1.0, y: -1.5, z: 0.0 },
        orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
      }
    },
    {
      header: { frame_id: 'map' },
      pose: {
        position: { x: -1.0, y: 2.0, z: 0.0 },
        orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
      }
    }
  ]
}"
