# Wait for /gazebo node to start
echo "Waiting for /gazebo node to start..."
until ros2 node list | grep -q "/gazebo"; do
    sleep 1
done
echo "/gazebo node has started!"
