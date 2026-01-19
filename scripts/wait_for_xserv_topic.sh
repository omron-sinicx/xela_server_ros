#!/bin/bash
# wait_for_xserv_topic.sh
# Waits until a message is received on /xServTopic

TOPIC="/xServTopic"
TIMEOUT=30  # seconds

echo "Waiting for $TOPIC to become active (timeout: ${TIMEOUT}s)..."

# Use rostopic echo with -n 1 to wait for one message
# timeout command ensures we don't wait forever
timeout $TIMEOUT rostopic echo -n 1 $TOPIC > /dev/null 2>&1

RET=$?

if [ $RET -eq 0 ]; then
    echo "$TOPIC detected. Starting node..."
    # Execute the command passed as arguments (the ROS node)
    exec "$@"
else
    echo "Timeout waiting for $TOPIC. Starting node anyway (or failing)..."
    # Even if timeout, we try to start, though it might fail or wait internally
    exec "$@"
fi
