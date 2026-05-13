#!/bin/bash


ros2 topic pub -1 /manip_node/pickup custom_msgs/msg/Blocks "{colors: [1, 1, 1, 1], count: 4}"
ros2 topic echo --once /manip_node/status | while read line; do
	code=$(echo "$line" | grep -o '[0-9]*')
	case $code in
		1) echo "Pickup started" ;;
		2) echo "Pickup done" ; break ;;
		3) echo "Pickup failed" ; break ;;
	esac
done

# read -p "Press Enter to continue..."


ros2 topic pub -1 /manip_node/home std_msgs/msg/UInt8 "{data: 1}"
ros2 topic echo --once /manip_node/status | while read line; do
	code=$(echo "$line" | grep -o '[0-9]*')
	case $code in
		7) echo "Home started" ;;
		8) echo "Home done" ; break ;;
		9) echo "Home failed" ; break ;;
	esac
done

# read -p "Press Enter to continue..."


ros2 topic pub -1 /manip_node/flip custom_msgs/msg/Blocks "{colors: [1, 2, 2, 1], count: 4}"
ros2 topic echo --once /manip_node/status | while read line; do
	code=$(echo "$line" | grep -o '[0-9]*')
	case $code in
		4) echo "Flip started" ;;
		5) echo "Flip done" ; break ;;
		6) echo "Flip failed" ; break ;;
	esac
done