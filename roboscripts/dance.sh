while :
do
    echo "j1: 0.444238, j2: 0.485596, j3: 0.569482, j4: 0.316602, j5: 0.600469, j6: 0.346924, safemode: 0, brake: 0, gripper: 0" > /dev/ttyUSB0
    sleep 2
    sh neutral_drive.sh
    sleep 2
done
