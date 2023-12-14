
# How to run

1. Create log folder
    ```
    mkdir -p log/latest
    ```
2. Source ROS 2 and Autoware
    ```
    source /opt/ros/humble/setup.bash
    source autoware/install/setup.bash
    ```
3. Run test script
    ```
    python3 test_ss2_topics_hz.py
    ```
4. In another terminal, run ss2 e.g.
    ```
    ros2 launch scenario_test_runner scenario_test_runner.launch.py \
        architecture_type:=awf/universe \
        record:=false \
        scenario:='$(find-pkg-share scenario_test_runner)/scenario/sample.yaml' \
        sensor_model:=sample_sensor_kit \
        vehicle_model:=sample_vehicle
    ```
5. Check results in `log/latest/topic_hz.csv`
