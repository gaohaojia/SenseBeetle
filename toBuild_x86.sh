cd src/tare_planner/or-tools/
git checkout x86
cd ../../..
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
