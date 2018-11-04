set -e

mkdir -p build
cd build
cmake ..
make -j4
cd ..

cd bin
if [ -f 'controller' ]; then
    cd resources/controller
    if [ ! -e 'kinova_graphics' ]; then
	ln -s ../../../resources/kinova_graphics .
    fi
    cd ../..
fi
cd ..

# Insert helper scripts into bin directory
cd bin

# Mico Controller + Simulator Script
if [ -f 'controller' ]; then
    cat <<EOF > init_controller.sh
sh set_gains.sh mico
./run_controller.sh controller resources/controller/world_mico.urdf resources/controller/mico.urdf mico
EOF
fi

# Mico Standalone Controller Script
if [ -f 'controller' ]; then
    cat <<EOF > init_standalone.sh
sh set_gains.sh mico
./run_standalone.sh controller resources/controller/world_mico.urdf resources/controller/mico.urdf mico
EOF
fi

# Set Gains Script
if [ -f 'controller' ]; then
    cat <<EOF > set_gains.sh
redis-cli flushall
redis-cli set cs225a::robot::\$1::tasks::kp_pos 5
redis-cli set cs225a::robot::\$1::tasks::kv_pos 2
redis-cli set cs225a::robot::\$1::tasks::kp_ori 5
redis-cli set cs225a::robot::\$1::tasks::kv_ori 2
redis-cli set cs225a::robot::\$1::tasks::kp_joint 5
redis-cli set cs225a::robot::\$1::tasks::kv_joint 2
redis-cli set cs225a::robot::\$1::tasks::kp_joint_init 5
redis-cli set cs225a::robot::\$1::tasks::kv_joint_init 2
redis-cli set cs225a::robot::\$1::tasks::jt_pos_des "4.71238898038 1.57079632679 1.57079632679 0.0 0.0 0.0"
EOF
fi

# Run Generic Controller + Simulator Script
if [ -f 'controller' ]; then
	cat <<EOF > run_controller.sh
if [ "\$#" -lt 4 ]; then
	cat <<EOM
This script calls ./visualizer, ./simulator, and the specified controller simultaneously.
All the arguments after the controller will be passed directly to it.

Usage: sh run.sh <controller-executable> <path-to-world.urdf> <path-to-robot.urdf> <robot-name> <extra_controller_args>
EOM
else
	trap 'kill %1; kill %2' SIGINT
	trap 'kill %1; kill %2' EXIT
	./simulator \$2 \$3 \$4 > simulator.log & ./visualizer \$2 \$3 \$4 > visualizer.log & ./"\$@"
fi
EOF
	chmod +x run_controller.sh
fi

# Run Generic Standalone Controller (No Simulator) Script
if [ -f 'controller' ]; then
	cat <<EOF > run_standalone.sh
if [ "\$#" -lt 4 ]; then
	cat <<EOM
This script calls ./visualizer and the specified controller simultaneously.
All the arguments after the controller will be passed directly to it.

Usage: sh run.sh <controller-executable> <path-to-world.urdf> <path-to-robot.urdf> <robot-name> <extra_controller_args>
EOM
else
	trap 'kill %1; kill %2' SIGINT
	trap 'kill %1; kill %2' EXIT
	./visualizer \$2 \$3 \$4 > visualizer.log & ./"\$@"
fi
EOF
	chmod +x run_standalone.sh
fi

cd ..
