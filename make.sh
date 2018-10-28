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

# Jaco Controller Script
if [ -f 'controller' ]; then
    cat <<EOF > init_jaco.sh
redis-cli flushall
redis-cli set cs225a::robot::jaco::tasks::kp_pos 300
redis-cli set cs225a::robot::jaco::tasks::kv_pos 40
redis-cli set cs225a::robot::jaco::tasks::kp_ori 300
redis-cli set cs225a::robot::jaco::tasks::kv_ori 30
redis-cli set cs225a::robot::jaco::tasks::kp_joint 0
redis-cli set cs225a::robot::jaco::tasks::kv_joint 30
redis-cli set cs225a::robot::jaco::tasks::kp_joint_init 300
redis-cli set cs225a::robot::jaco::tasks::kv_joint_init 10
redis-cli set cs225a::robot::jaco::tasks::jt_pos_des "4.71238898038 1.57079632679 1.57079632679 0.0 0.0 0.0"
./run_controller.sh controller resources/controller/world_jaco.urdf resources/controller/jaco.urdf jaco
EOF
fi

# Mico Controller Script
if [ -f 'controller' ]; then
    cat <<EOF > init_mico.sh
redis-cli flushall
redis-cli set cs225a::robot::mico::tasks::kp_pos 300
redis-cli set cs225a::robot::mico::tasks::kv_pos 40
redis-cli set cs225a::robot::mico::tasks::kp_ori 300
redis-cli set cs225a::robot::mico::tasks::kv_ori 30
redis-cli set cs225a::robot::mico::tasks::kp_joint 0
redis-cli set cs225a::robot::mico::tasks::kv_joint 30
redis-cli set cs225a::robot::mico::tasks::kp_joint_init 300
redis-cli set cs225a::robot::mico::tasks::kv_joint_init 10
redis-cli set cs225a::robot::mico::tasks::jt_pos_des "1.48481 3.22206 4.90119 2.04572 4.81604 1.88954"
redis-cli set cs225a::robot::mico::tasks::jt_pos_des "4.71238898038 1.57079632679 1.57079632679 0.0 0.0 0.0" 
./run_controller.sh controller resources/controller/world_mico.urdf resources/controller/mico.urdf mico
EOF
fi

# Run Generic Controller Script
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

cd ..
