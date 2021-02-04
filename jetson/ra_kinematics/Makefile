all: build_run

build :
	cp test/main_build.txt meson.build
	cd ../.. && pwd && ./jarvis build jetson/ra_kinematics/

build_run:
	cp test/main_build.txt meson.build
	cd ../.. && pwd && ./jarvis build jetson/ra_kinematics/ && ./jarvis exec jetson/ra_kinematics/

tests :
	cp test/arm_state_test_build.txt meson.build
	cd ../.. && pwd && ./jarvis build jetson/ra_kinematics/ && ./jarvis exec jetson/ra_kinematics/
	cp test/main_build.txt meson.build

arm_state_tests:
	cp test/arm_state_test_build.txt meson.build
	cd ../.. && pwd && ./jarvis build jetson/ra_kinematics/ && ./jarvis exec jetson/ra_kinematics/
	cp test/main_build.txt meson.build

kinematics_tests:
	cp test/kinematics_test_build.txt meson.build
	cd ../.. && pwd && ./jarvis build jetson/ra_kinematics/ && ./jarvis exec jetson/ra_kinematics/
	cp test/main_build.txt meson.build

motion_planner_tests:
	cp test/motion_planner_test_build.txt meson.build
	cd ../.. && pwd && ./jarvis build jetson/ra_kinematics/ && ./jarvis exec jetson/ra_kinematics/
	cp test/main_build.txt meson.build

exec :
	cd ../.. && ./jarvis exec jetson_ra_kinematics