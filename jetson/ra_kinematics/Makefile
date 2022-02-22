.PHONY: all build build_run unit_tests arm_state_tests kinematics_tests motion_planner_tests config_space_test exe

all: build_run

build :
	cp test/main_build.txt meson.build
	cd ../.. && pwd && ./jarvis build jetson/ra_kinematics/

build_run:
	cp test/main_build.txt meson.build
	cd ../.. && pwd && ./jarvis build jetson/ra_kinematics/ && ./jarvis exec jetson/ra_kinematics/

unit_tests: arm_state_tests kinematics_tests motion_planner_tests

arm_state_tests:
	cp test/arm_state_test_build.txt meson.build
	-cd ../.. && pwd && ./jarvis build jetson/ra_kinematics/ && ./jarvis exec jetson/ra_kinematics/
	cp test/main_build.txt meson.build

kinematics_tests:
	cp test/kinematics_test_build.txt meson.build
	-cd ../.. && pwd && ./jarvis build jetson/ra_kinematics/ && ./jarvis exec jetson/ra_kinematics/
	cp test/main_build.txt meson.build

motion_planner_tests:
	cp test/motion_planner_test_build.txt meson.build
	-cd ../.. && pwd && ./jarvis build jetson/ra_kinematics/ && ./jarvis exec jetson/ra_kinematics/
	cp test/main_build.txt meson.build

config_space_test:
	cp test/config_space_test_build.txt meson.build
	-cd ../.. && pwd && ./jarvis build jetson/ra_kinematics/ && ./jarvis exec jetson/ra_kinematics/
	cp test/main_build.txt meson.build

exec :
	cd ../.. && ./jarvis exec jetson_ra_kinematics
