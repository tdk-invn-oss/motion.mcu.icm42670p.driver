cmake_minimum_required(VERSION 3.13)

project(motion.emd.icm42670p.driver)

function(configure_executable _name)
	message(STATUS "Configuring ${_name}")
	add_executable(${_name} 
	               ${CMAKE_CURRENT_SOURCE_DIR}/examples/${_name}/${_name}.c
	               ${_sources})

	target_include_directories(${_name} PRIVATE
	                           ${CMAKE_CURRENT_SOURCE_DIR}/icm42670p/
	                           ${CMAKE_CURRENT_SOURCE_DIR}/examples/)
endfunction()

set(_sources
    ${CMAKE_CURRENT_SOURCE_DIR}/icm42670p/imu/inv_imu.h
    ${CMAKE_CURRENT_SOURCE_DIR}/icm42670p/imu/inv_imu_apex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/icm42670p/imu/inv_imu_apex.h
    ${CMAKE_CURRENT_SOURCE_DIR}/icm42670p/imu/inv_imu_defs.h
    ${CMAKE_CURRENT_SOURCE_DIR}/icm42670p/imu/inv_imu_driver.c
    ${CMAKE_CURRENT_SOURCE_DIR}/icm42670p/imu/inv_imu_driver.h
    ${CMAKE_CURRENT_SOURCE_DIR}/icm42670p/imu/inv_imu_extfunc.h
    ${CMAKE_CURRENT_SOURCE_DIR}/icm42670p/imu/inv_imu_regmap_rev_a.h
    ${CMAKE_CURRENT_SOURCE_DIR}/icm42670p/imu/inv_imu_selftest.c
    ${CMAKE_CURRENT_SOURCE_DIR}/icm42670p/imu/inv_imu_selftest.h
    ${CMAKE_CURRENT_SOURCE_DIR}/icm42670p/imu/inv_imu_transport.c
    ${CMAKE_CURRENT_SOURCE_DIR}/icm42670p/imu/inv_imu_transport.h
    ${CMAKE_CURRENT_SOURCE_DIR}/icm42670p/imu/inv_imu_version.h
    ${CMAKE_CURRENT_SOURCE_DIR}/examples/system_interface.h
    ${CMAKE_CURRENT_SOURCE_DIR}/examples/system_interface.c)

configure_executable(eis)
configure_executable(freefall)
configure_executable(pedometer)
configure_executable(raw)
configure_executable(selftest)
configure_executable(smd)
configure_executable(tilt)
configure_executable(wom)

