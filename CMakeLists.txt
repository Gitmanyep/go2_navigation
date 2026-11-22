cmake_minimum_required(VERSION 3.0.2)
project(go2_navigation)
find_package(catkin REQUIRED)
catkin_package()
install(PROGRAMS scripts/go2-navigation-node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY navigation control scripts bin launch provenance docs config tests
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        USE_SOURCE_PERMISSIONS
        PATTERN "__pycache__" EXCLUDE PATTERN "logs" EXCLUDE PATTERN "runs" EXCLUDE)
install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
