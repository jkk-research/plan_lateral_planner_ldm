dc_add_to_target(
    INCLUDE_IF

    SOURCES
        ./emg_linearDriverModel.cpp
        ./emg_linearDriverModel.hpp
        ./emg_linearDriverModel_controlLogic.cpp
        ./emg_linearDriverModel_controlLogic.hpp
        ./emg_linearDriverModel_interfaces.hpp
        ./linearDriverModelDriverModel/emg_linearDriverModel_driverModel.cpp
        ./linearDriverModelDriverModel/emg_linearDriverModel_driverModel.hpp
        ./linearDriverModelPlanner/emg_linearDriverModel_segmentPlanner.cpp
        ./linearDriverModelPlanner/emg_linearDriverModel_segmentPlanner.hpp
        ./linearDriverModelPlanner/emg_linearDriverModel_trajectoryEvaluation.cpp
        ./linearDriverModelPlanner/emg_linearDriverModel_trajectoryEvaluation.hpp
        ./linearDriverModelPlanner/emg_linearDriverModel_trajectoryPlanner.cpp
        ./linearDriverModelPlanner/emg_linearDriverModel_trajectoryPlanner.hpp
        ./linearDriverModelUtilities/emg_linearDriverModel_clothoidSubfunctions.cpp
        ./linearDriverModelUtilities/emg_linearDriverModel_clothoidSubfunctions.hpp
        ./linearDriverModelUtilities/emg_linearDriverModel_coordinateTransforms.cpp
        ./linearDriverModelUtilities/emg_linearDriverModel_coordinateTransforms.hpp
        ./linearDriverModelUtilities/emg_linearDriverModel_polynomialSubfunctions.cpp
        ./linearDriverModelUtilities/emg_linearDriverModel_polynomialSubfunctions.hpp
    SOURCES_TESTS

    TESTS

    UNIT_TESTS

    UNIT_TESTS_WITHOUT_ASSERTIONS

    UNIT_STANDALONE_TESTS

    UNIT_STANDALONE_TESTS_WITHOUT_ASSERTIONS

    CANTATA_TESTS

    CANTATA_TESTS_WITHOUT_ASSERTIONS

    CANTATA_STANDALONE_TESTS

    CANTATA_STANDALONE_TESTS_WITHOUT_ASSERTIONS

)
