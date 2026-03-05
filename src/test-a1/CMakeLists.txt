cmake_minimum_required(VERSION 3.12)

project(test-a1)

add_executable(${PROJECT_NAME}
    test-id-check.cpp
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC "crl::utils" #
    PUBLIC "crl::simAndControl" #
)

add_test(${PROJECT_NAME} "test-a1")
set_tests_properties(test-a1 PROPERTIES LABELS "a1")

add_executable(test-a1-id-check
    test-id-check.cpp
)
target_link_libraries(test-a1-id-check
    PUBLIC "crl::utils" #
    PUBLIC "crl::simAndControl" #
)
add_test(test-a1-id-check "test-a1-id-check")
set_tests_properties(test-a1-id-check PROPERTIES LABELS "a1")

add_executable(test-a1-ex2-fk-independence
    test-ex2-fk-independence.cpp
)
target_link_libraries(test-a1-ex2-fk-independence
    PUBLIC "crl::utils" #
    PUBLIC "crl::simAndControl" #
)
add_test(test-a1-ex2-fk-independence "test-a1-ex2-fk-independence")
set_tests_properties(test-a1-ex2-fk-independence PROPERTIES LABELS "a1")

add_executable(test-a1-ex2-fd-base-dofs
    test-ex2-fd-base-dofs.cpp
)
target_link_libraries(test-a1-ex2-fd-base-dofs
    PUBLIC "crl::utils" #
    PUBLIC "crl::simAndControl" #
)
add_test(test-a1-ex2-fd-base-dofs "test-a1-ex2-fd-base-dofs")
set_tests_properties(test-a1-ex2-fd-base-dofs PROPERTIES LABELS "a1")

add_executable(test-a1-ex2-traj-dt
    test-ex2-traj-dt.cpp
)
target_link_libraries(test-a1-ex2-traj-dt
    PUBLIC "crl::utils" #
    PUBLIC "crl::simAndControl" #
)
add_test(test-a1-ex2-traj-dt "test-a1-ex2-traj-dt")
set_tests_properties(test-a1-ex2-traj-dt PROPERTIES LABELS "a1")

add_executable(test-a1-ex2-traj-base-integration
    test-ex2-traj-base-integration.cpp
)
target_link_libraries(test-a1-ex2-traj-base-integration
    PUBLIC "crl::utils" #
    PUBLIC "crl::simAndControl" #
)
add_test(test-a1-ex2-traj-base-integration "test-a1-ex2-traj-base-integration")
set_tests_properties(test-a1-ex2-traj-base-integration PROPERTIES LABELS "a1")

add_executable(test-a1-ex3-analytic-base
    test-ex3-analytic-base.cpp
)
target_link_libraries(test-a1-ex3-analytic-base
    PUBLIC "crl::utils" #
    PUBLIC "crl::simAndControl" #
)
add_test(test-a1-ex3-analytic-base "test-a1-ex3-analytic-base")
set_tests_properties(test-a1-ex3-analytic-base PROPERTIES LABELS "a1")

add_executable(test-a1-ex3-analytic-all-limbs
    test-ex3-analytic-all-limbs.cpp
)
target_link_libraries(test-a1-ex3-analytic-all-limbs
    PUBLIC "crl::utils" #
    PUBLIC "crl::simAndControl" #
)
add_test(test-a1-ex3-analytic-all-limbs "test-a1-ex3-analytic-all-limbs")
set_tests_properties(test-a1-ex3-analytic-all-limbs PROPERTIES LABELS "a1")
