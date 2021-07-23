set(tests
    attitude
    copyto
    dual
    filter
    hatvee
    helper
    integration
    inverse
    least_squares
    matrixAssignment
    matrixMult
    matrixScalarMult
    pseudoInverse
    setIdentity
    slice
    squareMatrix
    transpose
    upperRightTriangle
    vector
    vector2
    vector3
    vectorAssignment
)

add_custom_target(test_build)
foreach(test_name ${tests})
    add_executable(${test_name} ${test_name}.cpp)
    add_test(test_${test_name} ${test_name})
    add_dependencies(test_build ${test_name})
endforeach()

px4_add_unit_gtest(SRC sparseVector.cpp)
