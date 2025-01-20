check_include_file_cxx(stdint.h HAVE_STDINT_H)
if(HAVE_STDINT_H)
    add_definitions(-DHAVE_STDINT_H)
endif()

set(examples_as_tests_sources)
if(${ENABLE_EXAMPLES})
    set(examples_as_tests_sources
        #test/FirstOrderDeterministicPathLoss-examples-test-suite.cc
        )
endif()

build_lib(
    LIBNAME FirstOrderDeterministicPathLoss
    SOURCE_FILES model/first-order-building-aware-propagation-loss-model.cc
                 model/NLOSassess.cc
                 #helper/FirstOrderDeterministicPathLoss-helper.cc
    HEADER_FILES model/first-order-building-aware-propagation-loss-model.h
                 model/NLOSassess.h
                 #helper/FirstOrderDeterministicPathLoss-helper.h
    LIBRARIES_TO_LINK ${libcore}
    TEST_SOURCES test/FirstOrderDeterministicPathLoss-test-suite.cc
                 ${examples_as_tests_sources}
)
