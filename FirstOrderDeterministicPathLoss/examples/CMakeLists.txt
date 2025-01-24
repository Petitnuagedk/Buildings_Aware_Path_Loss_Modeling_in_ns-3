build_lib_example(
    NAME FirstOrderDeterministicPathLoss-example
    SOURCE_FILES FOBAPathloss_1.cc FOBAPathloss_2.cc FOBAPathloss_3.cc
    LIBRARIES_TO_LINK ${libFirstOrderDeterministicPathLoss}
)
