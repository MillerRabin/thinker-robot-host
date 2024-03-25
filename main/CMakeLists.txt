idf_component_register(SRCS "main.cpp"  
  "altitude/altitude.cpp"
  "powerManagement/powerManagement.cpp"  
  "bus/bus.cpp"
  "detectors/detectors.cpp"  
  INCLUDE_DIRS ".")
  target_compile_options(${COMPONENT_LIB} PRIVATE -std=gnu++11)