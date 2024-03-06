idf_component_register(SRCS "main.cpp"  
  "altitude/altitude.cpp"
  "twai/twai.cpp"  
  INCLUDE_DIRS ".")
  target_compile_options(${COMPONENT_LIB} PRIVATE -std=gnu++11)