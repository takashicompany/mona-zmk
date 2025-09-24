if(CONFIG_ZMK_BEHAVIOR_OMNI_SCROLL)
  zephyr_library()
  zephyr_library_sources(src/behaviors/behavior_omni_scroll.c)
endif()