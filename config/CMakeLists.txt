# Add custom behavior sources
if(CONFIG_ZMK_POINTING)
  target_sources(app PRIVATE src/behaviors/behavior_omni_scroll.c)
endif()