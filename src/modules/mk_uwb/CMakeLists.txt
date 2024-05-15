px4_add_module(
    MODULE moudules__mk_uwb_main
    MAIN mk_uwb
    STACK_MAIN 2000
    SRCS
        mk_uwb.cpp
        mk_uwb_main.cpp
    DEPENDS
        px4_work_queue
    MODULE_CONFIG
        module.yaml
    )
