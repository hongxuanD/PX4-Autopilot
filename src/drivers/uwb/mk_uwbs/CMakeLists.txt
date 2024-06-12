px4_add_module(
    MODULE drivers__mk_uwbs_main
    MAIN mk_uwbs
    STACK_MAIN 2000
    SRCS
        mk_uwbs.cpp
        mk_uwbs_main.cpp
    DEPENDS
        px4_work_queue
    MODULE_CONFIG
        module.yaml
    )
