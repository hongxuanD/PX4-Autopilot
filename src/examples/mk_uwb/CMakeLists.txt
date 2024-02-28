px4_add_module(
    MODULE examples__mk_uwb_main
    MAIN mk_uwb
    STACK_MAIN 2000
    SRCS
        mk_uwb.cpp
        mk_uwb_main.cpp
    DEPENDS
    )
